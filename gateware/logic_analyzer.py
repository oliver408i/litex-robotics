"""Streaming logic-analyzer capture core for the IcePi Zero LA bitstream.

Samples a wide bus of external 3.3 V probe pins into a ring buffer in SDRAM,
with a programmable sample rate (decimator), a maskable pattern/edge trigger
and configurable pre/post-trigger depth. Firmware drains the ring out of SDRAM
and ships it to the host over the WINC (see docs/logic_analyzer.md).

Architecture (all in `sys`, 50 MHz):

    probe pins --2FF--> decimator --> elastic FIFO --> LiteDRAMDMAWriter --> SDRAM ring

  * 2FF (MultiReg) sync: the probe pins are asynchronous external signals; they
    MUST be synchronized before sampling or metastability corrupts the capture.
    This costs a fixed 2-cycle latency, uniform across channels (no added skew).
  * Decimator: sample once every (sample_div+1) sys cycles -> rate = sys/(div+1).
  * Elastic FIFO: absorbs SDRAM refresh / arbitration gaps so a steady sample
    rate doesn't stall on DRAM latency. If a sample arrives while the FIFO is
    full, `overrun` latches (the chosen rate exceeds sustainable DRAM bandwidth).
  * DMA writer: one 32-bit word per sample (channels zero-extended), addressed
    as a ring [ring_base, ring_base + 4*ring_size).

Capture model (benchtop-LA style): sample FAST into the deep SDRAM ring, then
offload SLOWLY over WiFi. The ring keeps the last `ring_size` samples; on
trigger it captures `post_trig` more samples then stops, so the buffer holds a
pre/post window around the trigger. See docs/logic_analyzer.md for the readback
math (wptr / trig_addr / wrapped).

STATUS: first cut, NOT yet simulated or hardware-verified. Bring it up with a
cocotb bench (sim/cocotb) before trusting capture timing/trigger placement.
"""
from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import AutoCSR, CSR, CSRStorage, CSRStatus, CSRField

from litedram.frontend.dma import LiteDRAMDMAWriter


class LogicAnalyzer(LiteXModule, AutoCSR):
    """N-channel SDRAM-streaming logic analyzer.

    dram_port  : a LiteDRAM native WRITE port (data_width must be >= n_channels;
                 32 in the current build).
    probe      : Signal(n_channels) of raw async probe inputs (board IO pins).
    guard_mask : channels that must read LOW for "no module attached". With an
                 internal pull-down on those pins, an external on-module pull-up
                 (e.g. the touch I2C lines) reads HIGH -> we infer a still-wired
                 LCD/touch module and expose `status.lcd_present` so firmware can
                 refuse to arm. 0 disables the guard.
    """
    def __init__(self, dram_port, probe, n_channels, guard_mask=0, fifo_depth=512):
        assert dram_port.data_width >= n_channels, \
            f"DMA port data_width {dram_port.data_width} < n_channels {n_channels}"
        self.probe = probe

        # ---- CSRs --------------------------------------------------------------------------------
        self._arm        = CSR()  # write-pulse: reset state and start a capture
        self._abort      = CSR()  # write-pulse: stop immediately (-> done)
        self._sample_div = CSRStorage(32, description=
            "Sample period = (sample_div+1) sys clocks. Rate = sys_clk/(sample_div+1). "
            "0 = full rate (one sample per sys clock).")
        self._trig_mask  = CSRStorage(n_channels, description=
            "Trigger care-mask: only channels with a 1 here participate. 0 => trigger immediately.")
        self._trig_value = CSRStorage(n_channels, description="Trigger match value (under trig_mask).")
        self._trig_edge  = CSRStorage(1, description=
            "1: fire on the transition INTO match (one-shot). 0: fire on any matching sample (level).")
        self._ring_base  = CSRStorage(32, description=
            "Ring buffer base as a BYTE offset into SDRAM from MAIN_RAM_BASE (4-byte aligned).")
        self._ring_size  = CSRStorage(32, description="Ring buffer length in SAMPLES (32-bit words).")
        self._post_trig  = CSRStorage(32, description="Samples to capture AFTER the trigger fires.")
        self._status     = CSRStatus(fields=[
            CSRField("running",     size=1),
            CSRField("triggered",   size=1),
            CSRField("done",        size=1),
            CSRField("overrun",     size=1, description="Sample rate outran SDRAM; samples were dropped."),
            CSRField("wrapped",     size=1, description="Write pointer wrapped at least once (whole ring valid)."),
            CSRField("lcd_present", size=1, description="Guard: an LCD/touch module appears to still be attached."),
        ])
        self._wr_count   = CSRStatus(32, description="Total samples written this run.")
        self._wptr       = CSRStatus(32, description="Current/final write pointer (sample index within ring).")
        self._trig_addr  = CSRStatus(32, description="Sample index within ring where the trigger fired.")
        self._probe_live = CSRStatus(n_channels, description="Live synchronized probe value (debug + guard read).")

        # ---- synchronize async probe inputs ------------------------------------------------------
        probe_sync = Signal(n_channels)
        self.specials += MultiReg(probe, probe_sync)
        self.comb += self._probe_live.status.eq(probe_sync)
        if guard_mask:
            self.comb += self._status.fields.lcd_present.eq((probe_sync & guard_mask) != 0)

        # ---- DMA writer + elastic capture FIFO ---------------------------------------------------
        self.dma = dma = LiteDRAMDMAWriter(dram_port, fifo_depth=16, fifo_buffered=True)
        self.cap_fifo = cap = stream.SyncFIFO([("data", n_channels)], fifo_depth, buffered=True)

        running   = Signal()
        triggered = Signal()
        done      = Signal()
        overrun   = Signal()
        wrapped   = Signal()
        prev_match= Signal()
        wptr      = Signal(32)
        trig_addr = Signal(32)
        post_cnt  = Signal(32)
        div_cnt   = Signal(32)
        wr_count  = Signal(32)

        self.comb += [
            self._status.fields.running.eq(running),
            self._status.fields.triggered.eq(triggered),
            self._status.fields.done.eq(done),
            self._status.fields.overrun.eq(overrun),
            self._status.fields.wrapped.eq(wrapped),
            self._wr_count.status.eq(wr_count),
            self._wptr.status.eq(wptr),
            self._trig_addr.status.eq(trig_addr),
        ]

        # SDRAM word address of the current ring slot (byte base -> 32-bit words).
        word_base = Signal(dram_port.address_width)
        self.comb += word_base.eq(self._ring_base.storage[2:])

        # ---- decimator: one sample strobe every (sample_div+1) sys clocks ------------------------
        samp = Signal()
        self.sample_stb = samp  # debug tap for the sim bench (sample-taken strobe)
        self.comb += samp.eq(running & (div_cnt == 0))
        self.sync += If(running,
            If(div_cnt == 0, div_cnt.eq(self._sample_div.storage)).Else(div_cnt.eq(div_cnt - 1)),
        ).Else(div_cnt.eq(0))

        # ---- push samples into the elastic FIFO; flag overrun if it can't keep up ----------------
        self.comb += [
            cap.sink.valid.eq(samp),
            cap.sink.data.eq(probe_sync),
        ]

        # ---- DMA feed: pop FIFO, generate ring address, write one word per sample ----------------
        beat  = Signal()  # a sample word was accepted into the DRAM write path this cycle
        match = Signal()
        fire  = Signal()
        self.comb += [
            dma.sink.data.eq(cap.source.data),         # zero-extended to port data_width
            dma.sink.address.eq(word_base + wptr),
            dma.sink.valid.eq(cap.source.valid & ~done),
            cap.source.ready.eq(dma.sink.ready & ~done),
            beat.eq(dma.sink.valid & dma.sink.ready),
            match.eq((cap.source.data & self._trig_mask.storage) ==
                     (self._trig_value.storage & self._trig_mask.storage)),
            fire.eq(Mux(self._trig_edge.storage, match & ~prev_match, match)),
        ]

        # ---- control + ring / trigger bookkeeping ------------------------------------------------
        self.sync += [
            If(self._arm.re,
                running.eq(1), triggered.eq(0), done.eq(0), wrapped.eq(0),
                wptr.eq(0), wr_count.eq(0), prev_match.eq(0), trig_addr.eq(0),
            ).Elif(self._abort.re,
                running.eq(0), done.eq(1),
            ).Elif(beat,
                prev_match.eq(match),
                wr_count.eq(wr_count + 1),
                # advance ring pointer with wrap
                If(wptr + 1 >= self._ring_size.storage,
                    wptr.eq(0), wrapped.eq(1),
                ).Else(
                    wptr.eq(wptr + 1),
                ),
                # trigger detection (one-shot latch)
                If(running & ~triggered & fire,
                    triggered.eq(1),
                    trig_addr.eq(wptr),
                    post_cnt.eq(self._post_trig.storage),
                ),
                # post-trigger countdown -> stop
                If(triggered,
                    If(post_cnt == 0,
                        running.eq(0), done.eq(1),
                    ).Else(
                        post_cnt.eq(post_cnt - 1),
                    ),
                ),
            ),
            # overrun is sticky for the run; cleared only on arm.
            If(self._arm.re,
                overrun.eq(0),
            ).Elif(samp & ~cap.sink.ready,
                overrun.eq(1),
            ),
        ]
