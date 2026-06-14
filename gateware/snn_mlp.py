#!/usr/bin/env python3
"""LiteX wrapper for the SNN-MLP MNIST inference core.

Instantiates verilog/snn_mlp_core.v and exposes it as a LiteX peripheral with:

- control / status CSRs
- pixel write port (host writes 784 Q4.12 pixel values into the core's BRAM)
- bias write port (host writes HIDDEN + OUT_SIZE Q4.12 bias values once)
- weight base pointer + per-cycle/cycle-count CSRs (configured once)
- spike-count readback (per-output 8-bit counters, packed)
- a burst weight loader: a Migen address-gen FSM feeding a LiteDRAMDMAReader
  on a dedicated native SDRAM port (replaces the old single-beat Wishbone
  verilog/snn_weight_loader.v -- now unused). Pipelined reads stream the
  weight blob as row-hit bursts at ~SDRAM bandwidth, which is what lets the
  half-rate (2x) SDRAM clock actually speed up weight loading. The core's
  stream protocol (w_valid/w_ready/w_data) is unchanged.

Per-image flow from the firmware:
    write 784 pixels via pixel_addr / pixel_data / pixel_ctl.we
    pulse control.clear_state (resets membrane between images)
    pulse control.start (kicks core + loader together)
    poll status.done
    read status.classification and spike_counts
"""
from __future__ import annotations

import os

from migen import *
from migen.genlib.fsm import FSM, NextState, NextValue
from litex.gen import LiteXModule
from litex.soc.interconnect.csr import AutoCSR, CSRField, CSRStatus, CSRStorage

from litedram.frontend.dma import LiteDRAMDMAReader


class SNNMLP(LiteXModule, AutoCSR):
    """Trainable-weight LIF SNN-MLP, sized for 784->64->10 MNIST by default."""

    def __init__(
        self,
        platform,
        dram_port,
        in_size: int = 784,
        hidden: int = 64,
        out_size: int = 10,
        timesteps: int = 25,
        n_mac: int = 2,   # 2 Q4.12 weights per 32-bit Wishbone word; see icepi_zero_mnist.py
        data_width: int = 16,
        frac_bits: int = 12,
        beta_shift: int = 3,
        spk_width: int = 8,
    ):
        # Verilog sources --------------------------------------------------
        repo_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
        # snn_weight_loader.v is superseded by the DMA loader below.
        platform.add_source(os.path.join(repo_root, "verilog/snn_mlp_core.v"))

        self.n_mac = n_mac
        self.out_size = out_size
        self.spk_width = spk_width

        # --- CSRs ---------------------------------------------------------
        self.control = CSRStorage(fields=[
            CSRField("start", size=1, pulse=True,
                     description="Pulse to begin inference. Triggers core + loader together."),
            CSRField("clear_state", size=1, pulse=True,
                     description="Pulse to clear membrane / spike state (between images)."),
        ])

        self.status = CSRStatus(fields=[
            CSRField("busy", size=1, description="snn_mlp_core busy"),
            CSRField("done", size=1, description="snn_mlp_core inference complete"),
            CSRField("loader_busy", size=1, description="snn_weight_loader busy"),
            CSRField("loader_done", size=1, description="snn_weight_loader completed"),
            CSRField("classification_valid", size=1),
            CSRField("classification", size=4, description="argmax of output spike counts"),
        ])

        # Weight stream layout — configured once at boot.
        # The core hoists layer-1 out of the timestep loop, so the loader streams
        # the W1 prefix (weight_preamble_beats) ONCE, then the W2 block
        # (weight_beats_per_cycle) weight_num_cycles times. The W2 block lives
        # right after the W1 prefix in SDRAM, so no second base address is needed.
        self.weight_base = CSRStorage(
            32, description="SDRAM byte address of the packed weight blob (W1 prefix then W2 block)."
        )
        self.weight_preamble_beats = CSRStorage(
            32, description="W1 beats streamed once up front (= L1_TILES*IN_SIZE). 0 = legacy single-segment."
        )
        self.weight_beats_per_cycle = CSRStorage(
            32, description="W2 beats consumed per timestep (= L2_TILES*HIDDEN)."
        )
        self.weight_num_cycles = CSRStorage(
            32, description="Number of timesteps to replay the W2 block (T)."
        )

        # Pixel and bias write ports — host writes addr then data then pulses we.
        self.pixel_ctl = CSRStorage(fields=[
            CSRField("we", size=1, pulse=True,
                     description="Pulse to commit pixel_addr -> pixel_mem[pixel_addr] = pixel_data."),
        ])
        self.pixel_addr = CSRStorage(16)
        self.pixel_data = CSRStorage(data_width)

        self.bias_ctl = CSRStorage(fields=[
            CSRField("we", size=1, pulse=True,
                     description="Pulse to commit bias_addr -> bias_mem[bias_addr] = bias_data."),
        ])
        self.bias_addr = CSRStorage(16)
        self.bias_data = CSRStorage(data_width)

        # Spike count readback: one CSRStatus per output neuron. Keeps each
        # accessor a clean single-word read from the firmware side and
        # sidesteps multi-word CSR endianness on the packed alternative.
        for i in range(out_size):
            name = f"spike_count_{i}"
            setattr(self, name, CSRStatus(spk_width, name=name,
                description=f"Spike count for output neuron {i} after the last inference."))

        # --- Shared signals between loader and core ----------------------
        w_valid_int = Signal()
        w_ready_int = Signal()
        w_data_int  = Signal(n_mac * data_width)

        core_busy = Signal()
        core_done = Signal()
        cls = Signal(4)
        cls_valid = Signal()
        spk_packed = Signal(out_size * spk_width)

        # --- Burst weight loader: address-gen FSM + LiteDRAMDMAReader -----
        # Reads the weight blob from a dedicated native SDRAM port (bypassing
        # the CPU's Wishbone/L2 path). The port width is fixed at 32 bits to
        # mirror the old WB_DATA_WIDTH=32: each beat is one 32-bit word and the
        # core takes the low N_MAC*DATA_WIDTH bits. The DMA reader pipelines
        # reads (internal FIFO), so the blob streams as row-hit bursts.
        DMA_WORD_BITS = 32
        assert dram_port.data_width == DMA_WORD_BITS, (
            f"SNN DMA needs a {DMA_WORD_BITS}-bit native port, got "
            f"{dram_port.data_width} (see add_snn_mlp / SDRAM controller width)."
        )
        assert n_mac * data_width <= DMA_WORD_BITS

        self.dma = dma = LiteDRAMDMAReader(dram_port, fifo_depth=32)

        aw    = dram_port.address_width
        shift = (DMA_WORD_BITS // 8).bit_length() - 1   # byte addr -> word addr (==2)

        # CSRs carry SDRAM BYTE addresses; the native port is 32-bit-word
        # addressed. preamble/beats counts are already in words (== beats).
        base_word = Signal(aw)
        self.comb += base_word.eq(self.weight_base.storage[shift:shift + aw])
        preamble = self.weight_preamble_beats.storage
        bpc      = self.weight_beats_per_cycle.storage
        ncyc     = self.weight_num_cycles.storage

        addr      = Signal(aw)
        cyc_base  = Signal(aw)   # word base of the repeated (W2) block
        beat_idx  = Signal(32)
        cycle_idx = Signal(32)
        in_pre    = Signal()     # streaming the one-shot W1 prefix

        # Outstanding native reads (issued to sink, not yet drained at source).
        # Bounded by the reader's internal FIFOs (sink backpressures), so 8
        # bits is ample; lets us know when the last word has reached the core.
        outstanding = Signal(8)
        sink_fire   = Signal()
        source_fire = Signal()
        self.comb += [
            sink_fire.eq(dma.sink.valid & dma.sink.ready),
            source_fire.eq(dma.source.valid & dma.source.ready),
        ]
        self.sync += [
            If(sink_fire & ~source_fire, outstanding.eq(outstanding + 1)),
            If(source_fire & ~sink_fire, outstanding.eq(outstanding - 1)),
        ]

        # Pipe DMA data -> core stream (low N_MAC*DATA_WIDTH bits per beat).
        self.comb += [
            w_valid_int.eq(dma.source.valid),
            dma.source.ready.eq(w_ready_int),
            w_data_int.eq(dma.source.data[:n_mac * data_width]),
        ]

        # Address generator: same schedule as the old verilog loader --
        # `preamble` words from base_word ONCE, then `bpc` words replayed
        # `ncyc` times from cyc_base (= base_word + preamble). preamble=0
        # collapses to the legacy single-segment behavior.
        self.gen_fsm = gen = FSM(reset_state="IDLE")
        gen.act("IDLE",
            If(self.control.fields.start,
                NextValue(addr, base_word),
                NextValue(cyc_base, base_word + preamble[:aw]),
                NextValue(in_pre, preamble != 0),
                NextValue(beat_idx, 0),
                NextValue(cycle_idx, 0),
                NextState("GEN"),
            )
        )
        gen.act("GEN",
            dma.sink.valid.eq(1),
            dma.sink.address.eq(addr),
            If(dma.sink.ready,
                If(in_pre,
                    If(beat_idx == (preamble - 1),
                        NextValue(in_pre, 0),
                        NextValue(beat_idx, 0),
                        NextValue(cycle_idx, 0),
                        NextValue(addr, cyc_base),
                    ).Else(
                        NextValue(beat_idx, beat_idx + 1),
                        NextValue(addr, addr + 1),
                    )
                ).Elif(beat_idx == (bpc - 1),
                    If(cycle_idx == (ncyc - 1),
                        NextState("DRAIN"),
                    ).Else(
                        NextValue(cycle_idx, cycle_idx + 1),
                        NextValue(beat_idx, 0),
                        NextValue(addr, cyc_base),
                    )
                ).Else(
                    NextValue(beat_idx, beat_idx + 1),
                    NextValue(addr, addr + 1),
                )
            )
        )
        # All addresses issued; wait for the last data word to reach the core.
        gen.act("DRAIN",
            If(outstanding == 0, NextState("IDLE")),
        )

        # Status: busy from start through drain; done sticky until next start
        # (matches the old loader's level semantics).
        loader_busy = Signal()
        loader_done = Signal()
        self.comb += loader_busy.eq(~gen.ongoing("IDLE"))
        self.sync += [
            If(gen.ongoing("DRAIN") & (outstanding == 0), loader_done.eq(1)),
            If(self.control.fields.start, loader_done.eq(0)),
        ]

        # --- Instantiate the core ----------------------------------------
        self.specials += Instance(
            "snn_mlp_core",
            p_IN_SIZE    = in_size,
            p_HIDDEN     = hidden,
            p_OUT_SIZE   = out_size,
            p_TIMESTEPS  = timesteps,
            p_N_MAC      = n_mac,
            p_DATA_WIDTH = data_width,
            p_FRAC_BITS  = frac_bits,
            p_BETA_SHIFT = beta_shift,
            p_SPK_WIDTH  = spk_width,

            i_clk = ClockSignal(),
            i_rst = ResetSignal(),

            i_start       = self.control.fields.start,
            i_clear_state = self.control.fields.clear_state,
            o_busy        = core_busy,
            o_done        = core_done,

            i_pixel_we   = self.pixel_ctl.fields.we,
            i_pixel_addr = self.pixel_addr.storage,
            i_pixel_data = self.pixel_data.storage,
            i_bias_we    = self.bias_ctl.fields.we,
            i_bias_addr  = self.bias_addr.storage,
            i_bias_data  = self.bias_data.storage,

            i_w_valid = w_valid_int,
            o_w_ready = w_ready_int,
            i_w_data  = w_data_int,

            o_classification        = cls,
            o_classification_valid  = cls_valid,
            o_spike_counts_packed   = spk_packed,
        )

        # --- Status routing ----------------------------------------------
        self.comb += [
            self.status.fields.busy.eq(core_busy),
            self.status.fields.done.eq(core_done),
            self.status.fields.loader_busy.eq(loader_busy),
            self.status.fields.loader_done.eq(loader_done),
            self.status.fields.classification_valid.eq(cls_valid),
            self.status.fields.classification.eq(cls),
        ]
        for i in range(out_size):
            csr = getattr(self, f"spike_count_{i}")
            self.comb += csr.status.eq(spk_packed[i*spk_width:(i+1)*spk_width])
