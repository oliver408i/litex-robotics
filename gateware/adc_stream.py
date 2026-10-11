"""Hardware-timed single-channel ADC capture, streamed to a host over raw UDP.

The radar capture path. One MCP3008 channel is sampled by a free-running rate
generator in gateware -- never by the CPU -- and the samples go straight out a
UDP socket in ~1 KB packets. Nothing on the board looks at the data; all DSP
happens on the host (`tools/adc_stream_rx.py`).

**Why the CPU is not in the loop.** The measurement here IS frequency: a
Doppler shift of 161 Hz per m/s. Sample-interval jitter smears every bin of the
FFT, and a CPU that polls an ADC between interrupts has jitter measured in
microseconds. The rate generator below is a counter off `sys`, so the interval
is exact to one sys clock and the only error is the crystal's.

**Why UDP and not Etherbone.** Etherbone is request/response -- the host asks
for each word -- and bulk transfers over it have wedged the Wishbone bus on
this board. This is push: the board emits, the host holds a socket open. The
two coexist (different UDP ports on the same core), so Etherbone stays
available for poking CSRs while a capture runs.

**Loss is made visible, because a silent drop looks exactly like signal.**
Every packet carries a sequence number, the running count of samples already
sent, and the running count of samples the gateware had to discard. The two
failure modes are therefore distinguishable at the host:

  * a gap in `seq`/`emitted` with `dropped` unchanged -- the *network* lost a
    packet (switch, socket buffer, host too slow to drain);
  * `dropped` climbing -- the *board* could not keep up, because the sample
    rate outran the link or the TX path stalled.

Neither can be mistaken for a quiet patch in the waveform, which is the whole
point.

Packet layout -- 8 header words then the samples, all 32-bit little-endian:

    w0  magic 0x43444152, i.e. the bytes "RADC" in wire order
    w1  seq            packet counter, +1 per packet, wraps at 2^32
    w2  emitted        samples successfully sent BEFORE this packet
    w3  dropped        samples discarded by the gateware, running total
    w4  n_samples[15:0] | channel[18:16] | version[31:24]
    w5  period         sys clocks between conversions
    w6  sys_clk_freq   Hz, so the host can compute the true sample rate
    w7  reserved, 0
    then n_samples/2 words, each holding two 10-bit samples right-justified in
    16 bits, earlier sample in the low half.

`sys_clk_freq / period` is the sample rate. It is sent rather than assumed
because `period` is a CSR and the rate is not a build constant.
"""
from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule
from litex.build.generic_platform import Pins, Subsignal, IOStandard, Misc
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus

from gateware.mcp3008 import COLORLIGHT_I9_MCP3008_PINS, _mcp3008_io, sclk_div_for


def _pack_ip(s):
    """"a.b.c.d" -> the 32-bit form LiteEth uses (first octet in the high byte).

    Same result as liteeth.common.convert_ip, open-coded so this module -- and
    the simulation bench that imports it -- does not need liteeth on the path.
    """
    if not isinstance(s, str):
        return s
    ip = 0
    for octet in s.split("."):
        ip = (ip << 8) | int(octet)
    return ip


MAGIC   = 0x43444152   # "RADC" little-endian on the wire
VERSION = 1

# 8 words of header. Keep the payload under the 1472-byte UDP/IPv4 MTU limit:
# LiteEth does not fragment, so an oversized datagram is simply broken.
HEADER_WORDS = 8


class MCP3008Sampler(LiteXModule):
    """Single-channel MCP3008 SPI master on a hardware timer.

    Emits a one-cycle `sample_stb` with `sample_data` every `period` sys
    clocks. `period` is a live input, not a parameter: the rate is a CSR.

    The transaction is the MCP3008's usual three bytes -- 0x01, then
    {start, single-ended, channel[2:0], 0000}, then a dummy -- and because the
    result lands in the low 10 bits of the 24-bit receive register, the sample
    is just `rx[0:10]`. That is the same alignment the (verified-working)
    Verilog reader uses, so the two cores agree by construction.

    SPI mode 0: MOSI is presented while SCLK is low and MISO is sampled at the
    rising edge, which is where the MCP3008 has it stable (it drives on the
    falling edge). MISO goes through a two-flop synchronizer, so the sample
    point is two sys clocks late -- irrelevant at any sane `sclk_div`, where a
    half-bit is tens of clocks.

    `overrun_stb` pulses when the rate generator fires while a conversion is
    still running. That means `period` is shorter than a conversion takes
    (24 bits * 2 * sclk_div clocks) and the requested rate is simply not
    achievable; the tick is dropped rather than sampling at the wrong time.
    """
    def __init__(self, pads, sclk_div=24):
        self.enable  = Signal()
        self.channel = Signal(3)
        self.period  = Signal(32)

        self.sample_stb  = Signal()
        self.sample_data = Signal(10)
        self.overrun_stb = Signal()
        self.busy        = Signal()

        # # #

        assert sclk_div >= 2

        miso = Signal()
        self.specials += MultiReg(pads.miso, miso)

        sclk  = Signal()
        cs_n  = Signal(reset=1)
        mosi  = Signal()
        self.comb += [
            pads.sclk.eq(sclk),
            pads.cs_n.eq(cs_n),
            pads.mosi.eq(mosi),
        ]

        # Rate generator. `period` is clamped to 2 so that a zero written to
        # the CSR cannot make (period - 1) wrap to 0xffffffff and stop the
        # capture dead -- a rate of "as fast as possible" is what overrun is
        # for, not something to express by accident.
        period  = Signal(32)
        cnt     = Signal(32)
        tick    = Signal()
        self.comb += period.eq(Mux(self.period < 2, 2, self.period))
        self.sync += [
            If(~self.enable,
                cnt.eq(0),
            ).Elif(cnt >= (period - 1),
                cnt.eq(0),
            ).Else(
                cnt.eq(cnt + 1),
            )
        ]
        self.comb += tick.eq(self.enable & (cnt >= (period - 1)))

        tx      = Signal(24)
        rx      = Signal(24)
        bitcnt  = Signal(5)
        divcnt  = Signal(max=sclk_div)
        half    = Signal()      # 0 = SCLK low half, 1 = SCLK high half
        running = Signal()

        self.comb += [
            self.busy.eq(running),
            self.overrun_stb.eq(tick & running),
            mosi.eq(tx[23]),
            sclk.eq(running & half),
            cs_n.eq(~running),
        ]

        div_end = Signal()
        self.comb += div_end.eq(divcnt == (sclk_div - 1))

        self.sync += [
            self.sample_stb.eq(0),
            If(~self.enable,
                running.eq(0),
                half.eq(0),
                divcnt.eq(0),
            ).Elif(~running,
                If(tick,
                    # start bit, then {1=single-ended, channel}, then a dummy
                    tx.eq(Cat(C(0, 8), C(0, 4), self.channel, C(1, 1), C(0x01, 8))),
                    rx.eq(0),
                    bitcnt.eq(24),
                    divcnt.eq(0),
                    half.eq(0),
                    running.eq(1),
                )
            ).Else(
                If(div_end,
                    divcnt.eq(0),
                    If(~half,
                        # low -> high: the MCP3008 has driven MISO, latch it
                        half.eq(1),
                        rx.eq(Cat(miso, rx[0:23])),
                    ).Else(
                        # high -> low: advance to the next bit
                        half.eq(0),
                        tx.eq(Cat(C(0, 1), tx[0:23])),
                        bitcnt.eq(bitcnt - 1),
                        If(bitcnt == 1,
                            running.eq(0),
                            self.sample_stb.eq(1),
                        )
                    )
                ).Else(
                    divcnt.eq(divcnt + 1),
                )
            )
        ]
        # rx has just been shifted for the final bit when bitcnt hits 1 in the
        # high->low branch, so the low 10 bits are the result at that moment.
        self.comb += self.sample_data.eq(rx[0:10])


class ADCStreamer(LiteXModule, AutoCSR):
    """Sampler + packetizer. `source` is a 32-bit stream ready for UDP.

    Samples are packed in pairs into 32-bit words before the FIFO, so the
    whole datapath downstream of the sampler is 32 bits wide and the packet
    body is a straight 1:1 pop. `samples_per_packet` is therefore even, and
    is a build-time constant because it sizes the FIFOs.

    Drops happen at word (two-sample) granularity and only when the FIFO is
    full, which on a healthy gigabit link means never: 20 ksps is 40 KB/s.
    """
    def __init__(self, pads, sys_clk_freq, sclk_div=24,
                 samples_per_packet=512, sample_rate=20e3,
                 fifo_depth=512, enable_reset=1,
                 dest_ip="255.255.255.255", udp_port=5005):
        assert samples_per_packet % 2 == 0
        words_per_packet = samples_per_packet // 2
        payload_words    = HEADER_WORDS + words_per_packet
        assert payload_words * 4 <= 1472, \
            f"packet payload {payload_words*4} B exceeds the 1472 B a non-fragmenting UDP stack can send"
        assert fifo_depth >= words_per_packet

        default_period = max(2, int(sys_clk_freq // int(sample_rate)))

        self._enable  = CSRStorage(reset=enable_reset, description="Enable capture and streaming.")
        self._channel = CSRStorage(3, description="MCP3008 channel to stream.")
        self._period  = CSRStorage(32, reset=default_period,
                                   description="Sys clocks between conversions. "
                                               "Sample rate = sys_clk_freq / period.")
        self._reset_counters = CSRStorage(description="Write 1 to zero seq/emitted/dropped.")
        # Destination as CSRs rather than build constants: the host's address is
        # the one thing about this design most likely to change, and a rebuild
        # to chase a new DHCP lease is a silly price. add_adc_stream() wires
        # these into the UDP streamer's own address signals.
        #
        # Default is the broadcast address, not a specific host, and that is
        # load-bearing rather than a style choice. liteeth's IP TX only ARPs
        # for a unicast destination -- a trailing ".255" octet (bcast_ip_mask
        # in liteeth/common.py) goes straight to the broadcast MAC with no ARP
        # involved. A unicast default pointed at a bench host that is not on
        # the network this boots into means the TX path retries that lookup
        # forever, and liteeth's ARP resolver tracks exactly one outstanding
        # request at a time -- so that endless retry starves ARP for every
        # *other* outbound packet too, including Etherbone's own replies and
        # ICMP. Symptom: the board answers ARP (a reply needs no resolution)
        # but nothing else -- ping, Etherbone, all of it looks dead, on a link
        # whose PHY and cabling are completely fine. Cost a full debugging
        # session (2026-10-09) on a host move that was otherwise a non-event.
        # `tools/adc_stream_ctl.py --dest-ip`/`--auto` re-point it at one host
        # at runtime for anyone who wants a non-broadcast capture.
        self._dest_ip   = CSRStorage(32, reset=_pack_ip(dest_ip),
                                     description="Destination IPv4, first octet in bits 31:24.")
        self._dest_port = CSRStorage(16, reset=udp_port, description="Destination UDP port.")

        self._emitted   = CSRStatus(32, description="Samples sent so far.")
        self._dropped   = CSRStatus(32, description="Samples discarded (FIFO full).")
        self._overruns  = CSRStatus(32, description="Rate ticks that arrived mid-conversion.")
        self._seq       = CSRStatus(32, description="Packets sent so far.")
        self._last      = CSRStatus(16, description="Most recent raw sample (diagnostic).")

        self.source = source = stream.Endpoint([("data", 32)])

        # # #

        enable = self._enable.storage

        self.sampler = sampler = MCP3008Sampler(pads, sclk_div=sclk_div)
        self.comb += [
            sampler.enable.eq(enable),
            sampler.channel.eq(self._channel.storage),
            sampler.period.eq(self._period.storage),
        ]

        counters_rst = Signal()
        self.comb += counters_rst.eq(self._reset_counters.re & self._reset_counters.storage[0])

        # Pack sample pairs into 32-bit words.
        self.fifo = fifo = stream.SyncFIFO([("data", 32)], fifo_depth, buffered=True)

        pair_lo   = Signal(16)
        pair_hi   = Signal(16)
        have_lo   = Signal()
        push      = Signal()

        # Both halves are LATCHED rather than read combinationally off the
        # sampler's receive register. `sample_data` is a wire off that register,
        # which the next conversion clears -- and the next conversion can start
        # one cycle after the previous one ends, so a combinational read here
        # would push zeros whenever the rate is high enough for the two to meet.
        self.comb += [
            fifo.sink.valid.eq(push),
            fifo.sink.data.eq(Cat(pair_lo, pair_hi)),
        ]
        self.sync += [
            push.eq(0),
            If(sampler.sample_stb,
                self._last.status.eq(sampler.sample_data),
                If(have_lo,
                    have_lo.eq(0),
                    pair_hi.eq(sampler.sample_data),
                    push.eq(1),
                ).Else(
                    have_lo.eq(1),
                    pair_lo.eq(sampler.sample_data),
                )
            ),
            If(~enable,
                have_lo.eq(0),
            )
        ]

        dropped  = self._dropped.status
        overruns = self._overruns.status
        emitted  = self._emitted.status
        seq      = self._seq.status

        # Every counter is driven from this ONE sync block. The packetizer FSM
        # reports completion with a comb strobe instead of bumping seq/emitted
        # itself: a signal written by both an FSM NextValue and a plain sync
        # statement has two drivers merged by source order, which is exactly
        # the kind of thing that works until the file is reordered.
        pkt_done = Signal()
        self.sync += [
            If(counters_rst,
                dropped.eq(0),
                overruns.eq(0),
                seq.eq(0),
                emitted.eq(0),
            ).Else(
                If(push & ~fifo.sink.ready, dropped.eq(dropped + 2)),
                If(sampler.overrun_stb,     overruns.eq(overruns + 1)),
                If(pkt_done,
                    seq.eq(seq + 1),
                    emitted.eq(emitted + samples_per_packet),
                ),
            )
        ]

        # Packetizer.
        widx      = Signal(max=payload_words + 1)
        hdr_seq   = Signal(32)
        hdr_emit  = Signal(32)
        hdr_drop  = Signal(32)
        hdr_chan  = Signal(3)
        hdr_per   = Signal(32)

        hdr = Array([
            Constant(MAGIC, 32),
            hdr_seq,
            hdr_emit,
            hdr_drop,
            Cat(Constant(samples_per_packet, 16), hdr_chan, Constant(0, 5), Constant(VERSION, 8)),
            hdr_per,
            Constant(int(sys_clk_freq), 32),
            Constant(0, 32),
        ])

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(widx, 0),
            If(enable & (fifo.level >= words_per_packet),
                NextValue(hdr_seq,  seq),
                NextValue(hdr_emit, emitted),
                NextValue(hdr_drop, dropped),
                NextValue(hdr_chan, self._channel.storage),
                NextValue(hdr_per,  self._period.storage),
                NextState("HEADER"),
            )
        )
        fsm.act("HEADER",
            source.valid.eq(1),
            source.data.eq(hdr[widx]),
            source.last.eq(0),
            If(source.ready,
                NextValue(widx, widx + 1),
                If(widx == (HEADER_WORDS - 1),
                    NextValue(widx, 0),
                    NextState("BODY"),
                )
            )
        )
        fsm.act("BODY",
            source.valid.eq(fifo.source.valid),
            source.data.eq(fifo.source.data),
            source.last.eq(widx == (words_per_packet - 1)),
            If(source.valid & source.ready,
                fifo.source.ready.eq(1),
                NextValue(widx, widx + 1),
                If(source.last,
                    pkt_done.eq(1),
                    NextState("IDLE"),
                )
            )
        )


def add_adc_stream(soc, sclk_freq=1e6, sample_rate=20e3, samples_per_packet=512,
                   dest_ip="255.255.255.255", udp_port=5005, fifo_depth=512,
                   pins=COLORLIGHT_I9_MCP3008_PINS, name="adc_stream",
                   ethcore_name="ethcore_etherbone"):
    """Attach the capture path to `soc` and point it at a host.

    `soc` must already have had `add_eth()` called on it -- this hangs a second
    UDP port off the core Etherbone is using, so both work at once.

    The clock-domain dance is the same one LiteX does for Etherbone, and it is
    not optional. With an 8-bit datapath the whole LiteEth core is renamed into
    the PHY's `eth_rx` domain, so the string "sys" means `eth_rx` inside it.
    Asking the UDP crossbar for a port `cd="sys"` would therefore hand us a
    port in `eth_rx` with no synchronizer between it and our real-`sys` logic.
    Passing a domain name the renamer does not touch, whose clock we drive from
    `sys`, gets a genuine CDC in the crossbar and a port we can talk to.
    """
    from liteeth.frontend.stream import LiteEthUDPStreamer

    ethcore = getattr(soc, ethcore_name, None)
    if ethcore is None:
        raise ValueError(
            f"add_adc_stream: no {ethcore_name} on this SoC -- call add_eth() first "
            "(with_etherbone=True, which is the default)."
        )

    soc.platform.add_extension(_mcp3008_io(pins))
    div = sclk_div_for(soc.sys_clk_freq, sclk_freq)

    core = ADCStreamer(
        pads               = soc.platform.request("mcp3008"),
        sys_clk_freq       = soc.sys_clk_freq,
        sclk_div           = div,
        samples_per_packet = samples_per_packet,
        sample_rate        = sample_rate,
        fifo_depth         = fifo_depth,
        dest_ip            = dest_ip,
        udp_port           = udp_port,
    )
    setattr(soc, name, core)
    soc.add_csr(name)

    cd_name = f"{name}_cd"
    cd = ClockDomain(cd_name)
    setattr(soc, f"cd_{cd_name}", cd)
    soc.comb += [
        cd.clk.eq(ClockSignal("sys")),
        cd.rst.eq(ResetSignal("sys")),
    ]

    # tx_fifo_depth must hold a whole packet: LiteEthStream2UDPTX flushes on
    # `last` OR when its FIFO fills, and a flush-on-full mid-packet would cut
    # the datagram in two and desynchronise the host's framing.
    payload_words = HEADER_WORDS + samples_per_packet // 2
    streamer = LiteEthUDPStreamer(
        ethcore.udp,
        ip_address    = dest_ip,
        udp_port      = udp_port,
        data_width    = 32,
        tx_fifo_depth = max(512, 2 * payload_words),
        rx_fifo_depth = 4,
        cd            = cd_name,
    )
    setattr(soc, f"{name}_udp", streamer)
    soc.comb += [
        core.source.connect(streamer.sink),
        # The streamer samples these per packet (it latches them in its IDLE
        # state), so re-pointing the stream mid-capture switches cleanly at a
        # packet boundary instead of tearing one in half.
        streamer.tx.ip_address.eq(core._dest_ip.storage),
        streamer.tx.udp_port.eq(core._dest_port.storage),
    ]

    up = name.upper()
    soc.add_constant(f"{up}_UDP_PORT", udp_port)
    soc.add_constant(f"{up}_SAMPLES_PER_PACKET", samples_per_packet)
    soc.add_constant(f"{up}_SCLK_DIV", div)
    soc.add_constant(f"{up}_SCLK_FREQ", int(soc.sys_clk_freq // (2 * div)))
    soc.add_constant(f"{up}_DEFAULT_PERIOD", max(2, int(soc.sys_clk_freq // int(sample_rate))))
    return core
