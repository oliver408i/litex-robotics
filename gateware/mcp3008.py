"""MCP3008 10-bit SPI ADC reader.

A free-running sampler: gateware walks the enabled channels on its own timer
and parks the latest 10-bit result for each one in a CSR. Firmware never
touches SPI, which is the property the radar capture path needs:
sample-interval jitter from a polling CPU smears the FFT, and frequency *is*
the measurement there.

Restored from the pre-`c282071` tree (the IcePi Zero WINC-era build, where the
ADC hung off the shared aux bus as cs[2]) and re-pointed at the Colorlight i9,
where it gets a bus of its own on P3/`pmodh`. The Verilog core
(`verilog/spi/mcp3008_reader.v`) is unchanged; what is new here is the pin map
and the `add_mcp3008()` helper.

Sampling rate: one conversion is 3 SPI bytes = 24 SCLK, so a channel is
sampled every `24 / sclk_freq` seconds plus the interval timer, times the
number of channels in the mask. At the 1 MHz default SCLK that is ~41 ksps
aggregate with `interval_ticks=0` (free run).

**Keep SCLK at or below ~1.8 MHz on a 3.3 V part.** The MCP3008 is specced
3.6 MHz at 5 V but only 1.35 MHz at 2.7 V; 3.3 V interpolates to roughly
1.8 MHz, and past it the SAR does not settle -- you get plausible-looking
numbers that are quietly wrong, which is the worst failure mode for an
instrument.
"""
import os

from migen import *

from litex.gen import *
from litex.build.generic_platform import Pins, Subsignal, IOStandard, Misc
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus


# Colorlight i9 v7.2: P3 / `pmodh`, which litex-boards spells
# "- E19 B3 K5 - B2 K4 A2" (the two dashes are the connector's power pins).
# Raw ball names here rather than "pmodh:n" indices so the map reads against
# the silkscreen.
COLORLIGHT_I9_MCP3008_PINS = {
    "sclk": "K5", "cs_n": "A2", "mosi": "B3", "miso": "K4",
}


def _mcp3008_io(pins):
    """Platform extension for one board's pin map.

    MISO is pulled UP on purpose. With no ADC on the header the reader then
    clocks in all ones and every channel reads 1023, which is a signature you
    can tell apart from a working part; a pull-down would read 0, which is
    also what a correctly-wired ADC sitting at 0 V reads. Once the MCP3008 is
    attached it drives the line and the pull is irrelevant.
    """
    return [
        ("mcp3008", 0,
            Subsignal("sclk", Pins(pins["sclk"])),
            Subsignal("cs_n", Pins(pins["cs_n"])),
            Subsignal("mosi", Pins(pins["mosi"])),
            Subsignal("miso", Pins(pins["miso"]), Misc("PULLMODE=UP")),
            IOStandard("LVCMOS33"),
        ),
    ]


class MCP3008Reader(LiteXModule, AutoCSR):
    """Wrapper around `verilog/spi/mcp3008_reader.v`.

    `sclk_div` is in sys clocks per SPI half-bit, so SCLK = sys / (2*sclk_div);
    use `sclk_div_for()` rather than working it out at each call site.
    """
    def __init__(self, pads, sys_clk_freq, platform, sclk_div=16, default_interval_ms=10):
        self._enable = CSRStorage(reset=1, description="Enable MCP3008 sampling.")
        self._channel_mask = CSRStorage(8, reset=0xff, description="Channel mask (bit0=CH0..bit7=CH7).")
        self._sample_interval = CSRStorage(32, reset=int(sys_clk_freq * default_interval_ms / 1000),
                                           description="Sample interval ticks (0=continuous).")
        self._clear_update = CSRStorage(8, description="Write mask to clear update bits.")

        self._busy = CSRStatus(description="Sampler busy flag.")
        self._update_mask = CSRStatus(8, description="Channel update flags.")
        self._last_channel = CSRStatus(3, description="Last sampled channel.")
        self._sample_count = CSRStatus(32, description="Total samples captured.")
        self._sample_ch0 = CSRStatus(16, description="Channel 0 raw sample.")
        self._sample_ch1 = CSRStatus(16, description="Channel 1 raw sample.")
        self._sample_ch2 = CSRStatus(16, description="Channel 2 raw sample.")
        self._sample_ch3 = CSRStatus(16, description="Channel 3 raw sample.")
        self._sample_ch4 = CSRStatus(16, description="Channel 4 raw sample.")
        self._sample_ch5 = CSRStatus(16, description="Channel 5 raw sample.")
        self._sample_ch6 = CSRStatus(16, description="Channel 6 raw sample.")
        self._sample_ch7 = CSRStatus(16, description="Channel 7 raw sample.")

        clear_mask = Signal(8)
        self.comb += clear_mask.eq(Mux(self._clear_update.re, self._clear_update.storage, 0))

        verilog_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "verilog"))
        platform.add_source(os.path.join(verilog_root, "spi", "SPI_host.v"))
        platform.add_source(os.path.join(verilog_root, "spi", "mcp3008_reader.v"))

        busy = Signal()
        update_mask = Signal(8)
        last_channel = Signal(3)
        sample_count = Signal(32)
        sample_ch = [Signal(10) for _ in range(8)]

        self.specials += Instance(
            "mcp3008_reader",
            p_SCLK_DIV=int(sclk_div),
            i_clk=ClockSignal("sys"),
            i_rstn=~ResetSignal("sys"),
            i_enable=self._enable.storage,
            i_channel_mask=self._channel_mask.storage,
            i_sample_interval_ticks=self._sample_interval.storage,
            i_clear_update_mask=clear_mask,
            o_busy=busy,
            o_update_mask=update_mask,
            o_last_channel=last_channel,
            o_sample_count=sample_count,
            o_adc_sclk=pads.sclk,
            o_adc_cs_n=pads.cs_n,
            o_adc_mosi=pads.mosi,
            i_adc_miso=pads.miso,
            o_sample_ch0=sample_ch[0],
            o_sample_ch1=sample_ch[1],
            o_sample_ch2=sample_ch[2],
            o_sample_ch3=sample_ch[3],
            o_sample_ch4=sample_ch[4],
            o_sample_ch5=sample_ch[5],
            o_sample_ch6=sample_ch[6],
            o_sample_ch7=sample_ch[7],
        )

        self.comb += [
            self._busy.status.eq(busy),
            self._update_mask.status.eq(update_mask),
            self._last_channel.status.eq(last_channel),
            self._sample_count.status.eq(sample_count),
            self._sample_ch0.status.eq(sample_ch[0]),
            self._sample_ch1.status.eq(sample_ch[1]),
            self._sample_ch2.status.eq(sample_ch[2]),
            self._sample_ch3.status.eq(sample_ch[3]),
            self._sample_ch4.status.eq(sample_ch[4]),
            self._sample_ch5.status.eq(sample_ch[5]),
            self._sample_ch6.status.eq(sample_ch[6]),
            self._sample_ch7.status.eq(sample_ch[7]),
        ]


def sclk_div_for(sys_clk_freq, sclk_freq):
    """Half-bit divisor giving the highest SCLK that does not exceed `sclk_freq`.

    Rounded up, and floored at the core's own minimum of 2.
    """
    div = -(-int(sys_clk_freq) // (2 * int(sclk_freq)))
    return max(2, div)


def add_mcp3008(soc, sclk_freq=1e6, default_interval_ms=10,
                pins=COLORLIGHT_I9_MCP3008_PINS, name="mcp3008"):
    """Attach an MCP3008 reader to `soc` on `pins`.

    Exports the achieved SCLK as a generated constant: the divisor is an
    integer, so the real rate is not the one asked for, and firmware that
    prints a sample rate should print the true one.
    """
    soc.platform.add_extension(_mcp3008_io(pins))
    div = sclk_div_for(soc.sys_clk_freq, sclk_freq)
    core = MCP3008Reader(
        pads                = soc.platform.request("mcp3008"),
        sys_clk_freq        = soc.sys_clk_freq,
        platform            = soc.platform,
        sclk_div            = div,
        default_interval_ms = default_interval_ms,
    )
    setattr(soc, name, core)
    soc.add_csr(name)

    up = name.upper()
    soc.add_constant(f"{up}_SCLK_DIV", div)
    soc.add_constant(f"{up}_SCLK_FREQ", int(soc.sys_clk_freq // (2 * div)))
    soc.add_constant(f"{up}_DEFAULT_INTERVAL_MS", int(default_interval_ms))
    return core
