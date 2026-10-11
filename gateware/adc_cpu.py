"""MCP3008 Doppler sampling exposed to the CPU directly -- no Ethernet, no UDP.

gateware/adc_stream.py's ADCStreamer hangs a UDP port off the Etherbone core
and exists to feed a *host* over the network. This is the on-board sibling:
the FFT now runs on the main CPU (software/fft_bench proved a plain
fixed-point FFT clears the LCD's frame budget with room to spare, see
docs/next-radar.md 2026-10-09), so the CPU needs the raw samples itself, not
a UDP packet.

Sample-interval jitter still matters -- frequency is still the measurement --
so the rate generator is the exact same hardware-timed core adc_stream.py
uses, `MCP3008Sampler`, reused unchanged. What's different is the sink: no
FIFO, no packetizer, just a free-running `count` CSR the CPU polls. `count`
incrementing is the "new sample" edge; `last` holds the sample itself. A CPU
busy-polling `count` at 20 ksps has ~4500 sys clocks between samples at
90 MHz -- ample margin to notice every increment without an interrupt.
"""
from migen import *

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus

from gateware.adc_stream import MCP3008Sampler
from gateware.mcp3008 import COLORLIGHT_I9_MCP3008_PINS, _mcp3008_io, sclk_div_for


class AdcCpuSampler(LiteXModule, AutoCSR):
    """MCP3008 sampler with a CPU-polling-friendly CSR interface.

    `count` wraps at 2**32 and is never reset by firmware (no `reset_counters`
    CSR here, unlike ADCStreamer) -- a poll loop only ever compares successive
    reads of it, so wrapping is transparent as long as firmware uses unsigned
    subtraction, same convention as timer0 in software/lvgl_demo and friends.
    """
    def __init__(self, pads, sys_clk_freq, sclk_div=24, sample_rate=20e3, enable_reset=1):
        default_period = max(2, int(sys_clk_freq // int(sample_rate)))

        self._enable   = CSRStorage(reset=enable_reset, description="Enable capture.")
        self._channel  = CSRStorage(3, description="MCP3008 channel to sample.")
        self._period   = CSRStorage(32, reset=default_period,
                                    description="Sys clocks between conversions. "
                                                "Sample rate = sys_clk_freq / period.")
        self._last     = CSRStatus(16, description="Most recent raw sample.")
        self._count    = CSRStatus(32, description="Samples taken so far, wraps. "
                                                    "Poll this to notice a new sample "
                                                    "without missing or double-reading one.")
        self._overruns = CSRStatus(32, description="Rate ticks that arrived mid-conversion "
                                                    "-- period is shorter than a conversion "
                                                    "takes, the requested rate is unachievable.")

        # # #

        self.sampler = sampler = MCP3008Sampler(pads, sclk_div=sclk_div)
        self.comb += [
            sampler.enable.eq(self._enable.storage),
            sampler.channel.eq(self._channel.storage),
            sampler.period.eq(self._period.storage),
        ]

        count    = self._count.status
        overruns = self._overruns.status
        self.sync += [
            If(sampler.sample_stb,
                self._last.status.eq(sampler.sample_data),
                count.eq(count + 1),
            ),
            If(sampler.overrun_stb,
                overruns.eq(overruns + 1),
            ),
        ]


def add_adc_cpu(soc, sclk_freq=1e6, sample_rate=20e3,
               pins=COLORLIGHT_I9_MCP3008_PINS, name="adc_cpu"):
    """Attach a CPU-polled ADC sampler to `soc`. Does not need add_eth()."""
    soc.platform.add_extension(_mcp3008_io(pins))
    div = sclk_div_for(soc.sys_clk_freq, sclk_freq)

    core = AdcCpuSampler(
        pads         = soc.platform.request("mcp3008"),
        sys_clk_freq = soc.sys_clk_freq,
        sclk_div     = div,
        sample_rate  = sample_rate,
    )
    setattr(soc, name, core)
    soc.add_csr(name)

    up = name.upper()
    soc.add_constant(f"{up}_SCLK_DIV", div)
    soc.add_constant(f"{up}_SCLK_FREQ", int(soc.sys_clk_freq // (2 * div)))
    soc.add_constant(f"{up}_DEFAULT_PERIOD", max(2, int(soc.sys_clk_freq // int(sample_rate))))
    return core
