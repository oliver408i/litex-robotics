#!/usr/bin/env python3
"""Colorlight i9 SoC: ADC Doppler capture + on-board FFT + live spectrum on
the ST7796S LCD. No host needed -- this is the "DPU" end of next-radar.md's
long-running thread, now that software/fft_bench proved a plain fixed-point
FFT on vexii-superscalar clears the LCD's frame budget with room to spare
(docs/next-radar.md, 2026-10-09: 3.14ms for a full 1024-point FFT against a
20-50ms refresh budget).

ADC sampling is gateware/adc_cpu.py's AdcCpuSampler, NOT adc_stream.py's
ADCStreamer -- that one hangs a UDP port off the Etherbone core and exists to
feed a *host* over the network. This one exposes the hardware-timed sample and
a free-running counter as CSRs for firmware to poll directly; no Ethernet is
required for the capture path. Etherbone stays on by default anyway, for the
same reason lcd.py keeps it: an image meant to live in flash with no network
path back in has no recovery but the probe.

    .venv/bin/python targets/colorlight_i9/radar_lcd.py --build --load --eth-phy 1

CPU is vexii-superscalar by default, same as lcd.py -- firmware does both the
FFT and the LCD drawing on one core, and lcd.py already established on
hardware that the bigger core is worth it here (docs/cpu_vexiiriscv.md). Needs
sbt+JDK21 on PATH if this SoC's memory-region shape doesn't match an
already-cached VexiiRiscv netlist (see bench-host-quirks memory / README).

Wiring: LCD/touch on P4 (same pins as lcd.py), MCP3008 on P3/pmodh (same pins
as adc_stream.py/adc.py) -- the same two headers used independently elsewhere
on this board, now on one image together.
"""
from base import BaseSoC, make_parser, run_build, add_eth_arguments, eth_kwargs

from gateware.soc_features import add_lcd_touch, add_eth, COLORLIGHT_I9_LCD_PINS
from gateware.adc_cpu import add_adc_cpu


class RadarLCDSoC(BaseSoC):
    def __init__(self, sys_clk_freq=90e6, sdram_rate="1:2", lcd_spi_clk_freq=185e6,
                 adc_sclk_freq=1e6, adc_sample_rate=20e3, eth=None, **kwargs):
        super().__init__(
            sys_clk_freq   = sys_clk_freq,
            sdram_rate     = sdram_rate,
            spi_clk_freq   = lcd_spi_clk_freq,
            # add_lcd_touch requests these pins itself; BaseSoC must not park them.
            force_lcd_idle = False,
            **kwargs,
        )
        add_lcd_touch(self, lcd_spi_clk_freq, pins=COLORLIGHT_I9_LCD_PINS)
        add_adc_cpu(self, sclk_freq=adc_sclk_freq, sample_rate=adc_sample_rate)
        if eth is not None and eth["with_etherbone"]:
            add_eth(self, **eth)


def main():
    # vexii-superscalar, not the base default: this top is the ADC+LCD+FFT
    # combo, and the CPU carries all three jobs. `--cpu vexriscv` still gets
    # the small core back if sbt/JDK21 aren't on PATH.
    parser = make_parser(description="Colorlight i9 SoC: radar FFT + live spectrum on the LCD.",
                         default_cpu="vexii-superscalar")
    add_eth_arguments(parser)
    # 90 MHz sys / 181 MHz SDRAM / 92.5 MHz SCK -- same numbers lcd.py measured
    # on hardware; see that module's docstring for the sweep.
    parser.set_defaults(sys_clk_freq=90e6)
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. "
                                    "Do not raise past ~200 MHz -- the ST7796S panel "
                                    "gives up around 100 MHz SCK before the FPGA does.")
    parser.add_target_argument("--adc-sclk-freq", default=1e6, type=float,
                               help="MCP3008 SCLK (Hz). Keep under ~1.8 MHz on a 3.3V part.")
    parser.add_target_argument("--adc-sample-rate", default=20e3, type=float,
                               help="ADC sample rate reset value (Hz), realised as an "
                                    "integer divisor of sys. Runtime-changeable over "
                                    "Etherbone (same CSRs adc_stream_ctl.py's pattern uses).")
    args = parser.parse_args()

    soc = RadarLCDSoC(
        sys_clk_freq     = args.sys_clk_freq,
        sdram_rate       = "1:2" if args.sdram_half_rate else "1:1",
        lcd_spi_clk_freq = args.lcd_spi_clk_freq,
        adc_sclk_freq    = args.adc_sclk_freq,
        adc_sample_rate  = args.adc_sample_rate,
        eth              = eth_kwargs(args),
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
