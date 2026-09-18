#!/usr/bin/env python3
"""Colorlight i9 SoC for the ST7796S LCD + FT6336U touch project.

The i9 port of targets/icepi_zero/lcd.py: same LCDEngine, same firmware, a
different pin map (gateware/soc_features.py: COLORLIGHT_I9_LCD_PINS). The
module hangs off P4; see targets/colorlight_i9/README.md for the wiring table.

Two board differences worth knowing, both absorbed by add_lcd_touch():
  * LCD_RST and CTP_RST are separate wires here (E4 / H4) rather than one tied
    pin, and are driven from the same CSR bit -- so software/common/lcd.c works
    unchanged.
  * SD_CS (J5) is wired and held high in gateware; nothing talks to the card.

The engine's SPI shifter runs in the dedicated cd_spi domain, so this top
constructs BaseSoC with spi_clk_freq. Unlike the IcePi Zero there is no
backlight-pin conflict to resolve -- the i9's BaseSoC does not claim H3.

    .venv/bin/python targets/colorlight_i9/lcd.py --build --load

Clocks, swept on hardware 2026-09-16. Every one of these is a build nextpnr
reports as a TIMING FAILURE, and every one of them runs:

    sys 90 MHz      reported 41.7    BIOS banner + 4 MB Memtest OK
    SDRAM 181 MHz   reported 145.7   4 MB Memtest OK (half rate, 2*sys)
    SCK 92.5 MHz    reported ok      6302 fps at 150, so the shifter is not the limit

**The limit is the ST7796S, not the FPGA, and it is about 100 MHz SCK.** Above
it the panel goes WHITE while the console still reports full frame rates, a
clean DMA self-test and a healthy touch controller -- because all of those
measure the FPGA side of the wire. 125 MHz SCK produced 5287 fps and a dead
display. That is why 92.5 MHz is the number here and in
targets/icepi_zero/turbo.py: it is just under a panel wall, not the fastest
thing that would build. Do not "optimise" it upward on console evidence alone.

Frame rate against SCK is otherwise linear (the panel link is the bottleneck,
not sys): 50 MHz -> 2137 fps, 92.5 -> 3891, 125 -> 5287, 150 -> 6302. Raising
sys from 50 to 90 MHz moved fps by 0.5%, so sys is worth raising only for
CPU-bound firmware.
"""
from base import BaseSoC, make_parser, run_build, add_eth_arguments, eth_kwargs

from gateware.soc_features import add_lcd_touch, add_eth, COLORLIGHT_I9_LCD_PINS


class LCDSoC(BaseSoC):
    def __init__(self, sys_clk_freq=90e6, sdram_rate="1:2", lcd_spi_clk_freq=185e6,
                 eth=None, **kwargs):
        super().__init__(
            sys_clk_freq = sys_clk_freq,
            sdram_rate   = sdram_rate,
            spi_clk_freq = lcd_spi_clk_freq,
            # add_lcd_touch requests these pins itself; BaseSoC must not park them.
            force_lcd_idle = False,
            **kwargs,
        )
        add_lcd_touch(self, lcd_spi_clk_freq, pins=COLORLIGHT_I9_LCD_PINS)
        # Etherbone is on by default -- this is a build meant to end up in
        # flash, and an image without it has no way back in but the probe.
        if eth is not None and eth["with_etherbone"]:
            add_eth(self, **eth)


def main():
    parser = make_parser(description="Colorlight i9 SoC + ST7796S LCD + FT6336U touch.")
    add_eth_arguments(parser)
    # 90 MHz sys / 181 MHz SDRAM / 92.5 MHz SCK: all three measured on hardware,
    # all three reported as timing failures by nextpnr. See the module docstring.
    parser.set_defaults(sys_clk_freq=90e6)
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. "
                                    "Default 185 MHz -> 92.5 MHz SCK. Do NOT raise this "
                                    "past ~200 MHz: the ST7796S itself gives up around "
                                    "100 MHz SCK and the panel goes white while the "
                                    "gateware happily reports full frame rates.")
    parser.add_target_argument("--sdram-1to1", action="store_true",
                               help="Full-rate SDRAM (clock = sys) instead of the "
                                    "half-rate default (clock = 2*sys = 181 MHz). "
                                    "Half rate is already the default here, so the "
                                    "shared --sdram-half-rate flag is a no-op on this "
                                    "top; this is the switch that does something.")
    args = parser.parse_args()

    soc = LCDSoC(
        sys_clk_freq     = args.sys_clk_freq,
        sdram_rate       = "1:1" if args.sdram_1to1 else "1:2",
        lcd_spi_clk_freq = args.lcd_spi_clk_freq,
        eth              = eth_kwargs(args),
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
