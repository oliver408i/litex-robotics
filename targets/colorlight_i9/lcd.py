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
"""
from base import BaseSoC, make_parser, run_build, add_eth_arguments, eth_kwargs

from gateware.soc_features import add_lcd_touch, add_eth, COLORLIGHT_I9_LCD_PINS


class LCDSoC(BaseSoC):
    def __init__(self, sys_clk_freq=48e6, sdram_rate="1:1", lcd_spi_clk_freq=100e6,
                 eth=None, **kwargs):
        super().__init__(
            sys_clk_freq = sys_clk_freq,
            sdram_rate   = sdram_rate,
            spi_clk_freq = lcd_spi_clk_freq,
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
    parser.add_target_argument("--lcd-spi-clk-freq", default=100e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. "
                                    "Default 100 MHz -> 50 MHz SCK.")
    args = parser.parse_args()

    soc = LCDSoC(
        sys_clk_freq     = args.sys_clk_freq,
        sdram_rate       = "1:2" if args.sdram_half_rate else "1:1",
        lcd_spi_clk_freq = args.lcd_spi_clk_freq,
        eth              = eth_kwargs(args),
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
