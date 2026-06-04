#!/usr/bin/env python3
"""IcePi Zero SoC for the ST7796S LCD + FT6336U touch project.

BaseSoC + the LCD/touch feature (gateware/soc_features.py: add_lcd_touch).
The engine's SPI shifter runs in the dedicated cd_spi domain, so this top
constructs BaseSoC with spi_clk_freq and owns P1 (backlight) via lcd_ctrl.
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.soc_features import add_lcd_touch


class LCDSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None,
                 lcd_spi_clk_freq=100e6, **kwargs):
        super().__init__(
            sys_clk_freq             = sys_clk_freq,
            with_spi_flash           = with_spi_flash,
            flash_boot_offset        = flash_boot_offset,
            spi_clk_freq             = lcd_spi_clk_freq,
            force_lcd_backlight_off  = False,  # this project owns P1 via lcd_ctrl
            **kwargs,
        )
        add_lcd_touch(self, lcd_spi_clk_freq)


def main():
    parser = make_parser(description="IcePi Zero SoC + ST7796S LCD + FT6336U touch.")
    parser.add_target_argument("--lcd-spi-clk-freq", default=100e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. Default 100 MHz -> 50 MHz SCK.")
    args = parser.parse_args()

    soc = LCDSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args),
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        lcd_spi_clk_freq  = args.lcd_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
