#!/usr/bin/env python3
"""IcePi Zero SoC for the 3.3 V logic-analyzer variant.

A deployable LA bitstream: the mandatory WiFi-OTA baseline (WINC aux bus +
boot_ctl, see gateware/soc_features.add_flashing_baseline) plus the SDRAM-
streaming capture core (add_logic_analyzer). The 18 probe channels are the
GPIO-bank pins that free up once the LCD/touch module is unplugged; the aux
bus (WINC/IMU/MCP) is untouched, so WiFi flashing still works during a session.

The LCD/touch module MUST be physically unplugged before capturing: a boot-time
guard (status.lcd_present) detects the on-module touch pull-ups and the firmware
refuses to arm if they're still present. See docs/logic_analyzer.md.

Pair with software/logic_analyzer (firmware + la_host.py).
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.soc_features import add_flashing_baseline, add_logic_analyzer


class LogicAnalyzerSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None,
                 flash_master=False, winc_spi_clk_freq=12.5e6, **kwargs):
        super().__init__(
            sys_clk_freq            = sys_clk_freq,
            with_spi_flash          = with_spi_flash,
            flash_boot_offset       = flash_boot_offset,
            flash_master            = flash_master,
            # The LA owns P1 (IO14) as a probe channel, so BaseSoC must not also
            # claim it to tie the LCD backlight low.
            force_lcd_backlight_off = False,
            **kwargs,
        )
        add_flashing_baseline(self, winc_spi_clk_freq=winc_spi_clk_freq, busy_led=0)
        add_logic_analyzer(self)


def main():
    parser = make_parser(description="IcePi Zero SoC + 3.3 V logic analyzer.")
    parser.add_target_argument("--winc-spi-clk-freq", default=12.5e6, type=float,
                               help="WINC SPI SCK frequency (Hz). Default 12.5 MHz (tested "
                                    "ceiling; 25 MHz corrupts MISO). Runtime-adjustable via "
                                    "the aux_spi clk_divider CSR.")
    parser.add_target_argument("--flash-master", action="store_true",
                               help="Expose the LiteSPI master CSRs so SDRAM-resident firmware "
                                    "(the WiFi flash-loader) can erase/program the SPI flash. "
                                    "Implies the XIP flash build. See docs/boot_chain.md.")
    args = parser.parse_args()

    soc = LogicAnalyzerSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args) or args.flash_master,
        flash_boot_offset = args.flash_boot_offset,
        flash_master      = args.flash_master,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        winc_spi_clk_freq = args.winc_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
