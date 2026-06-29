#!/usr/bin/env python3
"""IcePi Zero deployable: the on-FPGA MNIST touch demo (WiFi-updatable).

The flagship feature build. Composes, on the mandatory WiFi-OTA baseline:

  - the ST7796S LCD engine + FT6336U capacitive touch (add_lcd_touch)
  - the SNN-MLP MNIST inference peripheral (add_snn_mlp)
  - the OTA-flashing baseline: XIP BIOS + LiteSPI master + ATWINC1500 aux bus
    + boot-manager flag/reset (add_flashing_baseline)

so a single firmware lets the user finger-draw a digit on the touchscreen, run
it through the on-FPGA SNN, show the prediction on the same panel, AND have the
whole board updated in place over WiFi with ./flash.py.

This is the heaviest combination that fits the 25F: LCD + SNN + WINC routes
(SNN ~78% BRAM, ~2% PnR timing margin at N_MAC=2), but adding the SD card on
top does NOT -- SD is mutually exclusive with the LCD+SNN pair, so the data
logger is its own bitstream (icepi_zero_logger.py). LED 0/1 = SNN busy/done, so
the WINC busy LED uses index 2. Timing gate: nextpnr runs with
--timing-allow-fail, so always confirm "Max frequency for clock" >= 50 MHz.

Forces the deployment shape (with_spi_flash + flash_master always on), so a
plain `--build` is OTA-ready. This supersedes icepi_zero_all.py, which is the
same SoC once SD is dropped. Boot chain / flash layout: docs/boot_chain.md.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_lcd_touch, add_snn_mlp, add_flashing_baseline


class MnistLCDSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None,
                 lcd_spi_clk_freq=185e6, winc_spi_clk_freq=12.5e6, **kwargs):
        super().__init__(
            sys_clk_freq             = sys_clk_freq,
            with_spi_flash           = True,   # deployment shape: always XIP
            flash_master             = True,   # always WiFi-flashable
            flash_boot_offset        = flash_boot_offset,
            spi_clk_freq             = lcd_spi_clk_freq,
            force_lcd_backlight_off  = False,  # this project owns P1 via lcd_ctrl
            **kwargs,
        )
        add_lcd_touch(self, lcd_spi_clk_freq)
        add_snn_mlp(self, leds=(0, 1))
        add_flashing_baseline(self, winc_spi_clk_freq, busy_led=2)  # SNN owns 0,1


def main():
    parser = make_parser(description="IcePi Zero SoC + LCD/touch + SNN-MLP MNIST "
                                     "(touch demo, WiFi-updatable).")
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. "
                                    "Default 185 MHz -> 92.5 MHz SCK (production-proven).")
    parser.add_target_argument("--winc-spi-clk-freq", default=12.5e6, type=float,
                               help="WINC SPI SCK frequency (Hz). Default 12.5 MHz = tested "
                                    "ceiling (25 MHz corrupts MISO on jumper wiring). "
                                    "Runtime-adjustable via the aux_spi clk_divider CSR.")
    args = parser.parse_args()

    # Standalone boot by default: BIOS flash-boots the firmware at the firmware
    # offset; serial boot stays available as the fallback.
    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = MnistLCDSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        lcd_spi_clk_freq  = args.lcd_spi_clk_freq,
        winc_spi_clk_freq = args.winc_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
