#!/usr/bin/env python3
"""IcePi Zero SoC for the on-FPGA MNIST touch demo.

Fuses the two previously-separate projects into one SoC:

  - the ST7796S LCD engine + FT6336U capacitive touch (from icepi_zero_lcd.py)
  - the SNN-MLP MNIST inference peripheral (from icepi_zero_mnist.py)

so a single firmware can let the user finger-draw a digit on the touchscreen,
run it through the on-FPGA SNN, and show the prediction on the same panel.

Both peripherals add a Wishbone master to the system bus (lcd_dma + snn_wb);
both expose CSRs. The LCD engine only adds depth-4 async FIFOs (LUTRAM, not
block RAM), so the combined SoC stays within the MNIST build's ~78% BRAM budget
-- the gate to confirm is PnR timing, whose margin at N_MAC=2 is thin (~2%).

Feature blocks live in gateware/soc_features.py; the kitchen-sink variant
(this + WiFi) is icepi_zero_all.py.
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.soc_features import add_lcd_touch, add_snn_mlp


class MnistLCDSoC(BaseSoC):
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
        add_snn_mlp(self, leds=(0, 1))


def main():
    parser = make_parser(description="IcePi Zero SoC + LCD/touch + SNN-MLP MNIST (touch demo).")
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. Default 185 MHz -> 92.5 MHz SCK.")
    args = parser.parse_args()

    # Boot the demo straight from SPI flash by default so the board runs
    # standalone after power-on (no host attached). The BIOS still falls back
    # to serial boot if no valid image is present, so `litex_term --kernel`
    # dev flow keeps working. Flash the firmware to the same offset with
    # --flash-firmware. Pass --flash-boot-offset to override the default.
    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = MnistLCDSoC(
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
