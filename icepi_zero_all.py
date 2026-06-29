#!/usr/bin/env python3
"""IcePi Zero "everything" SoC -- all non-conflicting gateware in one build.

DEPRECATED for deployment: the SD card cannot route alongside LCD+SNN on the
25F (see the capacity note below), so `--no-sdcard` is the only shape that
builds -- and that is exactly icepi_zero_mnist_lcd.py, now the canonical
WiFi-updatable flagship. Keep this top only for A/B PnR experiments on the SD
cost; ship icepi_zero_mnist_lcd.py instead.

Composes every proven peripheral (gateware/soc_features.py) on one BaseSoC:

  - ST7796S LCD engine + FT6336U touch    (lcd, ctp_i2c, ctp_int; 10 pins)
  - SNN-MLP MNIST classifier              (snn; no pins, native SDRAM burst DMA)
  - shared aux SPI bus + ATWINC1500 WiFi  (aux_spi, winc_*; 9 pins)
  - native 4-bit SDIO SD card             (sdcard; dedicated pins, +2 DMA masters)
  - SPI flash with XIP BIOS + LiteSPI master (always on, see below)

Pin budget: 20 of 27 GPIO claimed (incl. the WINC EN), disjoint by
construction -- see docs/icepi_zero_pin_mapping.md. Wishbone masters: cpu +
lcd_dma (the SNN streams weights over its own native LiteDRAM burst port, not
Wishbone). The LSM6DS3 IMU has no block of its own -- it rides the aux
bus as AUX_IMU (add_winc_aux, cs[1]).

Unlike the per-feature tops this always builds the deployment shape: XIP
BIOS + LiteSPI flash master + the boot-manager flash layout, so every image
of this SoC is WiFi-updatable in place with ./flash.py and JTAG stays the
recovery path. Boot chain, flash layout and update flows:
docs/boot_chain.md.

Timing gate: the critical path is in the LiteDRAM L2 (thin margin at
N_MAC=2); nextpnr runs with --timing-allow-fail, so always check
"Max frequency for clock" >= 50 MHz in its output after PnR.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import (add_lcd_touch, add_snn_mlp, add_winc_aux,
                                   add_boot_ctl)


class AllSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None,
                 lcd_spi_clk_freq=185e6, winc_spi_clk_freq=12.5e6,
                 with_sdcard=True, **kwargs):
        super().__init__(
            sys_clk_freq             = sys_clk_freq,
            with_spi_flash           = True,   # deployment shape: always XIP
            flash_master             = True,   # always WiFi-flashable
            flash_boot_offset        = flash_boot_offset,
            spi_clk_freq             = lcd_spi_clk_freq,
            force_lcd_backlight_off  = False,  # LCD owns P1 via lcd_ctrl
            **kwargs,
        )
        # Same order as icepi_zero_mnist_lcd.py, WiFi appended last.
        add_lcd_touch(self, lcd_spi_clk_freq)
        add_snn_mlp(self, leds=(0, 1))
        add_winc_aux(self, winc_spi_clk_freq, busy_led=2)  # led2 optional
        add_boot_ctl(self)  # sticky boot flag + FTDI DTR/RTS sense
        # Native 4-bit SDIO for the IMU data logger (FAT32 via libfatfs). SPI-mode
        # SD is read-only in LiteX, so the logger needs the native core. Uses the
        # dedicated sdcard pins (P15/N16 + P14/R14/M15/M14), no GPIO contention,
        # and adds two block-DMA Wishbone masters. This is the tight build:
        # confirm Fmax >= 50 MHz and BRAM fit after PnR. --no-sdcard drops it.
        if with_sdcard:
            self.add_sdcard("sdcard")


def main():
    parser = make_parser(description="IcePi Zero SoC + LCD/touch + SNN MNIST + ATWINC1500 WiFi "
                                     "(XIP BIOS + flash master, WiFi-updatable).")
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. "
                                    "Default 185 MHz -> 92.5 MHz SCK (production-proven).")
    parser.add_target_argument("--winc-spi-clk-freq", default=12.5e6, type=float,
                               help="WINC SPI SCK frequency (Hz). Default 12.5 MHz = tested "
                                    "ceiling (25 MHz corrupts MISO on jumper wiring). "
                                    "Runtime-adjustable via the aux_spi clk_divider CSR.")
    parser.add_target_argument("--no-sdcard", action="store_true",
                               help="Drop the SDIO SD card core. Use to A/B the PnR cost or "
                                    "if timing/BRAM won't fit the full build. Default: SD on.")
    args = parser.parse_args()

    # Standalone boot by default: BIOS flash-boots the firmware at the
    # firmware offset; serial boot stays available as the fallback.
    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = AllSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        lcd_spi_clk_freq  = args.lcd_spi_clk_freq,
        winc_spi_clk_freq = args.winc_spi_clk_freq,
        with_sdcard       = not args.no_sdcard,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
