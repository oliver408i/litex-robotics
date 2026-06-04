#!/usr/bin/env python3
"""IcePi Zero "everything" SoC -- all non-conflicting gateware in one build.

Composes every proven peripheral (gateware/soc_features.py) on one BaseSoC:

  - ST7796S LCD engine + FT6336U touch    (lcd, ctp_i2c, ctp_int; 10 pins)
  - SNN-MLP MNIST classifier              (snn; no pins, snn_wb DMA master)
  - shared aux SPI bus + ATWINC1500 WiFi  (aux_spi, winc_*; 9 pins)
  - SPI flash with XIP BIOS + LiteSPI master (always on, see below)

Pin budget: 20 of 27 GPIO claimed (incl. the WINC EN), disjoint by
construction -- see docs/icepi_zero_pin_mapping.md. Wishbone masters: cpu +
lcd_dma + snn_wb. Excluded as duplicates: icepi_zero_imu.py's imu_spi (the
LSM6DS3 sits on the aux bus as AUX_IMU) and the snn_estimator debug core
(CSR-name clash with SNNMLP; its own bring-up top).

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

from gateware.soc_features import add_lcd_touch, add_snn_mlp, add_winc_aux, add_boot_ctl


class AllSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None,
                 lcd_spi_clk_freq=185e6, winc_spi_clk_freq=12.5e6, **kwargs):
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
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
