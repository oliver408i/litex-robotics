#!/usr/bin/env python3
"""IcePi Zero deployable: the IMU data logger with LCD/touch UI.

The field logger SoC. Composes, on the deployable baseline:

  - ST7796S LCD engine + FT6336U capacitive touch (add_lcd_touch) -- the field
    UI (live IMU readout, SD/log status, start/stop logging).
  - native 4-bit SDIO SD card (add_sdcard: SDPHY + SDCore + read/write block
    DMA, FAT32 via libfatfs) -- the non-volatile field store. SPI-mode SD is
    deliberately NOT used: LiteX's spisdcard.c is read-only (no block write),
    so a logger MUST use the native core. The card lives on dedicated pins
    (P15/N16 + P14/R14/M15/M14), so there is no GPIO contention.
  - the deployable baseline (add_flashing_baseline): XIP BIOS + LiteSPI
    master + shared aux SPI bus + boot-manager flag/reset. The LSM6DS3 IMU
    rides this aux bus on cs[1] FOR FREE (firmware drives it via aux_spi.c's
    AUX_IMU) -- so the logger needs no separate IMU SPI block.

It omits the SNN: on the 25F, SD's two block-DMA masters fit alongside LCD only
once the SNN (the ~78%-BRAM heavy block) is dropped. Verified to route
2026-06-16: post-route Fmax sys 54.56 / sys2x 131.91 / LCD-SPI 221.88 MHz, util
BRAM 58% / COMB 43%. (The LCD+SNN MNIST demo is its own bitstream,
icepi_zero_mnist_lcd.py; SD is mutually exclusive with the LCD+SNN pair.)

Post-WINC (2026-06-28): the ATWINC1500 is gone (smoked), so flash.py's WiFi OTA
path is dark until the ESP32-C3 loader lands -- field units must be loaded over
UART (litex_term) or JTAG for now. No SNN means the user LEDs are free, so the
aux busy LED stays at index 0.

Firmware is an LVGL app: the panel/touch UI plus logging (drain the LSM6DS3 FIFO
-> FatFs write to SD) driven from the LVGL main loop. Reuses the LVGL plumbing
from mnist_lcd_demo and the drivers in software/common (lsm6ds3, imu_log, power,
adc). The on-card record format carries a timer0 timestamp from day one so
IMU-only logs stay forward-compatible; a UART GPS (second UART) is the planned
next addition.

Validate SD first with zero firmware via the BIOS `sdcard` command. Build with
--yosys-abc9 --nextpnr-seed 2 and confirm "Max frequency for clock" >= 50 MHz.
Boot chain / flash layout: docs/boot_chain.md.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_lcd_touch, add_flashing_baseline, add_gps_uart


class LoggerSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None,
                 lcd_spi_clk_freq=185e6, aux_spi_clk_freq=12.5e6,
                 gps_baudrate=9600, **kwargs):
        super().__init__(
            sys_clk_freq            = sys_clk_freq,
            with_spi_flash          = True,   # deployment shape: always XIP
            flash_master            = True,   # always WiFi-flashable
            flash_boot_offset       = flash_boot_offset,
            spi_clk_freq            = lcd_spi_clk_freq,
            force_lcd_backlight_off = False,  # this project owns P1 via lcd_ctrl
            **kwargs,
        )
        add_lcd_touch(self, lcd_spi_clk_freq)
        # Deployable baseline: brings the aux bus the IMU (cs[1]) rides on. No
        # SNN here, so the aux busy LED keeps index 0.
        add_flashing_baseline(self, aux_spi_clk_freq, busy_led=0)
        # NMEA GPS on the second UART (rx=IO5/E1, tx=IO11/G2). Position fixes
        # become IMU_REC_GPS records on the IMU's timer0 timebase.
        add_gps_uart(self, baudrate=gps_baudrate)
        # Native 4-bit SDIO. Called after super().__init__() so SDRAM + the SoC
        # bus exist: add_sdcard hangs the block DMA masters on the bus and they
        # move 512-byte blocks straight to/from SDRAM-resident buffers.
        self.add_sdcard("sdcard")


def main():
    parser = make_parser(description="IcePi Zero SoC + LCD/touch + SDIO SD card + LSM6DS3 IMU "
                                     "(data logger, WiFi-updatable).")
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. "
                                    "Default 185 MHz -> 92.5 MHz SCK (production-proven).")
    parser.add_target_argument("--aux-spi-clk-freq", default=12.5e6, type=float,
                               help="Aux-bus default SCK frequency (Hz), used by the LSM6DS3 "
                                    "IMU (cs[1]). Default 12.5 MHz; the bus is capped at "
                                    "sys/2 = 25 MHz and runtime-adjustable per device via the "
                                    "aux_spi clk_divider CSR (LSM6DS3 tops out at 10 MHz).")
    parser.add_target_argument("--gps-baudrate", default=9600, type=int,
                               help="NMEA GPS UART baud (rx=IO5, tx=IO11). Default 9600 = the "
                                    "typical module power-on rate; raise it if the module is "
                                    "pre-configured faster.")
    args = parser.parse_args()

    # Boot the logger straight from SPI flash by default so the board runs
    # standalone after power-on (no host in the field). The BIOS still falls
    # back to serial boot if no valid image is present, so the litex_term
    # --kernel dev flow keeps working during bring-up.
    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = LoggerSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        lcd_spi_clk_freq  = args.lcd_spi_clk_freq,
        aux_spi_clk_freq  = args.aux_spi_clk_freq,
        gps_baudrate      = args.gps_baudrate,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
