#!/usr/bin/env python3
"""IcePi Zero deployable: the on-FPGA MNIST touch demo.

The flagship feature build. Composes, on the deployable baseline:

  - the ST7796S LCD engine + FT6336U capacitive touch (add_lcd_touch)
  - the SNN-MLP MNIST inference peripheral (add_snn_mlp)
  - the deployable baseline: XIP BIOS + LiteSPI master + shared aux SPI bus
    + boot-manager flag/reset (add_flashing_baseline)

so a single firmware lets the user finger-draw a digit on the touchscreen, run
it through the on-FPGA SNN, and show the prediction on the same panel.

Post-WINC (2026-06-28): the ATWINC1500 is gone (smoked), so flash.py's WiFi OTA
path is dark until the ESP32-C3 loader lands -- load over UART (litex_term) or
JTAG for now.

This is the heaviest combination that fits the 25F: LCD + SNN routes (SNN ~78%
BRAM, ~2% PnR timing margin at N_MAC=2), but adding the SD card on top does NOT
-- SD is mutually exclusive with the LCD+SNN pair, so the data logger is its own
bitstream (icepi_zero_logger.py). LED 0/1 = SNN busy/done, so the aux busy LED
uses index 2. Timing gate: nextpnr runs with --timing-allow-fail, so always
confirm "Max frequency for clock" >= 50 MHz.

Forces the deployment shape (with_spi_flash + flash_master always on), so the
flash slot map is retained for the eventual C3 OTA. This supersedes
icepi_zero_all.py, which is the same SoC once SD is dropped. Boot chain / flash
layout: docs/boot_chain.md.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import (add_lcd_touch, add_snn_mlp, add_aux_imu,
                                    add_c3_loader_baseline)


class MnistLCDSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None,
                 lcd_spi_clk_freq=185e6, aux_spi_clk_freq=12.5e6, **kwargs):
        super().__init__(
            sys_clk_freq             = sys_clk_freq,
            with_spi_flash           = True,   # deployment shape: always XIP
            flash_master             = True,   # always WiFi-flashable
            flash_boot_offset        = flash_boot_offset,
            spi_clk_freq             = lcd_spi_clk_freq,
            force_lcd_backlight_off  = False,  # this project owns P1 via lcd_ctrl
            **kwargs,
        )
        # LCD/CTP reset is driven via the MCP23S17 expander (GPB0/GPB1), not a
        # direct pin: drop the LCD reset pad (frees IO10/L2 for the expander's
        # own RESET) and add the expander to the aux bus (with_iox). Firmware
        # (software/common/lcd.c) pulses reset over SPI. See docs/reset_sidebands.md.
        add_lcd_touch(self, lcd_spi_clk_freq, with_reset_pad=False)
        add_snn_mlp(self, leds=(0, 1))
        # Aux bus for the MCP23S17 expander (LCD/CTP reset). for_c3=True drops the
        # dead WINC cs[0] so M2 is free for the C3 SPIBone MISO below. SNN owns
        # LEDs 0,1 so the aux busy-LED uses 2.
        add_aux_imu(self, imu_spi_clk_freq=aux_spi_clk_freq, busy_led=2,
                    with_iox=True, for_c3=True)
        # Embed the C3 flash loader (standing rule): this bitstream is now
        # C3-reflashable and boots via the boot manager (loader @0x200000 ->
        # chain-boot app @0x280000). See docs/boot_chain.md, docs/c3_loader.md.
        add_c3_loader_baseline(self)


def main():
    parser = make_parser(description="IcePi Zero SoC + LCD/touch + SNN-MLP MNIST "
                                     "(touch demo, WiFi-updatable).")
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. "
                                    "Default 185 MHz -> 92.5 MHz SCK (production-proven).")
    parser.add_target_argument("--aux-spi-clk-freq", default=12.5e6, type=float,
                               help="Aux-bus default SCK frequency (Hz). Default 12.5 MHz = "
                                    "tested ceiling (25 MHz corrupts MISO on jumper wiring). "
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
        aux_spi_clk_freq  = args.aux_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
