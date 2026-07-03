#!/usr/bin/env python3
"""IcePi Zero baseline SoC -- the pure WiFi-OTA flashing image, no features.

This is the smallest DEPLOYABLE bitstream: VexRiscv + UART + the deployable
baseline (XIP BIOS from flash, LiteSPI master, the shared aux SPI bus, and the
boot-manager flag/reset). It carries no LCD, SNN, SD or PISC -- nothing but the
common machinery every feature top builds on.

Post-WINC (2026-06-28): the ATWINC1500 is gone (smoked), so flash.py's WiFi OTA
path is dark until the ESP32-C3 loader (on separate free SPI pins) lands. Load
this over UART (litex_term --kernel) or bit-banged JTAG. It still always routes
with huge margin, so it remains the known-good recovery / fallback image: if a
feature build bricks or won't fit, flash this to get a board back to a loadable
state. The XIP-BIOS + LiteSPI-master flash slot map is retained for the eventual
C3 OTA. See docs/boot_chain.md and the winc-archive branch (archived WINC stack).
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_flashing_baseline


class BaselineSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None,
                 aux_spi_clk_freq=12.5e6, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = True,   # deployment shape: always XIP
            flash_master      = True,   # always WiFi-flashable
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )
        add_flashing_baseline(self, aux_spi_clk_freq, busy_led=0)


def main():
    parser = make_parser(description="IcePi Zero baseline SoC (recovery/fallback "
                                     "image: XIP BIOS + flash master + aux bus, no features).")
    parser.add_target_argument("--aux-spi-clk-freq", default=12.5e6, type=float,
                               help="Aux-bus default SCK frequency (Hz). Default 12.5 MHz "
                                    "(matches the other deployables). Runtime-adjustable "
                                    "via the aux_spi clk_divider CSR.")
    args = parser.parse_args()

    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = BaselineSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        aux_spi_clk_freq  = args.aux_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
