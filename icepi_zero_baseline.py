#!/usr/bin/env python3
"""IcePi Zero baseline SoC -- the pure WiFi-OTA flashing image, no features.

This is the smallest DEPLOYABLE bitstream: VexRiscv + UART + the OTA-flashing
baseline (XIP BIOS from flash, LiteSPI master, the ATWINC1500 aux bus, and the
boot-manager flag/reset). It carries no LCD, SNN, SD or PISC -- nothing but the
machinery flash.py needs to talk to the board over WiFi.

Why it exists: flash.py is WiFi-only, so every updatable bitstream must include
the WINC baseline (see gateware/soc_features.py add_flashing_baseline). This top
is that baseline on its own, which makes it the known-good recovery / fallback
image -- it always routes with huge margin, so if a feature build bricks or
won't fit, this is what you flash to get a board back to a WiFi-updatable state.
It is also the template every feature top builds on.

Pair with software/winc_loader (the OTA loader runs here unchanged -- it is
CSR-coupled, so flash a loader built against THIS SoC). See docs/boot_chain.md.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_flashing_baseline


class BaselineSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None,
                 winc_spi_clk_freq=12.5e6, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = True,   # deployment shape: always XIP
            flash_master      = True,   # always WiFi-flashable
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )
        add_flashing_baseline(self, winc_spi_clk_freq, busy_led=0)


def main():
    parser = make_parser(description="IcePi Zero baseline SoC (WiFi-OTA flashing "
                                     "image: XIP BIOS + flash master + WINC, no features).")
    parser.add_target_argument("--winc-spi-clk-freq", default=12.5e6, type=float,
                               help="WINC SPI SCK frequency (Hz). Default 12.5 MHz = tested "
                                    "ceiling (matches the other deployables). Runtime-adjustable "
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
        winc_spi_clk_freq = args.winc_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
