#!/usr/bin/env python3
"""IcePi Zero SoC -- ESP32-C3 SPIBone flash loader (hybrid: C3 brain, FPGA programmer).

The post-WINC loader, rebuilt on the verified SPIBone transport (icepi_zero_c3spibone.py
proved the C3 is a solid Wishbone master over SPI at up to 10 MHz). Architecture:
the C3 stages the image over USB-CDC, writes a command + page data into a mailbox
RAM over SPIBone, and rings a doorbell; a tiny SRAM-resident FPGA firmware
(software/c3_flash) polls the mailbox and drives the LiteSPI MASTER to erase/
program/verify, reusing the validated flash_w25q.c verbatim.

XIP-safety: the loader firmware runs from main_ram (SDRAM, not flash), so it never
instruction-fetches through the flash mmap while it issues master commands (the
mode-knock hazard in flash_w25q.h / docs/boot_chain.md). SDRAM is required (the
half-rate SDRAM fix is hardware-confirmed -- see docs on the CRG phase/DDROutput
fix). Forces the flash-master deployment shape (with_spi_flash + flash_master)
so flash_w25q.c has its spiflash_master_* CSRs.

Bring-up: serial-boot the loader over UART:
    litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/c3_flash/c3_flash.bin

Then the C3 (software/c3_flash_esp) drives it over SPIBone. Stage 1 = PING (read
the flash JEDEC id back through the mailbox); erase/program/verify follow.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_spibone, add_c3_mailbox, add_boot_flag


class C3FlashSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = True,    # loader firmware drives the flash...
            flash_master      = True,    # ...via the LiteSPI master CSRs
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )
        add_c3_spibone(self)     # ESP32-C3 as a Wishbone master over SPI
        add_c3_mailbox(self)     # command/data mailbox RAM @ 0x90000000
        add_boot_flag(self)      # sticky boot-to-app flag for chain-boot (docs/c3_loader.md)


def main():
    parser = make_parser(description="IcePi Zero SoC + ESP32-C3 SPIBone flash loader.")
    args = parser.parse_args()

    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = C3FlashSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
