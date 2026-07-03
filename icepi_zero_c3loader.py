#!/usr/bin/env python3
"""IcePi Zero SoC for the ESP32-C3 SPI flash-loader bring-up.

The post-WINC loader: the FPGA is an SPI SLAVE on a dedicated bus and the
ESP32-C3 (master) streams flash erase/program/verify commands; a small
SRAM-resident firmware drains the SPISlave FIFO and drives the LiteSPI master to
program the SPI flash. Pins + the SPISlave transport core live in
gateware/soc_features.py (add_c3_loader) / gateware/spi_slave.py. See
docs/c3_loader.md for the architecture, wiring and protocol.

Forces the flash-master deployment shape (with_spi_flash + flash_master) because
the loader firmware MUST drive the LiteSPI master to program flash. SDRAM is
OMITTED (with_sdram=False) -- main_ram is on-chip BRAM, so the BIOS + firmware
run from real memory with the SDRAM dead. There is no WiFi OTA post-WINC, so
serial-boot the loader kernel over UART during bring-up (default main_ram addr):

    litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/c3_loader/c3_loader.bin

Pair with the C3-side firmware (USB-CDC <-> SPI master) and the FPGA loader
firmware. Build the gateware first so generated/csr.h carries the c3_* CSRs the
firmware is coupled to.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_loader, add_boot_ctl


class C3LoaderSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None, **kwargs):
        # SDRAM is dead, and a broken SDRAM main_ram breaks the BIOS itself (it
        # keeps its .data/.bss/stack there -- it limps to a console from XIP but
        # garbles serialboot). So this bitstream OMITS SDRAM and uses a 32 KB
        # on-chip BRAM as main_ram: the BIOS and the loader firmware both run
        # from real memory. Dropping the WINC host driver keeps the firmware
        # small (~8.5 KB), so 32 KB is ample (firmware + BIOS stack/data).
        kwargs["integrated_main_ram_size"] = 0x8000   # 32 KB BRAM main_ram
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = True,    # loader firmware drives the flash...
            flash_master      = True,    # ...via the LiteSPI master CSRs
            with_sdram        = False,   # no SDRAM (it's dead); main_ram is BRAM
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )
        add_c3_loader(self)      # FPGA-slave SPI link to the ESP32-C3
        add_boot_ctl(self)       # sticky boot flag + FTDI DTR/RTS sense (reused)


def main():
    parser = make_parser(description="IcePi Zero SoC + ESP32-C3 SPI flash-loader link "
                                     "(FPGA-slave bring-up).")
    args = parser.parse_args()

    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = C3LoaderSoC(
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
