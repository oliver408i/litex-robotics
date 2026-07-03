#!/usr/bin/env python3
"""IcePi Zero SoC for the ESP32-C3 link CONTINUITY DIAGNOSTIC.

The C3 <-> FPGA SPI link has never reliably passed a byte even though both sides
appear healthy. This bitstream replaces the SPISlave transport with a raw 5-bit
tristate GPIO on the *same five balls* (P3/L1/M2/J3/R1), so a tiny SRAM-resident
firmware (software/c3_diag) can toggle and read each pin directly -- no shift
register, no clock domain, no command protocol. Pair it with the matching C3
firmware mode to prove signal continuity (and its direction) wire by wire before
trusting the real SPI stack. See docs/c3_loader.md for the pinout.

Mirrors icepi_zero_c3loader.py's deployment shape so it serial-boots identically:
SDRAM is OMITTED (it's dead) -- main_ram is a 32 KB on-chip BRAM, so the BIOS and
the diag firmware run from real memory. Serial-boot the kernel over UART:

    litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/c3_diag/c3_diag.bin

Flash is NOT needed for the diagnostic, but the diag SoC keeps with_spi_flash off
to stay minimal and route easily (the diag never touches flash).
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_diag


class C3DiagSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, **kwargs):
        # Match the c3loader bring-up shape: no SDRAM (dead), 32 KB BRAM main_ram
        # so the BIOS + diag firmware run from real memory and serial-boot works.
        kwargs["integrated_main_ram_size"] = 0x8000   # 32 KB BRAM main_ram
        super().__init__(
            sys_clk_freq = sys_clk_freq,
            with_sdram   = False,   # SDRAM is dead; main_ram is BRAM
            **kwargs,
        )
        add_c3_diag(self)      # raw tristate GPIO on the C3 SPI balls


def main():
    parser = make_parser(description="IcePi Zero SoC + ESP32-C3 link continuity "
                                     "diagnostic (raw GPIO on the C3 SPI balls).")
    args = parser.parse_args()

    soc = C3DiagSoC(
        sys_clk_freq      = args.sys_clk_freq,
        bios_flash_offset = args.bios_flash_offset,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
