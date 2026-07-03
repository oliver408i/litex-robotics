#!/usr/bin/env python3
"""IcePi Zero SoC -- ESP32-C3 link via the verified SPIBone Wishbone bridge.

PHASE 1 (transport proof). The raw-GPIO diagnostic (icepi_zero_c3diag.py) proved
every C3<->FPGA wire is electrically perfect in both directions, so the old
"never worked" link bug lived in the custom SPISlave stack. This SoC replaces
that entire stack -- custom slave + PING/ERASE/PROGRAM protocol + READY pin --
with LiteX's maintained SPIBone bridge (gateware/soc_features.py add_c3_spibone,
core litex.soc.cores.spi.spi_bone). The ESP32-C3 becomes a Wishbone MASTER and
reads/writes the SoC bus directly; pair with software/c3_spibone_esp.

Validate the transport with ZERO custom RTL by reading the built-in control
scratch register (ctrl_scratch, reset value 0x12345678, R/W) and round-tripping a
write -- the modern replacement for the flaky PING. This bitstream is deliberately
minimal (no flash, no SDRAM): the CPU just idles in its BIOS while SPIBone masters
the bus. Once the transport is green, a Phase-2 bitstream adds the LiteSPI flash
master so the C3 can program flash over the same bridge.

No firmware serial-boot is required for the transport test -- the CPU boots its
integrated-ROM BIOS and sits at the prompt; SPIBone works independently. SDRAM is
omitted (it's dead) so main_ram is a 32 KB BRAM and the BIOS runs from real memory.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_spibone


class C3SPIBoneSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, **kwargs):
        kwargs["integrated_main_ram_size"] = 0x8000   # 32 KB BRAM main_ram
        super().__init__(
            sys_clk_freq = sys_clk_freq,
            with_sdram   = False,   # SDRAM is dead; main_ram is BRAM
            **kwargs,
        )
        add_c3_spibone(self)      # ESP32-C3 as a Wishbone master over SPI


def main():
    parser = make_parser(description="IcePi Zero SoC + ESP32-C3 SPIBone Wishbone "
                                     "bridge (Phase 1: transport proof).")
    args = parser.parse_args()

    soc = C3SPIBoneSoC(
        sys_clk_freq      = args.sys_clk_freq,
        bios_flash_offset = args.bios_flash_offset,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
