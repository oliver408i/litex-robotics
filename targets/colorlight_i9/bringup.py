#!/usr/bin/env python3
"""Colorlight i9 bring-up SoC: VexRiscv + SDRAM + SPI flash + UART. Nothing else.

This is the board's "does it work at all" target, and the thing to build when
changing the clock or the SDRAM PHY, because the BIOS exercises both without
any firmware of ours having to be correct:

  * the BIOS banner arriving at all proves the CPU runs and the UART divisor is
    right, i.e. RV32 code executes from ROM;
  * `Memtest OK` plus the Memspeed figures prove the SDRAM PHY, whichever rate
    it was built with;
  * `Booting from serial` means litex_term can push a real program into SDRAM
    and run it -- the actual RV32 test.

Build and load (the probe is the only link to this board -- no FTDI):

    .venv/bin/python targets/colorlight_i9/bringup.py --build
    # then program build/.../gateware/*.bit through probe/icelink-fast ('p')

    # console, over the probe's second CDC:
    litex_term /dev/icelink-uart --speed 115200

    # run an RV32 program out of SDRAM:
    litex_term /dev/icelink-uart --speed 115200 --kernel software/diag/diag.bin

Half-rate SDRAM is the other thing worth testing here (--sdram-half-rate); see
base.py for what that changes.
"""
from base import BaseSoC, make_parser, run_build


def main():
    parser = make_parser(description="Colorlight i9 bring-up SoC (CPU + SDRAM + flash + UART).")
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq = args.sys_clk_freq,
        sdram_rate   = "1:2" if args.sdram_half_rate else "1:1",
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
