#!/usr/bin/env python3
"""IcePi Zero SoC for PSC16S (PISC v2) prototype bring-up.

PROTOTYPE against a DRAFT ISA -- docs/psc16s_isa_draft.md. The v1 bring-up top
is icepi_zero_pisc.py and is untouched; this is its sibling.

Deliberately lighter than icepi_zero_pisc.py: no WINC aux bus, no boot-manager,
no LCD. The subject under test is the core, and the v1 top's OTA hardware is
there for a flashing path this does not need -- load over JTAG (`--load`) and
iterate. Add `--with-spi-flash` if you want the XIP/OTA shape back.

Two things here are new versus v1 and are the actual point of the build:

- **The core is a Wishbone master.** `add_psc16s` registers it on the SoC bus
  with its 64K window pointed at the CSR base, so `BUS` can reach every CSR and
  structurally nothing else -- not SDRAM, not the flash mmap.
- **imem is 18 bits and split.** `--psc16s-init` supplies a $readmemh image for
  the read-only low region (what stage-0 would live in); the region above
  `--psc16s-ro-words` takes CSR writes like v1's imem.

**Autostart is OFF by default and should stay off here.** The stage-0 role has
the core holding the CPU in reset out of configuration; on a bring-up board
that means a wedged program takes the SoC that is debugging it with it. Bring
the ISA up as a peripheral first, then turn on autostart when there is a golden
model saying the program does what it should.

The watchdog is likewise off by default (`--psc16s-wdt-ms 0`): it exists to
force-release the CPU reset line, and nothing here drives one. Turn it on
together with autostart, not before.

Flow, from firmware or over litex_server/wishbone:

    python3 tools/psc16s_asm.py prog.s --py     # words to load
    ... write them at addr >= ro_words via imem_addr/imem_data/imem_ctl_we
    start_pc = ro_words ; control_run = 1 ; poll status.halted ; read result
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.psc16s import add_psc16s


class Psc16sSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6,
                 psc16s_imem_words=1024, psc16s_ro_words=256,
                 psc16s_prescale=1024, psc16s_autostart=False,
                 psc16s_wdt_cycles=0, psc16s_init="", **kwargs):
        super().__init__(sys_clk_freq=sys_clk_freq, **kwargs)

        # gpio_in/gpio_out stay CSR-only (unpinned), as in the v1 top -- no pad
        # conflicts, and the host can drive/observe both sides of WAIT/BITOP.
        add_psc16s(self,
                   imem_words     = psc16s_imem_words,
                   ro_words       = psc16s_ro_words,
                   num_out        = 4,
                   num_in         = 4,
                   delay_prescale = psc16s_prescale,
                   autostart      = psc16s_autostart,
                   wdt_cycles     = psc16s_wdt_cycles,
                   init_file      = psc16s_init)


def main():
    parser = make_parser(description="IcePi Zero SoC + PSC16S prototype core "
                                     "(PISC v2, DRAFT ISA -- see docs/psc16s_isa_draft.md).")
    parser.add_target_argument("--psc16s-imem-words", default=1024,
                               type=lambda x: int(x, 0),
                               help="Unified imem depth in 18-bit words (power of 2, "
                                    "default 1024 = 1 DP16KD).")
    parser.add_target_argument("--psc16s-ro-words", default=256,
                               type=lambda x: int(x, 0),
                               help="Low words reserved for the bitstream-resident "
                                    "stage-0 program; host writes below this are "
                                    "ignored (default 256).")
    parser.add_target_argument("--psc16s-prescale", default=1024,
                               type=lambda x: int(x, 0),
                               help="Core cycles per DELAY tick (default 1024: "
                                    "12.05 us/tick at 85 MHz, 197 ms max).")
    parser.add_target_argument("--psc16s-autostart", action="store_true",
                               help="Run at pc=0 out of reset with no host "
                                    "involvement. This is the stage-0 role -- see "
                                    "the module docstring before enabling it.")
    parser.add_target_argument("--psc16s-wdt-ms", default=0.0, type=float,
                               help="Watchdog deadline in ms (0 = disabled). Forces "
                                    "the CPU-reset port bit released on expiry; only "
                                    "meaningful with --psc16s-autostart.")
    parser.add_target_argument("--psc16s-init", default="",
                               help="$readmemh image for the read-only low imem "
                                    "region (tools/psc16s_asm.py --hex).")
    args = parser.parse_args()

    wdt_cycles = int(args.psc16s_wdt_ms * 1e-3 * args.sys_clk_freq)

    soc = Psc16sSoC(
        sys_clk_freq      = args.sys_clk_freq,
        psc16s_imem_words = args.psc16s_imem_words,
        psc16s_ro_words   = args.psc16s_ro_words,
        psc16s_prescale   = args.psc16s_prescale,
        psc16s_autostart  = args.psc16s_autostart,
        psc16s_wdt_cycles = wdt_cycles,
        psc16s_init       = args.psc16s_init,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
