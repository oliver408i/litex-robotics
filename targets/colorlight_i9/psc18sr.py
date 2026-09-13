#!/usr/bin/env python3
"""Colorlight i9 SoC with PSC18SR attached as a NON-BOOT SIDE BLOCK.

This is the rig for proving the core works on real hardware, and it is
deliberately the *safe* half of the core's two roles:

- **The CPU boots normally.** BIOS out of ROM, SDRAM, serial boot over the
  probe -- exactly `targets/colorlight_i9/bringup.py`, which is already known
  good on this board. Nothing about the boot path goes through PSC18SR.
- **The core is a peripheral hanging off the side.** It holds no reset line,
  gates nothing, and is not wired to anything the SoC needs. `autostart` is
  off, so it does not execute at all until firmware pulses `run`; the watchdog
  is off because there is no CPU reset for it to force-release.

So a program that wedges the core -- an unbounded `WAIT`, a `BUS` access that
never acks, a loop that never exits -- costs a `control.abort` write from the
CPU that is still perfectly alive and still holding the console. That is the
entire point of doing it in this order: the stage-0 role (autostart, holding
the CPU in reset, watchdog armed) takes the SoC down with the core when it
goes wrong, so it is not what you want to be debugging an unproven ISA on.

**Do not add `--psc18sr-autostart` to this top.** When the core is trusted
enough for the boot role, that is a different top, built once there is a golden
model to say the stage-0 program does what it should.

The gpio ports stay CSR-only (unpinned), as on the IcePi Zero top: no pad
conflicts, and firmware can drive `gpio_in` and observe `gpio_out`, so both
sides of `WAIT`/`BITOP`/`IN`/`OUT` are testable with nothing attached to the
board. Pinning them to the J-connectors is a later step, and a separate one.

Build, load, and test:

    .venv/bin/python targets/colorlight_i9/psc18sr.py --build --load

    cd software/psc18sr_host && make
    .venv/bin/python tools/socboot.py \
        --kernel software/psc18sr_host/psc18sr_host.bin \
        --expect "all passed" --fail FAILED

The firmware loads each program in `software/psc18sr_test/*.s`, runs it, and
reports PASS/FAIL against the result the program's own comments promise. All of
them pass on hardware as of 2026-09-12; see `docs/psc18sr_prototype.md`.

`tools/socboot.py` rather than `litex_term` because it needs no tty and exits
with a status, so the test is scriptable. It also handles the reset: `--reset
soft` is the BIOS `reboot`, `--reset hard --bitstream <file>` reconfigures the
FPGA through the probe.

Timing note: this closes 48 MHz with about 1 MHz of margin, where the plain
`bringup.py` has room to spare. The flow runs `--timing-allow-fail`, so read
the Fmax nextpnr reports rather than treating a bitstream as proof.
"""
from base import BaseSoC, make_parser, run_build

from gateware.psc18sr import add_psc18sr


class Psc18srSoC(BaseSoC):
    def __init__(self, sys_clk_freq=48e6, sdram_rate="1:1",
                 psc18sr_imem_words=1024, psc18sr_ro_words=256,
                 psc18sr_prescale=1024, **kwargs):
        super().__init__(sys_clk_freq=sys_clk_freq, sdram_rate=sdram_rate, **kwargs)

        # autostart / wdt_cycles are not parameters here on purpose -- see the
        # module docstring. Both belong to the boot role, which this top does
        # not have.
        add_psc18sr(self,
                    imem_words     = psc18sr_imem_words,
                    ro_words       = psc18sr_ro_words,
                    num_out        = 4,
                    num_in         = 4,
                    delay_prescale = psc18sr_prescale,
                    autostart      = False,
                    wdt_cycles     = 0,
                    init_file      = "")


def main():
    parser = make_parser(description="Colorlight i9 SoC + PSC18SR as a non-boot side "
                                     "block (PISC v2, DRAFT ISA -- see "
                                     "docs/psc18sr_isa_draft.md).")
    parser.add_target_argument("--psc18sr-imem-words", default=1024,
                               type=lambda x: int(x, 0),
                               help="Unified imem depth in 18-bit words (power of 2, "
                                    "default 1024 = 1 DP16KD).")
    parser.add_target_argument("--psc18sr-ro-words", default=256,
                               type=lambda x: int(x, 0),
                               help="Low words that reject host writes. Nothing lives "
                                    "there on this top (no init image, no autostart); "
                                    "kept at the real value so the address the firmware "
                                    "loads at is the address stage-0 would use.")
    parser.add_target_argument("--psc18sr-prescale", default=1024,
                               type=lambda x: int(x, 0),
                               help="Core cycles per DELAY tick (default 1024: 21.3 us/tick "
                                    "at 48 MHz, 349 ms max).")
    args = parser.parse_args()

    soc = Psc18srSoC(
        sys_clk_freq       = args.sys_clk_freq,
        sdram_rate         = "1:2" if args.sdram_half_rate else "1:1",
        psc18sr_imem_words = args.psc18sr_imem_words,
        psc18sr_ro_words   = args.psc18sr_ro_words,
        psc18sr_prescale   = args.psc18sr_prescale,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
