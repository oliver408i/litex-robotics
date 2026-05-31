#!/usr/bin/env python3
"""TEMPORARY test SoC for the ring-oscillator experiment.

Minimal BaseSoC + RingOscillator, nothing else. Pair with the
`software/ringosc_demo` firmware, which streams the measured ring frequency
over UART. Throwaway scaffolding for poking at the ring osc -- not part of any
real project.

    python icepi_zero_ringosc.py --build --load --nextpnr-ignoreloops
    (cd software/ringosc_demo && make)
    litex_term /dev/ttyUSB0 --kernel software/ringosc_demo/ringosc_demo.bin

The --nextpnr-ignoreloops flag is required: the ring oscillator is an
intentional combinational loop, which nextpnr-ecp5 otherwise refuses to route.
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.ring_osc import RingMonitor


class RingOscSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = with_spi_flash,
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )

        # num_heaters>0 keeps the self-heating demo bank; a real drop-in monitor
        # would leave it at 0. gate_ms/prescale defaults give ~100 Hz updates.
        self.ringosc = RingMonitor(self.platform, sys_clk_freq, num_heaters=32)
        self.add_csr("ringosc")

        # Sanity LED: lit whenever measurements are enabled.
        try:
            self.comb += self.platform.request("user_led", 0).eq(self.ringosc._enable.storage)
        except Exception:
            pass


def main():
    parser = make_parser(description="TEMPORARY IcePi Zero SoC + ring-oscillator test.")
    args = parser.parse_args()

    soc = RingOscSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args),
        flash_boot_offset = args.flash_boot_offset,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
