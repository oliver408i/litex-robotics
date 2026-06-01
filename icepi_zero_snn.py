#!/usr/bin/env python3
"""IcePi Zero SoC for the SNN tracking-estimator project."""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.snn_estimator import SNNTrackingEstimator


class SNNSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = with_spi_flash,
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )

        self.snn = SNNTrackingEstimator(self.platform)
        self.add_csr("snn")

        # Sanity LEDs: busy/done from the estimator.
        try:
            self.comb += [
                self.platform.request("user_led", 0).eq(self.snn.status.fields.busy),
                self.platform.request("user_led", 1).eq(self.snn.status.fields.done),
            ]
        except Exception:
            pass


def main():
    parser = make_parser(description="IcePi Zero SoC + SNN tracking estimator.")
    args = parser.parse_args()

    soc = SNNSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args),
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
