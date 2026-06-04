#!/usr/bin/env python3
"""IcePi Zero SoC for the SNN-MLP MNIST classifier.

Same shape as icepi_zero_snn.py: BaseSoC + one extra peripheral. The SNNMLP
block (Wishbone master for SDRAM weight streaming, busy/done LEDs, and the
N_MAC=2 rationale) lives in gateware/soc_features.py (add_snn_mlp).
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.soc_features import add_snn_mlp


class SNNMnistSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = with_spi_flash,
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )
        add_snn_mlp(self, leds=(0, 1))


def main():
    parser = make_parser(description="IcePi Zero SoC + SNN-MLP MNIST classifier.")
    args = parser.parse_args()

    soc = SNNMnistSoC(
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
