#!/usr/bin/env python3
"""IcePi Zero SoC for the SNN-MLP MNIST classifier.

Same shape as icepi_zero_snn.py: BaseSoC + one extra peripheral. Adds the
SNNMLP block, wires its Wishbone master into the system bus so it can read
the weight blob from SDRAM, and mirrors busy/done on the user LEDs when
they're available.
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.snn_mlp import SNNMLP

# MAC parallelism for the SNN core. N_MAC=2 is the sweet spot on this board:
# two Q4.12 weights pack exactly into one 32-bit Wishbone word, so it halves
# both core cycles AND SDRAM weight traffic with zero plumbing changes. Going
# higher does NOT help here — the 16-bit SDR SDRAM (~100 MB/s) is the wall, and
# at N_MAC=2 the core is already faster than the DRAM can feed it. See
# docs/snn_mnist.md for the bandwidth analysis. Must match the --n-mac passed
# to tools/pack_snn_mnist_weights.py.
N_MAC = 2


class SNNMnistSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = with_spi_flash,
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )

        self.snn = SNNMLP(self.platform, n_mac=N_MAC)
        self.add_csr("snn")
        self.bus.add_master(name="snn_wb", master=self.snn.wb)

        # Status LEDs: busy on LED0, done on LED1 when the board exposes them.
        try:
            self.comb += [
                self.platform.request("user_led", 0).eq(self.snn.status.fields.busy),
                self.platform.request("user_led", 1).eq(self.snn.status.fields.done),
            ]
        except Exception:
            pass


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
