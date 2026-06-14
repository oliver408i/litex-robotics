#!/usr/bin/env python3
"""IcePi Zero SoC for PISC sequencer-core bring-up.

VexRiscv + UART + the PISC peripheral (gateware/pisc.py), built in the same
WiFi-updatable shape as the deployed all-SoC: XIP BIOS from flash, LiteSPI
master, the ATWINC1500 aux bus, and the boot-manager flag/reset. No LCD and no
SNN, so it stays far lighter than icepi_zero_all.py (no 185 MHz LCD domain ->
fast builds, trivial timing) while keeping PISC isolated as the only new logic.

Like icepi_zero_all.py, this forces the deployment shape (with_spi_flash +
flash_master always on), so a plain `--build` produces a full XIP-BIOS +
LiteSPI-master bitstream and bios.bin -- ready to load over the fast WiFi OTA
path (./flash.py) exactly like an all-SoC build. The loader hardware is here
not because this is a deployment image but because OTA beats the bit-banged
JTAG load (kept as the slow recovery backup). The aux-bus config mirrors
icepi_zero_winc.py, so the hardware-verified winc_loader runs here unchanged
(it is CSR-coupled -- flash a winc_loader built against THIS SoC). PISC's
gpio_in/gpio_out stay CSR-only (not pinned), so they don't touch the aux pins.
See docs/boot_chain.md and docs/pisc_isa.md.

Pair with software/pisc_test: it loads programs into the core, runs them, and
prints the results (sum 1..10 -> 55, etc.).
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.pisc import PISC
from gateware.soc_features import add_winc_aux, add_boot_ctl


class PiscSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None,
                 winc_spi_clk_freq=12.5e6, pisc_imem_words=256, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = True,   # deployment shape: always XIP (mirror all-SoC)
            flash_master      = True,   # always WiFi-flashable -> `--build` alone is OTA-ready
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )
        # WiFi OTA loader hardware (same as icepi_zero_winc.py): shared aux SPI
        # bus + WINC sidebands, plus the boot flag / FTDI reset the chain uses.
        add_winc_aux(self, winc_spi_clk_freq, busy_led=0)
        add_boot_ctl(self)

        # The actual subject: PISC as a CSR-mapped peripheral. gpio stays
        # CSR-exposed (gpio_in/gpio_out), unpinned, so no aux-pin conflict.
        self.pisc = PISC(self.platform, imem_words=pisc_imem_words,
                         num_out=2, num_in=2)
        self.add_csr("pisc")


def main():
    parser = make_parser(description="IcePi Zero SoC + PISC sequencer core "
                                     "(WiFi-updatable: XIP BIOS + flash master + WINC loader bus).")
    parser.add_target_argument("--winc-spi-clk-freq", default=12.5e6, type=float,
                               help="WINC SPI SCK frequency (Hz). Default 12.5 MHz = tested "
                                    "ceiling (matches all-SoC; 25 MHz corrupts MISO on jumper "
                                    "wiring). Runtime-adjustable via the aux_spi clk_divider CSR.")
    parser.add_target_argument("--pisc-imem-words", default=256,
                               type=lambda x: int(x, 0),
                               help="PISC instruction-memory depth in 16-bit words "
                                    "(power of 2, default 256).")
    args = parser.parse_args()

    # Standalone boot by default: BIOS flash-boots the firmware (loader) at the
    # firmware offset; serial boot stays available as the fallback. Mirrors
    # icepi_zero_all.py.
    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = PiscSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        winc_spi_clk_freq = args.winc_spi_clk_freq,
        pisc_imem_words   = args.pisc_imem_words,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
