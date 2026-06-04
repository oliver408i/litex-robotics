#!/usr/bin/env python3
"""IcePi Zero SoC for the ATWINC1500 Wi-Fi bring-up project.

UART-only diagnostic SoC that puts the ATWINC1500 on the shared sensor/aux SPI
bus (same sclk/mosi/miso as the LSM6DS3 IMU + MCP3008, see
docs/icepi_zero_pin_mapping.md), driven by `gateware/aux_spi.py`'s AuxSPIMaster.
The pins, sidebands and constants live in gateware/soc_features.py
(add_winc_aux); this top is BaseSoC + that one feature.

This is the hardware half of the bring-up; pair it with software/winc_test
(and software/winc_loader when built with --flash-master).
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.soc_features import add_winc_aux, add_boot_ctl


class WincSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None,
                 flash_master=False, winc_spi_clk_freq=1e6, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = with_spi_flash,
            flash_boot_offset = flash_boot_offset,
            flash_master      = flash_master,
            **kwargs,
        )
        add_winc_aux(self, winc_spi_clk_freq, busy_led=0)
        add_boot_ctl(self)  # sticky boot flag + FTDI DTR/RTS sense


def main():
    parser = make_parser(description="IcePi Zero SoC + ATWINC1500 (Wi-Fi SPI bring-up).")
    parser.add_target_argument("--winc-spi-clk-freq", default=1e6, type=float,
                               help="Initial WINC SPI SCK frequency (Hz). Default 1 MHz "
                                    "(conservative for bring-up; WINC supports up to ~48 MHz, "
                                    "this bus is capped at sys/2 = 25 MHz). Runtime-adjustable "
                                    "via the aux_spi clk_divider CSR.")
    parser.add_target_argument("--flash-master", action="store_true",
                               help="Expose the LiteSPI master CSRs so SDRAM-resident firmware "
                                    "(the WiFi flash-loader) can erase/program the SPI flash. "
                                    "Implies the XIP flash build; the BIOS skips master init "
                                    "(SPIFLASH_SKIP_MASTER_INIT). See docs/boot_chain.md.")
    args = parser.parse_args()

    soc = WincSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args) or args.flash_master,
        flash_boot_offset = args.flash_boot_offset,
        flash_master      = args.flash_master,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        winc_spi_clk_freq = args.winc_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
