#!/usr/bin/env python3
"""IcePi Zero SoC for the MCP23S17 SPI GPIO-expander bring-up.

UART-only diagnostic SoC that puts the MCP23S17 I/O expander on the shared
sensor/aux SPI bus (same sclk/mosi/miso as the LSM6DS3 IMU + MCP3008, see
docs/icepi_zero_pin_mapping.md), driven by gateware/aux_spi.py's AuxSPIMaster.
The expander adds a 4th aux chip-select (AUX_CS_IOX, IO11/G2) plus a reset
(IO10/L2) and an interrupt (IO22/P2) sideband. Pins + CSRs live in
gateware/soc_features.py (add_mcp_expander); this top is BaseSoC + that feature.

This is the hardware half of the expander bring-up that replaces the retired
ATWINC1500 (board refactor 2026-06; an ESP32-C3 will later take over the
WiFi-loader role). There is no WiFi OTA path post-WINC, so serial-boot the test
kernel over UART:

    litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/mcp_test/mcp_test.bin

Pair it with software/mcp_test. Build the gateware first so generated/csr.h
carries the aux_spi + iox_* CSRs this firmware is coupled to.
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from gateware.soc_features import add_mcp_expander, add_gpa7_loopback, add_boot_ctl


class McpSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None,
                 flash_master=False, iox_spi_clk_freq=1e6, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = with_spi_flash,
            flash_boot_offset = flash_boot_offset,
            flash_master      = flash_master,
            **kwargs,
        )
        add_mcp_expander(self, iox_spi_clk_freq, busy_led=0)
        add_gpa7_loopback(self)  # bench fixture: IO24/L1 <-> MCP23S17 GPA7
        add_boot_ctl(self)       # sticky boot flag + FTDI DTR/RTS sense


def main():
    parser = make_parser(description="IcePi Zero SoC + MCP23S17 SPI GPIO expander (aux-bus bring-up).")
    parser.add_target_argument("--iox-spi-clk-freq", default=1e6, type=float,
                               help="Initial expander SPI SCK frequency (Hz). Default 1 MHz "
                                    "(conservative for bring-up; the MCP23S17 supports up to "
                                    "10 MHz, this bus is capped at sys/2 = 25 MHz). "
                                    "Runtime-adjustable via the aux_spi clk_divider CSR.")
    parser.add_target_argument("--flash-master", action="store_true",
                               help="Expose the LiteSPI master CSRs (XIP flash build). Not needed "
                                    "for expander bring-up -- serial-boot the test kernel instead.")
    args = parser.parse_args()

    soc = McpSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args) or args.flash_master,
        flash_boot_offset = args.flash_boot_offset,
        flash_master      = args.flash_master,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        iox_spi_clk_freq  = args.iox_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
