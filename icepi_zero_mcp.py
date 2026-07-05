#!/usr/bin/env python3
"""IcePi Zero SoC for the MCP23S17 SPI GPIO-expander bring-up, on the C3 loader.

Puts the MCP23S17 I/O expander on the shared sensor/aux SPI bus (same
sclk/mosi/miso as the LSM6DS3 IMU + MCP3008, see docs/icepi_zero_pin_mapping.md),
driven by gateware/aux_spi.py's AuxSPIMaster. The expander adds a 4th aux
chip-select (AUX_CS_IOX, IO17/R3) plus a reset (IO10/L2) and an interrupt
(IO22/P2) sideband. Pins + CSRs live in gateware/soc_features.py
(add_mcp_expander); this top is C3FlashSoC's shape (icepi_zero_c3flash.py) +
that feature, so the resident C3 loader (software/c3_flash) can flash this
bitstream/BIOS/loader and chain-boot software/mcp_test as the app -- see
docs/c3_loader.md. No JTAG/litex_term needed once the loader is resident:
    flash.py --bitstream --bios --loader --port /dev/ttyACM0   (+ power-cycle)
    flash.py --app software/mcp_test/mcp_test.bin --port /dev/ttyACM0
    flash.py --boot-app --port /dev/ttyACM0

NOTE: the GPA7 loopback bench fixture (add_gpa7_loopback) is NOT included here
-- it shares IO24/L1 with the C3 link's MOSI line, a real pin conflict, not
just a build one. Verify the expander via the register echo probe, output
walk, and GPB/INTA input watch instead; re-add loopback on a free pin later
if still wanted.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_mcp_expander, add_c3_spibone, add_c3_mailbox, add_boot_flag


class McpSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, flash_boot_offset=None, iox_spi_clk_freq=1e6, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = True,    # loader firmware drives the flash...
            flash_master      = True,    # ...via the LiteSPI master CSRs
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )
        add_c3_spibone(self)     # ESP32-C3 as a Wishbone master over SPI
        add_c3_mailbox(self)     # command/data mailbox RAM @ 0x90000000
        add_boot_flag(self)      # sticky boot-to-app flag for chain-boot
        add_mcp_expander(self, iox_spi_clk_freq, busy_led=0)


def main():
    parser = make_parser(description="IcePi Zero SoC + MCP23S17 SPI GPIO expander, on the C3 loader.")
    parser.add_target_argument("--iox-spi-clk-freq", default=1e6, type=float,
                               help="Initial expander SPI SCK frequency (Hz). Default 1 MHz "
                                    "(conservative for bring-up; the MCP23S17 supports up to "
                                    "10 MHz, this bus is capped at sys/2 = 25 MHz). "
                                    "Runtime-adjustable via the aux_spi clk_divider CSR.")
    args = parser.parse_args()

    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = McpSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        iox_spi_clk_freq  = args.iox_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
