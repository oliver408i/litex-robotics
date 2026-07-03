#!/usr/bin/env python3
"""IcePi Zero SoC -- ESP32-C3 link via the verified UARTBone Wishbone bridge.

The async twin of icepi_zero_c3spibone.py. After both SPI transports failed
identically ("MISO returns 00"), this makes the ESP32-C3 a Wishbone MASTER over
plain UART using LiteX's UARTBone -- the most battle-tested bridge in the tree
(litex_server/wishbone-tool speak it). No CS, no clock phase, no tristate, no
firmware. Same loader goal (C3 -> WB master -> flash), non-SPI transport.

Reuses the MOSI/MISO wires as the UART pair -- NO rewiring:
  FPGA rx = L1/IO24 <- C3 TX (GPIO1)     FPGA tx = M2/IO23 -> C3 RX (GPIO3)

Validate with software/c3_uartbone_esp: read ctrl_scratch (byte 0xF0000804 ->
WORD address 0x3C000201, reset 0x12345678) and round-trip a write. Just build +
load; the CPU idles in its BIOS while UARTBone masters the bus. SDRAM omitted
(dead) -> main_ram is BRAM.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_uartbone


class C3UartBoneSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, **kwargs):
        kwargs["integrated_main_ram_size"] = 0x8000   # 32 KB BRAM main_ram
        super().__init__(
            sys_clk_freq = sys_clk_freq,
            with_sdram   = False,
            **kwargs,
        )
        add_c3_uartbone(self, baudrate=115200)  # UARTBone's normal range; its 100ms
                                                 # FSM watchdog makes 1200 baud unsafe


def main():
    parser = make_parser(description="IcePi Zero SoC + ESP32-C3 UARTBone Wishbone "
                                     "bridge (verified UART transport).")
    args = parser.parse_args()

    soc = C3UartBoneSoC(
        sys_clk_freq      = args.sys_clk_freq,
        bios_flash_offset = args.bios_flash_offset,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
