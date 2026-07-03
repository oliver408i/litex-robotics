#!/usr/bin/env python3
"""IcePi Zero SoC -- ESP32-C3 link UART echo (transport sanity check).

Both SPI transports (custom SPISlave and LiteX SPIBone) failed identically
("MISO returns 00") though the raw-GPIO diagnostic proved every wire good both
ways. This bitstream tests a completely different protocol -- async UART -- to
answer: is the link bug SPI-specific, or is the C3<->FPGA channel more deeply
broken? The FPGA echoes every received byte back in PURE GATEWARE (RX -> FIFO ->
TX); no CPU, no firmware (gateware/soc_features.py add_c3_uart_echo).

Reuses the existing MOSI/MISO wires as the UART pair -- NO rewiring:
  FPGA RX = L1/IO24 <- C3 TX (GPIO1)     FPGA TX = M2/IO23 -> C3 RX (GPIO3)

Just build + load; the C3 firmware (software/c3_uart_esp) sends bytes and checks
the echo. No SDRAM (dead) -> main_ram is BRAM so the BIOS still comes up, but it
is irrelevant here (the echo is gateware-only).
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_uart_echo


class C3UartSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, **kwargs):
        kwargs["integrated_main_ram_size"] = 0x8000   # 32 KB BRAM main_ram
        super().__init__(
            sys_clk_freq = sys_clk_freq,
            with_sdram   = False,
            **kwargs,
        )
        add_c3_uart_echo(self, baudrate=1200)   # crawl: 833us/bit, huge RC margin


def main():
    parser = make_parser(description="IcePi Zero SoC + ESP32-C3 UART echo "
                                     "(transport sanity check).")
    args = parser.parse_args()

    soc = C3UartSoC(
        sys_clk_freq      = args.sys_clk_freq,
        bios_flash_offset = args.bios_flash_offset,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
