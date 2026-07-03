#!/usr/bin/env python3
"""IcePi Zero SoC -- UART RX-report (C3->FPGA receive-side isolation test).

The beacon proved FPGA->C3 works perfectly; every transport failed on the OTHER
direction (C3->FPGA). This bitstream gives us eyes on the FPGA's receive side:
the FPGA captures each byte arriving on L1 (RX) and continuously reflects that
last byte back out M2 (the proven-good TX path) -- gateware add_c3_uart_rxreport,
no CPU/firmware. The C3 (software/c3_uart_rxreport_esp) sends a known value and
checks what comes back:

  reflects the sent value -> FPGA receives on L1 fine; the bug was protocol framing.
  reflects 0x00 forever   -> FPGA receives nothing on L1 (dead RX pin/input).
  reflects garbage        -> receiving but mis-framed (baud/clock).

Just build + load (reload after any power cycle -- SRAM load is volatile).
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_uart_rxreport


class C3UartRxSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, **kwargs):
        kwargs["integrated_main_ram_size"] = 0x8000
        super().__init__(
            sys_clk_freq = sys_clk_freq,
            with_sdram   = False,
            **kwargs,
        )
        add_c3_uart_rxreport(self, baudrate=1200)


def main():
    parser = make_parser(description="IcePi Zero SoC + UART RX-report "
                                     "(C3->FPGA receive-side isolation test).")
    args = parser.parse_args()

    soc = C3UartRxSoC(
        sys_clk_freq      = args.sys_clk_freq,
        bios_flash_offset = args.bios_flash_offset,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
