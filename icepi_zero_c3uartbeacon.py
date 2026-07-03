#!/usr/bin/env python3
"""IcePi Zero SoC -- one-way UART BEACON (direction-isolation test).

Every C3->FPGA transport (echo, SPIBone, UARTBone) failed identically: the FPGA
never acts on what the C3 sends. This bitstream tests the OTHER direction alone.
The FPGA continuously transmits an incrementing byte on M2 (gateware
add_c3_uart_beacon); no RX, no CPU, no firmware. The C3 just listens on GPIO3
(software/c3_uart_listen_esp) -- a direction the GPIO diag (Test B) and the C3
self-loopback both proved good.

  FPGA tx = M2/IO23 -> C3 RX (GPIO3)     (rx = L1 present but unused/drained)

Interpretation:
  C3 sees a clean 00,01,02,... stream -> FPGA->C3 works + bitstream is alive +
    baud is right; therefore the fault is the C3->FPGA direction.
  C3 sees nothing/garbage -> FPGA->C3 (M2 TX) is broken or the baud is wrong.

Just build + load (reload after any power cycle -- JTAG SRAM is volatile).
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_uart_beacon


class C3UartBeaconSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, **kwargs):
        kwargs["integrated_main_ram_size"] = 0x8000
        super().__init__(
            sys_clk_freq = sys_clk_freq,
            with_sdram   = False,
            **kwargs,
        )
        add_c3_uart_beacon(self, baudrate=1200)


def main():
    parser = make_parser(description="IcePi Zero SoC + one-way UART beacon "
                                     "(FPGA->C3 direction-isolation test).")
    args = parser.parse_args()

    soc = C3UartBeaconSoC(
        sys_clk_freq      = args.sys_clk_freq,
        bios_flash_offset = args.bios_flash_offset,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
