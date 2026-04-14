#!/usr/bin/env python3
import os
import sys

from migen import Cat, ClockDomain, Signal

from litex.gen import LiteXModule

from litex.build.generic_platform import Pins, Subsignal
from litex_boards.platforms import icepi_zero
from litex.soc.cores.clock import ECP5PLL
from litex.soc.cores.gpio import GPIOIn, GPIOOut
from litex.soc.integration.builder import Builder
from litex.soc.integration.soc_core import SoCCore

from gateware.ws2812_status_verilog import WS2812StatusVerilog


class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.cd_sys = ClockDomain()

        clk50 = platform.request("clk50")
        rst = platform.request("rst")
        ext_reset_n = platform.request("ext_reset")

        self.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~rst | ~ext_reset_n | self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)


class BaseSoC(SoCCore):
    def __init__(self, device="LFE5U-25F", toolchain="trellis", sys_clk_freq=50e6, hc05_baudrate=38400, **kwargs):
        platform = icepi_zero.Platform(device=device, toolchain=toolchain)
        platform.add_extension([
            ("hc05", 0,
                Subsignal("tx", Pins("P1")),
                Subsignal("rx", Pins("N1")),
                Subsignal("en", Pins("N4")),
            ),
            ("shared_spi", 0,
                Subsignal("clk", Pins("T2")),
                Subsignal("cs_n", Pins("R2 R1")),
                Subsignal("mosi", Pins("H2")),
                Subsignal("miso", Pins("J2")),
            ),
            ("lr1121", 0,
                Subsignal("busy", Pins("J3")),
                Subsignal("dio9", Pins("H3")),
                Subsignal("reset_n", Pins("D4")),
            ),
        ])

        kwargs.setdefault("integrated_rom_size", 0x8000)
        kwargs.setdefault("integrated_main_ram_size", 0x4000)
        kwargs.setdefault("uart_name", "serial")
        kwargs.setdefault("uart_baudrate", 1_000_000)
        kwargs.setdefault("cpu_type", "vexriscv")

        self.crg = _CRG(platform, sys_clk_freq)

        SoCCore.__init__(
            self,
            platform,
            sys_clk_freq,
            ident="Minimal LiteX SoC on IcePi Zero",
            **kwargs,
        )

        hc05_pads = platform.request("hc05")
        self.add_uart(
            name="hc05_uart",
            uart_pads=hc05_pads,
            baudrate=hc05_baudrate,
        )
        self.hc05_en = GPIOOut(hc05_pads.en, reset=1)
        self.add_csr("hc05_en")

        self.add_spi_master(
            name="shared_spi",
            pads=platform.request("shared_spi"),
            data_width=8,
            spi_clk_freq=1_000_000,
            mode="aligned",
        )
        self.add_constant("SHARED_SPI_CS_MCP3008", 0x1)
        self.add_constant("SHARED_SPI_CS_LR1121", 0x2)

        lr1121_pads = platform.request("lr1121")
        self.lr1121_status = GPIOIn(Cat(lr1121_pads.busy, lr1121_pads.dio9))
        self.add_csr("lr1121_status")
        self.lr1121_reset = GPIOOut(lr1121_pads.reset_n, reset=1)
        self.add_csr("lr1121_reset")

        self.rgb_led = WS2812StatusVerilog(
            pad=platform.request("rgb_led"),
            sys_clk_freq=sys_clk_freq,
            platform=platform,
            led_count=300,
            status_led=False,
        )
        self.add_csr("rgb_led")


def main():
    from litex.build.parser import LiteXArgumentParser

    # Keep LiteX's nested software builds on the same interpreter that launched us.
    os.environ["PYTHON"] = sys.executable

    parser = LiteXArgumentParser(
        platform=icepi_zero.Platform,
        description="Minimal IcePi Zero LiteX SoC.",
    )
    parser.set_defaults(uart_baudrate=1_000_000)
    parser.add_target_argument(
        "--device",
        default="LFE5U-25F",
        help="FPGA device (LFE5U-25F or LFE5U-45F).",
    )
    parser.add_target_argument(
        "--sys-clk-freq",
        default=50e6,
        type=float,
        help="System clock frequency.",
    )
    parser.add_target_argument(
        "--hc05-baudrate",
        default=38400,
        type=int,
        help="HC-05 UART baudrate.",
    )
    args = parser.parse_args()

    soc = BaseSoC(
        device=args.device,
        toolchain=args.toolchain,
        sys_clk_freq=args.sys_clk_freq,
        hc05_baudrate=args.hc05_baudrate,
        **parser.soc_argdict,
    )
    builder = Builder(soc, **parser.builder_argdict)

    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".bit"))


if __name__ == "__main__":
    main()
