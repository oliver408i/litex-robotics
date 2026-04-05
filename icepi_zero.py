#!/usr/bin/env python3

from migen import ClockDomain, Signal

from litex.gen import LiteXModule

from litex_boards.platforms import icepi_zero
from litex.soc.cores.clock import ECP5PLL
from litex.soc.integration.builder import Builder
from litex.soc.integration.soc_core import SoCCore


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
    def __init__(self, device="LFE5U-25F", toolchain="trellis", sys_clk_freq=50e6, **kwargs):
        platform = icepi_zero.Platform(device=device, toolchain=toolchain)

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


def main():
    from litex.build.parser import LiteXArgumentParser

    parser = LiteXArgumentParser(
        platform=icepi_zero.Platform,
        description="Minimal IcePi Zero LiteX SoC.",
    )
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
    args = parser.parse_args()

    soc = BaseSoC(
        device=args.device,
        toolchain=args.toolchain,
        sys_clk_freq=args.sys_clk_freq,
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
