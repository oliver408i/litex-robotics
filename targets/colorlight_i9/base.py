#!/usr/bin/env python3
"""Shared SoC base for the Colorlight i9 (v7.2, ECP5 LFE5U-45F).

The second board this repo targets. Mirrors targets/icepi_zero/base.py so the
gateware in gateware/ can be hung off either one, but the two boards differ
enough that sharing a single base would be more confusing than helpful:

                        IcePi Zero              Colorlight i9 v7.2
    FPGA                LFE5U-25F               LFE5U-45F  (~2x the fabric)
    input clock         50 MHz                  25 MHz
    SDRAM               W9825G6KH6, 16-bit      M12L64322A, 32-bit
    SPI flash           W25Q128 (16 MB), quad   W25Q64 (8 MB), 1x
    console             on-board FTDI           MCU probe (probe/icelink-fast)

Run the tops, not this file.
"""
import os
import sys

# targets/colorlight_i9/base.py -> three levels up is the repo root. Running a
# top as a script only puts its own directory on sys.path, so the repo root has
# to be added explicitly or `gateware` stops importing.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LITEX_SETUP_ROOT = os.path.join(REPO_ROOT, "litex-setup")

if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

for rel_path in [
    "litex",
    "litex-boards",
    "migen",
    "litedram",
    "litespi",
    "litesdcard",
]:
    candidate = os.path.join(LITEX_SETUP_ROOT, rel_path)
    if os.path.isdir(candidate) and candidate not in sys.path:
        sys.path.insert(0, candidate)

from migen import *
from litex.gen import *

from litex_boards.platforms import colorlight_i5
from litex.build.io import DDROutput
from litex.soc.cores.clock import ECP5PLL
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder

from litedram.modules import M12L64322A
from litedram.phy import GENSDRPHY, HalfRateGENSDRPHY


# CRG ----------------------------------------------------------------------------------------------
class _CRG(LiteXModule):
    """Clock/reset generator.

    `sdram_rate` picks the SDRAM PHY clocking, same meaning as on the IcePi Zero:
      "1:1" -- full rate: PHY in cd_sys, SDRAM clock == sys.
      "1:2" -- half rate: PHY in cd_sys2x, SDRAM clock == 2*sys, so the memory
               sees twice the bandwidth per sys cycle at the cost of coupling
               the memory clock to 2*sys.

    Note the phase on the SDRAM clock output is 180 degrees, not the 90 you
    might expect -- that is what litex-boards' own colorlight target uses, with
    the comment that 90 needs to be increased in practice. Kept identical
    because it is the configuration that has been proven on these boards.
    """
    def __init__(self, platform, sys_clk_freq, sdram_rate="1:1", spi_clk_freq=None):
        assert sdram_rate in ("1:1", "1:2")
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        if sdram_rate == "1:2":
            self.cd_sys2x    = ClockDomain()
            self.cd_sys2x_ps = ClockDomain()
        else:
            self.cd_sys_ps = ClockDomain()
        if spi_clk_freq is not None:
            self.cd_spi = ClockDomain()

        clk25  = platform.request("clk25")
        rst_n  = platform.request("cpu_reset_n")

        self.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~rst_n | self.rst)
        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        if sdram_rate == "1:2":
            pll.create_clkout(self.cd_sys2x,    2 * sys_clk_freq)
            pll.create_clkout(self.cd_sys2x_ps, 2 * sys_clk_freq, phase=180)
        else:
            pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=180)

        # A second PLL for anything that needs its own fast clock (the LCD
        # engine's SPI domain, if that gets ported over). The 45F has 4 PLLs, so
        # unlike the 25F this does not immediately collide with USB.
        if spi_clk_freq is not None:
            self.pll2 = pll2 = ECP5PLL()
            self.comb += pll2.reset.eq(~rst_n | self.rst)
            pll2.register_clkin(clk25, 25e6)
            pll2.create_clkout(self.cd_spi, spi_clk_freq)

        # Drive the SDRAM clock pin through a DDR output register rather than a
        # comb assignment: a plain assignment routes the clock through general
        # fabric with placement-dependent delay that STA does not check as a
        # clock path, so timing reports clean while the real clock-to-DQ phase
        # drifts build to build.
        sdram_clk = ClockSignal("sys2x_ps" if sdram_rate == "1:2" else "sys_ps")
        self.specials += DDROutput(1, 0, platform.request("sdram_clock"), sdram_clk)


# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    """CPU + SDRAM + SPI flash on the i9.

    Deliberately plain: this is the substrate the board's bring-up runs on, and
    every feature added to it should be added knowingly.
    """
    def __init__(self, sys_clk_freq=48e6,
                 sdram_rate="1:1",
                 with_sdram=True,
                 with_spi_flash=True,
                 spi_clk_freq=None,
                 **kwargs):
        platform = colorlight_i5.Platform(board="i9", revision="7.2")

        self.crg = _CRG(platform, sys_clk_freq,
                        sdram_rate   = sdram_rate,
                        spi_clk_freq = spi_clk_freq)

        kwargs.setdefault("integrated_rom_size", 0x20000)   # 128 KB BIOS ROM
        kwargs.setdefault("uart_name", "serial")
        if with_sdram:
            kwargs.setdefault("integrated_main_ram_size", 0)
        else:
            kwargs.setdefault("integrated_main_ram_size", 0x8000)

        SoCCore.__init__(self, platform, sys_clk_freq,
                         ident="LiteX SoC on Colorlight i9",
                         **kwargs)

        # SDR SDRAM ----------------------------------------------------------------------------
        if with_sdram and not self.integrated_main_ram_size:
            sdrphy_cls  = HalfRateGENSDRPHY if sdram_rate == "1:2" else GENSDRPHY
            self.sdrphy = sdrphy_cls(platform.request("sdram"))
            self.add_sdram("sdram",
                phy           = self.sdrphy,
                module        = M12L64322A(sys_clk_freq, sdram_rate),
                l2_cache_size = kwargs.get("l2_size", 8192),
            )

        # SPI Flash ----------------------------------------------------------------------------
        # 1x only: the i9 does not bring out the quad pins the way the IcePi
        # Zero does, and litex-boards' own target uses 1x here too.
        if with_spi_flash:
            from litespi.modules import W25Q64
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            self.add_spi_flash(mode="1x", module=W25Q64(Codes.READ_1_1_1))


# Build helpers ------------------------------------------------------------------------------------
def make_parser(description):
    """The argparse parser shared by the i9 tops."""
    # Keep nested software builds on this interpreter and this checkout.
    os.environ["PYTHON"] = sys.executable
    extra = [os.path.join(LITEX_SETUP_ROOT, p)
             for p in ("litex", "litex-boards", "migen", "litedram", "litespi", "litesdcard")]
    existing = os.environ.get("PYTHONPATH", "")
    os.environ["PYTHONPATH"] = os.pathsep.join(p for p in extra + [existing] if p)

    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=colorlight_i5.Platform, description=description)
    parser.set_defaults(uart_baudrate=115200)
    # 60 MHz is litex-boards' default and it does NOT close timing on this
    # design (nextpnr reported 45.3 MHz, and builds only because the flow runs
    # with --timing-allow-fail). 48 MHz closes with margin; raise it
    # deliberately and read the reported Fmax, do not assume a bitstream means
    # timing passed.
    parser.add_target_argument("--sys-clk-freq", default=48e6, type=float,
                               help="System clock (Hz). Default 48e6 -- closes timing; "
                                    "litex-boards' 60e6 does not.")
    parser.add_target_argument("--sdram-half-rate", action="store_true",
                               help="Use the half-rate SDRAM PHY (SDRAM clock = 2*sys) "
                                    "instead of full rate (SDRAM clock = sys). Doubles "
                                    "memory bandwidth per sys cycle -- but the CPU path "
                                    "is the real limit for CPU-driven traffic, so it "
                                    "mostly pays off for DMA engines. Validate with the "
                                    "BIOS memtest.")
    # --load is bound to the probe rather than a JTAG dongle: this board has no
    # on-board FTDI, and openFPGALoader's CH347 backend does not work against
    # this chip (probe/icelink-fast/README.md).
    parser.add_target_argument("--probe-port", default=None,
                               help="icelink-fast probe console for --load "
                                    "(default /dev/icelink-console).")
    parser.add_target_argument("--flash", action="store_true",
                               help="NOT IMPLEMENTED on this board yet -- the probe does "
                                    "SRAM configuration only. See run_build().")
    return parser


def run_build(soc, args, parser):
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if getattr(args, "flash", False):
        # Say so loudly rather than silently doing nothing: a no-op --flash
        # looks like a successful flash right up until the board cold-boots to
        # whatever was there before.
        raise SystemExit(
            "--flash is not implemented for the i9.\n"
            "  The probe configures SRAM over JTAG but cannot yet write the board's\n"
            "  SPI flash; that needs ECP5 JTAG-to-SPI background mode in\n"
            "  probe/icelink-fast/src/ecp5.c. Use --load for a volatile load."
        )

    if args.load:
        # tools/ is importable because base.py put the repo root on sys.path.
        from tools.icelink import program_sram, IcelinkError, DEFAULT_PORT
        bit = builder.get_bitstream_filename(mode="sram")
        port = args.probe_port or DEFAULT_PORT
        print(f"Loading {bit} via the icelink-fast probe on {port}...")
        try:
            program_sram(bit, port=port)
        except IcelinkError as e:
            raise SystemExit(f"load failed: {e}")
        print("Loaded. The SoC is running; console is the probe's second CDC "
              "(/dev/icelink-uart).")

    return builder
