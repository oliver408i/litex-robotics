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
    "liteeth",
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
    def __init__(self, sys_clk_freq=87.5e6,
                 sdram_rate="1:2",
                 with_sdram=True,
                 with_spi_flash=True,
                 spi_clk_freq=None,
                 force_lcd_idle=True,
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

        # LCD module parked ------------------------------------------------------------------------
        # With the ST7796S module wired to P4 (see targets/colorlight_i9/lcd.py),
        # a build that does NOT instantiate the engine leaves those pins
        # unrequested -- and an unrequested ECP5 pin comes up as an input with
        # its weak pull-up on. Pulled high, the backlight line lights the panel
        # with no image on it. targets/icepi_zero/base.py has the same guard for
        # the same reason (force_lcd_backlight_off).
        #
        # lcd.py passes force_lcd_idle=False: add_lcd_touch requests these pins
        # itself, and requesting a pin twice is an error.
        #
        # Imported here rather than at module scope so the non-LCD tops do not
        # pull in the whole gateware feature set to build a bring-up SoC.
        if force_lcd_idle:
            from gateware.soc_features import add_lcd_idle
            add_lcd_idle(self)


# VexiiRiscv 2-way superscalar preset: two decode/execute lanes + full branch
# prediction (BTB/RAS/gshare) + late-ALU. Appended AFTER the `standard` variant
# base (mul/div, 2-way L1 I$/D$, relaxed-branch), so this is standard + IPC.
# Trades Fmax and area for instructions-per-clock. On the 25F this collapsed
# the clock to 46 MHz and filled 79% of the chip (docs/cpu_vexiiriscv.md); the
# 45F has the room the IcePi never had, which is the whole reason to retry it
# here.
VEXII_SUPERSCALAR_ARGS = (
    "--decoders=2 --lanes=2 --with-btb --with-ras --with-gshare --with-late-alu"
)


def _argv_value(flag, default=None):
    """Exact-match scan of sys.argv for `--flag value` or `--flag=value`.

    Used to peek at --cpu *before* parse_args() so we can preset the LiteX CPU
    args it registers. We can't use the parser's get_value_from_key() here: it
    matches by substring, so "--cpu" would falsely match "--cpu-type" etc.
    """
    for i, tok in enumerate(sys.argv):
        if tok == flag:
            return sys.argv[i + 1] if i + 1 < len(sys.argv) else default
        if tok.startswith(flag + "="):
            return tok.split("=", 1)[1]
    return default


# Build helpers ------------------------------------------------------------------------------------
def make_parser(description, default_cpu="vexriscv"):
    """The argparse parser shared by the i9 tops.

    `default_cpu` lets a CPU-bound top opt into a bigger core without changing
    it for everything else. It has to be threaded through here rather than set
    with `parser.set_defaults(cpu=...)`, because the CPU choice is read from
    argv *before* parse_args() runs -- see the comment at `_argv_value` below.
    """
    # Keep nested software builds on this interpreter and this checkout.
    os.environ["PYTHON"] = sys.executable
    extra = [os.path.join(LITEX_SETUP_ROOT, p)
             for p in ("litex", "litex-boards", "migen", "litedram", "liteeth",
                       "litespi", "litesdcard")]
    existing = os.environ.get("PYTHONPATH", "")
    os.environ["PYTHONPATH"] = os.pathsep.join(p for p in extra + [existing] if p)

    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=colorlight_i5.Platform, description=description)
    parser.set_defaults(uart_baudrate=115200)
    # 87.5 MHz, half-rate SDRAM (175 MHz). Every part of this is measured.
    #
    # It must be a frequency the PLL hits EXACTLY, because LiteX derives the
    # UART divisor -- and this project's ADC sample rate -- from the REQUESTED
    # value, not the achieved one. In half-rate the PLL has to produce sys AND
    # 2*sys, which is a harder constraint than it looks: 90 MHz comes out as
    # 90.62/181.25 (0.7% high), while 87.5 -> 87.50/175.00 and 93.75 ->
    # 93.75/187.50 land exactly. The old 48e6 default was never 48 either --
    # it was 47.73 (VCO 525/11). That is why this is 87.5 and not a round 90.
    #
    # nextpnr reports FAIL here as it does at every frequency this board has
    # ever run at; it times the -6 worst-case corner and is ~70% pessimistic
    # against this bench. The hardware sweep in README.md is the evidence:
    # 50 / 60 / 70 / 80 / 87.5 / 93.75 MHz all pass the BIOS memtest and
    # 96.875 does not. 93.75 is exact and passes too, but sits only 3 MHz under
    # a MEASURED cliff; 87.5 keeps 9 MHz of margin for a rounding error's worth
    # of speed.
    parser.add_target_argument("--sys-clk-freq", default=87.5e6, type=float,
                               help="System clock (Hz). Default 87.5e6 -- exactly "
                                    "achievable with the half-rate PLL, and 9 MHz "
                                    "below the 96.875 MHz cliff measured on hardware. "
                                    "nextpnr will call this a timing failure; see README.")
    # HALF RATE IS THE DEFAULT, and this is not a performance choice -- the
    # full-rate PHY is BROKEN on this board above ~48 MHz. Measured 2026-09-20
    # at 90 MHz sys: full rate (SDRAM = sys) gives 1048576/1048576 data errors,
    # a total failure; half rate (SDRAM = 2*sys = 181 MHz) gives Memtest OK on
    # the same build and the same board, minutes apart. It is the known
    # full-rate phase problem noted in docs/ -- 90 degrees is what the PHY
    # wants and 180 is what the target sets, and it survives at 48 MHz only
    # because the window is wide there.
    #
    # Nothing that touches SDRAM works on a full-rate build at this clock: it
    # is what corrupted a bitstream staged in SDRAM for flashing and produced
    # a CRC mismatch that looked like an Etherbone pacing problem for a week.
    # Do not flip this back without re-running `mem_test`.
    sdopts = parser.target_group.add_mutually_exclusive_group()
    sdopts.add_argument("--sdram-half-rate", dest="sdram_half_rate",
                        action="store_true", default=True,
                        help="Half-rate SDRAM PHY, clock = 2*sys (DEFAULT, and the "
                             "only rate that passes memtest above ~48 MHz).")
    sdopts.add_argument("--sdram-full-rate", dest="sdram_half_rate",
                        action="store_false",
                        help="Full-rate SDRAM PHY, clock = sys. BROKEN above ~48 MHz "
                             "on this board -- 100%% data errors at 90 MHz. Here for "
                             "experiments on the phase fix, not for use.")
    # --load is bound to the probe rather than a JTAG dongle: this board has no
    # on-board FTDI, and openFPGALoader's CH347 backend does not work against
    # this chip (probe/icelink-fast/README.md).
    parser.add_target_argument("--probe-port", default=None,
                               help="icelink-fast probe console for --load "
                                    "(default /dev/icelink-console).")
    parser.add_target_argument("--flash", action="store_true",
                               help="NOT IMPLEMENTED on this board yet -- the probe does "
                                    "SRAM configuration only. See run_build().")
    parser.add_target_argument("--cpu", default=default_cpu,
                               choices=["vexriscv", "vexii", "vexii-superscalar"],
                               help="Soft CPU core. 'vexriscv' (default): production "
                                    "VexRiscv -- highest Fmax, smallest. "
                                    "'vexii': VexiiRiscv 'standard' (single-issue, 2-way L1 "
                                    "I$/D$); build-on-demand, needs sbt+JDK21 on PATH. "
                                    "'vexii-superscalar': VexiiRiscv 2-way superscalar + branch "
                                    "prediction + late-ALU -- trades Fmax/area for IPC. Raw "
                                    "--cpu-type/--cpu-variant/--vexii-args still work and override "
                                    "this preset. See docs/cpu_vexiiriscv.md.")
    # Preset the LiteX CPU args from --cpu *before* parse_args(), so the parser
    # registers/reads the VexiiRiscv-specific args (--vexii-args etc.) and selects
    # the core. Peek at argv directly (parse hasn't run yet). Raw --cpu-type /
    # --cpu-variant / --vexii-args passed on the CLI still win (argparse CLI value
    # overrides these set_defaults).
    cpu_choice = _argv_value("--cpu", default_cpu)
    if cpu_choice in ("vexii", "vexii-superscalar"):
        parser.set_defaults(cpu_type="vexiiriscv", cpu_variant="standard")
        if cpu_choice == "vexii-superscalar":
            parser.set_defaults(vexii_args=VEXII_SUPERSCALAR_ARGS)
    return parser



def add_eth_arguments(parser, default_etherbone=True):
    """The Ethernet flags shared by the i9 tops.

    Etherbone defaults ON (`--no-etherbone` turns it off) because it is the
    board's flashing path: `tools/i9_flash_bitstream.sh` drives a running
    Etherbone image, so an image flashed without one boots with no way back in
    except the probe. See gateware/soc_features.py: add_eth, and the README
    section on flashing over Ethernet.

    The tradeoff is that nextpnr will report a post-route timing failure on the
    eth_rx/eth_tx domains in every build that has them. That report has been
    wrong every time it was checked against hardware; `--no-etherbone` is there
    for when a trustworthy STA number matters more than the network path.
    """
    ethopts = parser.target_group.add_mutually_exclusive_group()
    ethopts.add_argument("--with-etherbone", dest="with_etherbone", action="store_true",
                         default=default_etherbone,
                         help="Etherbone: a Wishbone master over UDP, so the host "
                              "reaches every CSR without the probe."
                              + (" (default)" if default_etherbone else ""))
    ethopts.add_argument("--no-etherbone", dest="with_etherbone", action="store_false",
                         help="Build without Etherbone."
                              + (" (default)" if not default_etherbone else ""))
    # 192.168.0.50 is the bench subnet (the host sits on 192.168.0.10/24) and
    # the address tools/i9_flash_bitstream.sh and the README's litex_server
    # invocation both assume. The old eth.py default was 192.168.1.50, which is
    # simply unreachable from here -- it cost a debugging round to notice.
    parser.add_target_argument("--local-ip", default="192.168.0.50",
                               help="This board's IP, for Etherbone. Must be on the "
                                    "host's subnet to be reachable.")
    parser.add_target_argument("--etherbone-buffer-depth", default=256,
                               type=lambda x: int(x, 0),
                               help="Etherbone record FIFO depth in 32-bit words "
                                    "(max 256). This is a hard cap on burst size -- "
                                    "a longer burst deadlocks the core. LiteX "
                                    "defaults to 16; do not lower this without "
                                    "reading the note in gateware/soc_features.py.")
    parser.add_target_argument("--eth-phy", default=0, type=int,
                               help="Which PHY gets the RGMII data pins (0 or 1). "
                                    "MDIO reaches both either way.")
    return parser


def eth_kwargs(args):
    """add_eth() keyword arguments from a parser built with add_eth_arguments."""
    return dict(
        phy_index              = args.eth_phy,
        with_etherbone         = args.with_etherbone,
        local_ip               = args.local_ip,
        etherbone_buffer_depth = args.etherbone_buffer_depth,
    )


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
