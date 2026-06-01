#!/usr/bin/env python3
"""Shared SoC base for IcePi Zero projects.

Per-project top files (icepi_zero_snn.py, icepi_zero_lcd.py) import from here
and add only the gateware their project needs. Run those tops directly; this
file is library-only.
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
LITEX_SETUP_ROOT = os.path.join(REPO_ROOT, "litex-setup")

for rel_path in [
    "litex",
    "litex-boards",
    "migen",
    "litedram",
    "litespi",
]:
    candidate = os.path.join(LITEX_SETUP_ROOT, rel_path)
    if os.path.isdir(candidate) and candidate not in sys.path:
        sys.path.insert(0, candidate)

from migen import *
from litex.gen import *

from litex_boards.platforms import icepi_zero
from litex.soc.cores.clock import ECP5PLL
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import Builder

from litedram import modules as litedram_modules
from litedram.phy import GENSDRPHY


# CRG ----------------------------------------------------------------------------------------------
class _CRG(LiteXModule):
    """Clock/reset generator. `spi_clk_freq` adds an extra cd_spi domain (LCD project uses it)."""
    def __init__(self, platform, sys_clk_freq, spi_clk_freq=None, ext_reset_n=None):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        self.cd_sdram = ClockDomain(reset_less=True)
        if spi_clk_freq is not None:
            self.cd_spi = ClockDomain()

        clk50 = platform.request("clk50")
        rst   = platform.request("rst")

        self.pll = pll = ECP5PLL()
        if ext_reset_n is None:
            self.comb += pll.reset.eq(~rst | self.rst)
        else:
            self.comb += pll.reset.eq(~rst | self.rst | ~ext_reset_n)
        pll.register_clkin(clk50, 50e6)

        pll.create_clkout(self.cd_sys, sys_clk_freq)
        # SDRAM clock: phase shifted 90deg so data is sampled in the middle of the valid window.
        pll.create_clkout(self.cd_sdram, sys_clk_freq, phase=90)

        if spi_clk_freq is not None:
            pll.create_clkout(self.cd_spi, spi_clk_freq)


# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    """SoC shell with SDRAM always enabled and optional SPI flash boot/firmware support.

    SDRAM is mandatory for every project on this board, so it is wired in unconditionally.
    SPI flash is enabled automatically when any flash-related CLI arg is passed (see build_and_run).

    With flash the BIOS is XIP'd from it (no EBR ROM); without flash the integrated
    EBR ROM is kept (self-contained --load / recovery build). See docs/xip_bios.md.
    """
    # Flash base; must stay below VexRiscv's uncached IO region (0x80000000+) so
    # the XIP fetch path stays cacheable. See docs/xip_bios.md.
    mem_map = {**SoCCore.mem_map, "spiflash": 0x20000000}

    def __init__(self, sys_clk_freq=50e6,
                 with_spi_flash=False, flash_boot_offset=None,
                 bios_flash_offset=0x100000, spiflash_1x=False,
                 spi_clk_freq=None,
                 platform=None,
                 force_lcd_backlight_off=True,
                 **kwargs):
        if platform is None:
            platform = icepi_zero.Platform()

        # The on-board LCD backlight is on P1. Without anyone driving it, the
        # pin floats high and the panel goes full white — annoying on every
        # non-LCD project. Drive it low by default; the LCD project opts out
        # (`force_lcd_backlight_off=False`) because it requests P1 itself as
        # part of lcd_ctrl.
        if force_lcd_backlight_off:
            from litex.build.generic_platform import IOStandard, Pins
            platform.add_extension([
                ("lcd_backlight", 0, Pins("P1"), IOStandard("LVCMOS33")),
            ])

        ext_reset_n = platform.request("ext_reset")
        self.crg = _CRG(platform, sys_clk_freq,
                        spi_clk_freq=spi_clk_freq,
                        ext_reset_n=ext_reset_n)

        if with_spi_flash:
            # XIP: no EBR ROM, reset vector into flash. Force (not setdefault) --
            # the parser always supplies these via soc_argdict. See docs/xip_bios.md.
            kwargs["integrated_rom_size"] = 0
            kwargs["cpu_reset_address"]   = self.mem_map["spiflash"] + bios_flash_offset
        else:
            kwargs.setdefault("integrated_rom_size", 0x8000)  # integrated EBR ROM
        kwargs.setdefault("integrated_main_ram_size", 0)  # main RAM is SDRAM
        kwargs.setdefault("uart_name", "serial")
        kwargs.setdefault("uart_baudrate", 1_000_000)

        SoCCore.__init__(
            self, platform, sys_clk_freq,
            ident="LiteX SoC on IcePi Zero",
            **kwargs
        )

        # Tie backlight low if this project hasn't claimed P1 for itself.
        if force_lcd_backlight_off:
            self.comb += platform.request("lcd_backlight").eq(0)

        # SDR SDRAM --------------------------------------------------------------------------------
        sdram_pads = platform.request("sdram")
        self.sdrphy = GENSDRPHY(sdram_pads, sys_clk_freq)
        # Drive the SDRAM clock pin from the phase-shifted domain.
        self.comb += platform.request("sdram_clock").eq(self.crg.cd_sdram.clk)
        self.add_sdram("sdram",
            phy           = self.sdrphy,
            module        = litedram_modules.W9825G6KH6(sys_clk_freq, "1:1"),
            l2_cache_size = 8192,
        )

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q128JV
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            # 4x default (QE bit assumed set); --spiflash-1x is the no-QE recovery
            # mode. with_master=False is required for XIP -- the BIOS must not drive
            # raw SPI commands on the flash it executes from. See docs/xip_bios.md.
            if spiflash_1x:
                self.add_spi_flash(mode="1x", module=W25Q128JV(Codes.READ_1_1_1),
                                   with_master=False)
            else:
                self.add_spi_flash(mode="4x", module=W25Q128JV(Codes.READ_1_1_4),
                                   with_master=False)

            # Skip BIOS SCLK auto-calibration -- it crashes XIP. See docs/xip_bios.md.
            self.add_constant("SPIFLASH_SKIP_FREQ_INIT")

            # BIOS XIP linker region (linker=True: no bus slave, flash MMAP covers it).
            self.bus.add_region("rom", SoCRegion(
                origin = self.mem_map["spiflash"] + bios_flash_offset,
                size   = 0x10000,  # >= BIOS size; headroom for growth
                linker = True,
            ))

            if flash_boot_offset is not None and "spiflash" in self.bus.regions:
                flash_origin = self.bus.regions["spiflash"].origin
                self.add_constant("FLASH_BOOT_ADDRESS", flash_origin + flash_boot_offset)


# Build helpers ------------------------------------------------------------------------------------
def make_parser(description):
    """Build the argparse parser shared by every project top."""
    # Keep LiteX's nested software builds on the same interpreter and local checkout.
    os.environ["PYTHON"] = sys.executable
    extra_pythonpaths = [
        os.path.join(LITEX_SETUP_ROOT, p)
        for p in ("litex", "litex-boards", "migen", "litedram", "litespi")
    ]
    existing_pythonpath = os.environ.get("PYTHONPATH", "")
    os.environ["PYTHONPATH"] = os.pathsep.join(
        p for p in extra_pythonpaths + [existing_pythonpath] if p
    )

    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=icepi_zero.Platform, description=description)
    parser.set_defaults(uart_baudrate=1_000_000)
    parser.add_target_argument("--flash", action="store_true", help="Flash bitstream to SPI flash.")
    parser.add_target_argument("--sys-clk-freq", default=50e6, type=float)
    parser.add_target_argument("--flash-boot-offset", default=None,
                               type=lambda x: int(x, 0),
                               help="Enable BIOS autoboot from SPI flash at this offset.")
    parser.add_target_argument("--flash-firmware", default=None,
                               help="Flash firmware to SPI Flash (path to .bin).")
    parser.add_target_argument("--firmware-offset", default="0x200000",
                               type=lambda x: int(x, 0),
                               help="SPI Flash offset for firmware (default: 0x200000).")
    parser.add_target_argument("--bios-flash-offset", default="0x100000",
                               type=lambda x: int(x, 0),
                               help="SPI Flash offset the BIOS is XIP'd from / reset vector "
                                    "(default: 0x100000). Must clear the bitstream below and "
                                    "the firmware above it.")
    parser.add_target_argument("--flash-bios", action="store_true",
                               help="Flash the built bios.bin to --bios-flash-offset. Combine "
                                    "with --build/--load/--flash or run on its own.")
    parser.add_target_argument("--spiflash-1x", action="store_true",
                               help="Use single-lane (1x) flash reads instead of quad (4x). "
                                    "No QE-bit dependency -> cold-boot recovery mode.")
    return parser


def resolve_spi_flash(args):
    """SPI flash is enabled whenever any flash-related CLI arg is set.

    With flash enabled the SoC switches to XIP-BIOS-from-flash (integrated ROM
    disabled), so any of these flags also selects the BRAM-optimized build.
    """
    return bool(args.flash or args.flash_firmware is not None
                or args.flash_boot_offset is not None or args.flash_bios)


def run_build(soc, args, parser):
    """Standard build/load/flash sequence used by every project top."""
    import binascii
    builder = Builder(soc, **parser.builder_argdict)

    if args.build:
        builder.build(**parser.toolchain_argdict)

    # Flash steps run BEFORE --load on purpose. --load configures the FPGA and
    # immediately releases the CPU, which on an XIP build does its first BIOS
    # fetch from flash right away. If the BIOS/firmware were written after the
    # load, the CPU would boot stale flash contents and hang (no UART). Flashing
    # first, then loading, guarantees the CPU's first fetch sees the just-written
    # image. (Plain EBR-ROM builds don't care -- the BIOS rides in the bitstream.)
    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"), external=True)

    if args.flash_bios:
        bios_bin = os.path.join(builder.software_dir, "bios", "bios.bin")
        if not os.path.exists(bios_bin):
            raise FileNotFoundError(
                f"BIOS not found: {bios_bin} (run --build first).")
        prog = soc.platform.create_programmer()
        prog.flash(args.bios_flash_offset, bios_bin, external=True)

    if args.flash_firmware is not None:
        firmware_path = os.path.abspath(args.flash_firmware)
        if not os.path.exists(firmware_path):
            raise FileNotFoundError(f"Firmware not found: {firmware_path}")
        fbi_path = firmware_path
        if not firmware_path.lower().endswith(".fbi"):
            with open(firmware_path, "rb") as f:
                data = f.read()
            length = len(data).to_bytes(4, byteorder="little")
            crc = binascii.crc32(data).to_bytes(4, byteorder="little")
            fbi_name = os.path.splitext(os.path.basename(firmware_path))[0] + ".fbi"
            fbi_path = os.path.join(builder.output_dir, fbi_name)
            with open(fbi_path, "wb") as f:
                f.write(length)
                f.write(crc)
                f.write(data)
        prog = soc.platform.create_programmer()
        prog.flash(args.firmware_offset, fbi_path, external=True)

    # Load last: the FPGA ends up configured with all flash content already in
    # place, so the CPU's first XIP fetch boots the correct BIOS. See the
    # ordering note above.
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".bit"))
