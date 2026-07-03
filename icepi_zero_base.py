#!/usr/bin/env python3
"""Shared SoC base for IcePi Zero projects.

Per-project top files (icepi_zero_mnist.py, icepi_zero_lcd.py) import from here
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
    "litesdcard",
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
from litedram.phy import HalfRateGENSDRPHY


# CRG ----------------------------------------------------------------------------------------------
class _CRG(LiteXModule):
    """Clock/reset generator. `spi_clk_freq` adds an extra cd_spi domain (LCD project uses it)."""
    def __init__(self, platform, sys_clk_freq, spi_clk_freq=None, ext_reset_n=None):
        self.rst      = Signal()
        self.user_rst = Signal()  # extra reset source (e.g. BootCtl's FTDI RTS reset)
        self.cd_sys      = ClockDomain()
        self.cd_sys2x    = ClockDomain()             # 2x sys: SDRAM PHY (HalfRateGENSDRPHY) lives here
        self.cd_sys2x_ps = ClockDomain(reset_less=True)  # 2x sys, +90deg: drives the SDRAM clock pin
        if spi_clk_freq is not None:
            self.cd_spi = ClockDomain()

        clk50 = platform.request("clk50")
        rst   = platform.request("rst")

        rst_comb = ~rst | self.rst | self.user_rst
        if ext_reset_n is not None:
            rst_comb = rst_comb | ~ext_reset_n

        # PLL #1 -- core clocks. sys and sys2x MUST share one VCO so the half-rate
        # PHY's sys<->sys2x serdes sees an edge-aligned 2x clock. sys2x_ps is the
        # same 2x clock shifted 90deg to sample the SDRAM in the middle of its
        # data-valid window (this replaces the old cd_sdram).
        self.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(rst_comb)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys,      sys_clk_freq)
        pll.create_clkout(self.cd_sys2x,    2*sys_clk_freq)
        pll.create_clkout(self.cd_sys2x_ps, 2*sys_clk_freq, phase=90)

        # PLL #2 -- LCD SPI clock. It's async-crossed (AsyncFIFO in lcd_engine), so
        # it needs no phase relationship to sys, and 185MHz can't share a VCO with
        # 50/100MHz -- so it gets its own PLL. See docs on the half-rate SDRAM plan.
        if spi_clk_freq is not None:
            self.pll2 = pll2 = ECP5PLL()
            self.comb += pll2.reset.eq(rst_comb)
            pll2.register_clkin(clk50, 50e6)
            pll2.create_clkout(self.cd_spi, spi_clk_freq)


# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    """SoC shell with SDRAM (default on) and optional SPI flash boot/firmware support.

    SDRAM is wired in by default (every normal project needs it). `with_sdram=False`
    omits it and makes main_ram on-chip BRAM instead -- used by the C3 loader, which
    must run while the SDRAM is dead (the BIOS itself keeps its stack/data in main_ram).
    SPI flash is enabled automatically when any flash-related CLI arg is passed (see build_and_run).

    With flash the BIOS is XIP'd from it (no EBR ROM); without flash the integrated
    EBR ROM is kept (self-contained --load / recovery build). See docs/boot_chain.md.
    """
    # Flash base; must stay below VexRiscv's uncached IO region (0x80000000+) so
    # the XIP fetch path stays cacheable. See docs/boot_chain.md.
    mem_map = {**SoCCore.mem_map, "spiflash": 0x20000000}

    def __init__(self, sys_clk_freq=50e6,
                 with_spi_flash=False, flash_boot_offset=None,
                 bios_flash_offset=0x100000, spiflash_1x=False,
                 flash_master=False, app_flash_offset=0x280000,
                 spiflash_clk_freq=25e6,
                 spi_clk_freq=None,
                 platform=None,
                 force_lcd_backlight_off=True,
                 with_sdram=True,
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
            # the parser always supplies these via soc_argdict. See docs/boot_chain.md.
            kwargs["integrated_rom_size"] = 0
            kwargs["cpu_reset_address"]   = self.mem_map["spiflash"] + bios_flash_offset
        else:
            kwargs.setdefault("integrated_rom_size", 0x8000)  # integrated EBR ROM
        if with_sdram:
            kwargs.setdefault("integrated_main_ram_size", 0)  # main RAM is SDRAM
        else:
            # No SDRAM in this bitstream: main RAM is on-chip BRAM. The BIOS puts
            # its own .data/.bss/stack in main_ram, so a working main_ram is
            # mandatory -- a broken SDRAM here breaks the BIOS itself (it limps to
            # a console from XIP but garbles serialboot). Caller sizes it.
            kwargs.setdefault("integrated_main_ram_size", 0x8000)  # 32 KB
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
        # Optional: omitted for the C3 loader bitstream, which must run with the
        # SDRAM dead (the BIOS keeps its stack/data in main_ram, so SDRAM main_ram
        # would break the BIOS). cd_sys2x/_ps stay generated by the CRG but go
        # unused. See icepi_zero_c3loader.py.
        if with_sdram:
            sdram_pads = platform.request("sdram")
            # Half-rate PHY: the SDRAM runs in cd_sys2x (2x sys) while the controller
            # and the rest of the SoC stay in cd_sys. This doubles raw SDRAM bandwidth
            # without raising the CPU clock. The chip (W9825G6KH6 -6) is rated 166MHz,
            # so 2*sys=100MHz is comfortable.
            self.sdrphy = HalfRateGENSDRPHY(sdram_pads, sys_clk_freq)
            # Drive the SDRAM clock pin from the 90deg-shifted 2x domain.
            self.comb += platform.request("sdram_clock").eq(self.crg.cd_sys2x_ps.clk)
            self.add_sdram("sdram",
                phy           = self.sdrphy,
                module        = litedram_modules.W9825G6KH6(sys_clk_freq, "1:2"),
                l2_cache_size = 8192,
            )

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q128JV
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            # 4x default (QE bit assumed set); --spiflash-1x = no-QE recovery mode.
            # spiflash_clk_freq is the LIVE speed (BIOS auto-cal is skipped below);
            # default 25e6 = sys/2 ceiling, hardware-validated. flash_master only
            # for builds with an SDRAM-resident flasher. All XIP rules and the
            # rationale: docs/boot_chain.md.
            if spiflash_1x:
                self.add_spi_flash(mode="1x", module=W25Q128JV(Codes.READ_1_1_1),
                                   clk_freq=spiflash_clk_freq,
                                   with_master=flash_master)
            else:
                self.add_spi_flash(mode="4x", module=W25Q128JV(Codes.READ_1_1_4),
                                   clk_freq=spiflash_clk_freq,
                                   with_master=flash_master)

            # Skip BIOS SCLK auto-calibration -- it crashes XIP. See docs/boot_chain.md.
            self.add_constant("SPIFLASH_SKIP_FREQ_INIT")
            if flash_master:
                # Keep the XIP BIOS off the master (read-ID/quad-enable would crash
                # the fetch path); local litex patch, see docs/boot_chain.md and
                # patches/litex-spiflash-skip-master-init.patch.
                self.add_constant("SPIFLASH_SKIP_MASTER_INIT")

            # BIOS XIP linker region (linker=True: no bus slave, flash MMAP covers it).
            self.bus.add_region("rom", SoCRegion(
                origin = self.mem_map["spiflash"] + bios_flash_offset,
                size   = 0x10000,  # >= BIOS size; headroom for growth
                linker = True,
            ))

            if flash_boot_offset is not None and "spiflash" in self.bus.regions:
                flash_origin = self.bus.regions["spiflash"].origin
                self.add_constant("FLASH_BOOT_ADDRESS", flash_origin + flash_boot_offset)

            # App slot chain-booted by the winc_loader (docs/boot_chain.md).
            self.add_constant("FLASH_APP_OFFSET", app_flash_offset)


# Build helpers ------------------------------------------------------------------------------------
def make_parser(description):
    """Build the argparse parser shared by every project top."""
    # Keep LiteX's nested software builds on the same interpreter and local checkout.
    os.environ["PYTHON"] = sys.executable
    extra_pythonpaths = [
        os.path.join(LITEX_SETUP_ROOT, p)
        for p in ("litex", "litex-boards", "migen", "litedram", "litespi", "litesdcard")
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
    parser.add_target_argument("--spiflash-clk-freq", default=25e6, type=float,
                               help="Requested SPI flash SCLK (Hz); rounded down to "
                                    "sys/(2*(div+1)). Default 25e6 = sys/2, the 1:1 PHY "
                                    "ceiling (hardware-validated). Lower it if a board "
                                    "shows flash read corruption.")
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

    # Flash steps run BEFORE --load on purpose: --load releases the CPU, which
    # XIP-fetches the BIOS from flash immediately. See docs/boot_chain.md.
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
