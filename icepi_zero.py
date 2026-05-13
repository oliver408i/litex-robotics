#!/usr/bin/env python3
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
from litex.soc.integration.builder import Builder

from litedram import modules as litedram_modules
from litedram.phy import GENSDRPHY

from hw.snn_estimator import SNNTrackingEstimator

# CRG ----------------------------------------------------------------------------------------------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, ext_reset_n=None):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        # Create a dedicated domain for the SDRAM output clock
        self.cd_sdram = ClockDomain(reset_less=True)

        clk50 = platform.request("clk50")
        rst   = platform.request("rst")

        self.pll = pll = ECP5PLL()
        if ext_reset_n is None:
            self.comb += pll.reset.eq(~rst | self.rst)
        else:
            self.comb += pll.reset.eq(~rst | self.rst | ~ext_reset_n)
        pll.register_clkin(clk50, 50e6)
        
        # System clock (SoC logic)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        
        # SDRAM clock (Phase shifted)
        # We shift it by 90 degrees so data is sampled in the middle of the valid window.
        pll.create_clkout(self.cd_sdram, sys_clk_freq, phase=90)


# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=50e6, with_sdram=False, with_spi_flash=False,
                 flash_boot_offset=None, with_snn_poc=False, **kwargs):
        platform = icepi_zero.Platform()

        # Handle CRG
        ext_reset_n = platform.request("ext_reset")
        self.crg = _CRG(platform, sys_clk_freq, ext_reset_n=ext_reset_n)

        # Default memory sizes
        kwargs.setdefault("integrated_rom_size", 0x8000)
        if with_sdram:
            kwargs.setdefault("integrated_main_ram_size", 0)
        else:
            kwargs.setdefault("integrated_main_ram_size", 0x4000)

        kwargs.setdefault("uart_name", "serial")
        kwargs.setdefault("uart_baudrate", 1_000_000)
        
        SoCCore.__init__(
            self, platform, sys_clk_freq,
            ident="LiteX SoC on IcePi Zero Rev2",
            **kwargs
        )

        # SDR SDRAM --------------------------------------------------------------------------------
        if with_sdram and not self.integrated_main_ram_size:
            sdram_pads = platform.request("sdram")
            
            # GENSDRPHY handles the data and control signals.
            self.sdrphy = GENSDRPHY(sdram_pads, sys_clk_freq)
            
            # Manually drive the SDRAM clock pin from our phase-shifted domain.
            self.comb += platform.request("sdram_clock").eq(self.crg.cd_sdram.clk)

            self.add_sdram("sdram",
                phy           = self.sdrphy,
                module        = litedram_modules.W9825G6KH6(sys_clk_freq, "1:1"),
                l2_cache_size = 8192
            )

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q128JV
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            self.add_spi_flash(mode="4x", module=W25Q128JV(Codes.READ_1_1_4))
            if flash_boot_offset is not None and "spiflash" in self.bus.regions:
                flash_origin = self.bus.regions["spiflash"].origin
                self.add_constant("FLASH_BOOT_ADDRESS", flash_origin + flash_boot_offset)

        # SNN PoC ---------------------------------------------------------------------------------
        if with_snn_poc:
            self.snn = SNNTrackingEstimator(platform)
            self.add_csr("snn")

            # Expose a couple of quick sanity signals on the user LEDs when available.
            try:
                self.comb += [
                    platform.request("user_led", 0).eq(self.snn.status.fields.busy),
                    platform.request("user_led", 1).eq(self.snn.status.fields.done),
                ]
            except Exception:
                pass

# Build --------------------------------------------------------------------------------------------
def main():
    import binascii
    from litex.build.parser import LiteXArgumentParser

    # Keep LiteX's nested software builds on the same interpreter and local checkout.
    os.environ["PYTHON"] = sys.executable
    extra_pythonpaths = [
        os.path.join(LITEX_SETUP_ROOT, "litex"),
        os.path.join(LITEX_SETUP_ROOT, "litex-boards"),
        os.path.join(LITEX_SETUP_ROOT, "migen"),
        os.path.join(LITEX_SETUP_ROOT, "litedram"),
        os.path.join(LITEX_SETUP_ROOT, "litespi"),
    ]
    existing_pythonpath = os.environ.get("PYTHONPATH", "")
    combined_pythonpath = os.pathsep.join(
        [path for path in extra_pythonpaths + [existing_pythonpath] if path]
    )
    os.environ["PYTHONPATH"] = combined_pythonpath

    parser = LiteXArgumentParser(platform=icepi_zero.Platform, description="IcePi Zero minimal LiteX SoC.")
    parser.set_defaults(uart_baudrate=1_000_000)
    parser.add_target_argument("--flash", action="store_true", help="Flash Bitstream.")
    parser.add_target_argument("--sys-clk-freq", default=50e6, type=float)
    parser.add_target_argument("--with-sdram", action="store_true", help="Use external SDRAM as main RAM.")
    parser.add_target_argument("--with-spi-flash", action="store_true", help="Enable SPI Flash (MMAPed).")
    parser.add_target_argument("--with-snn-poc", action="store_true", help="Enable the SNN tracking estimator peripheral.")
    parser.add_target_argument("--flash-boot-offset", default=None,
                               type=lambda x: int(x, 0),
                               help="Enable BIOS autoboot from SPI flash at this offset.")
    parser.add_target_argument("--flash-firmware", default=None,
                               help="Flash firmware to SPI Flash (path to .bin).")
    parser.add_target_argument("--firmware-offset", default="0x200000",
                               type=lambda x: int(x, 0),
                               help="SPI Flash offset for firmware (default: 0x200000).")
    args = parser.parse_args()

    with_spi_flash = args.with_spi_flash or args.flash or (args.flash_firmware is not None) or (args.flash_boot_offset is not None)
    soc = BaseSoC(
        sys_clk_freq   = args.sys_clk_freq,
        with_sdram     = args.with_sdram,
        with_spi_flash = with_spi_flash,
        flash_boot_offset = args.flash_boot_offset,
        with_snn_poc   = args.with_snn_poc,
        **parser.soc_argdict,
    )
    builder = Builder(soc, **parser.builder_argdict)

    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".bit"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"), external=True)

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


if __name__ == "__main__":
    main()
