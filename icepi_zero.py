#!/usr/bin/env python3
from migen import *
from litex.gen import *

from litex.build.generic_platform import *
from litex_boards.platforms import icepi_zero
from litex.soc.cores.clock import ECP5PLL
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.interconnect.csr import *

from litedram import modules as litedram_modules
from litedram.phy import GENSDRPHY

from litex.soc.cores.i2c import I2CMaster as HWI2CMaster
from litex.soc.cores.gpio import GPIOIn
from litex.soc.integration.soc import SoCRegion

from gateware.lcd_engine import LCDEngine

import os, sys
os.environ["PYTHON"] = sys.executable

# LCD / Touch IO -----------------------------------------------------------------------------------
_lcd_io = [
    ("lcd_spi", 0,
        Subsignal("clk",  Pins("T2")),
        Subsignal("cs_n", Pins("H3")),
        Subsignal("mosi", Pins("H2")),
        Subsignal("miso", Pins("J2")),
        IOStandard("LVCMOS33"),
        Misc("SLEWRATE=FAST"),
    ),
    ("lcd_ctrl", 0,
        Subsignal("dc",        Pins("G1")),
        Subsignal("reset_n",   Pins("J3")),
        Subsignal("backlight", Pins("P1")),
        IOStandard("LVCMOS33"),
    ),
    ("ctp_i2c", 0,
        Subsignal("scl", Pins("N1"), Misc("PULLMODE=UP")),
        Subsignal("sda", Pins("N4"), Misc("PULLMODE=UP")),
        IOStandard("LVCMOS33"),
    ),
    ("ctp_int", 0, Pins("E3"), IOStandard("LVCMOS33"), Misc("PULLMODE=UP")),
]

# CRG ----------------------------------------------------------------------------------------------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, spi_clk_freq=None, ext_reset_n=None):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        # Create a dedicated domain for the SDRAM output clock
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

        # System clock (SoC logic)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        # SDRAM clock (Phase shifted)
        # We shift it by 90 degrees so data is sampled in the middle of the valid window.
        pll.create_clkout(self.cd_sdram, sys_clk_freq, phase=90)

        # SPI clock (LCD engine), independent of sys clock.
        if spi_clk_freq is not None:
            pll.create_clkout(self.cd_spi, spi_clk_freq)


# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=50e6, with_sdram=False, with_spi_flash=False,
                 with_lcd=False, lcd_spi_clk_freq=100e6, flash_boot_offset=None, **kwargs):
        platform = icepi_zero.Platform()
        platform.add_extension(_lcd_io)

        # Handle CRG
        ext_reset_n = platform.request("ext_reset")
        self.crg = _CRG(platform, sys_clk_freq,
                        spi_clk_freq=lcd_spi_clk_freq if with_lcd else None,
                        ext_reset_n=ext_reset_n)

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
            ident="LiteX SoC on IcePi Zero Rev3",
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

        # ST7796S LCD ------------------------------------------------------------------------------
        if with_lcd:
            self.lcd = LCDEngine(
                pads      = platform.request("lcd_spi"),
                ctrl_pads = platform.request("lcd_ctrl"),
                platform  = platform,
                sclk_div  = 1,
            )
            self.bus.add_master(name="lcd_dma", master=self.lcd.bus)
            self.irq.add("lcd", use_loc_if_exists=True)
            self.add_constant("LCD_WIDTH",  320)
            self.add_constant("LCD_HEIGHT", 480)
            self.add_constant("LCD_SPI_FREQUENCY", int(lcd_spi_clk_freq // 2))

            # FT6336U capacitive touch (I2C + INT). RST is tied to LCD reset.
            # Hardware I2C master (wishbone-mapped, 2 word registers: xfer + config).
            # Interrupts: ev.idle fires on transfer completion.
            self.ctp_i2c = HWI2CMaster(platform.request("ctp_i2c"))
            self.bus.add_slave("ctp_i2c", self.ctp_i2c.bus,
                               SoCRegion(origin=None, size=0x10, cached=False))
            self.irq.add("ctp_i2c", use_loc_if_exists=True)

            # INT line: GPIOIn with IRQ. Firmware configures edge=falling at
            # runtime (FT6336U pulses INT low on touch events).
            self.ctp_int = GPIOIn(platform.request("ctp_int"), with_irq=True)
            self.irq.add("ctp_int", use_loc_if_exists=True)

# Build --------------------------------------------------------------------------------------------
def main():
    import os
    import binascii
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=icepi_zero.Platform, description="IcePi Zero minimal LiteX SoC.")
    parser.set_defaults(uart_baudrate=1_000_000)
    parser.add_target_argument("--flash", action="store_true", help="Flash Bitstream.")
    parser.add_target_argument("--sys-clk-freq", default=50e6, type=float)
    parser.add_target_argument("--with-sdram", action="store_true", help="Use external SDRAM as main RAM.")
    parser.add_target_argument("--with-spi-flash", action="store_true", help="Enable SPI Flash (MMAPed).")
    parser.add_target_argument("--with-lcd", action="store_true", help="Enable ST7796S LCD SPI test interface.")
    parser.add_target_argument("--lcd-spi-clk-freq", default=100e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. Default 100 MHz -> 50 MHz SCK.")
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
        sys_clk_freq      = args.sys_clk_freq,
        with_sdram        = args.with_sdram,
        with_spi_flash    = with_spi_flash,
        with_lcd          = args.with_lcd,
        lcd_spi_clk_freq  = args.lcd_spi_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
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
