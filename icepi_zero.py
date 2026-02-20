#!/usr/bin/env python3
from migen import *
from litex.gen import *
from functools import reduce
from operator import or_

from litex_boards.platforms import icepi_zero
from litex.soc.cores.clock import ECP5PLL
from litex.soc.cores.gpio import GPIOOut
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder

from litedram import modules as litedram_modules
from litedram.phy import GENSDRPHY

# CPU Activity LED --------------------------------------------------------------------------------
class CpuActivityLED(LiteXModule):
    def __init__(self, pad, activity, sys_clk_freq, hold_time=0.02, invert=False):
        hold_cycles = max(1, int(sys_clk_freq * hold_time))
        counter     = Signal(max=hold_cycles + 1)
        led         = Signal()

        # Stretch activity pulses to a visible LED blink.
        self.sync += [
            If(activity,
                counter.eq(hold_cycles)
            ).Elif(counter != 0,
                counter.eq(counter - 1)
            )
        ]
        self.comb += led.eq(counter != 0)
        self.comb += pad.eq(~led if invert else led)

# CRG ----------------------------------------------------------------------------------------------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        # Create a dedicated domain for the SDRAM output clock
        self.cd_sdram = ClockDomain(reset_less=True)

        clk50 = platform.request("clk50")
        rst   = platform.request("rst")

        self.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~rst | self.rst)
        pll.register_clkin(clk50, 50e6)
        
        # System clock (SoC logic)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        
        # SDRAM clock (Phase shifted)
        # We shift it by 90 degrees so data is sampled in the middle of the valid window.
        pll.create_clkout(self.cd_sdram, sys_clk_freq, phase=90)


# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=50e6, with_sdram=False, with_spi_flash=False, flash_boot_offset=None, **kwargs):
        platform = icepi_zero.Platform()

        # Handle CRG
        self.crg = _CRG(platform, sys_clk_freq)

        # Default memory sizes
        kwargs.setdefault("integrated_rom_size", 0x8000)
        if with_sdram:
            kwargs.setdefault("integrated_main_ram_size", 0)
        else:
            kwargs.setdefault("integrated_main_ram_size", 0x4000)

        kwargs.setdefault("uart_name", "serial")
        kwargs.setdefault("uart_baudrate", 115200)
        
        SoCCore.__init__(
            self, platform, sys_clk_freq,
            ident="LiteX SoC on IcePi Zero (SDRAM Fixed)",
            **kwargs
        )

        # CPU Activity LED -----------------------------------------------------------------------
        if getattr(self, "cpu", None) is not None:
            buses = []
            for bus_name in ("ibus", "dbus"):
                bus = getattr(self.cpu, bus_name, None)
                if bus is not None and hasattr(bus, "cyc") and hasattr(bus, "stb"):
                    buses.append(bus.cyc & bus.stb)
            if buses:
                activity = reduce(or_, buses)
                self.cpu_activity = CpuActivityLED(
                    pad          = platform.request("user_led", 0),
                    activity     = activity,
                    sys_clk_freq = sys_clk_freq,
                )

        # User LEDs (exclude LED0 used for CPU activity) -----------------------------------------
        user_leds = [platform.request("user_led", i) for i in range(1, 5)]
        self.leds = GPIOOut(pads=Cat(*user_leds))
        self.add_csr("leds")

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

# Build --------------------------------------------------------------------------------------------
def main():
    import os
    import binascii
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=icepi_zero.Platform, description="IcePi Zero minimal LiteX SoC.")
    parser.add_target_argument("--flash", action="store_true", help="Flash Bitstream.")
    parser.add_target_argument("--sys-clk-freq", default=50e6, type=float)
    parser.add_target_argument("--with-sdram", action="store_true", help="Use external SDRAM as main RAM.")
    parser.add_target_argument("--with-spi-flash", action="store_true", help="Enable SPI Flash (MMAPed).")
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
