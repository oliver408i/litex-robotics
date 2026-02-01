#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2022 Icenowy Zheng <icenowy@aosc.io>
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *
from litex.gen import *

from litex.build.generic_platform import IOStandard, Pins, Subsignal
from litex.build.io import DDROutput

from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.gpio import GPIOIn
from litex.soc.cores.led import LedChaser
from litex.soc.cores.video import VideoGowinHDMIPHY

from litedram.modules import M12L64322A  # FIXME: use the real model number
from litedram.phy import GENSDRPHY

from litex_boards.platforms import sipeed_tang_nano_20k
from gateware.ws2812_status_verilog import WS2812StatusVerilog
from gateware.mcp3008_verilog import MCP3008ReaderVerilog
from gateware.laser_control import LaserControl
from gateware.motor_pwm import MotorPWM
from gateware.servo_pwm import ServoPWM
from litex.soc.cores.bitbang import I2CMaster

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_hdmi=False, with_reset_button=True):
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()
        self.cd_por   = ClockDomain()
        if with_hdmi:
            self.cd_hdmi   = ClockDomain()
            self.cd_hdmi5x = ClockDomain()

        # Clk
        clk27 = platform.request("clk27")

        # Power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(clk27)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # Optional reset button (active-high on pin 87 -> btn[1]).
        if with_reset_button:
            reset_btn = platform.request("btn", 1)
            reset_in = reset_btn | self.rst | ~por_done
        else:
            reset_in = self.rst | ~por_done

        # PLL
        self.pll = pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        self.comb += pll.reset.eq(reset_in)
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        # HDMI PLL
        if with_hdmi:
            self.video_pll = video_pll = GW2APLL(devicename=platform.devicename, device=platform.device)
            video_pll.register_clkin(clk27, 27e6)
            video_pll.create_clkout(self.cd_hdmi5x, 125e6, margin=1e-2)
            self.specials += Instance("CLKDIV",
                p_DIV_MODE = "5",
                i_RESETN   = 1, # Disable reset signal.
                i_CALIB    = 0, # No calibration.
                i_HCLKIN   = self.cd_hdmi5x.clk,
                o_CLKOUT   = self.cd_hdmi.clk
            )

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, toolchain="gowin", sys_clk_freq=48e6,
        with_led_chaser = False,
        with_rgb_led    = True,
        with_buttons    = False,
        with_reset_button = True,
        with_mcp3008    = True,
        with_estop      = True,
        with_servo_pwm  = True,
        with_motor_pwm  = True,
        with_i2c        = True,
        with_spi_flash  = False,
        with_video_terminal  = False,
        with_video_colorbars = False,
        **kwargs):

        platform = sipeed_tang_nano_20k.Platform(toolchain=toolchain)
        if with_mcp3008:
            platform.add_extension([
                ("mcp3008", 0,
                    Subsignal("sclk", Pins("73")),
                    Subsignal("cs_n", Pins("74")),
                    Subsignal("mosi", Pins("49")),
                    Subsignal("miso", Pins("55")),
                    IOStandard("LVCMOS33"),
                ),
            ])
        if with_estop:
            platform.add_extension([
                ("estop_ext", 0, Pins("31"), IOStandard("LVCMOS33")),
            ])
        if with_servo_pwm:
            platform.add_extension([
                ("servo_pwm", 0, Pins("25 26 27 28 29"), IOStandard("LVCMOS33")),
            ])
        platform.add_extension([
            ("laser", 0, Pins("41"), IOStandard("LVCMOS33")),
        ])
        if with_motor_pwm:
            platform.add_extension([
                ("motor_pwm", 0,
                    Subsignal("pwm0", Pins("71")),
                    Subsignal("pwm1", Pins("72")),
                    IOStandard("LVCMOS33"),
                ),
            ])
        # I2C pads (shared with HDMI SDA/SCL).
        if with_i2c:
            platform.add_extension([
                ("i2c", 0,
                    Subsignal("scl", Pins("53")),
                    Subsignal("sda", Pins("52")),
                    IOStandard("LVCMOS33"),
                ),
            ])

        with_hdmi = with_video_terminal or with_video_colorbars
        kwargs.setdefault("uart_baudrate", 750000)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, with_hdmi=with_hdmi, with_reset_button=with_reset_button)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Tang Nano 20K", **kwargs)

        # SPI Flash: XT25F64B ----------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q64 as SpiFlashModule # compatible with XT25F64B
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            self.add_spi_flash(mode="1x", module=SpiFlashModule(Codes.READ_1_1_1))

        # SDR SDRAM --------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            class SDRAMPads:
                def __init__(self):
                    self.clk   = platform.request("O_sdram_clk")
                    self.cke   = platform.request("O_sdram_cke")
                    self.cs_n  = platform.request("O_sdram_cs_n")
                    self.cas_n = platform.request("O_sdram_cas_n")
                    self.ras_n = platform.request("O_sdram_ras_n")
                    self.we_n  = platform.request("O_sdram_wen_n")
                    self.dm    = platform.request("O_sdram_dqm")
                    self.a     = platform.request("O_sdram_addr")
                    self.ba    = platform.request("O_sdram_ba")
                    self.dq    = platform.request("IO_sdram_dq")
            sdram_pads = SDRAMPads()

            self.specials += DDROutput(0, 1, sdram_pads.clk, ClockSignal("sys"))

            self.sdrphy = GENSDRPHY(sdram_pads, sys_clk_freq)
            self.add_sdram("sdram",
                phy           = self.sdrphy,
                module        = M12L64322A(sys_clk_freq, "1:1"), # FIXME.
                l2_cache_size = 128,
            )

        # Video ------------------------------------------------------------------------------------
        if with_hdmi:
            self.videophy = VideoGowinHDMIPHY(platform.request("hdmi"), clock_domain="hdmi", true_lvds=True)
            if with_video_terminal:
                self.add_video_terminal(phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi")
            if with_video_colorbars:
                self.add_video_colorbars(phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi")

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("led_n"),
                sys_clk_freq = sys_clk_freq
            )

        # RGB Led ----------------------------------------------------------------------------------
        if with_rgb_led:
            self.rgb_led = WS2812StatusVerilog(
                pad          = platform.request("rgb_led"),
                sys_clk_freq = sys_clk_freq,
                platform     = platform,
                led_count    = 300,
            )
            self.add_csr("rgb_led")

        # Laser (shift register Qa) -----------------------------------------------------------------
        self.laser = LaserControl(reset=0)
        self.add_csr("laser")
        self.comb += platform.request("laser").eq(self.laser.enable)

        # I2C --------------------------------------------------------------------------------------
        if with_i2c:
            self.i2c = I2CMaster(pads=platform.request("i2c"), default_dev=True)
            self.add_csr("i2c")

        # Motor PWM --------------------------------------------------------------------------------
        if with_motor_pwm:
            self.motor_pwm = MotorPWM(pads=Signal(2))
            self.add_csr("motor_pwm")

        # MCP3008 ADC -------------------------------------------------------------------------------
        if with_mcp3008:
            self.mcp3008 = MCP3008ReaderVerilog(
                pads         = platform.request("mcp3008"),
                sys_clk_freq = sys_clk_freq,
                platform     = platform,
            )
            self.add_csr("mcp3008")

        # Servo PWM ---------------------------------------------------------------------------------
        if with_servo_pwm:
            servo_pads = platform.request("servo_pwm")
            if with_motor_pwm:
                motor_pads = platform.request("motor_pwm")
                pads = Cat(servo_pads, motor_pads.pwm0, motor_pads.pwm1)
                channels = 7
            else:
                pads = servo_pads
                channels = 5
            self.servo_pwm = ServoPWM(
                pads         = pads,
                sys_clk_freq = sys_clk_freq,
                channels     = channels,
            )
            self.add_csr("servo_pwm")

        # Buttons ----------------------------------------------------------------------------------
        if with_estop:
            self.estop_btns = GPIOIn(pads=Cat(platform.request("btn", 0), platform.request("estop_ext", 0)))
            self.add_csr("estop_btns")
        elif with_buttons:
            # Keep btn[1] for reset when enabled; only use btn[0] here.
            self.buttons = GPIOIn(pads=platform.request("btn", 0))


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform, description="LiteX SoC on Tang Nano 20K.")
    parser.set_defaults(uart_baudrate=750000, with_spi_flash=True)
    parser.add_target_argument("--flash",        action="store_true",      help="Flash Bitstream.")
    parser.add_target_argument("--sys-clk-freq", default=48e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-spi-flash", action="store_true", help="Enable SPI Flash (MMAPed).")
    parser.add_target_argument("--flash-firmware", action="store_true", help="Flash default firmware to SPI Flash.")
    parser.add_target_argument("--firmware-path", default="software/uart/uart_echo.bin",
                               help="Firmware binary to flash.")
    parser.add_target_argument("--firmware-offset", default="0x200000",
                               type=lambda x: int(x, 0),
                               help="SPI Flash offset for firmware (default: 0x200000).")
    parser.add_target_argument("--no-sd-boot", action="store_true",
                               help="Disable default SDCard boot (SPI mode).")
    sdopts = parser.target_group.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",            action="store_true", help="Enable SPI-mode SDCard support.")
    sdopts.add_argument("--with-sdcard",                action="store_true", help="Enable SDCard support.")
    viopts = parser.target_group.add_mutually_exclusive_group()
    viopts.add_argument("--with-video-terminal",   action="store_true", help="Enable Video Terminal (HDMI).")
    viopts.add_argument("--with-video-colorbars",  action="store_true", help="Enable Video Colorbars (HDMI).")
    args = parser.parse_args()

    with_spi_flash = args.with_spi_flash or args.flash_firmware
    auto_spi_sd = not args.no_sd_boot and not args.with_spi_sdcard and not args.with_sdcard
    soc = BaseSoC(
        toolchain    = args.toolchain,
        sys_clk_freq = args.sys_clk_freq,
        with_spi_flash       = with_spi_flash,
        with_video_terminal  = args.with_video_terminal,
        with_video_colorbars = args.with_video_colorbars,
        **parser.soc_argdict
    )
    if args.with_spi_sdcard or auto_spi_sd:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext=".fs"), external=True)

    if args.flash_firmware:
        firmware_path = os.path.abspath(args.firmware_path)
        if not os.path.exists(firmware_path):
            raise FileNotFoundError(f"Firmware not found: {firmware_path}")
        prog = soc.platform.create_programmer()
        prog.flash(args.firmware_offset, firmware_path, external=True)

if __name__ == "__main__":
    main()
