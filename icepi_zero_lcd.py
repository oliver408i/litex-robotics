#!/usr/bin/env python3
"""IcePi Zero SoC for the ST7796S LCD + FT6336U touch project."""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from migen import *
from litex.build.generic_platform import Pins, Subsignal, IOStandard, Misc
from litex.soc.cores.i2c import I2CMaster as HWI2CMaster
from litex.soc.cores.gpio import GPIOIn
from litex.soc.integration.soc import SoCRegion

from gateware.lcd_engine import LCDEngine


# LCD / Touch IO -----------------------------------------------------------------------------------
_lcd_io = [
    ("lcd_spi", 0,
        Subsignal("clk",  Pins("E4")),
        Subsignal("cs_n", Pins("H3")),
        Subsignal("mosi", Pins("D4")),
        Subsignal("miso", Pins("E3")),
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
    ("ctp_int", 0, Pins("F2"), IOStandard("LVCMOS33"), Misc("PULLMODE=UP")),
]


class LCDSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None,
                 lcd_spi_clk_freq=100e6, **kwargs):
        from litex_boards.platforms import icepi_zero as icepi_zero_platform
        platform = icepi_zero_platform.Platform()
        platform.add_extension(_lcd_io)

        super().__init__(
            sys_clk_freq             = sys_clk_freq,
            with_spi_flash           = with_spi_flash,
            flash_boot_offset        = flash_boot_offset,
            spi_clk_freq             = lcd_spi_clk_freq,
            platform                 = platform,
            force_lcd_backlight_off  = False,  # this project owns P1 via lcd_ctrl
            **kwargs,
        )

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

        # INT line: GPIOIn with IRQ. Firmware configures edge=falling at runtime
        # (FT6336U pulses INT low on touch events).
        self.ctp_int = GPIOIn(platform.request("ctp_int"), with_irq=True)
        self.irq.add("ctp_int", use_loc_if_exists=True)


def main():
    parser = make_parser(description="IcePi Zero SoC + ST7796S LCD + FT6336U touch.")
    parser.add_target_argument("--lcd-spi-clk-freq", default=100e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. Default 100 MHz -> 50 MHz SCK.")
    args = parser.parse_args()

    soc = LCDSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args),
        flash_boot_offset = args.flash_boot_offset,
        lcd_spi_clk_freq  = args.lcd_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
