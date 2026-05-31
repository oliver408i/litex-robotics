#!/usr/bin/env python3
"""IcePi Zero SoC for the on-FPGA MNIST touch demo.

Fuses the two previously-separate projects into one SoC:

  - the ST7796S LCD engine + FT6336U capacitive touch (from icepi_zero_lcd.py)
  - the SNN-MLP MNIST inference peripheral (from icepi_zero_mnist.py)

so a single firmware can let the user finger-draw a digit on the touchscreen,
run it through the on-FPGA SNN, and show the prediction on the same panel.

Both peripherals add a Wishbone master to the system bus (lcd_dma + snn_wb);
both expose CSRs. The LCD engine only adds depth-4 async FIFOs (LUTRAM, not
block RAM), so the combined SoC stays within the MNIST build's ~78% BRAM budget
-- the gate to confirm is PnR timing, whose margin at N_MAC=2 is thin (~2%).
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from migen import *
from litex.build.generic_platform import Pins, Subsignal, IOStandard, Misc
from litex.soc.cores.i2c import I2CMaster as HWI2CMaster
from litex.soc.cores.gpio import GPIOIn
from litex.soc.integration.soc import SoCRegion

from gateware.lcd_engine import LCDEngine
from gateware.snn_mlp import SNNMLP


# MAC parallelism for the SNN core. N_MAC=2 is the sweet spot on this board:
# two Q4.12 weights pack exactly into one 32-bit Wishbone word. Must match the
# --n-mac passed to tools/pack_snn_mnist_weights.py. See docs/snn_mnist.md.
N_MAC = 2


# LCD / Touch IO -----------------------------------------------------------------------------------
# Identical to icepi_zero_lcd.py; kept here so the combined target is self-contained.
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


class MnistLCDSoC(BaseSoC):
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

        # --- LCD engine + touch (from icepi_zero_lcd.py) ----------------------
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
        self.ctp_i2c = HWI2CMaster(platform.request("ctp_i2c"))
        self.bus.add_slave("ctp_i2c", self.ctp_i2c.bus,
                           SoCRegion(origin=None, size=0x10, cached=False))
        self.irq.add("ctp_i2c", use_loc_if_exists=True)

        self.ctp_int = GPIOIn(platform.request("ctp_int"), with_irq=True)
        self.irq.add("ctp_int", use_loc_if_exists=True)

        # --- SNN-MLP MNIST classifier (from icepi_zero_mnist.py) --------------
        self.snn = SNNMLP(self.platform, n_mac=N_MAC)
        self.add_csr("snn")
        self.bus.add_master(name="snn_wb", master=self.snn.wb)

        # Status LEDs: SNN busy on LED0, done on LED1 when the board exposes them.
        try:
            self.comb += [
                self.platform.request("user_led", 0).eq(self.snn.status.fields.busy),
                self.platform.request("user_led", 1).eq(self.snn.status.fields.done),
            ]
        except Exception:
            pass


def main():
    parser = make_parser(description="IcePi Zero SoC + LCD/touch + SNN-MLP MNIST (touch demo).")
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. Default 100 MHz -> 50 MHz SCK.")
    args = parser.parse_args()

    # Boot the demo straight from SPI flash by default so the board runs
    # standalone after power-on (no host attached). The BIOS still falls back
    # to serial boot if no valid image is present, so `litex_term --kernel`
    # dev flow keeps working. Flash the firmware to the same offset with
    # --flash-firmware. Pass --flash-boot-offset to override the default.
    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = MnistLCDSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args),
        flash_boot_offset = args.flash_boot_offset,
        lcd_spi_clk_freq  = args.lcd_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
