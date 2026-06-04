"""Composable per-feature SoC adders for the IcePi Zero project tops.

Each function appends one self-contained peripheral block (platform pins +
gateware + CSRs/IRQs + firmware constants) to an already-constructed BaseSoC.
The project tops (icepi_zero_lcd.py, icepi_zero_mnist.py, icepi_zero_winc.py,
icepi_zero_mnist_lcd.py) are thin wrappers over these, and icepi_zero_all.py
composes all of them. The blocks moved here verbatim from those tops; the pin
choices match the physical wiring in docs/icepi_zero_pin_mapping.md.

Not represented here (and why):
- imu_spi (icepi_zero_imu.py): superseded -- the LSM6DS3 sits on the shared
  aux bus (add_winc_aux, cs[1]) and firmware drives it via aux_spi's AUX_IMU.
- snn_estimator (icepi_zero_snn.py): tracking-experiment debug core; collides
  with SNNMLP on the CSR name "snn" and stays a standalone bring-up top.
"""
from litex.build.generic_platform import Pins, Subsignal, IOStandard, Misc
from litex.soc.cores.i2c import I2CMaster as HWI2CMaster
from litex.soc.cores.gpio import GPIOIn, GPIOOut
from litex.soc.integration.soc import SoCRegion

from gateware.lcd_engine import LCDEngine
from gateware.snn_mlp import SNNMLP
from gateware.aux_spi import AuxSPIMaster


# LCD / Touch --------------------------------------------------------------------------------------
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


def add_lcd_touch(soc, lcd_spi_clk_freq):
    """ST7796S LCD engine + FT6336U capacitive touch (I2C + INT).

    Caller contract: the BaseSoC must have been constructed with
    spi_clk_freq=lcd_spi_clk_freq (the engine's SPI shifter lives in cd_spi,
    SCK = lcd_spi_clk_freq / 2) and force_lcd_backlight_off=False (the
    lcd_ctrl extension owns P1).
    """
    assert hasattr(soc.crg, "cd_spi"), \
        "add_lcd_touch: BaseSoC must be built with spi_clk_freq=lcd_spi_clk_freq"
    platform = soc.platform
    platform.add_extension(_lcd_io)

    soc.lcd = LCDEngine(
        pads      = platform.request("lcd_spi"),
        ctrl_pads = platform.request("lcd_ctrl"),
        platform  = platform,
        sclk_div  = 1,
    )
    soc.bus.add_master(name="lcd_dma", master=soc.lcd.bus)
    soc.irq.add("lcd", use_loc_if_exists=True)
    soc.add_constant("LCD_WIDTH",  320)
    soc.add_constant("LCD_HEIGHT", 480)
    soc.add_constant("LCD_SPI_FREQUENCY", int(lcd_spi_clk_freq // 2))

    # FT6336U capacitive touch (I2C + INT). RST is tied to LCD reset.
    # Hardware I2C master (wishbone-mapped, 2 word registers: xfer + config).
    # Interrupts: ev.idle fires on transfer completion.
    soc.ctp_i2c = HWI2CMaster(platform.request("ctp_i2c"))
    soc.bus.add_slave("ctp_i2c", soc.ctp_i2c.bus,
                      SoCRegion(origin=None, size=0x10, cached=False))
    soc.irq.add("ctp_i2c", use_loc_if_exists=True)

    # INT line: GPIOIn with IRQ. Firmware configures edge=falling at runtime
    # (FT6336U pulses INT low on touch events).
    soc.ctp_int = GPIOIn(platform.request("ctp_int"), with_irq=True)
    soc.irq.add("ctp_int", use_loc_if_exists=True)


# SNN-MLP MNIST classifier -------------------------------------------------------------------------
# MAC parallelism sweet spot: N_MAC=2 packs two Q4.12 weights into one 32-bit
# Wishbone word, halving both core cycles AND SDRAM weight traffic. Going
# higher does NOT help -- the 16-bit SDR SDRAM (~100 MB/s) is the wall, and at
# N_MAC=2 the core is already faster than the DRAM can feed it. See
# docs/snn_mnist.md. Must match the --n-mac passed to
# tools/pack_snn_mnist_weights.py.
def add_snn_mlp(soc, n_mac=2, leds=None):
    """SNN-MLP MNIST inference peripheral (784->64->10, SDRAM-streamed weights).

    leds=(busy_idx, done_idx) mirrors status on user LEDs when the board
    exposes them (silently skipped otherwise).
    """
    soc.snn = SNNMLP(soc.platform, n_mac=n_mac)
    soc.add_csr("snn")
    soc.bus.add_master(name="snn_wb", master=soc.snn.wb)

    if leds is not None:
        try:
            soc.comb += [
                soc.platform.request("user_led", leds[0]).eq(soc.snn.status.fields.busy),
                soc.platform.request("user_led", leds[1]).eq(soc.snn.status.fields.done),
            ]
        except Exception:
            pass


# Shared aux SPI bus + ATWINC1500 sidebands --------------------------------------------------------
# The bus lines match the IMU bus (IO2/IO8/IO25); chip-selects are ordered
# WINC, IMU, MCP3008 so that no device CS floats (unused CS parked high).
#
# Sideband pins match the physical wiring: CS=IO23, IRQ=IO24, RST=IO20.
# The module's EN/CHIP_EN is NOT wired (onboard pull-up keeps it enabled), so
# chip_en goes to a free dummy pin (IO9) -- firmware writes to it harmlessly.
_winc_io = [
    ("aux_spi", 0,
        Subsignal("clk",  Pins("T2")),            # IO2  -- shared sclk
        Subsignal("mosi", Pins("H2")),            # IO8  -- shared mosi
        Subsignal("miso", Pins("J2")),            # IO25 -- shared miso
        Subsignal("cs_n", Pins("M2 F3 R2")),      # cs[0]=WINC(IO23) cs[1]=IMU(IO6) cs[2]=MCP(IO3)
        IOStandard("LVCMOS33"),
    ),
    ("winc_ctrl", 0,
        Subsignal("reset_n", Pins("F1")),         # IO20 -> WINC RESET_N (active low)
        Subsignal("chip_en", Pins("J1")),         # IO9  -- dummy, EN not wired (module pull-up)
        Subsignal("irq_n",   Pins("L1"), Misc("PULLMODE=UP")),  # IO24 <- WINC IRQN (active low)
        IOStandard("LVCMOS33"),
    ),
]

# Chip-select indices on the shared aux bus (must match software HAL table).
AUX_CS_WINC = 0
AUX_CS_IMU  = 1
AUX_CS_MCP  = 2


def add_winc_aux(soc, winc_spi_clk_freq, busy_led=None):
    """Shared aux SPI bus (runtime divider, software-held CS) + WINC sidebands.

    GPIOOut defaults to 0, so at power-on RESET_N=0 (held in reset) and
    CHIP_EN=0 (disabled) -- a safe state the firmware releases during
    nm_bsp_reset(). busy_led mirrors in-flight transfers on a user LED.
    """
    platform = soc.platform
    platform.add_extension(_winc_io)

    soc.aux_spi = AuxSPIMaster(
        pads                 = platform.request("aux_spi"),
        platform             = platform,
        sys_clk_freq         = soc.sys_clk_freq,
        default_spi_clk_freq = int(winc_spi_clk_freq),
    )
    soc.add_csr("aux_spi")

    ctrl = platform.request("winc_ctrl")
    soc.winc_reset = GPIOOut(ctrl.reset_n)   # write 0 = assert reset
    soc.winc_en    = GPIOOut(ctrl.chip_en)   # write 1 = enable chip
    soc.winc_irq   = GPIOIn(ctrl.irq_n, with_irq=True)
    # NOTE: if a build reports these CSRs are already registered, this LiteX
    # version auto-collects submodule CSRs -- drop the three add_csr() calls.
    soc.add_csr("winc_reset")
    soc.add_csr("winc_en")
    soc.add_csr("winc_irq")
    soc.irq.add("winc_irq", use_loc_if_exists=True)

    soc.add_constant("WINC_SPI_DEFAULT_FREQUENCY", int(winc_spi_clk_freq))
    soc.add_constant("AUX_CS_WINC", AUX_CS_WINC)
    soc.add_constant("AUX_CS_IMU",  AUX_CS_IMU)
    soc.add_constant("AUX_CS_MCP",  AUX_CS_MCP)

    if busy_led is not None:
        # Sanity LED: lit while an aux-bus SPI transfer is in flight.
        try:
            soc.comb += platform.request("user_led", busy_led).eq(soc.aux_spi.busy)
        except Exception:
            pass
