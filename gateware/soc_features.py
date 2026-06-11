"""Composable per-feature SoC adders for the IcePi Zero project tops.

Each function appends one self-contained peripheral block (platform pins +
gateware + CSRs/IRQs + firmware constants) to an already-constructed BaseSoC.
The project tops (icepi_zero_lcd.py, icepi_zero_mnist.py, icepi_zero_winc.py,
icepi_zero_mnist_lcd.py) are thin wrappers over these, and icepi_zero_all.py
composes all of them. The blocks moved here verbatim from those tops; the pin
choices match the physical wiring in docs/icepi_zero_pin_mapping.md.

The LSM6DS3 IMU is not a block of its own: it shares the aux bus
(add_winc_aux, cs[1]) and firmware drives it via aux_spi's AUX_IMU.
"""
from migen import Cat, Signal, If
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule
from litex.build.generic_platform import Pins, Subsignal, IOStandard, Misc
from litex.soc.cores.i2c import I2CMaster as HWI2CMaster
from litex.soc.cores.gpio import GPIOIn, GPIOOut
from litex.soc.interconnect.csr import AutoCSR, CSRStorage

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
        Subsignal("backlight", Pins("P1")),
        # reset_n on IO10 (L2): the LCD and CTP RST lines are tied together on
        # the board to this one pin (was the 74HC595 Qa/Qb, expander retired).
        Subsignal("reset_n",   Pins("L2")),
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

    # LCD_RST and CTP_RST are tied together on the board to one FPGA pin
    # (lcd_ctrl.reset_n on IO10/L2); LCDEngine drives it from the reset_n CSR
    # field. So touch reset follows LCD reset with no extra wiring here. (The
    # slow reset/enable sidebands will move onto a SPI GPIO expander on the aux
    # bus in the final build -- see docs/reset_sidebands.md.)

    # FT6336U capacitive touch (I2C + INT). RST follows LCD reset (above).
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
# Sideband pins match the physical wiring: CS=IO23, IRQ=IO24. RESET_N and
# CHIP_EN are on direct FPGA pins (IO20/F1, IO22/P2). CHIP_EN reaches the
# module's true power-down since 2026-06 (was a dummy before; the old external
# 3.3 V tie must be gone).
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
        Subsignal("chip_en", Pins("P2")),         # IO22 -> WINC CHIP_EN (low = power-down)
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
    CHIP_EN=0 (powered down) -- the WINC stays off until the firmware powers
    it up in nm_bsp_reset(). Both lines are on direct FPGA pins (see _winc_io).
    busy_led mirrors in-flight transfers.
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


# Boot control: sticky boot-request flag + FTDI sideband reset/sense -------------------------------
# FT231X DTR# -> L15, RTS# -> L16 (plain FPGA IO, active low). esptool-style
# cross-conditions -- both-asserted MUST be idle because the OS asserts both
# on every port open. Full semantics table: docs/boot_chain.md.
_ftdi_io = [
    ("ftdi_sideband", 0,
        Subsignal("dtr_n", Pins("L15"), Misc("PULLMODE=UP")),
        Subsignal("rts_n", Pins("L16"), Misc("PULLMODE=UP")),
        IOStandard("LVCMOS33"),
    ),
]


class BootCtl(LiteXModule, AutoCSR):
    """Sticky boot-request flag + FTDI host-reset (docs/boot_chain.md).

    flag: reset_less -- survives soft resets, 0 at power-on/reconfigure;
    written by loader_hook (LOADER_BOOT_MAGIC), cleared by the loader.
    reset_request: one-shot pulse after the FTDI reset combo (RTS asserted,
    DTR deasserted) holds for 50 ms; re-arms when the combo is released.
    Detector state is reset_less so the reset it causes can't re-trigger
    or truncate it; the filter rejects non-atomic port open/close states.
    """
    def __init__(self, dtr_n, rts_n, sys_clk_freq):
        self.flag = CSRStorage(32, reset_less=True,
            description="Boot request flag. Survives soft reset; 0 at power-on.")
        self.reset_request = Signal()   # -> crg.user_rst

        # 2FF synchronizers, converted to active-high "asserted" levels.
        dtr = Signal()   # 1 = host asserted DTR (pin low)
        rts = Signal()
        self.specials += MultiReg(~dtr_n, dtr)
        self.specials += MultiReg(~rts_n, rts)

        combo  = Signal()
        stable = int(50e-3 * sys_clk_freq)
        count  = Signal(max=stable + 1, reset_less=True)
        armed  = Signal(reset=1, reset_less=True)
        self.comb += combo.eq(rts & ~dtr)
        self.sync += [
            If(~combo,
                count.eq(0),
                armed.eq(1),
            ).Elif(count != stable,
                count.eq(count + 1),
            ),
            If(self.reset_request,
                armed.eq(0),
            ),
        ]
        self.comb += self.reset_request.eq(combo & armed & (count == stable))


def add_boot_ctl(soc):
    """Sticky boot flag + FTDI DTR/RTS host reset + level sense."""
    soc.platform.add_extension(_ftdi_io)
    pads = soc.platform.request("ftdi_sideband")

    soc.boot_ctl = BootCtl(pads.dtr_n, pads.rts_n, soc.sys_clk_freq)
    soc.add_csr("boot_ctl")
    if hasattr(soc.crg, "user_rst"):
        soc.comb += soc.crg.user_rst.eq(soc.boot_ctl.reset_request)

    # bit0 = dtr_n, bit1 = rts_n: the loader's "stay in loader" level + wiring check.
    soc.ftdi_sense = GPIOIn(Cat(pads.dtr_n, pads.rts_n))
    soc.add_csr("ftdi_sense")
