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
from litex.soc.cores.gpio import GPIOIn, GPIOOut, GPIOTristate
from litex.soc.interconnect.csr import AutoCSR, CSRStorage

from litex.soc.integration.soc import SoCRegion

from gateware.lcd_engine import LCDEngine
from gateware.snn_mlp import SNNMLP
from gateware.aux_spi import AuxSPIMaster
from gateware.logic_analyzer import LogicAnalyzer


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
    # Dedicated native SDRAM read port for the burst weight loader (32-bit to
    # match the core's N_MAC*DATA_WIDTH=32 beat). Bypasses the CPU Wishbone/L2
    # path, so weight streaming rides the SDRAM bandwidth directly.
    snn_port = soc.sdram.crossbar.get_port(mode="read", data_width=32)
    soc.snn = SNNMLP(soc.platform, dram_port=snn_port, n_mac=n_mac)
    soc.add_csr("snn")

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


# Aux SPI bus, IMU only (no WiFi) ------------------------------------------------------------------
# Same bus pins and CS ordering as add_winc_aux, so the aux_spi.c HAL
# (AUX_CS_IMU = cs[1]) is byte-for-byte unchanged. The WINC (cs[0]) and MCP
# (cs[2]) lines simply park high -- nothing is wired to them in a logger build.
# Keep these pins in sync with _winc_io's aux_spi subsignal.
_aux_imu_io = [
    ("aux_spi", 0,
        Subsignal("clk",  Pins("T2")),            # IO2  -- shared sclk
        Subsignal("mosi", Pins("H2")),            # IO8  -- shared mosi
        Subsignal("miso", Pins("J2")),            # IO25 -- shared miso
        Subsignal("cs_n", Pins("M2 F3 R2")),      # cs[0]=WINC cs[1]=IMU(IO6) cs[2]=MCP
        IOStandard("LVCMOS33"),
    ),
]


def add_aux_imu(soc, imu_spi_clk_freq=1e6):
    """Shared aux SPI bus for the LSM6DS3 IMU alone (no WINC sidebands).

    For builds that want the IMU but not WiFi (the SD data logger). Bus pins
    and CS indices match add_winc_aux exactly, so firmware reuses aux_spi.c's
    AUX_IMU device with no changes. CS is software-held; runtime divider via
    the aux_spi clk_divider CSR (LSM6DS3 tops out at 10 MHz; bus at sys/2).
    """
    platform = soc.platform
    platform.add_extension(_aux_imu_io)

    soc.aux_spi = AuxSPIMaster(
        pads                 = platform.request("aux_spi"),
        platform             = platform,
        sys_clk_freq         = soc.sys_clk_freq,
        default_spi_clk_freq = int(imu_spi_clk_freq),
    )
    soc.add_csr("aux_spi")

    soc.add_constant("AUX_CS_WINC", AUX_CS_WINC)
    soc.add_constant("AUX_CS_IMU",  AUX_CS_IMU)
    soc.add_constant("AUX_CS_MCP",  AUX_CS_MCP)


# MCP23S17 SPI GPIO expander on the aux bus -------------------------------------------------------
# Replaces the retired ATWINC1500 in the aux-bus slot (board refactor 2026-06-25):
# the WINC is physically gone (an ESP32-C3 will later take over the WiFi-loader
# role), and the long-planned SPI GPIO expander (see docs/reset_sidebands.md +
# the sr595 retirement) is finally wired in. The expander shares the same
# sclk/mosi/miso as the IMU + MCP3008 and adds a 4th chip-select plus a reset and
# an interrupt sideband.
#
# Pins (physical wiring, 2026-06-25):
#   CS    = IO11 / G2   -- 4th aux-bus chip-select (cs[3], AUX_CS_IOX)
#   RESET = IO10 / L2   -- active low (this was the *direct* LCD/CTP reset pin;
#                          in an expander build the LCD/CTP resets move onto
#                          expander OUTPUT pins, freeing L2 to reset the expander)
#   INTA  = IO22 / P2   -- interrupt in (this was the WINC CHIP_EN pin)
#
# TWO namespace/pin traps to remember:
#  (1) "MCP" on this bus historically means the MCP3008 ADC (cs[2], AUX_CS_MCP).
#      This expander is a DISTINCT chip -> it uses AUX_CS_IOX / iox_* / "IOX".
#  (2) CS pin IO11/G2 is ALSO the NMEA GPS UART TX (add_gps_uart). The expander
#      and a GPS-on-IO11 build are mutually exclusive until one is repinned.
# Both reset (L2) and INTA (P2) collide with add_lcd_touch / add_winc_aux pins,
# so this adder is for the standalone bring-up top (icepi_zero_mcp.py) until the
# deployables are migrated off the WINC and the LCD reset is rerouted.
_mcp_io = [
    ("aux_spi", 0,
        Subsignal("clk",  Pins("T2")),               # IO2  -- shared sclk
        Subsignal("mosi", Pins("H2")),               # IO8  -- shared mosi
        Subsignal("miso", Pins("J2")),               # IO25 -- shared miso
        Subsignal("cs_n", Pins("M2 F3 R2 G2")),      # cs0=unused(was WINC,IO23) cs1=IMU(IO6) cs2=MCP3008(IO3) cs3=IOX(IO11)
        IOStandard("LVCMOS33"),
    ),
    ("iox_ctrl", 0,
        Subsignal("reset_n", Pins("L2")),            # IO10 -> MCP23S17 RESET (active low)
        Subsignal("inta",    Pins("P2"), Misc("PULLMODE=UP")),  # IO22 <- MCP23S17 INTA
        IOStandard("LVCMOS33"),
    ),
]

# Chip-select index for the MCP23S17 I/O expander on the shared aux bus (4th line).
AUX_CS_IOX = 3


def add_mcp_expander(soc, iox_spi_clk_freq=1e6, busy_led=None):
    """Shared aux SPI bus + MCP23S17 SPI GPIO-expander sidebands (reset + INTA).

    The expander rides the same AuxSPIMaster as the IMU/MCP3008 on a 4th CS
    (AUX_CS_IOX). reset_n is a GPIOOut that defaults to 0, so the expander powers
    up held in reset until firmware releases it (mcp23s17_reset()). inta is a
    GPIOIn with IRQ (firmware picks the edge; MCP23S17 INTA defaults active-low).
    busy_led mirrors in-flight aux transfers. This is the post-WINC replacement
    for add_winc_aux during expander bring-up; cs[0] (the old WINC line) parks
    high -- nothing is wired to it.
    """
    platform = soc.platform
    platform.add_extension(_mcp_io)

    soc.aux_spi = AuxSPIMaster(
        pads                 = platform.request("aux_spi"),
        platform             = platform,
        sys_clk_freq         = soc.sys_clk_freq,
        default_spi_clk_freq = int(iox_spi_clk_freq),
    )
    soc.add_csr("aux_spi")

    ctrl = platform.request("iox_ctrl")
    soc.iox_reset = GPIOOut(ctrl.reset_n)        # write 0 = assert reset, 1 = release
    soc.iox_inta  = GPIOIn(ctrl.inta, with_irq=True)
    soc.add_csr("iox_reset")
    soc.add_csr("iox_inta")
    soc.irq.add("iox_inta", use_loc_if_exists=True)

    soc.add_constant("IOX_SPI_DEFAULT_FREQUENCY", int(iox_spi_clk_freq))
    soc.add_constant("AUX_CS_IMU", AUX_CS_IMU)
    soc.add_constant("AUX_CS_MCP", AUX_CS_MCP)   # MCP3008 ADC, NOT the expander
    soc.add_constant("AUX_CS_IOX", AUX_CS_IOX)   # MCP23S17 GPIO expander

    if busy_led is not None:
        # Sanity LED: lit while an aux-bus SPI transfer is in flight.
        try:
            soc.comb += platform.request("user_led", busy_led).eq(soc.aux_spi.busy)
        except Exception:
            pass


# GPA7 loopback test fixture (expander bring-up only) ---------------------------------------------
# IO24 / L1 (the freed WINC IRQ pin) is wired on the bench to the MCP23S17's
# GPA7. Exposed as a tristate GPIO so firmware can drive BOTH directions of a
# closed-loop test through the expander's real GPIO silicon:
#   - expander GPA7 = output -> FPGA reads IO24      (proves the expander drives pins)
#   - FPGA drives IO24       -> expander reads GPA7   (proves expander input + SPI read)
# This is a bench fixture, not part of the expander interface -- drop the adder
# once the expander is verified. CSR accessors:
#   gpa7_loop_oe_write / gpa7_loop_out_write / gpa7_loop_in_read.
_gpa7_loop_io = [
    ("gpa7_loop", 0, Pins("L1"), IOStandard("LVCMOS33")),   # IO24 <-> MCP23S17 GPA7
]


def add_gpa7_loopback(soc):
    """Tristate GPIO on IO24/L1 for the MCP23S17 GPA7 bench loopback test."""
    soc.platform.add_extension(_gpa7_loop_io)
    soc.gpa7_loop = GPIOTristate(soc.platform.request("gpa7_loop"))
    soc.add_csr("gpa7_loop")


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


# NMEA GPS UART ------------------------------------------------------------------------------------
# A second hardware UART for an NMEA GPS module (the logger's position source;
# fixes become IMU_REC_GPS records on the same timer0 timebase as the IMU). The
# module's own TX is on IO5 (E1) and its RX on IO11 (G2), so from the FPGA side
# rx=E1 (we receive the module's NMEA stream) and tx=G2 (we transmit, e.g. UBX
# config). Pins are free: the LCD/aux/SD blocks don't touch E1 or G2.
#
# Most NMEA modules power up at 9600 8N1; override via add_gps_uart(baudrate=).
# add_uart wires a full CSR UART named "gps" (gps_rxtx / gps_rxempty / gps_txfull
# + an ev IRQ), so firmware reads it independently of the console "serial" UART.
_gps_io = [
    ("gps_serial", 0,
        Subsignal("tx", Pins("G2")),   # IO11 -> GPS RX  (FPGA transmits)
        Subsignal("rx", Pins("E1")),   # IO5  <- GPS TX  (FPGA receives)
        IOStandard("LVCMOS33"),
    ),
]


def add_gps_uart(soc, baudrate=9600):
    """Second hardware UART (CSR + IRQ) for an NMEA GPS on IO5(rx)/IO11(tx)."""
    soc.platform.add_extension(_gps_io)
    soc.add_uart(name="gps", uart_name="gps_serial", baudrate=baudrate)
    soc.add_constant("GPS_UART_BAUDRATE", baudrate)


# Logic analyzer -----------------------------------------------------------------------------------
# A 3.3 V logic analyzer that captures into an SDRAM ring (gateware/logic_analyzer.py).
# It coexists with the mandatory WINC aux bus: the probe pins are all OUTSIDE that
# bus, so capturing can't disturb WiFi flashing. The IMU (cs[1]) and MCP3008 (cs[2])
# stay parked-deasserted on the aux bus exactly as in any WINC build.
#
# The 18 probe channels are the GPIO-bank pins that are free once the LCD/touch
# module is unplugged (see docs/icepi_zero_pin_mapping.md + docs/logic_analyzer.md).
# Bit order below == channel index (bit0..bit17); host channel labels follow it.
#   ch:  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17
#   IO:  1    4    5    7    9   10   11   12   13   14   15   16   17   18   19   21   26   27
#  pin: K3   R1   E1   G1   J1   L2   G2   J3   E3   P1   N1   H3   R3   N4   E4   F2   D4   P3
#
# All channels get an internal pull-DOWN: unconnected probes idle low (clean LA
# trace), AND it powers the LCD-present guard -- the FT6336U touch I2C lines
# (ch10/IO15 = CTP_SCL, ch13/IO18 = CTP_SDA) carry on-module pull-ups, so if the
# module is still attached those two channels read HIGH against our pull-down.
# guard_mask = (1<<10)|(1<<13) = 0x2400; firmware refuses to arm if it trips.
#
# P1/IO14 is the LCD backlight pin: the LA top MUST build BaseSoC with
# force_lcd_backlight_off=False so this block can own P1 as a probe channel.
_la_io = [
    ("la_probe", 0,
        Pins("K3 R1 E1 G1 J1 L2 G2 J3 E3 P1 N1 H3 R3 N4 E4 F2 D4 P3"),
        IOStandard("LVCMOS33"), Misc("PULLMODE=DOWN")),
]

LA_N_CHANNELS = 18
LA_GUARD_MASK = (1 << 10) | (1 << 13)   # ch10 (IO15/CTP_SCL) + ch13 (IO18/CTP_SDA)


def add_logic_analyzer(soc, n_channels=LA_N_CHANNELS, guard_mask=LA_GUARD_MASK,
                       fifo_depth=512):
    """SDRAM-streaming 3.3 V logic analyzer on the freed GPIO-bank pins.

    Caller contract: BaseSoC must be built with force_lcd_backlight_off=False
    (this block owns P1/IO14 as a probe channel). The capture core takes a
    dedicated native SDRAM write port, like the SNN's read port -- it streams
    samples straight to DRAM bandwidth, independent of the CPU Wishbone path.
    """
    platform = soc.platform
    platform.add_extension(_la_io)
    probe = platform.request("la_probe")

    port = soc.sdram.crossbar.get_port(mode="write", data_width=32)
    soc.la = LogicAnalyzer(port, probe, n_channels=n_channels,
                           guard_mask=guard_mask, fifo_depth=fifo_depth)
    soc.add_csr("la")

    soc.add_constant("LA_N_CHANNELS", n_channels)
    soc.add_constant("LA_GUARD_MASK", guard_mask)


# OTA-flashing baseline ----------------------------------------------------------------------------
# flash.py talks to the board ONLY over WiFi (the WINC), so without the WINC aux
# bus + the LiteSPI flash master a bitstream can only be updated over bit-banged
# JTAG (the slow recovery path). The WINC is therefore part of the baseline, not
# a feature: every DEPLOYABLE top composes this. The IMU (cs[1]) and MCP3008
# (cs[2]) ride the same aux bus for free, so a logger needs no extra SPI block.
#
# Caller contract: construct BaseSoC with with_spi_flash=True and
# flash_master=True (the XIP-BIOS + LiteSPI-master deployment shape); this then
# adds the WINC bus + the boot-manager flag/reset. busy_led mirrors aux-bus
# traffic -- pass an LED index NOT used by another block (the SNN takes 0,1, so
# WINC uses 2 in builds that include the SNN). See docs/boot_chain.md.
def add_flashing_baseline(soc, winc_spi_clk_freq=12.5e6, busy_led=0):
    """WINC aux bus + boot_ctl: the mandatory WiFi-OTA baseline for deployables."""
    add_winc_aux(soc, winc_spi_clk_freq, busy_led=busy_led)
    add_boot_ctl(soc)
