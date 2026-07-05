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
from litex.soc.cores.spi.spi_bone import SPIBone
from litex.soc.interconnect.csr import AutoCSR, CSRStorage

from litex.soc.integration.soc import SoCRegion

from gateware.lcd_engine import LCDEngine
from gateware.snn_mlp import SNNMLP
from gateware.aux_spi import AuxSPIMaster
from gateware.logic_analyzer import LogicAnalyzer
from gateware.spi_slave import SPISlave


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


def add_aux_imu(soc, imu_spi_clk_freq=1e6, busy_led=None):
    """Shared aux SPI bus without WINC sidebands (IMU cs[1] / MCP3008 cs[2]).

    The WINC-free aux bus: bus pins and CS indices match add_winc_aux exactly,
    so firmware reuses aux_spi.c's AUX_IMU device with no changes. cs[0] (the
    old WINC line) simply parks high -- nothing is wired to it post-WINC. CS is
    software-held; runtime divider via the aux_spi clk_divider CSR (LSM6DS3 tops
    out at 10 MHz; bus at sys/2). busy_led, if given, lights an LED while an
    aux-bus transfer is in flight.
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

    soc.add_constant("AUX_CS_WINC", AUX_CS_WINC)  # cs[0] parks unused post-WINC
    soc.add_constant("AUX_CS_IMU",  AUX_CS_IMU)
    soc.add_constant("AUX_CS_MCP",  AUX_CS_MCP)

    if busy_led is not None:
        # Sanity LED: lit while an aux-bus SPI transfer is in flight.
        try:
            soc.comb += platform.request("user_led", busy_led).eq(soc.aux_spi.busy)
        except Exception:
            pass


# MCP23S17 SPI GPIO expander on the aux bus -------------------------------------------------------
# Replaces the retired ATWINC1500 in the aux-bus slot (board refactor 2026-06-25):
# the WINC is physically gone (an ESP32-C3 will later take over the WiFi-loader
# role), and the long-planned SPI GPIO expander (see docs/reset_sidebands.md +
# the sr595 retirement) is finally wired in. The expander shares the same
# sclk/mosi/miso as the IMU + MCP3008 and adds a 4th chip-select plus a reset and
# an interrupt sideband.
#
# Pins:
#   CS    = IO17 / R3   -- 3rd aux-bus chip-select (cs[2], AUX_CS_IOX)
#
# THE CS SAGA, and how it actually ended (2026-07-04): the CS was chased across
# FIVE pins (IO11/G2 -> IO9/J1 -> IO4/R1 -> IO17/R3 -> IO16/H3) on a "CS stuck at
# 0V / dead pin" theory, including a scary "IO17 bare header pin reads 0V with
# nothing attached" measurement that spawned a whole "the 74HC595-freed pin pool
# (IO5/9/11/12/17) shares a dead I/O bank" hypothesis. That theory was WRONG.
# The real bug was never the pin or the gateware -- it was INTERMITTENT PHYSICAL
# WIRING on the MCP side (floating A2:A0 straps / loose jumpers). Proof, in order:
#   - IMU WHO_AM_I passed the whole time  -> shared sclk/mosi/miso datapath good.
#   - a CS-metering diag (drive cs_n straight from the CSR, hold 3s, meter the
#     pin) showed the CS pin tracks the CSR perfectly              -> pin good.
#   - the register probe INTERMITTENTLY passed (echoed 0x55/0xAA/... back
#     correctly for seconds, then went silent)  -> chip ALIVE, link flaky.
# A chip that echoes walking patterns even once is alive and connected; the "0V"
# readings were a mix of stale-bitstream/measurement artifacts and a genuinely
# silent bus floating to a pull-up during a dropout. See memory
# [[mcp23s17-was-intermittent-wiring]]. Lesson for next time: when the IMU cross-
# check passes, STOP touching gateware/pins -- the datapath is proven; read the
# slave's own POR registers and watch for pass/fail flipping over seconds (=
# wiring), and hard-tie A2:A0 to GND first. Same bisect-the-layer lesson as the
# C3 bring-up ([[c3-link-gpio-diag]]).
#
# CS now lives on IO17/R3 (a dedicated free pin) rather than the borrowed
# IO16/H3 (= LCD_CS), which frees H3 for future LCD coexistence. If IO17 ever
# misbehaves again, re-verify with the CS-metering diag before assuming the pin
# -- do not reopen the pin-pool rabbit hole.
#   RESET = IO10 / L2   -- active low (this was the *direct* LCD/CTP reset pin;
#                          in an expander build the LCD/CTP resets move onto
#                          expander OUTPUT pins, freeing L2 to reset the expander)
#   INTA  = IO22 / P2   -- interrupt in (this was the WINC CHIP_EN pin)
#
# TWO namespace/pin traps to remember:
#  (1) "MCP" on this bus historically means the MCP3008 ADC (cs[1], AUX_CS_MCP).
#      This expander is a DISTINCT chip -> it uses AUX_CS_IOX / iox_* / "IOX".
#  (2) IO11/G2 (the pin the CS was originally mis-documented as) is the NMEA
#      GPS UART TX (add_gps_uart) -- genuinely already wired that way on this
#      board, not just a future planning conflict.
# Both reset (L2) and INTA (P2) collide with add_lcd_touch / add_winc_aux pins,
# so this adder is for the standalone bring-up top (icepi_zero_mcp.py) until the
# deployables are migrated off the WINC and the LCD reset is rerouted.
#
# cs_n is 3-wide here, NOT the WINC-era 4-wide layout (_winc_io/_aux_io above):
# the dead WINC cs[0] slot (M2/IO23) is dropped entirely, because M2 is now the
# ESP32-C3 link's MISO pin (add_c3_spibone) -- keeping a 4th unused cs_n driver
# on M2 collides with that at the pin-binding level (two TRELLIS_IO cells on one
# bel), found when combining this with the C3 loader for MCP bring-up. So the
# AUX_CS_* indices below are LOCAL to this 3-wide bus, not the shared
# AUX_CS_IMU/AUX_CS_MCP module constants used by the WINC-era 4-wide extensions.
_mcp_io = [
    ("aux_spi", 0,
        Subsignal("clk",  Pins("T2")),               # IO2  -- shared sclk
        Subsignal("mosi", Pins("H2")),               # IO8  -- shared mosi
        Subsignal("miso", Pins("J2")),               # IO25 -- shared miso
        Subsignal("cs_n", Pins("F3 R2 R3")),         # cs0=IMU(IO6) cs1=MCP3008(IO3) cs2=IOX(IO17)
        IOStandard("LVCMOS33"),
    ),
    ("iox_ctrl", 0,
        Subsignal("reset_n", Pins("L2")),            # IO10 -> MCP23S17 RESET (active low)
        Subsignal("inta",    Pins("P2"), Misc("PULLMODE=UP")),  # IO22 <- MCP23S17 INTA
        IOStandard("LVCMOS33"),
    ),
]

# Chip-select indices for the MCP23S17 bring-up's 3-wide aux bus (_mcp_io above).
_MCP_AUX_CS_IMU = 0
_MCP_AUX_CS_MCP = 1   # MCP3008 ADC, NOT the expander
AUX_CS_IOX      = 2   # MCP23S17 GPIO expander


def add_mcp_expander(soc, iox_spi_clk_freq=1e6, busy_led=None):
    """Shared aux SPI bus + MCP23S17 SPI GPIO-expander sidebands (reset + INTA).

    The expander rides the same AuxSPIMaster as the IMU/MCP3008 on a 3rd CS
    (AUX_CS_IOX). reset_n is a GPIOOut that defaults to 0, so the expander powers
    up held in reset until firmware releases it (mcp23s17_reset()). inta is a
    GPIOIn with IRQ (firmware picks the edge; MCP23S17 INTA defaults active-low).
    busy_led mirrors in-flight aux transfers. This is the post-WINC replacement
    for add_winc_aux during expander bring-up; the old WINC cs[0] slot is gone
    entirely (see _mcp_io comment -- it now collides with the C3 link's MISO).
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
    soc.add_constant("AUX_CS_IMU", _MCP_AUX_CS_IMU)
    soc.add_constant("AUX_CS_MCP", _MCP_AUX_CS_MCP)   # MCP3008 ADC, NOT the expander
    soc.add_constant("AUX_CS_IOX", AUX_CS_IOX)        # MCP23S17 GPIO expander

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


# ESP32-C3 SPI flash-loader link ------------------------------------------------------------------
# Dedicated SPI bus where the FPGA is the SLAVE and the ESP32-C3 (master) streams
# flash erase/program/verify commands -- the post-WINC loader transport (see
# docs/c3_loader.md and gateware/spi_slave.py). All pins were freed by the WINC
# removal and are fully separate from the aux bus (IMU/MCP), so no contention.
#   SCLK=IO27/P3  MOSI=IO24/L1  MISO=IO23/M2  CS#=IO12/J3   (C3 drives all but MISO)
#   READY=IO4/R1  -- FPGA->C3 BUSY/READY flow-control (the C3's "GPIO10").
_c3_io = [
    ("c3_spi", 0,
        # Pull-downs on the clocked inputs so a marginal/open contact reads LOW
        # (no spurious edges -> no rx flood) instead of floating and spewing
        # noise; CS pulls UP so a floating CS reads deasserted. Real drive from
        # the C3 overrides these weak pulls.
        Subsignal("sclk",  Pins("P3"), Misc("PULLMODE=DOWN")),  # IO27 <- C3 SCLK
        Subsignal("mosi",  Pins("L1"), Misc("PULLMODE=DOWN")),  # IO24 <- C3 MOSI
        Subsignal("miso",  Pins("M2")),                         # IO23 -> C3 MISO
        Subsignal("cs_n",  Pins("J3"), Misc("PULLMODE=UP")),    # IO12 <- C3 CS# (idle high)
        Subsignal("ready", Pins("R1")),                         # IO4  -> C3 BUSY/READY
        IOStandard("LVCMOS33"),
    ),
]


def add_c3_loader(soc):
    """ESP32-C3 SPI flash-loader link: the FPGA is the SPI SLAVE.

    Instantiates the SPISlave transport core (gateware/spi_slave.py) on the
    dedicated C3 bus. Firmware drains its RX FIFO, parses the C3 command stream,
    and drives the LiteSPI master to program flash -- so the caller must build
    BaseSoC with with_spi_flash=True + flash_master=True. The READY pad (IO4) is
    firmware-driven flow-control. No SDRAM dependency (the C3 stages the image).
    """
    platform = soc.platform
    platform.add_extension(_c3_io)
    soc.c3 = SPISlave(platform.request("c3_spi"))
    soc.add_csr("c3")


# C3 link bring-up diagnostic ---------------------------------------------------------------------
# The SPI link to the ESP32-C3 has "never worked" despite both sides looking
# healthy. This strips the link down to raw bidirectional GPIO on the *exact same
# five balls* as the c3 SPI bus (no shift register, no clock domain, no protocol)
# so firmware can drive any pin to any level and read any pin's actual voltage.
# It isolates wiring / pin damage (the 5V incident) / strapping-pin conflicts from
# SPI timing + protocol. Pins MUST match add_c3_loader's _c3_io exactly.
#   bit0=SCLK/P3  bit1=MOSI/L1  bit2=MISO/M2  bit3=CS#/J3  bit4=READY/R1
# No pull Misc() here on purpose: a diag must see the *undriven* level (float/short)
# without a weak pull masking it. Real drive from either end still dominates.
_c3_diag_io = [
    ("c3_diag", 0,
        Subsignal("pins", Pins("P3 L1 M2 J3 R1")),
        IOStandard("LVCMOS33"),
    ),
]


def add_c3_diag(soc):
    """Raw 5-bit tristate GPIO on the C3 SPI balls -- link continuity diagnostic.

    LiteX GPIOTristate exposes per-pin CSRs the firmware drives directly:
      c3diag_oe_write(mask)   1 = drive that pin (output), 0 = high-Z input.
      c3diag_out_write(mask)  output level per pin (when oe bit set).
      c3diag_in_read()        live readback of every pin's actual level.
    Bit order matches _c3_diag_io: 0=SCLK 1=MOSI 2=MISO 3=CS# 4=READY.
    """
    platform = soc.platform
    platform.add_extension(_c3_diag_io)
    soc.c3diag = GPIOTristate(platform.request("c3_diag").pins)
    soc.add_csr("c3diag")


# C3 link via SPIBone (verified Wishbone-over-SPI bridge) -----------------------------------------
# Post-diagnostic direction: the raw-GPIO diag (add_c3_diag) proved all 5 wires
# are electrically perfect both ways, so the "never worked" bug was in the custom
# SPISlave stack -- we replace that whole stack (custom slave + PING/ERASE/PROGRAM
# command protocol + READY pin) with LiteX's maintained SPIBone bridge (the core
# wishbone-tool speaks). The ESP32-C3 becomes a Wishbone master and reads/writes
# the SoC bus directly. 4-wire, CPOL0/CPHA0, big-endian, MSB-first; SCLK must be
# <= sys/4 (~12.5 MHz at 50 MHz sys). Same four balls as the c3 SPI bus; the READY
# pad (R1/IO4) is no longer needed (SPIBone signals busy in-band on MISO).
#   SCLK=P3/IO27  MOSI=L1/IO24  MISO=M2/IO23  CS#=J3/IO12   (C3 is the master)
_c3_spibone_io = [
    ("c3_spibone", 0,
        Subsignal("clk",  Pins("P3")),   # IO27 <- C3 SCLK
        Subsignal("mosi", Pins("L1")),   # IO24 <- C3 MOSI
        Subsignal("miso", Pins("M2")),   # IO23 -> C3 MISO (driven only while CS# low)
        Subsignal("cs_n", Pins("J3")),   # IO12 <- C3 CS# (active low)
        IOStandard("LVCMOS33"),
    ),
]


def add_c3_spibone(soc):
    """ESP32-C3 link as a Wishbone master via the verified SPIBone bridge.

    The C3 (SPI master) can then read/write the whole SoC bus -- CSRs, RAM, and
    (once flash_master is enabled) the LiteSPI master registers to program flash
    -- with no custom slave gateware and no command protocol. Added as a bus
    master; LiteX inserts an arbiter so it coexists with the CPU.
    """
    platform = soc.platform
    platform.add_extension(_c3_spibone_io)
    soc.c3_spibone = SPIBone(platform.request("c3_spibone"), wires=4)
    soc.bus.add_master(name="c3_spibone", master=soc.c3_spibone.bus)


# C3 link UART echo -- transport sanity check (is it SPI-specific, or dead?) -----------------------
# Two independent, well-tested SPI implementations (custom SPISlave + LiteX
# SPIBone) both failed with "MISO returns 00", while the raw-GPIO diag proved
# every wire good both ways. This tests a totally different protocol -- async
# UART -- with none of SPI's traps (no CS, no clock phase, no MISO tristate, no
# bus-master framing). Pure-gateware echo (RX stream -> FIFO -> TX stream): no
# CPU, no firmware. If bytes echo, the channel + a real protocol work and the bug
# is SPI-specific; if not, something deeper is wrong.
#
# Reuses the existing MOSI/MISO wires as the UART pair (NO rewiring):
#   RX = L1/IO24  <- C3 TX (GPIO1, the old MOSI wire)
#   TX = M2/IO23  -> C3 RX (GPIO3, the old MISO wire)
# RX pulls UP so it idles high (UART idle) before the C3 drives it -> no boot glitch.
_c3_uart_io = [
    ("c3_uart", 0,
        Subsignal("rx", Pins("L1"), Misc("PULLMODE=UP")),  # <- C3 TX (GPIO1)
        Subsignal("tx", Pins("M2")),                       # -> C3 RX (GPIO3)
        IOStandard("LVCMOS33"),
    ),
]


def add_c3_uart_echo(soc, baudrate=115200):
    """Pure-gateware UART loopback on the C3 link (no CPU/firmware).

    Instantiates an RS232 PHY on the reused MOSI/MISO wires and echoes every
    received byte straight back through a small FIFO. The ESP32-C3 sends bytes on
    its UART and checks they come back -- a transport sanity test independent of
    the (twice-failed) SPI path.
    """
    from litex.soc.cores.uart import RS232PHY
    from litex.soc.interconnect import stream
    soc.platform.add_extension(_c3_uart_io)
    pads = soc.platform.request("c3_uart")
    soc.submodules.c3_uart_phy = phy = RS232PHY(pads, soc.sys_clk_freq, baudrate)
    soc.submodules.c3_uart_fifo = fifo = stream.SyncFIFO([("data", 8)], 16)
    soc.comb += [
        phy.source.connect(fifo.sink),   # RX -> FIFO
        fifo.source.connect(phy.sink),   # FIFO -> TX  (echo)
    ]


# C3 link via UARTBone (verified Wishbone-over-UART bridge) ----------------------------------------
# The async twin of add_c3_spibone: LiteX's UARTBone is the most battle-tested
# transport in the tree (it's how litex_server/wishbone-tool talk over serial).
# After BOTH SPI paths failed with "MISO returns 00", this makes the ESP32-C3 a
# Wishbone master over plain UART -- no CS, no clock phase, no tristate, no
# firmware. Same loader goal (C3 -> WB master -> flash) on a non-SPI transport.
#
# Reuses the MOSI/MISO wires as the UART pair (NO rewiring), same as the echo:
#   rx = L1/IO24 <- C3 TX (GPIO1)     tx = M2/IO23 -> C3 RX (GPIO3)
# Protocol (software/c3_uartbone_esp mirrors it), all over 8N1 UART:
#   READ : 0x02, len(words), addr[WORD, big-endian] ; FPGA returns len*4 data bytes (BE).
#   WRITE: 0x01, len(words), addr[WORD, big-endian], data[4 BE per word] ; no reply.
# NOTE the address is a *word* address (byte_addr >> 2) -- Stream2Wishbone drives
# wishbone.adr directly (unlike SPIBone, which shifted internally).
_c3_uartbone_io = [
    ("c3_uartbone", 0,
        Subsignal("rx", Pins("L1"), Misc("PULLMODE=UP")),  # <- C3 TX (GPIO1)
        Subsignal("tx", Pins("M2")),                       # -> C3 RX (GPIO3)
        IOStandard("LVCMOS33"),
    ),
]


def add_c3_uartbone(soc, baudrate=115200):
    """ESP32-C3 as a Wishbone master via the verified UARTBone bridge (no firmware)."""
    soc.platform.add_extension(_c3_uartbone_io)
    soc.add_uartbone(name="c3_uartbone", uart_name="c3_uartbone", baudrate=baudrate)


def add_c3_mailbox(soc, origin=0x90000000, size=0x400):
    """Dedicated on-chip RAM slave used as the C3<->firmware loader mailbox.

    Both the SPIBone master (C3) and the CPU (loader firmware) access it as plain
    memory -- no CSRs, no clobber risk (separate from main_ram where the firmware
    keeps its stack/data). Layout is a convention shared by software/c3_flash
    (FPGA side) and software/c3_flash_esp (C3 side):
        +0x00 cmd (doorbell; 0=idle/done)  +0x04 arg0  +0x08 arg1
        +0x0C status                       +0x10 result   +0x40.. data[256]
    The C3 writes args+data then cmd last; the firmware processes, writes
    result+status, then clears cmd to 0 to signal completion.

    UNCACHED on purpose: it is shared between two bus masters (the CPU running the
    firmware, and the SPIBone master driven by the C3). A cached region would let
    the VexRiscv D-cache hold the firmware's writes so the C3 sees stale RAM (and
    vice-versa). cached=False makes every access hit the real BRAM -> coherent.
    LiteX only permits uncached regions inside the IO space, so it lives at
    0x90000000 (the CPU treats IO-region accesses as uncached).
    """
    from litex.soc.interconnect.wishbone import SRAM
    mailbox = SRAM(size)
    soc.submodules.c3_mailbox = mailbox
    soc.bus.add_slave("mailbox", mailbox.bus,
                      SoCRegion(origin=origin, size=size, cached=False))


def add_c3_uart_beacon(soc, baudrate=1200):
    """One-way UART beacon: FPGA continuously transmits an incrementing byte on M2.

    Direction-isolation test. Every C3->FPGA transport has failed identically
    (FPGA never acts on what the C3 sends); this tests the OTHER direction alone.
    No RX, no CPU, no firmware -- if the C3 (listening on GPIO3, proven good)
    sees a clean 00,01,02,... stream, then FPGA->C3 works and the fault is the
    C3->FPGA direction. Reuses the same M2/L1 pads as the echo (tx=M2 used; rx
    drained). See software/c3_uart_listen_esp.
    """
    from litex.soc.cores.uart import RS232PHY
    soc.platform.add_extension(_c3_uart_io)
    pads = soc.platform.request("c3_uart")
    soc.submodules.c3_uart_phy = phy = RS232PHY(pads, soc.sys_clk_freq, baudrate)
    counter = Signal(8)
    soc.comb += [
        phy.sink.valid.eq(1),            # always have a byte to send
        phy.sink.data.eq(counter),
        phy.source.ready.eq(1),          # drain RX (unused)
    ]
    soc.sync += If(phy.sink.valid & phy.sink.ready, counter.eq(counter + 1))


def add_c3_uart_rxreport(soc, baudrate=1200):
    """RX-report: FPGA reflects the last byte it received on L1 back out M2.

    The beacon proved FPGA->C3 works; this gives us eyes on the RECEIVE side
    (C3->FPGA on L1), the direction every transport failed on. The FPGA captures
    each byte arriving on its RX and continuously re-transmits that last byte on
    TX (the proven-good M2->C3 path). No CPU/firmware. The C3 sends a known value
    and checks what comes back:
      reflects the sent value  -> FPGA IS receiving on L1 correctly (link fine;
                                  bug was in protocol framing).
      reflects 0x00 forever    -> FPGA receives nothing on L1 (dead RX pin/input).
      reflects garbage         -> receiving, but mis-framed (baud/clock issue).
    """
    from litex.soc.cores.uart import RS232PHY
    soc.platform.add_extension(_c3_uart_io)
    pads = soc.platform.request("c3_uart")
    soc.submodules.c3_uart_phy = phy = RS232PHY(pads, soc.sys_clk_freq, baudrate)
    last_byte = Signal(8)
    soc.comb += [
        phy.source.ready.eq(1),          # accept every received byte
        phy.sink.valid.eq(1),            # always transmitting
        phy.sink.data.eq(last_byte),     # ...the last byte we received
    ]
    soc.sync += If(phy.source.valid & phy.source.ready,
                   last_byte.eq(phy.source.data))


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


class BootFlag(LiteXModule, AutoCSR):
    """Sticky one-shot boot-to-app request flag -- the CSR half of BootCtl,
    with none of its FTDI coupling. reset_less: survives a soft/ext reset
    (e.g. the C3's GPIO7 pulse), 0 only at power-on/reconfigure. Deliberately
    has no reset_request/ftdi_sense wiring -- unlike add_boot_ctl(), this does
    NOT claim L15/L16 or feed crg.user_rst, so a link with its own dedicated
    reset line (docs/c3_loader.md) doesn't grow a second, surprising reset
    path off a stray FTDI DTR/RTS toggle."""
    def __init__(self):
        self.flag = CSRStorage(32, reset_less=True,
            description="Boot-to-app request magic. Survives soft reset; 0 at power-on.")


def add_boot_flag(soc):
    soc.boot_ctl = BootFlag()
    soc.add_csr("boot_ctl")


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


# Deployable baseline ------------------------------------------------------------------------------
# The shared aux SPI bus + the boot-manager flag/reset: the common machinery
# every DEPLOYABLE top composes. The IMU (cs[1]) and MCP3008 (cs[2]) ride the
# aux bus for free, so a logger needs no extra SPI block.
#
# NOTE (post-WINC, 2026-06-28): the ATWINC1500 was removed (smoked); this no
# longer instantiates the WINC sidebands, and flash.py's WiFi OTA path is dark
# until the ESP32-C3 loader (on separate free SPI pins) lands. Until then, load
# over UART (litex_term --kernel) or bit-banged JTAG. The aux bus cs[0] (old
# WINC line) parks unused. with_spi_flash=True + flash_master=True (XIP BIOS +
# LiteSPI master) is still the deployment shape so the flash slot map survives
# for the eventual C3 OTA. See docs/boot_chain.md and the winc-archive branch.
#
# Caller contract: construct BaseSoC with with_spi_flash=True and
# flash_master=True; this adds the aux bus + boot_ctl. busy_led mirrors aux-bus
# traffic -- pass an LED index NOT used by another block (the SNN takes 0,1, so
# pass 2 in builds that include the SNN).
def add_flashing_baseline(soc, aux_spi_clk_freq=12.5e6, busy_led=0):
    """Aux SPI bus (IMU cs[1] / MCP3008 cs[2]) + boot_ctl: baseline for deployables."""
    add_aux_imu(soc, imu_spi_clk_freq=aux_spi_clk_freq, busy_led=busy_led)
    add_boot_ctl(soc)
