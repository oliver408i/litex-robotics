#!/usr/bin/env python3
"""IcePi Zero SoC for the ATWINC1500 Wi-Fi bring-up project.

UART-only diagnostic SoC that puts the ATWINC1500 on the shared sensor/aux SPI
bus (same sclk/mosi/miso as the LSM6DS3 IMU + MCP3008, see
docs/icepi_zero_pin_mapping.md), driven by `gateware/aux_spi.py`'s AuxSPIMaster.
That master has a runtime SCLK divider and software-held chip-selects, so the
WINC, IMU and ADC can each run at their own clock on one bus.

The WINC also needs three sidebands on free GPIO: RESET_N and CHIP_EN (outputs)
and IRQN (input, active-low, wired into the interrupt controller). GPIOOut
defaults to 0, so at power-on RESET_N=0 (held in reset) and CHIP_EN=0
(disabled) -- a safe state the firmware releases during nm_bsp_reset().

This is the hardware half of the bring-up; pair it with software/winc_test.
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from migen import Cat
from litex.build.generic_platform import Pins, Subsignal, IOStandard, Misc
from litex.soc.cores.gpio import GPIOIn, GPIOOut

from gateware.aux_spi import AuxSPIMaster


# Shared sensor/aux SPI bus + WINC sidebands. The bus lines match the IMU bus
# (IO2/IO8/IO25); chip-selects are ordered WINC, IMU, MCP3008 so that no device
# CS floats in this build (the WINC test only drives WINC's, others stay high).
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


class WincSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None,
                 flash_master=False, winc_spi_clk_freq=1e6, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = with_spi_flash,
            flash_boot_offset = flash_boot_offset,
            flash_master      = flash_master,
            **kwargs,
        )

        self.platform.add_extension(_winc_io)

        # --- Shared aux SPI bus master (runtime divider, software-held CS) ---
        self.aux_spi = AuxSPIMaster(
            pads                 = self.platform.request("aux_spi"),
            platform             = self.platform,
            sys_clk_freq         = sys_clk_freq,
            default_spi_clk_freq = int(winc_spi_clk_freq),
        )
        self.add_csr("aux_spi")

        # --- WINC sidebands: RESET_N + CHIP_EN (out), IRQN (in, w/ IRQ) ------
        ctrl = self.platform.request("winc_ctrl")
        self.winc_reset = GPIOOut(ctrl.reset_n)   # write 0 = assert reset
        self.winc_en    = GPIOOut(ctrl.chip_en)   # write 1 = enable chip
        self.winc_irq   = GPIOIn(ctrl.irq_n, with_irq=True)
        # NOTE: if a build reports these CSRs are already registered, this LiteX
        # version auto-collects submodule CSRs -- drop the three add_csr() calls.
        self.add_csr("winc_reset")
        self.add_csr("winc_en")
        self.add_csr("winc_irq")
        self.irq.add("winc_irq", use_loc_if_exists=True)

        self.add_constant("WINC_SPI_DEFAULT_FREQUENCY", int(winc_spi_clk_freq))
        self.add_constant("AUX_CS_WINC", AUX_CS_WINC)
        self.add_constant("AUX_CS_IMU",  AUX_CS_IMU)
        self.add_constant("AUX_CS_MCP",  AUX_CS_MCP)

        # Sanity LED: lit while an aux-bus SPI transfer is in flight.
        try:
            self.comb += self.platform.request("user_led", 0).eq(self.aux_spi.busy)
        except Exception:
            pass


def main():
    parser = make_parser(description="IcePi Zero SoC + ATWINC1500 (Wi-Fi SPI bring-up).")
    parser.add_target_argument("--winc-spi-clk-freq", default=1e6, type=float,
                               help="Initial WINC SPI SCK frequency (Hz). Default 1 MHz "
                                    "(conservative for bring-up; WINC supports up to ~48 MHz, "
                                    "this bus is capped at sys/2 = 25 MHz). Runtime-adjustable "
                                    "via the aux_spi clk_divider CSR.")
    parser.add_target_argument("--flash-master", action="store_true",
                               help="Expose the LiteSPI master CSRs so SDRAM-resident firmware "
                                    "(the WiFi flash-loader) can erase/program the SPI flash. "
                                    "Implies the XIP flash build; the BIOS skips master init "
                                    "(SPIFLASH_SKIP_MASTER_INIT). See docs/xip_bios.md.")
    args = parser.parse_args()

    soc = WincSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args) or args.flash_master,
        flash_boot_offset = args.flash_boot_offset,
        flash_master      = args.flash_master,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        winc_spi_clk_freq = args.winc_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
