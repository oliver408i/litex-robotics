#!/usr/bin/env python3
"""IcePi Zero SoC for the LSM6DS3 IMU bring-up project.

UART-only diagnostic SoC: a single firmware-driven LiteX SPIMaster on the
second SPI bus (physically shared with the MCP3008 -- sclk/mosi/miso unchanged,
see docs/icepi_zero_pin_mapping.md). The IMU rides chip-select 0; the MCP3008's
chip-select is wired as cs[1] but parked deasserted (IMU-only bring-up), which
also keeps the ADC off the shared MISO line.
"""
from icepi_zero_base import BaseSoC, make_parser, resolve_spi_flash, run_build

from litex.build.generic_platform import Pins, Subsignal, IOStandard
from litex.soc.cores.spi import SPIMaster


# Second SPI bus -- shared lines with the MCP3008, plus a dedicated IMU CS.
#   clk  = T2  (IO2)   mosi = H2  (IO8)   miso = J2  (IO25)   -- MCP3008 bus
#   cs_n = "F3 R2": cs[0] = IMU (IO6 = F3), cs[1] = MCP3008 (R2, parked high)
_imu_io = [
    ("imu_spi", 0,
        Subsignal("clk",  Pins("T2")),
        Subsignal("mosi", Pins("H2")),
        Subsignal("miso", Pins("J2")),
        Subsignal("cs_n", Pins("F3 R2")),
        IOStandard("LVCMOS33"),
    ),
]


class ImuSoC(BaseSoC):
    def __init__(self, sys_clk_freq=50e6, with_spi_flash=False, flash_boot_offset=None,
                 imu_spi_clk_freq=2e6, **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = with_spi_flash,
            flash_boot_offset = flash_boot_offset,
            **kwargs,
        )

        self.platform.add_extension(_imu_io)
        pads = self.platform.request("imu_spi")

        # LSM6DS3 is fine in SPI mode 0 (SPIMaster default CPOL=0/CPHA=0).
        # data_width=8: byte-at-a-time; firmware uses manual CS mode to hold
        # CS low across multi-byte (auto-incrementing) register bursts.
        self.imu_spi = SPIMaster(
            pads,
            data_width   = 8,
            sys_clk_freq = sys_clk_freq,
            spi_clk_freq = int(imu_spi_clk_freq),
            with_csr     = True,
        )
        self.add_csr("imu_spi")
        self.add_constant("IMU_SPI_FREQUENCY", int(imu_spi_clk_freq))

        # Sanity LED: lit while an SPI transfer is in flight.
        try:
            self.comb += self.platform.request("user_led", 0).eq(~self.imu_spi.done)
        except Exception:
            pass


def main():
    parser = make_parser(description="IcePi Zero SoC + LSM6DS3 IMU (SPI bring-up).")
    parser.add_target_argument("--imu-spi-clk-freq", default=2e6, type=float,
                               help="IMU SPI SCK frequency (Hz). Default 2 MHz "
                                    "(LSM6DS3 max is 10 MHz).")
    args = parser.parse_args()

    soc = ImuSoC(
        sys_clk_freq      = args.sys_clk_freq,
        with_spi_flash    = resolve_spi_flash(args),
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        imu_spi_clk_freq  = args.imu_spi_clk_freq,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
