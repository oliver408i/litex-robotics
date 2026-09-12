#!/usr/bin/env python3
"""IcePi Zero SoC -- USB: a device-mode USB scaffold (UNTESTED on hardware).

Purpose: bring up a **USB device** interface via ValentyUSB so the board
enumerates on a host PC. This first cut wires a USB-CDC ACM port as a *secondary*
UART (a `/dev/ttyACM*` on the host) while leaving the primary serial UART intact
for the C3 loader / console. See docs/usb_device.md for the full plan.

Clocking: ValentyUSB's CDCUsb needs BOTH a 48 MHz (4x full-speed oversample) and
a 12 MHz (bit-rate) clock domain, and it does its OWN sys<->usb clock-domain
crossing internally (AsyncFIFOs). So sys is unconstrained by USB -- we keep the
proven sys = 100 MHz / half-rate SDRAM = 200 MHz on PLL1, and put the USB 48/12
domains on PLL2 (one 480 MHz VCO -> 48 = /10, 12 = /40). The 25F has only 2 PLLs,
so PLL2 is either LCD *or* USB -- this variant has no LCD.

Shape: BaseSoC (CPU + SDRAM) + XIP flash + C3 flash loader (so the board stays
programmable) + a secondary USB-CDC ACM UART. No LCD, no IO expander.

NOT HARDWARE-VALIDATED. A produced bitstream is only a routing / Fmax / capacity
data point until the physical USB port (D+ F15 / D- E16 + series R + D+ pullup)
is confirmed and enumeration is observed on a host.
"""
from migen import ClockDomainsRenamer

from base import BaseSoC, make_parser, run_build

from gateware.soc_features import add_c3_loader_baseline


class UsbSoC(BaseSoC):
    def __init__(self, sys_clk_freq=100e6, flash_boot_offset=None,
                 sdram_rate="1:2", **kwargs):
        super().__init__(
            sys_clk_freq      = sys_clk_freq,
            with_spi_flash    = True,   # XIP BIOS ...
            flash_master      = True,   # ... + LiteSPI master for the C3 loader
            flash_boot_offset = flash_boot_offset,
            sdram_rate        = sdram_rate,
            with_usb          = True,   # PLL2 -> cd_usb_48 (48 MHz) + cd_usb_12 (12 MHz)
            **kwargs,
        )
        # C3 flash loader so the board is programmable (BIOS -> loader @0x200000
        # -> chain-boot app @0x280000). Keeps the primary serial UART.
        add_c3_loader_baseline(self)

        self.add_usb_acm()

    def add_usb_acm(self, name="usb"):
        """Secondary USB-CDC ACM port via ValentyUSB's CDCUsb (eptri).

        CDCUsb runs its PHY in cd_usb_12/cd_usb_48 (provided by the CRG) and
        exposes a UART-shaped CSR interface (rxtx/txfull/rxempty + tx/rx events)
        in the sys domain, crossing between them with its own AsyncFIFOs. We add
        it as an extra UART module (the primary "serial" UART stays the console).
        """
        import valentyusb.usbcore.io as usbio
        import valentyusb.usbcore.cpu.cdc_eptri as cdc_eptri

        usb_pads  = self.platform.request("usb")  # d_p F15 / d_n E16 / pullup
        usb_iobuf = usbio.IoBuf(usb_pads.d_p, usb_pads.d_n, usb_pads.pullup)
        # CDCUsb's sys-side logic sits in the SoC sys domain as-is. (LiteX's stock
        # usb_acm renames sys->sys_usb to dodge sys warm-reset; skipped here for
        # clarity -- a warm reset simply re-enumerates.)
        usb = cdc_eptri.CDCUsb(usb_iobuf,
                               product="IcePi Zero CDC",
                               manufacturer="IcePi")
        self.add_module(name=name, module=usb)
        if self.irq.enabled:
            self.irq.add(name, use_loc_if_exists=True)


def main():
    parser = make_parser(description="IcePi Zero SoC USB (device-mode ValentyUSB scaffold, UNTESTED).")
    parser.set_defaults(sys_clk_freq=100e6)
    parser.add_target_argument("--bios-in-bram", action="store_true",
                               help="Boot the BIOS from on-chip EBR ROM instead of XIP'ing it "
                                    "from SPI flash. Makes the bitstream self-contained for "
                                    "bring-up: no flash content needed, serial-boot the USB "
                                    "test firmware into SDRAM.")
    args = parser.parse_args()

    # Standalone boot by default: BIOS flash-boots the loader at the firmware
    # offset (0x200000), which chain-boots the app. Serial boot stays available.
    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = UsbSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        bios_in_bram      = args.bios_in_bram,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
