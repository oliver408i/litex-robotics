#!/usr/bin/env python3
"""IcePi Zero SoC -- TURBO: fast, experimental, gaming-only. DO NOT USE IN THE FIELD.

This variant exists to be overclocked. It is the leanest *programmable* SoC that
can run a framebuffer game (DOOM port):

  - CPU + SDRAM (BaseSoC)
  - ST7796S LCD engine + FT6336U capacitive touch (add_lcd_touch, reset via IOX)
  - MCP23S17 IO expander (add_aux_imu with_iox) -- ALWAYS required: it drives
    LCD/CTP reset over the aux bus. Mostly a software feature; tiny gateware.
  - ESP32-C3 flash loader baseline (add_c3_loader_baseline) -- required so the
    board can actually be programmed (C3-reflashable, boots via the boot manager:
    BIOS -> loader @0x200000 -> chain-boot app @0x280000).

Dropped vs the deployable mnist_lcd shape: only the SNN-MLP peripheral (DOOM
doesn't need it), freeing its ~78% BRAM + logic as headroom for a future
pixel/palette accelerator.

The default sys_clk_freq is pushed WELL above the conservative 50 MHz baseline
so nextpnr optimizes the design hard and the reported Fmax reflects the true
ceiling of this shape. Timing is expected to be tight or failing; that is the
point. Builds rely on `--timing-allow-fail`, so a produced bitstream does NOT
mean timing closed -- always read the reported "Max frequency for clock 'sys'"
and validate empirically (BIOS memtest) before trusting a clock. The half-rate
SDRAM couples mem = 2*sys, so raising sys raises the memory clock too (sys=83 ->
166 MHz mem = the W9825G6KH6-6 rated ceiling).

Reliability across temperature / voltage / part-to-part is explicitly a
non-goal. This is a bench stunt. Do not ship it.
"""
from icepi_zero_base import BaseSoC, make_parser, run_build

from gateware.soc_features import (add_lcd_touch, add_aux_imu,
                                    add_c3_loader_baseline)


class TurboSoC(BaseSoC):
    def __init__(self, sys_clk_freq=75e6, flash_boot_offset=None,
                 lcd_spi_clk_freq=185e6, aux_spi_clk_freq=12.5e6,
                 sdram_rate="1:2", bios_in_bram=False, **kwargs):
        super().__init__(
            sys_clk_freq             = sys_clk_freq,
            with_spi_flash           = True,   # deployment shape: XIP BIOS...
            flash_master             = True,   # ...+ LiteSPI master for the C3 loader
            flash_boot_offset        = flash_boot_offset,
            spi_clk_freq             = lcd_spi_clk_freq,
            force_lcd_backlight_off  = False,  # lcd_ctrl owns P1
            sdram_rate               = sdram_rate,
            bios_in_bram             = bios_in_bram,  # opt: BIOS in EBR ROM, not XIP
            **kwargs,
        )
        # LCD/CTP reset is driven via the MCP23S17 expander (GPB0/GPB1), so drop
        # the direct LCD reset pad to free IO10/L2 for the expander's own RESET.
        add_lcd_touch(self, lcd_spi_clk_freq, with_reset_pad=False)
        # IOX on the shared aux bus. for_c3=True drops the dead WINC cs[0] so M2
        # is free for the C3 SPIBone MISO (mandatory alongside the C3 loader).
        add_aux_imu(self, imu_spi_clk_freq=aux_spi_clk_freq, busy_led=0,
                    with_iox=True, for_c3=True)
        # C3 flash loader so the board is programmable over USB (no SNN here).
        add_c3_loader_baseline(self)


def main():
    parser = make_parser(description="IcePi Zero SoC TURBO (overclock/gaming, not for field use).")
    parser.add_target_argument("--lcd-spi-clk-freq", default=185e6, type=float,
                               help="LCD engine SPI core clock (Hz). SCK = this / 2. "
                                    "Default 185 MHz -> 92.5 MHz SCK (production-proven).")
    parser.add_target_argument("--aux-spi-clk-freq", default=12.5e6, type=float,
                               help="Aux-bus default SCK frequency (Hz). Runtime-adjustable "
                                    "via the aux_spi clk_divider CSR.")
    parser.add_target_argument("--sdram-1to1", action="store_true",
                               help="Use a full-rate (1:1) SDRAM PHY: SDRAM clock == sys "
                                    "instead of the default half-rate (1:2) where SDRAM "
                                    "clock == 2*sys. Decouples the memory clock from sys so "
                                    "sys can be overclocked past 83 MHz without pushing the "
                                    "SDRAM past its 166 MHz rating (at the cost of ~half the "
                                    "per-clock SDRAM bandwidth). Untested phase; validate with "
                                    "a BIOS memtest.")
    parser.add_target_argument("--bios-in-bram", action="store_true",
                               help="Boot the BIOS from on-chip EBR ROM instead of XIP'ing "
                                    "it from SPI flash. The flash peripheral stays (master + "
                                    "app slot @0x280000), only the BIOS moves to BRAM. Removes "
                                    "the flash fetch path from the reset critical path (useful "
                                    "when overclocking makes XIP flaky) at the cost of EBR and "
                                    "a rebuilt bitstream per BIOS change.")
    args = parser.parse_args()

    # Standalone boot by default: BIOS flash-boots the loader at the firmware
    # offset (0x200000), which chain-boots the app. Serial boot stays available.
    if args.flash_boot_offset is None:
        args.flash_boot_offset = args.firmware_offset

    soc = TurboSoC(
        sys_clk_freq      = args.sys_clk_freq,
        flash_boot_offset = args.flash_boot_offset,
        bios_flash_offset = args.bios_flash_offset,
        spiflash_1x       = args.spiflash_1x,
        spiflash_clk_freq = args.spiflash_clk_freq,
        lcd_spi_clk_freq  = args.lcd_spi_clk_freq,
        aux_spi_clk_freq  = args.aux_spi_clk_freq,
        sdram_rate        = "1:1" if args.sdram_1to1 else "1:2",
        bios_in_bram      = args.bios_in_bram,
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
