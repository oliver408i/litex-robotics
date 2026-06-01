# IcePi Zero Pin Mapping

This note is derived from the LiteX board platform file at
`litex-setup/litex-boards/litex_boards/platforms/icepi_zero.py`.

It is a board/platform mapping reference, not a record of which pins this
project currently uses.

## Minimal SoC Pins

The current minimal `icepi_zero.py` uses only these board resources:

| Resource | Signal | FPGA Pin | Notes |
| --- | --- | --- | --- |
| `clk50` | clock input | `M1` | 50 MHz input clock |
| `rst` | reset input | `C4` | Pulled up in platform file |
| `ext_reset` | external reset, active-low | `G3` | Pulled up in platform file |
| `serial` | `tx` | `K15` | LiteX BIOS UART transmit |
| `serial` | `rx` | `K16` | LiteX BIOS UART receive |

## Buttons

| Resource | Index | FPGA Pin | Notes |
| --- | --- | --- | --- |
| `user_btn` | 0 | `C4` | Shares the same pin as `rst` in the platform file |
| `user_btn` | 1 | `C5` | Pulled up |

## User LEDs

| Resource | Index | FPGA Pin |
| --- | --- | --- |
| `user_led` | 0 | `E13` |
| `user_led` | 1 | `D14` |
| `user_led` | 2 | `E12` |
| `user_led` | 3 | `C13` |
| `user_led` | 4 | `D13` |

## SDR SDRAM

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `sdram_clock` | clock | `A3` |
| `sdram` | `a[0:12]` | `B10 A9 B9 A8 B8 A7 B7 A6 B6 A5 A10 B5 A4` |
| `sdram` | `dq[0:15]` | `B16 C14 C16 C15 D16 A15 B15 A14 A2 B2 E2 D1 C2 C1 C3 B1` |
| `sdram` | `we_n` | `A13` |
| `sdram` | `ras_n` | `A12` |
| `sdram` | `cas_n` | `B13` |
| `sdram` | `cs_n` | `B12` |
| `sdram` | `cke` | `B4` |
| `sdram` | `ba[0:1]` | `A11 B11` |
| `sdram` | `dm[0:1]` | `B14 B3` |

## GPIO Bank

The platform exposes a 28-pin `gpio` resource:

`K3 T2 R2 R1 E1 F3 G1 H2 J1 L2 G2 J3 E3 P1 N1 H3 R3 N4 E4 F1 F2 P2 M2 L1 J2 D4 P3`

Physical pins from this bank are labeled as `IOn` on the board, with `IO1` equal to `K3`, `IO2` equal to `T2` and so on.

## Named Peripheral Mappings

These resources are aliases or grouped views built on top of the platform pins:

### RGB LED / NeoPixel

| Resource | Signal | FPGA Pin |
| --- | --- | --- |
| `rgb_led` | data | `K3` |

### MCP3008

| Resource | Signal | FPGA Pin |
| --- | --- | --- |
| `mcp3008` | `sclk` | `T2` |
| `mcp3008` | `cs_n` | `R2` |
| `mcp3008` | `mosi` | `H2` |
| `mcp3008` | `miso` | `J2` |

### LCD / Touch Project Mapping

This project uses an ST7796S SPI LCD controller with a 320x480 panel and an
FT6336U capacitive touch controller. The first bring-up target is an LCD test
pattern. The eventual software target is LVGL running on the LiteX CPU.

The LCD SPI bus now has its own dedicated pins (no longer shared with the
MCP3008 ADC). `CTP_INT` was also relocated because its old pin (`E3`) collided
with the new SPI routing — `E3` is now reused by `LCD_MISO`.

| Device | Signal | FPGA Pin | Board IO | Notes |
| --- | --- | --- | --- | --- |
| LCD SPI | `sclk` | `E4` | `IO19` | `LCD_SCK` |
| LCD SPI | `mosi` | `D4` | `IO26` | `LCD_MOSI` |
| LCD SPI | `miso` | `E3` | `IO13` | `LCD_MISO`; not required for basic LCD writes |
| ST7796S LCD | `cs_n[0]` | `H3` | — | `LCD_CS` |
| ST7796S LCD | `reset` | `J3` | — | `LCD_RST` |
| ST7796S LCD | `dc` / `rs` | `G1` | — | `LCD_RS`; command/data select |
| LCD backlight | `led` | `P1` | — | `backlightLED` |
| FT6336U touch | `scl` | `N1` | — | `CTP_SCL` |
| FT6336U touch | `sda` | `N4` | — | `CTP_SDA` |
| FT6336U touch | `int` | `F2` | `IO21` | `CTP_INT` (moved from `E3`) |

#### Original shared SPI bus pins (kept for reference)

Before the move, the LCD SPI clock/data pins were shared with the MCP3008 SPI
bus. Retained here because these pins will be reused soon.

| Device | Signal | FPGA Pin | Notes |
| --- | --- | --- | --- |
| shared SPI | `sclk` | `T2` | Shared with `mcp3008.sclk` |
| shared SPI | `mosi` | `H2` | Shared with `mcp3008.mosi` |
| shared SPI | `miso` | `J2` | Shared with `mcp3008.miso` |
| MCP3008 | `cs_n[1]` | `R2` | Used only as a shared-SPI bus test in this LCD bring-up |
| FT6336U touch | `int` | `E3` | `CTP_INT` original pin |

### USB 0

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `usb` | `d_p` | `F15` |
| `usb` | `d_n` | `E16` |
| `usb` | `pullup` | `G15 H14` |

### USB 1

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `usb` | `d_p` | `J16` |
| `usb` | `d_n` | `J15` |
| `usb` | `pullup` | `E14 E11` |

### SPI Flash

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `spiflash` | `cs_n` | `N8` |
| `spiflash` | `miso` | `T7` |
| `spiflash` | `mosi` | `T8` |
| `spiflash` | `wp` | `M7` |
| `spiflash` | `hold` | `N7` |
| `spiflash4x` | `cs_n` | `N8` |
| `spiflash4x` | `dq[0:3]` | `T8 T7 M7 N7` |
| board note | flash clock | `N9` |

### SD Card

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `spisdcard` | `clk` | `P15` |
| `spisdcard` | `mosi` | `N16` |
| `spisdcard` | `cs_n` | `M14` |
| `spisdcard` | `miso` | `P14` |
| `sdcard` | `clk` | `P15` |
| `sdcard` | `cmd` | `N16` |
| `sdcard` | `data[0:3]` | `P14 R14 M15 M14` |

### GPDI

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `gpdi` | `clk_p` | `R12` |
| `gpdi` | `data0_p` | `R13` |
| `gpdi` | `data1_p` | `R15` |
| `gpdi` | `data2_p` | `P16` |
| `gpdi` | `cec` | `R5` |
| `gpdi` | `scl` | `T3` |
| `gpdi` | `sda` | `T4` |
| `gpdi` | `hpd` | `L14` |
| `gpdi` | `util` | `P5` |
