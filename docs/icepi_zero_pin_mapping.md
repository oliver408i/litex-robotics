# IcePi Zero Pin Mapping

This note is derived from the LiteX board platform file at
`litex-setup/litex-boards/litex_boards/platforms/icepi_zero.py`.

It is a board/platform mapping reference, not a record of which pins this
project currently uses.

## Hardware Peripherals Overview

These are the main board-level peripherals exposed by the LiteX platform file.

| Peripheral | LiteX resource | Signal(s) | FPGA pin(s) | Notes |
| --- | --- | --- | --- | --- |
| 50 MHz clock | `clk50` | clock input | `M1` | Main input clock |
| Reset input | `rst` | reset | `C4` | Also overlaps with `user_btn[0]` in the platform file |
| External reset | `ext_reset` | active-low reset | `G3` | Pulled up |
| UART | `serial` | `tx`, `rx` | `K15`, `K16` | BIOS UART used by the minimal SoC |
| SDR SDRAM | `sdram`, `sdram_clock` | addr/data/control | `A3`, `A4-A15`, `B1-B16`, `C1-C3`, `C14-C16`, `D1`, `D16`, `E2` | External SDRAM interface |
| General GPIO header/bank | `gpio` | 27 GPIOs | `K3 T2 R2 R1 E1 F3 G1 H2 J1 L2 G2 J3 E3 P1 N1 H3 R3 N4 E4 F1 F2 P2 M2 L1 J2 D4 P3` | Several named peripherals are aliases on these pins |
| RGB / NeoPixel data | `rgb_led` | data | `K3` | Alias on `gpio[0]` |
| SPI ADC | `mcp3008` | `sclk`, `cs_n`, `mosi`, `miso` | `T2`, `R2`, `H2`, `J2` | Alias on GPIO pins |
| USB port 0 | `usb[0]` | `d_p`, `d_n`, `pullup` | `F15`, `E16`, `G15 H14` | USB PHY pins |
| USB port 1 | `usb[1]` | `d_p`, `d_n`, `pullup` | `J16`, `J15`, `E14 E11` | USB PHY pins |
| SPI flash | `spiflash`, `spiflash4x` | flash bus | `N8`, `T7`, `T8`, `M7`, `N7`, `N9` | `N9` is noted as flash clock |
| SD card | `spisdcard`, `sdcard` | SPI/SDIO signals | `P15`, `N16`, `M14`, `P14`, `R14`, `M15` | Same pins exposed in different modes |
| GPDI / HDMI-style video | `gpdi` | TMDS/control | `R12`, `R13`, `R15`, `P16`, `R5`, `T3`, `T4`, `L14`, `P5` | Video output pins |
| Buttons | `user_btn` | button inputs | `C4`, `C5` | `user_btn[0]` shares `rst` pin |
| User LEDs | `user_led` | LED outputs | `E13`, `D14`, `E12`, `C13`, `D13` | Five discrete LEDs |

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

The platform exposes a 27-pin `gpio` resource:

`K3 T2 R2 R1 E1 F3 G1 H2 J1 L2 G2 J3 E3 P1 N1 H3 R3 N4 E4 F1 F2 P2 M2 L1 J2 D4 P3`

Indexed by position, that is:

| GPIO index | FPGA pin | Notes |
| --- | --- | --- |
| `gpio[0]` | `K3` | Also exposed as `rgb_led` |
| `gpio[1]` | `T2` | Used by `mcp3008.sclk` |
| `gpio[2]` | `R2` | Used by `mcp3008.cs_n` |
| `gpio[3]` | `R1` | General GPIO |
| `gpio[4]` | `E1` | General GPIO |
| `gpio[5]` | `F3` | General GPIO |
| `gpio[6]` | `G1` | General GPIO |
| `gpio[7]` | `H2` | Used by `mcp3008.mosi` |
| `gpio[8]` | `J1` | General GPIO |
| `gpio[9]` | `L2` | General GPIO |
| `gpio[10]` | `G2` | General GPIO |
| `gpio[11]` | `J3` | General GPIO |
| `gpio[12]` | `E3` | General GPIO |
| `gpio[13]` | `P1` | General GPIO |
| `gpio[14]` | `N1` | General GPIO |
| `gpio[15]` | `H3` | General GPIO |
| `gpio[16]` | `R3` | General GPIO |
| `gpio[17]` | `N4` | General GPIO |
| `gpio[18]` | `E4` | General GPIO |
| `gpio[19]` | `F1` | General GPIO |
| `gpio[20]` | `F2` | General GPIO |
| `gpio[21]` | `P2` | General GPIO |
| `gpio[22]` | `M2` | General GPIO |
| `gpio[23]` | `L1` | General GPIO |
| `gpio[24]` | `J2` | Used by `mcp3008.miso` |
| `gpio[25]` | `D4` | General GPIO |
| `gpio[26]` | `P3` | General GPIO |

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

## Archived Project Wiring

These mappings come from the earlier project wiring that used this board.
They are preserved here as reference only and are not part of the current
minimal `icepi_zero.py`.

### HC-05 UART

| Function | FPGA pin | Notes |
| --- | --- | --- |
| HC-05 TX from FPGA | `N1` | FPGA UART TX to HC-05 RXD |
| HC-05 RX to FPGA | `P1` | FPGA UART RX from HC-05 TXD |
| HC-05 enable / key | `N4` | Module enable / AT-mode style control |

### Shared SPI Bus

The old project used one shared SPI clock/data bus for multiple peripherals:

| Signal | FPGA pin | Notes |
| --- | --- | --- |
| `sclk` | `T2` | Shared SPI clock |
| `mosi` | `H2` | Shared controller-out data |
| `miso` | `J2` | Shared controller-in data |

### SPI Device: MCP3008 ADC

| Signal | FPGA pin | Notes |
| --- | --- | --- |
| `cs_n` | `R2` | Chip select for the MCP3008 on the shared SPI bus |

### SPI Device: LR1121 Radio

| Signal | FPGA pin | Notes |
| --- | --- | --- |
| `cs_n` | `R1` | Chip select for the LR1121 on the shared SPI bus |
| `busy` | `J3` | LR1121 busy status |
| `dio9` | `H3` | LR1121 DIO9 status |
| `reset_n` | `D4` | LR1121 reset, active low |
