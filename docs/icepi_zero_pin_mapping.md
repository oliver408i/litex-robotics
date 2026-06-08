# IcePi Zero Pin Mapping

Derived from the LiteX board platform file at
`litex-setup/litex-boards/litex_boards/platforms/icepi_zero.py`.

This note has two parts:

- **Part 1 — Board / Platform Reference**: the static map of what the board
  brings out, exactly as the platform file defines it. Independent of any
  project.
- **Part 2 — Current Project Wiring**: what this project's SoCs actually drive
  on the GPIO bank, via `icepi_zero_imu.py` and the LCD SoC. This is the part
  that changes as the project evolves.

When in doubt, Part 2 is the source of truth for how the GPIO pins are used
right now; Part 1 is the source of truth for the fixed board resources
(SDRAM, USB, flash, SD, GPDI, etc.).

---

# Part 1 — Board / Platform Reference

## Core: clock, reset, UART

| Resource | Signal | FPGA Pin | Notes |
| --- | --- | --- | --- |
| `clk50` | clock input | `M1` | 50 MHz input clock |
| `rst` | reset input | `C4` | Pulled up in platform file; shares pin with `user_btn` 0 |
| `ext_reset` | external reset, active-low | `G3` | Pulled up in platform file |
| `serial` | `tx` | `K15` | LiteX BIOS UART transmit |
| `serial` | `rx` | `K16` | LiteX BIOS UART receive |

## Buttons

| Resource | Index | FPGA Pin | Notes |
| --- | --- | --- | --- |
| `user_btn` | 0 | `C4` | Shares the same pin as `rst` |
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

The platform exposes a **27-pin** `gpio` resource:

`K3 T2 R2 R1 E1 F3 G1 H2 J1 L2 G2 J3 E3 P1 N1 H3 R3 N4 E4 F1 F2 P2 M2 L1 J2 D4 P3`

Physical pins are labeled `IOn` on the board, with `IO1` = `K3`, `IO2` = `T2`,
and so on through `IO27` = `P3`.

> Note: the platform comment marks `IO1` as "old `gpio[1]` = new `gpio[0]`" —
> an earlier revision had a `gpio[0]` that was dropped, which is why this bank
> is sometimes mistakenly called "28-pin." It is 27 pins.

The platform file also predefines `rgb_led` and `mcp3008` aliases that sit on
top of these GPIO pins. For how the bank is actually driven today (including
the IMU added by `icepi_zero_imu.py` and the LCD/touch panel), see
**Part 2 — Current Project Wiring**.

## USB 0

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `usb` | `d_p` | `F15` |
| `usb` | `d_n` | `E16` |
| `usb` | `pullup` | `G15 H14` |

## USB 1

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `usb` | `d_p` | `J16` |
| `usb` | `d_n` | `J15` |
| `usb` | `pullup` | `E14 E11` |

## SPI Flash

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

`spiflash` (1x) and `spiflash4x` (quad) are two views of the same physical
flash pins.

## SD Card

| Resource | Signal | FPGA Pin(s) |
| --- | --- | --- |
| `spisdcard` | `clk` | `P15` |
| `spisdcard` | `mosi` | `N16` |
| `spisdcard` | `cs_n` | `M14` |
| `spisdcard` | `miso` | `P14` |
| `sdcard` | `clk` | `P15` |
| `sdcard` | `cmd` | `N16` |
| `sdcard` | `data[0:3]` | `P14 R14 M15 M14` |

`spisdcard` (SPI mode) and `sdcard` (native SD) are two views of the same
physical card pins.

## GPDI

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

---

# Part 2 — Current Project Wiring

Everything below sits on the GPIO bank from Part 1. The peripherals are added
as platform extensions in `gateware/soc_features.py` (shared by the per-feature
tops and the combined `icepi_zero_all.py`); `rgb_led` comes from the platform
file but is listed here because this is where its real usage lives. All
allocations are disjoint, so every peripheral coexists in `icepi_zero_all.py`.

## GPIO Bank Usage (IO1–IO27)

| Board IO | FPGA Pin | Function |
| --- | --- | --- |
| IO1 | `K3` | RGB LED / NeoPixel data |
| IO2 | `T2` | aux SPI bus `sclk` (WINC + IMU + MCP3008) |
| IO3 | `R2` | MCP3008 `cs_n` (aux bus `cs[2]`) |
| IO4 | `R1` | *free* |
| IO5 | `E1` | 74HC595 `SRCLK` |
| IO6 | `F3` | IMU `cs_n` (aux bus `cs[1]`) |
| IO7 | `G1` | `LCD_RS` (DC / command-data select) |
| IO8 | `H2` | aux SPI bus `mosi` |
| IO9 | `J1` | 74HC595 `SER` (was WINC `CHIP_EN` direct, moved to Qc 2026-06) |
| IO10 | `L2` | *free* |
| IO11 | `G2` | 74HC595 `RCLK` |
| IO12 | `J3` | *free* (was `LCD_RST`, moved to 74HC595 Qa) |
| IO13 | `E3` | `LCD_MISO` |
| IO14 | `P1` | LCD backlight |
| IO15 | `N1` | `CTP_SCL` (touch) |
| IO16 | `H3` | `LCD_CS` |
| IO17 | `R3` | *free* |
| IO18 | `N4` | `CTP_SDA` (touch) |
| IO19 | `E4` | `LCD_SCK` |
| IO20 | `F1` | *free* (was WINC `RESET_N`, moved to 74HC595 Qh) |
| IO21 | `F2` | `CTP_INT` (touch) |
| IO22 | `P2` | *free* |
| IO23 | `M2` | WINC `cs_n` (aux bus `cs[0]`) |
| IO24 | `L1` | WINC `IRQN` (active low, pull-up) |
| IO25 | `J2` | aux SPI bus `miso` |
| IO26 | `D4` | `LCD_MOSI` |
| IO27 | `P3` | *free* |

**20 used, 7 free.** Free pins: IO4, IO10, IO12,
IO17, IO20, IO22, IO27.

## RGB LED / NeoPixel

| Signal | FPGA Pin | Board IO |
| --- | --- | --- |
| `rgb_led` data | `K3` | IO1 |

## 74HC595 Reset/Enable Expander

A 3.3 V 74HC595 (added 2026-06) carries all slow reset/enable sidebands so
they stop costing one FPGA pin each (`gateware/sr595.py`, wired lazily by
`_sr595_connect` in `gateware/soc_features.py`). Transparent to firmware:
the existing CSRs (`lcd.pads_ctrl.reset_n`, `winc_reset`, `winc_en`)
comb-drive expander bits and the driver re-shifts (MSB first, 2 MHz SRCLK,
~4.5 us per update) and pulses RCLK on any change. One forced shift runs
right after (soft) reset -- the '595 powers up with random latch contents,
and this restores the GPIOOut power-on guarantees (WINC off, LCD reset
deasserted). `SRCLR` ties high, `OE` ties low.

| Signal | FPGA Pin | Board IO |
| --- | --- | --- |
| `SER` | `J1` | IO9 |
| `RCLK` | `G2` | IO11 |
| `SRCLK` | `E1` | IO5 |

| Output | Net | Notes |
| --- | --- | --- |
| Qa | `LCD_RST` | Active low; follows `lcd.pads_ctrl.reset_n` |
| Qb | `CTP_RST` | Active low; ganged with Qa (replicates old shared line) |
| Qc | WINC `CHIP_EN` | Low = power-down; follows `winc_en` CSR |
| Qd-Qg | -- | Unconnected, driven 0 |
| Qh | WINC `RESET_N` | Active low; follows `winc_reset` CSR |

## Shared Aux SPI Bus (ATWINC1500 + LSM6DS3 IMU + MCP3008)

One SPI bus (runtime-divider `AuxSPIMaster`, software-held chip-selects --
`gateware/aux_spi.py`, wired by `add_winc_aux` in `gateware/soc_features.py`)
serves three devices at their own clocks. Distinct from the LCD's dedicated
SPI bus. Historical note: `icepi_zero_imu.py` drove the same physical pins
with a LiteX SPIMaster (`imu_spi`, IMU on cs[0]); that arrangement is
superseded by this bus -- firmware reaches the IMU via the `AUX_IMU` device
in `software/winc_test/aux_spi.h`.

Shared lines:

| Signal | FPGA Pin | Board IO |
| --- | --- | --- |
| `sclk` | `T2` | IO2 |
| `mosi` | `H2` | IO8 |
| `miso` | `J2` | IO25 |

Per-device chip-selects (`AUX_CS_*` constants in csr.h match the indices):

| Device | Signal | FPGA Pin | Board IO | Notes |
| --- | --- | --- | --- | --- |
| ATWINC1500 WiFi | `cs_n[0]` | `M2` | IO23 | SPI mode 0; 12.5 MHz = tested ceiling (25 MHz corrupts MISO) |
| LSM6DS3 IMU | `cs_n[1]` | `F3` | IO6 | SPI mode 0; SCK max 10 MHz (bring-up uses 1-2 MHz) |
| MCP3008 ADC | `cs_n[2]` | `R2` | IO3 | Parked deasserted (no firmware driver wired up currently) |

WINC sidebands (also `add_winc_aux`):

| Signal | Route | Notes |
| --- | --- | --- |
| `RESET_N` | 74HC595 Qh | Active low; GPIOOut CSR, 0 at power-on (held in reset) |
| `CHIP_EN` | 74HC595 Qc | Low = power-down; GPIOOut CSR, 0 at power-on (chip off). Reaches the module's true power-down since 2026-06 (the old external 3.3 V tie must be gone) |
| `IRQN` | `L1` (IO24) | Active low, pull-up; GPIOIn with IRQ (polled in current firmware) |

## LCD / Touch

This project uses an ST7796S SPI LCD controller with a 320x480 panel and an
FT6336U capacitive touch controller. The first bring-up target is an LCD test
pattern. The eventual software target is LVGL running on the LiteX CPU.

The LCD SPI bus has its own dedicated pins (not shared with the sensor bus).
`CTP_INT` lives on `F2` (IO21); its former pin `E3` (IO13) is now `LCD_MISO`.

| Device | Signal | FPGA Pin | Board IO | Notes |
| --- | --- | --- | --- | --- |
| LCD SPI | `sclk` | `E4` | IO19 | `LCD_SCK` |
| LCD SPI | `mosi` | `D4` | IO26 | `LCD_MOSI` |
| LCD SPI | `miso` | `E3` | IO13 | `LCD_MISO`; not required for basic LCD writes |
| ST7796S LCD | `cs_n[0]` | `H3` | IO16 | `LCD_CS` |
| ST7796S LCD | `reset` | 74HC595 Qa | -- | `LCD_RST`; was `J3` (IO12) until 2026-06 |
| ST7796S LCD | `dc` / `rs` | `G1` | IO7 | `LCD_RS`; command/data select |
| LCD backlight | `led` | `P1` | IO14 | `backlightLED` |
| FT6336U touch | `scl` | `N1` | IO15 | `CTP_SCL` |
| FT6336U touch | `sda` | `N4` | IO18 | `CTP_SDA` |
| FT6336U touch | `int` | `F2` | IO21 | `CTP_INT` |
| FT6336U touch | `rst` | 74HC595 Qb | -- | `CTP_RST`; ganged with LCD reset (was hard-tied to it) |
