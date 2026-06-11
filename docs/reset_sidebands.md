# Slow Reset/Enable Sidebands

The project has a handful of slow, set-once control lines — `LCD_RST`,
`CTP_RST`, WINC `RESET_N`, WINC `CHIP_EN` — that don't need to toggle fast and
are wasteful to spend a dedicated FPGA pin on each. This note records where they
live now and where they're headed.

## Current state (interim): direct FPGA pins

All four are plain GPIO on direct pins today:

| Line | Pin | Driven by |
|------|-----|-----------|
| `LCD_RST` + `CTP_RST` | IO10 / `L2` | `lcd.pads_ctrl.reset_n` (one CSR bit) |
| WINC `RESET_N` | IO20 / `F1` | `winc_reset` GPIOOut CSR |
| WINC `CHIP_EN` | IO22 / `P2` | `winc_en` GPIOOut CSR |

`LCD_RST` and `CTP_RST` are physically tied together on the board to the single
IO10 net, so touch reset follows LCD reset with no separate control — matching
the panel's long-standing shared-reset wiring. `lcd_hw_reset()` in firmware
(`software/common/lcd.c`) toggles the one `reset_n` CSR bit and it reaches both.

## What was tried and abandoned: 74HC595 shift register

In 2026-06 a 3.3 V 74HC595 output expander folded all four sidebands onto three
shared FPGA pins (SER/RCLK/SRCLK), driven by a small glitch-hardened shift
FSM. WINC was moved back to direct pins first (the expander was unreliable on
breadboard wiring), then the physical IC failed outright with no spares on hand,
so the remaining lines (LCD/CTP reset) were reverted to direct pins too.

The gateware is **not** deleted from history — it's preserved at git tag
`sr595-expander` (driver `gateware/sr595.py`, wiring `_sr595_connect` /
`add_sr595_loopback` in `gateware/soc_features.py`, sim `sim/sr595.py`). It is
**not** the path forward, for the reason below.

## Plan (final build): SPI GPIO expander on the aux bus

The eventual home for these sidebands is a proper **SPI GPIO expander**
(MCP23S17-class) hung off the existing aux SPI bus (`gateware/aux_spi.py`,
`AuxSPIMaster`) as one more chip-select device — firmware writes its config and
output registers, exactly like the `AUX_IMU` device does today
(`software/winc_test/aux_spi.h`). That needs **no new gateware**: just one more
aux-bus CS line and a small firmware driver.

This is a different mechanism from the retired 74HC595 (register-addressed SPI,
not a write-only shift register), so `sr595.py` does not carry forward — the tag
is for reference only. When the expander lands, the interim direct-pin
assignments above (IO10, IO20, IO22) free up again.
