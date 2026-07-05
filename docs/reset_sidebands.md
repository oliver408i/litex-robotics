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

## Implemented (mnist_lcd): SPI GPIO expander on the aux bus

The sidebands now live on a **MCP23S17 SPI GPIO expander** hung off the aux SPI
bus (`gateware/aux_spi.py`, `AuxSPIMaster`) as one more chip-select device —
firmware writes its config and output registers, exactly like the `AUX_IMU`
device. As predicted this needed **no new gateware datapath**: just one more
aux-bus CS line + the expander sidebands, and a firmware driver.

**Wired into `icepi_zero_mnist_lcd.py` (2026-07-04):**

| Line | Where | Driven by |
|------|-------|-----------|
| `LCD_RST` | expander **GPB0** | `software/common/lcd.c` (SPI write to `OLATB`) |
| `CTP_RST` | expander **GPB1** | `software/common/lcd.c` (pulsed with LCD_RST) |
| MCP23S17 `RESET` | IO10 / `L2` | `iox_reset` GPIOOut (freed from LCD reset) |
| MCP23S17 `CS` | IO17 / `R3` | aux bus cs[3] (`AUX_CS_IOX`) |
| MCP23S17 `INTA` | IO22 / `P2` | `iox_inta` GPIOIn (with IRQ) |

Gateware: `add_lcd_touch(soc, ..., with_reset_pad=False)` drops the direct L2
reset pad, and `add_aux_imu(soc, ..., with_iox=True)` (via
`add_flashing_baseline(..., with_iox=True)`) adds cs[3] + `iox_reset`/`iox_inta`.
Firmware: `common/lcd.c` routes reset through the shared `common/mcp23s17.c`
driver when `CSR_IOX_RESET_BASE` is defined; the direct-CSR path is unchanged on
`icepi_zero_lcd.py` / `_all.py` / `_logger.py`.

**LCD_RST and CTP_RST are now separate lines** (GPB0/GPB1), no longer tied — the
board wiring must split them. During the brief power-on window before firmware
configures the expander, both float (MCP23S17 POR = inputs); `panel_iox_init()`
drives them high before enabling the outputs, and a board pull-up on each RST is
belt-and-suspenders. This is a different mechanism from the retired 74HC595
(register-addressed SPI, not a write-only shift register), so `sr595.py` does
not carry forward — that tag is for reference only.
