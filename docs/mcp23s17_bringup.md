# MCP23S17 SPI GPIO expander — bring-up

The MCP23S17 16-bit SPI GPIO expander replaces the retired ATWINC1500 in the
aux-bus slot (board refactor 2026-06-25). It is the long-planned SPI GPIO
expander from the 74HC595 retirement / `docs/reset_sidebands.md`: the slow
reset/enable sidebands (LCD_RST, CTP_RST, …) eventually move onto its output
pins, firmware-driven. The WINC is physically gone; an ESP32-C3 will later take
over the WiFi-loader role (so there is **no WiFi OTA path right now** — load over
UART / serial-boot / JTAG).

## Wiring (physical, 2026-06-25)

| Signal | FPGA pin | Notes |
|--------|----------|-------|
| SCLK / MOSI / MISO | IO2 (T2) / IO8 (H2) / IO25 (J2) | shared aux bus (same as IMU + MCP3008) |
| CS    | IO11 / G2 | 4th aux chip-select, `AUX_CS_IOX = 3` |
| RESET | IO10 / L2 | active low, `iox_reset` GPIOOut (powers up at 0 = held in reset) |
| INTA  | IO22 / P2 | `iox_inta` GPIOIn (with IRQ; active-low by default) |
| GPA7 loopback | IO24 / L1 | bench fixture: FPGA tristate GPIO jumpered to expander GPA7 |

Hardware address pins A2:A0 are strapped **000** → SPI opcodes `0x40` (write) /
`0x41` (read).

## Components

- **Gateware:** `gateware/soc_features.py` → `add_mcp_expander(soc, iox_spi_clk_freq, busy_led)`
  + `_mcp_io` extension + `AUX_CS_IOX`. Reuses `gateware/aux_spi.py`'s
  `AuxSPIMaster` (4th CS line).
- **Top:** `icepi_zero_mcp.py` — UART-only serial-bootable bring-up SoC
  (BaseSoC + `add_mcp_expander` + `add_boot_ctl`).
- **Firmware:** `software/mcp_test/` — `aux_spi.{c,h}` (bus HAL, `AUX_IOX` +
  `AUX_IMU`), `mcp23s17.{c,h}` (reset / read / write / 16-bit / `probe`), `main.c`.

## Build & run

```sh
# gateware (user's env: PATH=.venv/bin + oss-cad-suite/bin + system ninja)
# --yosys-abc9 --nextpnr-seed 2 are MANDATORY: this board's half-rate SDRAM
# (HalfRateGENSDRPHY, 100 MHz mem / 50 MHz sys) only closes timing with them.
# nextpnr runs with --timing-allow-fail, so a bitstream that misses timing still
# builds and loads -- it just fails the SDRAM memtest (0 addr / ~100% data
# errors). ALWAYS grep the build log for "Max frequency for clock" and confirm
# sys >= 50 MHz and sys2x >= 100 MHz; if seed 2 doesn't close, sweep seeds.
.venv/bin/python icepi_zero_mcp.py --build --load --yosys-abc9 --nextpnr-seed 2

# firmware
make -C software/mcp_test
litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/mcp_test/mcp_test.bin
```

`main.c` runs: (1) IMU `WHO_AM_I` as an aux-bus datapath cross-check, (2) reset +
register-echo probe (the MCP23S17 has no WHO_AM_I), (3) the **GPA7 loopback** (see
below), (4) a walking-1 on GPA outputs verified via GPIOA, (5) a GPB pulled-up
input + INTA watch loop.

### GPA7 loopback (the headline bring-up test)

IO24/L1 (the freed WINC IRQ pin) is jumpered to the expander's GPA7 and exposed
as a tristate GPIO (`add_gpa7_loopback`, fixture `gpa7_loop`). `gpa7_loopback()`
drives a closed loop through GPA7's real silicon, both directions, ordering the
drives so the two ends never contend:

- **A — expander → FPGA:** GPA7 = output, FPGA pin Hi-Z input; drive `OLATA` bit7,
  read the FPGA pin. Proves the expander can drive a pin.
- **B — FPGA → expander:** GPA7 = input, FPGA drives the pin; read `GPIOA` bit7
  over SPI. Proves expander input sensing + the SPI read path.

A clean run prints `LOOPBACK PASS`. This is a bench fixture — remove
`add_gpa7_loopback` (and the jumper) once the expander is verified.

## Integration debt (NOT yet resolved — bring-up is standalone)

Both reset and INTA, and the CS pin, collide with other blocks. The standalone
bring-up top has no LCD/GPS so it is clean, but before folding the expander into
the deployables:

1. **CS IO11/G2 == NMEA GPS UART TX** (`add_gps_uart`). The logger uses GPS on
   IO11 → the expander and a GPS-on-IO11 logger are mutually exclusive until one
   is repinned.
2. **RESET IO10/L2 == the direct LCD/CTP reset pin** — **RESOLVED (2026-07-04)**
   for `icepi_zero_mnist_lcd.py`: `add_lcd_touch(with_reset_pad=False)` drops the
   L2 pad, `add_aux_imu(with_iox=True)` puts the expander on cs[3] (R3) with
   RESET on L2, and `software/common/lcd.c` drives LCD_RST/CTP_RST from expander
   outputs **GPB0/GPB1** over SPI (see docs/reset_sidebands.md). Note the two
   reset lines are now SEPARATE (no longer tied) — the board wiring must split
   them onto GPB0 and GPB1.
3. **INTA IO22/P2 == WINC CHIP_EN** and **AUX_CS_MCP** already names the MCP3008
   ADC — the expander deliberately uses `AUX_CS_IOX` / `iox_*` / "IOX" to avoid
   the name clash. The dead `add_winc_aux` / `winc_ctrl` code is still in
   `soc_features.py` pending the post-WINC cleanup.

## Status

Gateware elaborates; all firmware CSR accessors verified present in the generated
headers. **Not yet hardware-tested** and **not yet PnR'd** (Fmax unchecked — the
project builds nextpnr with `--timing-allow-fail`, so grep Fmax after `--build`).
PISC Phase 3/4 (driving this expander from the sequencer core) is unblocked by
this chip arriving.
