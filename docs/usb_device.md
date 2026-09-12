# USB device-mode controller (plan)

Goal: give the IcePi Zero a **USB device** interface (it enumerates on a host PC)
for future use — the obvious payoffs being a USB-CDC serial console and, longer
term, a USB-DFU / bulk firmware loader that beats the 1 Mbaud UART and complements
the ESP32-C3 flash path.

This is a *plan + scaffold* doc. The `targets/icepi_zero/usb_device.py` variant it describes is
**UNTESTED on hardware** — it exists so the shape can be built (routing / Fmax /
capacity checked) the moment the physical USB port is confirmed.

## What already exists

- **Pins are defined** in `litex_boards/platforms/icepi_zero.py`:
  - `usb:0` → D+ `F15`, D- `E16`, pullup `G15 H14`
  - `usb:1` → D+ `J16`, D- `J15`, pullup `E14 E11`
- **ValentyUSB is vendored** (`litex-setup/valentyusb/`) and LiteX integrates it
  natively: `SoCCore.add_uart(uart_name="usb_acm")` (`soc.py` ~line 1637).
- **Reference design**: `litex_boards/targets/gsd_orangecrab.py` is also an ECP5
  and brings up USB-ACM over ValentyUSB — copy its patterns.
- `pythondata-misc-usb_ohci` is also vendored, but that is a USB **host** core —
  not device mode. Ignore it here.

## Recommended core: ValentyUSB, full-speed (12 Mbps), fabric PHY

No external PHY chip. The FPGA drives D+/D- directly through series resistors,
with a 1.5 kΩ pullup on D+ (a gateware-controlled pullup, per the `pullup` pins).
Full-speed is the only sane option on a 25F:

- **LUNA** (the modern alternative) is Amaranth-based — a large detour off the
  migen/LiteX flow. Not worth it.
- **High-speed (480 Mbps)** needs an external ULPI PHY (USB3300) on a ~12-pin bus.
  No spare pins, no capacity. Overkill.

## Clocking — how it actually works (implemented)

The key realization: **ValentyUSB's `CDCUsb` is NOT clocked from `sys`.** It runs
its PHY in dedicated `usb_48` (48 MHz, 4× FS oversample — `usbcore/rx/clock.py`)
and `usb_12` (12 MHz bit-rate) domains, and does its own `sys`↔`usb` crossing
internally (AsyncFIFO / MultiReg / PulseSynchronizer, `cdc_eptri.py:126-165`). Its
UART-shaped CSR interface sits in `sys`.

Consequence: **`sys` is unconstrained by USB.** It can stay at the proven
**100 MHz / half-rate SDRAM 200 MHz** on PLL1. USB just needs its own 48 + 12 MHz
domains, which come off **PLL2**. (LiteX's stock `usb_acm` renames `sys`→`sys_usb`
and still requires the board CRG to supply `usb_48`/`usb_12`; we skip that wrapper
and instantiate `CDCUsb` directly — see `targets/icepi_zero/usb_device.py:add_usb_acm`.)

Both USB clocks use **`margin=0`** in `create_clkout`. The default 1% PLL margin
lets the solver pick VCO=525 → 47.73 MHz (0.57% off), *outside* USB FS's ±0.25%
tolerance → flaky enumeration. `margin=0` forces the exact solution VCO=480 →
48 (/10) + 12 (/40), 0 ppm error.

Earlier drafts assumed USB had to ride the `sys` domain (forcing sys=48, or a
sys=96 "2×48" PLL trick). That was based on the stock wrapper, not on `CDCUsb`'s
real contract — it's moot now that USB has its own PLL2 domains.

## PLL budget on the 25F

The **LFE5U-25F has only 2 PLLs**. Each has 4 outputs, but all 4 divide from one
shared VCO — so the limit is the *VCO*, not the output count. In the USB variant:

| PLL | VCO | Outputs |
|-----|-----|---------|
| PLL1 (`_CRG.pll`)  | 400 MHz | `cd_sys` 100 (/4), `cd_sys2x` 200 (/2), `cd_sys2x_ps` 200 (/2, φ180) |
| PLL2 (`_CRG.pll2`) | 480 MHz | `cd_usb_48` 48 (/10), `cd_usb_12` 12 (/40) |

This fits cleanly because USB got its **own** PLL — 48/12 share a trivial VCO
(480). What does *not* work is trying to also make 48 on **PLL1**: a VCO that
divides to 200, 100 *and* 48 would need to be a common multiple ≈ LCM(200,48) =
1200 MHz, above the 800 MHz VCO ceiling (closest a 200-compatible VCO gets is
50 MHz, 4% off). So USB must live on PLL2.

**Trade-off:** PLL2 is the same PLL the LCD variant uses for its 185 MHz SPI
clock. With only 2 PLLs, **USB and LCD are mutually exclusive** on this board
(the CRG asserts this). Fine here — the USB variant has no LCD. A USB+LCD build
would need a 3rd PLL the 25F doesn't have.

## Staged plan

- **Phase 0 — Hardware.** Confirm D+/D- on `F15`/`E16` reach a physical USB
  connector. If not, this is a bench-wiring task first: connector, ~22 Ω series
  resistors on D+/D-, 1.5 kΩ pullup on D+.
- **Phase 1 — Clocking.** *Done in the scaffold.* `_CRG(..., with_usb=True)` adds
  `cd_usb_48`/`cd_usb_12` off PLL2 (exact, `margin=0`); sys/mem stay 100/200 on
  PLL1. Both PLLs verified to solve.
- **Phase 2 — Enumeration proof-of-life.** `CDCUsb` is wired as a **secondary**
  USB-CDC serial port (keeps the primary UART for the C3 loader/console). Goal:
  a `/dev/ttyACM*` appears on the host and echoes. This is what
  `targets/icepi_zero/usb_device.py` scaffolds — but it is **not yet HW-tested**.
- **Phase 3 — Real use case.** Move to the `eptri` register interface and port
  firmware for the target class. High-value target: **USB-DFU / bulk loader** to
  replace the 1 Mbaud UART.
- **Phase 4 — Capacity.** ValentyUSB eptri is ~1–2k LUT. Build the scaffold and
  read the nextpnr utilization/Fmax to confirm the real cost; USB will not
  coexist with the heavy variants (SNN, full LCD+SD) on the 25F.

## Scaffold

`targets/icepi_zero/usb_device.py` — BaseSoC at **sys = 100 MHz / half-rate SDRAM 200 MHz**
(PLL1, unchanged) + `with_usb=True` (PLL2 → 48/12 MHz) + the C3 flash loader (so
the board stays programmable) + a **secondary** USB-CDC ACM UART via `CDCUsb`. No
LCD, no IOX. Build:

```
.venv/bin/python targets/icepi_zero/usb_device.py --build          # elaborate + place & route
# read "Max frequency for clock 'sys'" and LUT/EBR utilization from the log
```

Nothing here is HW-validated — treat a produced bitstream only as a capacity/Fmax
data point until the USB port is wired and enumeration is confirmed.
