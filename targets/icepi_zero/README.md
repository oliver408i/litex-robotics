# IcePi Zero SoC targets

Every file here is a thin `BaseSoC` subclass that picks a few feature adders
from `gateware/soc_features.py`. Run them from the repo root:

```bash
.venv/bin/python targets/icepi_zero/mnist_lcd.py --build
```

`base.py` is library-only — it holds `BaseSoC`, the CRG, and the shared
argparse/build helpers, and it is what puts the repo root and the pinned
`litex-setup/` checkouts on `sys.path`. Every other file imports from it.

All variants build into the **same** `build/icepi_zero/` directory, so building
one clobbers the previous one's matched bitstream/BIOS/csr.csv. Snapshot a build
with `./syspkg.py pack <name>` before moving on — see the repo README.

## Status

Board hardware changed under this tree twice (the ATWINC1500 was destroyed, the
74HC595 expander failed), so some tops target hardware that no longer exists.
Nothing is deleted — a dead top still documents a working design and its pin
map — but check this column before picking one to build on.

| Top | What it is | Status |
|---|---|---|
| `mnist_lcd.py` | LCD + touch + SNN-MLP + IOX + C3 loader | **Deployable.** The reference "everything that still works" build |
| `logger.py` | IMU data logger with LCD/touch UI, GPS UART, SD | Deployable |
| `c3flash.py` | ESP32-C3 SPIBone flash loader (the `flash.py` target) | **Current.** This is how the board gets programmed |
| `turbo.py` | Overclocked, gaming-only (the DOOM port) | Bench stunt — explicitly not for field use |
| `lcd.py` | ST7796S LCD + FT6336U touch only | Works |
| `mnist.py` | SNN-MLP MNIST classifier | Works |
| `baseline.py` | Minimal flashing baseline, no features | Works |
| `c3spibone.py` | C3 link bring-up over SPIBone | Bring-up scaffold |
| `c3uartbone.py` | C3 link bring-up over UARTBone | Bring-up scaffold |
| `mcp.py` | MCP23S17 GPIO-expander bring-up | Bring-up scaffold |
| `psc18sr.py` | PSC18SR (PISC v2) prototype | **Prototype against a draft ISA.** No golden model yet — see `docs/psc18sr_prototype.md` |
| `pisc.py` | PISC v1 sequencer bring-up | Works, but still instantiates the dead WINC block |
| `la.py` | 18-channel logic analyzer | Gateware + sims pass; its firmware/host offload is WiFi-only, so the transport is gone |
| `usb_device.py` | ValentyUSB CDC device scaffold | **Untested on hardware.** PLL2 is shared with the LCD clock, so USB and LCD are mutually exclusive on the 25F |
| `all.py` | Everything non-conflicting in one build | Instantiates the dead WINC block |
| `winc.py` | ATWINC1500 WiFi bring-up | **Dead hardware.** The chip was destroyed; kept for the design + pin map (tag `winc-final`, branch `winc-archive`) |

Named `usb_device.py`, not `usb.py`: running a top puts this directory on
`sys.path`, and a bare `usb.py` would shadow **pyusb** for anything later in the
import chain.

## Related

- `gateware/soc_features.py` — the feature adders these compose
- `docs/soc_layout.md` — block diagram and address map of the `all.py` shape
- `docs/boot_chain.md` — how a build actually reaches the board
- `docs/icepi_zero_pin_mapping.md` — what each FPGA pin is doing
