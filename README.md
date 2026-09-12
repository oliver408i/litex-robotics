# fpga-mcu — IcePi Zero (ECP5) SoC

A LiteX/Migen SoC for the **IcePi Zero** (Lattice ECP5) with a VexRiscv soft
CPU, plus gateware accelerators (spiking-neural-network MLP core, LCD/DMA
engine), C firmware, and a host-side tooling/training stack.

## Quick start on a new machine

```bash
git clone --recursive <repo-url> fpga-mcu
cd fpga-mcu
./setup.sh                # core: toolchain + LiteX + venv + patch + submodules
# ./setup.sh --with-ml    # add torch/snntorch for SNN training tools
```

That reconstructs everything that is **not** in git (see below). Then:

```bash
cd sim/cocotb && ./run.sh                       # run the cocotb SNN tests
.venv/bin/python targets/icepi_zero/mnist_lcd.py --build  # build a bitstream
make -C software/snn_mnist_demo                  # build firmware (needs riscv gcc)
.venv/bin/python flash.py                         # flash the board (needs hardware)
```

## What ships in git vs. what `setup.sh` rebuilds

The repo holds **only source** — the ~10 GB toolchain is gitignored and
reconstructed by `setup.sh`:

| Path | In git? | What it is | Size |
|------|:------:|------------|-----:|
| `targets/`, `gateware/`, `verilog/`, `software/`, `tools/`, `sim/`, `docs/`, `patches/`, `probe/` | ✅ | The actual project | small |
| `software/lvgl/` | ✅ submodule | LVGL GUI lib (pinned) | — |
| `.venv/` | ❌ | Python 3.12 venv (LiteX + tools) | 6.9 GB |
| `oss-cad-suite/` | ❌ | yosys + nextpnr-ecp5 + iverilog + openFPGALoader | 2.4 GB |
| `litex-setup/` | ❌ | 27 pinned LiteX/Migen source repos (editable) | 691 MB |
| `build/` | ❌ | generated bitstream/firmware artifacts | regenerated |
| `software/winc_test/wifi_secrets.h` | ❌ | WiFi SSID/password (copy from `wifi_secrets.h.example`) | you create it |

## Prerequisites you install on the host

`setup.sh` checks for these and tells you what's missing:

- **Python 3.12** (`python3.12`) — cocotb 2.0.1 and the LiteX stack target 3.12.
  - Fedora: `sudo dnf install python3.12`
  - Debian/Ubuntu: `sudo apt install python3.12 python3.12-venv`
- **RISC-V cross-compiler** `riscv64-linux-gnu-gcc` (only for building firmware):
  - Fedora: `sudo dnf install gcc-riscv64-linux-gnu binutils-riscv64-linux-gnu`
  - Debian/Ubuntu: `sudo apt install gcc-riscv64-linux-gnu`
- **git**, **curl** (or wget), **tar** — standard.

> Gateware bitstream builds and cocotb simulations work **without** the RISC-V
> compiler and **without** the physical board — you can do most development on a
> laptop offline.

## Pinned versions (reproducibility)

`setup.sh` pins exact commits/versions to match the reference machine:

- **OSS CAD Suite**: release `2026-06-02` (yosys 0.66+2). Override with
  `OSS_CAD_DATE=YYYY-MM-DD ./setup.sh`.
- **LiteX**: `enjoy-digital/litex@9748a9b` (upstream master) + the local
  `patches/litex-spiflash-skip-master-init.patch`.
- **Migen**: `m-labs/migen@4c2ae8d`.
- All other LiteX peripheral/CPU-data repos: see the `LITEX_REPOS` table in
  `setup.sh` for the full pinned list.
- **LVGL**: pinned via the git submodule.

### The liblitespi patch

`patches/litex-spiflash-skip-master-init.patch` adds a `SPIFLASH_SKIP_MASTER_INIT`
guard so the BIOS does not issue master SPI commands while executing XIP from the
same flash (which would knock it out of continuous-read mode and crash). `setup.sh`
applies it automatically and is idempotent. **Re-apply it after any LiteX update**
(re-run `setup.sh`, or `cd litex-setup/litex && git apply ../../patches/...`).
Details in [`docs/boot_chain.md`](docs/boot_chain.md).

## Build entry points

SoC configurations live under `targets/<board>/`, one thin file per variant
composed from the feature adders in `gateware/soc_features.py`:

```bash
.venv/bin/python targets/icepi_zero/mnist_lcd.py --build
```

| Path | What |
|------|------|
| `targets/icepi_zero/` | the IcePi Zero (ECP5 25F) variants — **see its README for a per-top status table**, since some target hardware that no longer exists |
| `targets/icepi_zero/base.py` | library-only: `BaseSoC`, the CRG, and the shared parser/build helpers. Puts the repo root and `litex-setup/*` on `sys.path`, so the directory layout is load-bearing |

The deployable is `targets/icepi_zero/mnist_lcd.py`; the board is programmed
through `targets/icepi_zero/c3flash.py` + `./flash.py`.

Every variant builds into the same `build/icepi_zero/`, so building one
clobbers the last one's matched artifacts — snapshot with `./syspkg.py pack`.

## What still requires the physical hardware

These cannot run on a bare laptop — they need the IcePi Zero board + FTDI cable:

- `flash.py` — program bitstream/BIOS/app over serial/WiFi
- `tools/stream_snn*_uart.py` — stream trained weights to the device
- WINC/LCD/IMU/touch bring-up and tests under `software/`

The board carries: ECP5 (LFE5U-25F), W25Q128 16 MB SPI flash, W9825G6KH6 32 MB
SDRAM, ST7796S LCD + FT6336U touch, ATWINC1500 WiFi, LSM6DS3 IMU, and MCP3008
ADC. Slow reset/enable lines are on direct FPGA pins. Pinout in
[`docs/icepi_zero_pin_mapping.md`](docs/icepi_zero_pin_mapping.md); boot chain in
[`docs/boot_chain.md`](docs/boot_chain.md); SoC layout in
[`docs/soc_layout.md`](docs/soc_layout.md).
