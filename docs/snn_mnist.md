# SNN-MLP MNIST Classifier on IcePi Zero (ECP5)

Snapshot of the MNIST classifier work landed on 2026-05-25. Read this to pick
the project back up, understand what works, and pick the next improvement.

## Status

**Working end-to-end on hardware.** Trained 784→64→10 LIF SNN-MLP runs on the
IcePi Zero ECP5 board, classifying real MNIST test images. The hardware is
bit-exactly equivalent to the numpy reference simulator at every layer of the
stack.

Validated result (1000-image run, captured 2026-05-25):

| metric | value |
|---|---|
| Hardware vs simulator agreement | **1000 / 1000 images (100.00%)** |
| Hardware classification correctness | **961 / 1000 (96.10%)** |
| Trained snntorch ceiling | 95.67% (on the full 10K test set) |
| Throughput | 3.9 img/s (~255 ms/image) over 1 Mbaud UART |
| Inference cycles per image | ~1.27M @ 50 MHz ≈ 25 ms |

Everything from training to hardware deployment is reproducible from the
artifacts in the repo.

## Architecture

```
host (PC)                            FPGA (ECP5 LFE5U-25)
─────────                            ────────────────────
train_snn_mnist.py    ╮              vexriscv ─── snn_mlp peripheral
   snntorch + QAT     │                   │           │
   95.67% trained     │                   │           ├── snn_mlp_core (verilog)
        ↓             │                   │           │       MAC + LIF, N_MAC=1
build/snn_mnist.pt    │                   │           │       parameterized
        ↓             │                   │           │       FIN_A/FIN_B pipelined
pack_snn_mnist_weights.py                 │           │
   tile-major blob    │                   │           ├── snn_weight_loader (verilog)
        ↓             │                   │           │       Wishbone master, wraps
build/snn_mnist_weights.bin               │           │       beats_per_cycle × T
   203 KB             │                   │
        ↓             │                   ↓           ↓
stream_snn_mnist_uart.py ───UART(1Mbaud)──▶ snn_mnist_demo (firmware in SDRAM)
   chunked-ack proto  ╯              ╰── CSR control + pixel/bias writes
                                                      │
                                                      ↓
                                              LiteDRAM ←── snn_wb master
                                              (SDRAM, 32 MB)
                                              ├── firmware
                                              └── weight blob @ +1 MiB
```

## File inventory

### Training & simulation (host-side)

- `tools/train_snn_mnist.py` — snntorch BPTT trainer for the 784→64→10 SNN-MLP.
  Q4.12 quantization-aware training with `_Quantize` STE (clip-aware backward),
  `fast_sigmoid` surrogate, direct-current input encoding, cross-entropy on
  spike-count readout. Defaults: 15 epochs, batch 128, lr 2e-3. Output:
  `build/snn_mnist.pt`.
- `sim/snn_mlp.py` — Schedule-bit-exact numpy integer simulator. Mirrors the
  hardware datapath: Q4.12 arithmetic, leak as `mem - (mem >>> beta_shift)`,
  membrane clip ±3.999, threshold-subtract reset, sequential per-neuron MACs.
  Used as the bit-exact reference for hardware validation.
- `tools/pack_snn_mnist_weights.py` — Packs a `.pt` checkpoint into the
  tile-major SDRAM blob the loader consumes. Each beat is one 32-bit word
  holding `N_MAC` Q4.12 weights in the low bits. Prints the CSR values
  (`weight_base`, `beats_per_cycle`, `num_cycles`) for the firmware.

### Gateware

- `verilog/snn_mlp_core.v` — Parameterizable SNN-MLP inference core.
  Parameters: `IN_SIZE`, `HIDDEN`, `OUT_SIZE`, `TIMESTEPS`, `N_MAC`, `DATA_WIDTH`,
  `FRAC_BITS`, `BETA_SHIFT`, `SPK_WIDTH`. Image-major schedule:
  load pixels → for each `t` in `T`: layer-1 MAC then LIF tiles → layer-2 MAC
  then LIF tiles → accumulate output spikes → argmax. **FIN is split into
  `FIN_A` (post-MAC `leak + shifted + bias` add, registered to `pre_reg`) and
  `FIN_B` (clip + threshold + write back)** to break a ~21 ns combinational
  chain that was missing 50 MHz timing on ECP5.
- `verilog/snn_weight_loader.v` — Wishbone-classic master streaming weights
  from SDRAM. Holds one timestep's worth of weights in SDRAM and replays it
  `num_cycles` times by wrapping `base_addr → base_addr + beats_per_cycle`.
  Per-beat: one Wishbone read; data held until downstream `w_ready`.
- `gateware/snn_mlp.py` — LiteX wrapper. Instantiates the core + loader,
  exposes them as a peripheral with CSRs: `control` (start/clear pulses),
  `status` (busy/done/classification), `weight_base/beats_per_cycle/num_cycles`,
  `pixel_addr/pixel_data/pixel_ctl.we`, `bias_addr/bias_data/bias_ctl.we`, and
  10 individual `spike_count_<i>` readback CSRs. Owns a Wishbone master
  (`self.wb`) that connects to the system bus.
- `icepi_zero_mnist.py` — SoC target wrapping `BaseSoC` with the `SNNMLP`
  peripheral attached as a CSR + bus master, plus LED status indicators.

### Firmware

- `software/snn_mnist_demo/main.c` — Standalone firmware running from SDRAM
  under the LiteX BIOS. Implements a binary UART command protocol:
  - `P` → `p` (ping)
  - `C` + u32 base + u32 beats + u32 cycles → `c` (configure loader CSRs)
  - `B` + 74 × int16 → `b` (load biases via CSR write port)
  - `W` + u32 length + length bytes → `.` per 256-byte chunk + final `w`
    (chunked-ack weight blob upload to SDRAM @ `MAIN_RAM_BASE + 0x100000`)
  - `I` + 784 × int16 → `i` + 1 B class + 10 B spike counts (inference)
- `software/snn_mnist_demo/{Makefile, linker.ld, crt0.d}` — Build scaffolding
  adapted from `software/snn_demo/`. Builds against the LiteX-generated headers
  in `build/icepi_zero/software/include/generated/`.

### Host driver

- `tools/stream_snn_mnist_uart.py` — Sends biases + chunked weight blob over
  UART to the firmware, then streams MNIST test images. For each image,
  compares the DUT classification + spike count vector against `sim/snn_mlp.py`
  (the bit-exact reference). Reports per-image agreement and overall accuracy.

### Cocotb verification (gateware unit/integration tests)

- `verilog/snn_mlp_core.v` is tested at two scales:
  - **tiny** (4→2→2, T=3, hand-crafted weights) — fast smoke test
  - **MNIST** (784→64→10, T=25, trained checkpoint) — full-scale bit-exact
    check against `sim/snn_mlp.py`
- `verilog/snn_weight_loader.v` has its own cocotb tests with a mocked
  Wishbone slave (sequential reads + backpressure handling).
- `sim/cocotb/run.sh` — single entry point. Sources oss-cad-suite's iverilog
  but uses a custom `vvp-shim/` so cocotb's Python 3.12 libs and the system
  glibc load correctly (the bundled vvp wrapper would otherwise force an
  older glibc that breaks `libpython3.12.so`).
- Targets switched via `TARGET=core` (default) or `TARGET=loader`.

## End-to-end workflow

Assuming a fresh checkout with `.venv` set up (Python 3.12 with torch+snntorch
+ torchvision, see `pythondata-*` submodules in `litex-setup`):

```bash
# 1. Train the model (~3 min on the RTX 3080 Ti, CUDA-accelerated)
.venv/bin/python tools/train_snn_mnist.py --epochs 15
# → build/snn_mnist.pt at ~95.67% test accuracy

# 2. Sanity-check via the numpy simulator on the full 10K MNIST test set (~2 s)
.venv/bin/python sim/snn_mlp.py
# → hw-schedule accuracy: 95.67% (9567/10000)

# 3. (Optional) cocotb tests for the core + loader
sim/cocotb/run.sh                                       # tiny core
MNIST_IMAGES=1 sim/cocotb/run.sh IN_SIZE=784 HIDDEN=64 OUT_SIZE=10 TIMESTEPS=25
sim/cocotb/run.sh TARGET=loader                         # loader unit tests

# 4. Build + flash gateware (~3 min PnR)
.venv/bin/python icepi_zero_mnist.py --build --load
# Default output dir is build/icepi_zero/ (NOT build/snn_mnist_soc/ — the
# Makefile expects build/icepi_zero/)

# 5. Build firmware against the generated headers
PATH=/path/to/oss-cad-suite/bin:$PATH make -C software/snn_mnist_demo
# → software/snn_mnist_demo/snn_mnist_demo.bin (~5.6 KB)

# 6. Pack the weight blob
.venv/bin/python tools/pack_snn_mnist_weights.py
# → build/snn_mnist_weights.bin (203 KB, 50816 beats × 25 cycles)

# 7. Boot the firmware over UART
.venv/bin/python -m litex.tools.litex_term /dev/ttyUSB0 \
    --speed 1000000 --kernel software/snn_mnist_demo/snn_mnist_demo.bin
# Wait for the "snn_mnist_demo ready" banner, then Ctrl+C to release the port

# 8. Run inference. Host driver pings instead of waiting for the banner so
# attaching after the firmware has already booted works fine.
.venv/bin/python tools/stream_snn_mnist_uart.py --port /dev/ttyUSB0 --n-images 1000
```

## FPGA resource budget (LFE5U-25, post-synthesis)

| Resource | Used | Capacity | % | Notes |
|---|---:|---:|---:|---|
| LUT4 | 7,129 | ~24,000 | 30% | tons of room |
| TRELLIS_FF | 4,693 | ~24,000 | 20% | tons of room |
| CCU2C (carry) | 581 | ~12,000 | 5% | abundant |
| DP16KD (BRAM) | 44 | 56 | **79%** | tight — most is vexriscv caches, BIOS ROM, LiteDRAM L2 |
| MULT18X18D (DSP) | 5 | 28 | 18% | 23 free |
| TRELLIS_DPR16X4 | 48 | thousands | <1% | abundant |

**Headroom story**: scaling the SNN core (more N_MAC, bigger hidden) costs LUTs
+ DSPs, both abundant. Adding a *new peripheral* depends on BRAM — 12 free
blocks ≈ 192 Kbit; small FIFOs and lookup tables fit, a 320×240 framebuffer
doesn't (needs SDRAM, as the LCD project already proved).

## Memory map (current SoC)

- `0x00000000` — integrated ROM (BIOS, 128 KB)
- `0x10000000` — integrated SRAM (8 KB)
- `0x40000000` — SDRAM (32 MB)
  - `0x40000000` → firmware (loaded by BIOS, ~5.6 KB)
  - `0x40100000` → weight blob (203 KB; `WEIGHT_BLOB_OFFSET` in firmware)
- `0xf0000000` — CSR space
  - SNN CSR base at offset 0; `snn_control`, `snn_status`, `snn_weight_base`,
    `snn_weight_beats_per_cycle`, `snn_weight_num_cycles`,
    `snn_pixel_{addr,data,ctl}`, `snn_bias_{addr,data,ctl}`,
    `snn_spike_count_<0..9>`

## What carried over from prior work

This implementation reused a lot. None of it was free, but none of it had to
be rebuilt today either:

- **From the 8-neuron SNN tracker PoC** (`software/snn_demo`,
  `gateware/snn_estimator.py`, `verilog/lif_bank_debug.v`,
  `sim/lif_reservoir_1d.py`, `tools/train_snn_torch.py`):
  the entire Q4.12 numeric convention (`mem_clip = 3.999`, `threshold = 1.0`,
  beta as `mem - (mem >>> 3)`), the snntorch QAT + STE quantizer pattern, the
  schedule-bit-exact simulator pattern, the LiteX peripheral wrapping pattern
  (LiteXModule + AutoCSR + Instance), the standalone-firmware Makefile and
  linker.ld structure, the UART command-protocol idiom. Even the two-stage
  pipeline trick (FIN_A/FIN_B) was inspired by `lif_bank_debug.v`'s existing
  stage-A/stage-B split.
- **From the LCD project**: confidence that LiteDRAM + L2 cache + custom
  Wishbone masters work reliably on this board. This was the deciding factor
  for choosing SDRAM streaming over a BRAM-resident weight cache.
- **From the dead `mlp-testing-new` branch** (`gateware/mlp_accel.py`,
  `software/mlp2_sdram_demo`): no surviving code, but the *shape* of the
  Wishbone-master + CSR base-pointer + tile-major MAC parallelism is the same
  pattern, generalized for Q4.12. The dead attempt also surfaced the
  "BRAM-too-tight for MNIST" constraint that drove the SDRAM-streaming choice.

The retrospective that drives this: the substrate-first strategy
(8-neuron PoC → MNIST scale) was the right move *because* a brute-force MLP
attempt had failed earlier. When MNIST finally landed today, almost every
load-bearing component had a working template — the new work was scaling and
plumbing, not invention.

## Known follow-on improvements (ranked by leverage)

### High-leverage (would actually change something)

1. **Throughput**: 255 ms/image is UART + firmware-loop bound, not core bound.
   - **Easy**: batch pixel writes. Currently 784 pixels × 3 CSR writes per
     pixel via firmware. Could pack 2 pixels per 32-bit CSR write or even
     replace per-pixel CSR writes with a single CSR that points at an SDRAM
     pixel buffer (host writes pixels to SDRAM the same way it writes weights,
     SNN reads via a second Wishbone master). Probably gets to ~25 ms/image
     (inference-bound).
   - **Medium**: increase UART baudrate. 1 Mbaud is the current `BaseSoC`
     default; LiteX supports higher rates with appropriate clock.

2. **Scale `N_MAC`**: the core already supports it via parameter. `N_MAC=8`
   would give ~8× core throughput (~310 img/s ceiling). Required changes:
   - `gateware/snn_mlp.py` widens `w_data` from 16 to `N_MAC*16` bits
   - `verilog/snn_weight_loader.v` either widens the wishbone bus or packs
     multiple beats per wishbone read
   - `tools/pack_snn_mnist_weights.py` already takes `--n-mac` and packs
     tile-major
   - DSP cost ~7 (still well under the 23 free)
   - Only worth doing *after* throughput improvement #1, since UART would
     dominate until then.

### Medium-leverage (would push the model)

3. **Push training accuracy above 95.67%**:
   - `--full-qat` flag in `train_snn_mnist.py` — quantize membrane state during
     training, not just weights. Slightly noisier gradients but better
     training-vs-deployment match.
   - Bigger hidden layer (96, 128). Substrate accepts any 784→N→10 with no
     Verilog changes; just retrain + repack + reload.
   - Cosine LR schedule + ~30 epochs.

4. **Full 10K test set on hardware**. Current best run was 1000 images. At
   255 ms/image, full 10K takes ~42 minutes. Mostly to confirm overall accuracy
   matches the trained 95.67%.

### Lower-leverage (nice-to-have / research)

5. **Multi-layer SNN-MLP** (`784→32→32→10` or similar). Would need the core
   to support an arbitrary layer count, not just two. Probably ~30% more
   Verilog complexity.

6. **More aggressive numeric format** (e.g., Q3.13 or Q4.8). Smaller weights
   = smaller SDRAM footprint = fewer DSP bits used. Probably not necessary
   given current headroom.

7. **On-FPGA training** (gradient computation in gateware). Research-grade
   project, not a small extension.

## Gotchas / things future-me should know

- **LiteX default output dir is `build/<platform>` = `build/icepi_zero/`**,
  not `build/snn_mnist_soc/`. The firmware Makefile was originally wrong about
  this; it now points to `build/icepi_zero/`.
- **Don't pass `--no-compile-software`** to LiteX `--build` for actual
  hardware deployment; that skips libc / picolibc.h generation and breaks the
  firmware build.
- **Cocotb 2.0 + Icarus from oss-cad-suite glibc-conflict**: solved by the
  `sim/cocotb/vvp-shim/` wrappers. Don't source `oss-cad-suite/environment`
  for cocotb runs — it sets `LD_LIBRARY_PATH` to oss-cad-suite's older libs
  which break system `libpython3.12.so`.
- **Cocotb signal reads must happen in a stable phase**. `await
  RisingEdge(clk)` returns in the active region before NBAs apply on some
  setups; use `FallingEdge` for the host-style driver or `ReadOnly` + a phase
  transition for the same effect.
- **The 1 MiB SDRAM offset for the weight blob** is a firmware constant
  (`WEIGHT_BLOB_OFFSET = 0x100000`). Firmware itself lives below that.
- **LCD backlight tie-down**: `BaseSoC` now drives P1 low by default via
  `force_lcd_backlight_off=True`. The LCD project overrides this to
  `False` because it owns P1 itself.

## Where the bodies are buried

If something breaks in the future, these are the high-risk seams:

- **The `FIN_A`/`FIN_B` pipeline split** in `snn_mlp_core.v` is correct but
  adds 1 cycle per tile. If `N_MAC` is increased, validate the split still
  holds timing at 50 MHz; might need a third pipeline stage.
- **The Wishbone-master byte→word address conversion** in `gateware/snn_mlp.py`
  drops the bottom 2 bits via `self.wb.adr.eq(loader_wb_adr_byte[2:])`. This
  assumes `WB_DATA_WIDTH=32`. If the bus widens, this needs updating.
- **Chunked-ack weight upload** assumes both ends agree on `WEIGHT_CHUNK_SIZE`
  (256). If you change it in firmware, also change it in the host driver.
- **LiteDRAM L2 cache coherence** between CPU writes and SNN reads relies on
  `flush_cpu_dcache()` + `flush_l2_cache()` after the `W` command. Don't
  remove those calls or weights become invisible to the SNN's Wishbone master.

## Test commands worth remembering

```bash
# Quick cocotb smoke (tiny network, <1 s)
sim/cocotb/run.sh

# Bit-exact cocotb at MNIST scale (~80 s, 1 image)
MNIST_IMAGES=1 sim/cocotb/run.sh IN_SIZE=784 HIDDEN=64 OUT_SIZE=10 TIMESTEPS=25

# Full simulator MNIST eval (~2 s)
.venv/bin/python sim/snn_mlp.py

# Hardware MNIST with 100 images (~25 s + ~2 s setup)
.venv/bin/python tools/stream_snn_mnist_uart.py --port /dev/ttyUSB0 --n-images 100

# Skip the upload phase on subsequent runs (firmware retains state)
.venv/bin/python tools/stream_snn_mnist_uart.py --port /dev/ttyUSB0 --n-images 100 --skip-setup
```
