# SNN-MLP MNIST Classifier on IcePi Zero (ECP5)

Snapshot of the MNIST classifier work. Originally landed 2026-05-25; the
**chapter was closed on 2026-05-26** after landing the N_MAC=2 capstone and an
honest look at where the architecture can and can't go. Read this to pick the
project back up, understand what works, and — importantly — understand *why we
stopped here* and what the substrate is actually good for next.

## Status

**Working end-to-end on hardware.** Trained 784→64→10 LIF SNN-MLP runs on the
IcePi Zero ECP5 board, classifying real MNIST test images. The hardware is
bit-exactly equivalent to the numpy reference simulator at every layer of the
stack.

Validated result (1000-image run, captured 2026-05-25 at N_MAC=1):

| metric | value |
|---|---|
| Hardware vs simulator agreement | **1000 / 1000 images (100.00%)** |
| Hardware classification correctness | **961 / 1000 (96.10%)** |
| Trained snntorch ceiling | 95.67% (on the full 10K test set) |
| Throughput (UART-bound) | 3.9 img/s (~255 ms/image) over 1 Mbaud UART |

Core latency (cocotb, pure core, 1 beat/cycle):

| N_MAC | core cycles/image | speedup | timing | status |
|---:|---:|---:|---|---|
| 1 | ~1,270,000 | 1.0× | — | original |
| **2** | **637,087** | **2.0×** | **50.97 MHz PASS** | **landed (current default)** |
| 4 | 319,387 | 4.0× | — | proven bit-exact in sim, *not* worth building (see ceiling analysis) |

Everything from training to hardware deployment is reproducible from the
artifacts in the repo.

## Read this first: the honest retrospective

The original project goal — "run MNIST on this FPGA" — is **met**. But two things
became clear closing it out, and they matter more than any remaining polish:

1. **MNIST didn't need to be an SNN.** Functionally this is a quantized MLP
   wearing a LIF activation, run for 25 timesteps over a constant (DC-encoded)
   input to get a spike-count readout. A plain quantized 784→64→10 ReLU MLP would
   beat it on *every* axis: ~97% vs 95.67% accuracy, one forward pass instead of
   25, and ~25× less memory traffic. The "SNN-ness" is pure overhead on this task.

2. **The real deliverable is the substrate, not the classifier.** A parameterized
   LIF inference core, Q4.12 numerics, a snntorch QAT training pipeline, a
   schedule-bit-exact simulator, and — critically — **SDRAM weight streaming over
   a Wishbone master**. That last piece is exactly what killed the earlier
   classical-MLP attempts (Perceptron A/B died on "BRAM too tight for MNIST
   weights"). We solved the MLP blocker sideways: this substrate *is* an MLP
   accelerator. Swap LIF→ReLU and drop the timestep loop and you have the MLP
   that beat you, reusing ~90% of the gateware.

3. **The one thing that would justify an SNN — event-driven sparsity — is not
   exploited** (see "Performance ceiling" below). So as it stands it has neither
   the MLP's efficiency nor the SNN's signature advantage.

**Takeaway for the next project:** pick the *task* to fit the capability you want
to show, not the other way around. SNNs earn their keep on temporal / event-driven
sparse data (DVS event cameras, audio keyword spotting, sensor anomaly detection),
**with** event-driven sparsity actually built in. Both the tracker PoC and MNIST
were tasks that didn't exercise that — don't make it three.

## Architecture

```
host (PC)                            FPGA (ECP5 LFE5U-25)
─────────                            ────────────────────
train_snn_mnist.py    ╮              vexriscv ─── snn_mlp peripheral
   snntorch + QAT     │                   │           │
   95.67% trained     │                   │           ├── snn_mlp_core (verilog)
        ↓             │                   │           │       MAC + LIF, N_MAC=2
build/snn_mnist.pt    │                   │           │       parameterized
        ↓             │                   │           │       FIN_A/FIN_B pipelined
pack_snn_mnist_weights.py                 │           │
   --n-mac 2          │                   │           ├── snn_weight_loader (verilog)
   tile-major blob    │                   │           │       Wishbone master, wraps
        ↓             │                   │           │       beats_per_cycle × T
build/snn_mnist_weights.bin               │           │
   101 KB             │                   │
        ↓             │                   ↓           ↓
stream_snn_mnist_uart.py ───UART(1Mbaud)──▶ snn_mnist_demo (firmware in SDRAM)
   chunked-ack proto  ╯              ╰── CSR control + pixel/bias writes
                                                      │
                                                      ↓
                                              LiteDRAM ←── snn_wb master
                                              (16-bit SDR SDRAM, 32 MB)
                                              ├── firmware
                                              └── weight blob @ +1 MiB
```

## Performance ceiling — why N_MAC stops paying off at 2

This is the analysis that closed the chapter. The headline: **the 16-bit SDR
SDRAM is the wall, not the MAC count.**

The board's DRAM is a Winbond **W9825G6KH6** — 16-bit data bus, single-data-rate,
driven by `GENSDRPHY` at **1:1 with sys_clk (50 MHz)**. Raw ceiling:

```
16 bits × 50 MHz = 100 MB/s   (peak; ~70–85 realistic after refresh/activate)
```

Every weight (Q4.12 = 16 bits) must cross the bus from SDRAM once per timestep,
and the loader replays the whole blob `num_cycles = 25` times. A 32-bit Wishbone
word holds exactly **two** Q4.12 weights:

| N_MAC | weights/beat | bits/beat | words/beat | core cycles/img | SDRAM words/img | verdict |
|---:|---:|---:|---:|---:|---:|---|
| 1 | 1 | 16 | 1 | ~1.27M | ~1.27M | 50% of every word wasted (padding) |
| **2** | 2 | 32 | 1 | **637K** | **635K** | **bus packed perfectly — the sweet spot** |
| 4 | 4 | 64 | 2 | 319K | ~637K | core 2× faster than the loader can feed → starves |
| 8 | 8 | 128 | 4 | 160K | ~640K | core 8× faster, loader 4× slower than core |

So:

- **N_MAC=2 is a genuine, near-free 2×.** It just stops wasting the upper 16 bits
  of every word. Both core cycles *and* SDRAM traffic halve. No loader / packer /
  host structural change — the 32-bit word already fits two weights.
- **At N_MAC=2 you're already at the DRAM floor.** 2.54 MB of weight traffic /
  100 MB/s ≈ **25 ms/image**, while the core could finish in 12.7 ms. The core
  sits idle half the time waiting on weights.
- **N_MAC>2 does not help on this board.** A wider on-chip port (e.g. a 128-bit
  LiteDRAM native port) lets you *request* more per beat, but the controller still
  has to pull those bits out of a 16-bit chip — the port width changes the shape
  of delivery, not the chip's bandwidth.

What *would* push past 25 ms (ranked by leverage / risk):

1. **Stop re-reading constant data 25×** (highest leverage, bit-exact, contained).
   The input is DC-encoded — the same pixel vector every timestep — so layer-1's
   pre-activation `W1·x + b1` is *identical* across all 25 steps. Today the core
   recomputes it and the loader re-streams all of W1 every step. Compute it
   **once**, cache the 64 hidden input-currents on-chip, then run the 25-step LIF
   loop over that cached value. Layer-2 weights are tiny (10×64 = 640 ≈ 1.3 KB) —
   cache them on-chip too. That collapses DRAM weight traffic ~25× (→ ~1 ms floor),
   is bit-exact by construction (factoring out a constant), and makes the system
   core-bound — at which point N_MAC parallelism pays off again. Composes with
   pixel-sparsity skipping (only fetch/MAC the non-zero ~20% of MNIST pixels).
2. **Faster SDRAM clock** (~2×, bounded, much bigger lift). The W9825G6KH6 is rated
   ≥133 MHz. `GENSDRPHY` is 1:1, so the only way to run DRAM faster is a separate,
   faster clock domain for {LiteDRAM + loader + core} with a CDC to the 50 MHz CPU
   — custom CRG, a CDC boundary, PHY timing re-closure, and re-pipelining the core
   for 100 MHz (the FIN split was needed just for 50). **This is really a separate
   memory/SoC-clocking project, not MNIST polish.**

## Event-driven sparsity — present for correctness, not performance

The SNN's signature advantage (compute/energy scaling with spike activity) is
**not** harvested here. The only sparsity-aware line is the layer-2 spike gate in
`snn_mlp_core.v` (`if (spk1_buf[in_idx])`), and it's there for *correctness*: it
decides whether to *add* the weight, but the cycle is still spent and the weight is
still read from SDRAM whether or not the neuron spiked. Layer 1 (98% of the work)
has no gating at all — every pixel, including the ~75–80% background zeros, gets a
full beat + multiply.

Key rule for a memory-bound accelerator: **sparsity only pays if it skips the
weight *fetch***, not just the multiply. And the layers cut the wrong way here —
layer 2 (genuine spike sparsity) is ~1% of the work, while layer 1 (98%) has no
*spike* sparsity by design because its input is DC-coded pixels, not spikes. To
make it real you'd need per-image weight gathering (skip W1 columns for zero
pixels), which breaks the static "replay one blob" loader model. Worth it only on
a task where the input is genuinely sparse-in-time.

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
  holding `N_MAC` Q4.12 weights. **Run with `--n-mac 2`** to match the current
  gateware. Prints the CSR values (`weight_base`, `beats_per_cycle`,
  `num_cycles`) for the firmware.

### Gateware

- `verilog/snn_mlp_core.v` — Parameterizable SNN-MLP inference core.
  Parameters: `IN_SIZE`, `HIDDEN`, `OUT_SIZE`, `TIMESTEPS`, `N_MAC`, `DATA_WIDTH`,
  `FRAC_BITS`, `BETA_SHIFT`, `SPK_WIDTH`. Image-major schedule:
  load pixels → for each `t` in `T`: layer-1 MAC then LIF tiles → layer-2 MAC
  then LIF tiles → accumulate output spikes → argmax. `N_MAC` neurons per tile
  computed in parallel; partial tiles padded and guarded by `neuron < HIDDEN`.
  **FIN is split into `FIN_A` (post-MAC `leak + shifted + bias` add) and `FIN_B`
  (clip + threshold + write back)** to break a ~21 ns combinational chain that
  missed 50 MHz on ECP5. The N_MAC>1 path is now verified bit-exact in cocotb
  (it had never actually been simulated before 2026-05-26).
- `verilog/snn_weight_loader.v` — Wishbone-classic master streaming weights
  from SDRAM. Holds one timestep's worth of weights in SDRAM and replays it
  `num_cycles` times by wrapping `base_addr → base_addr + beats_per_cycle`.
  Per-beat: one Wishbone read; data held until downstream `w_ready`. Slices the
  low `N_MAC*DATA_WIDTH` bits of each 32-bit word — so **N_MAC≤2 only** without
  a multi-word-per-beat rework.
- `gateware/snn_mlp.py` — LiteX wrapper. Default **`n_mac=2`**. Instantiates the
  core + loader, exposes CSRs: `control` (start/clear pulses), `status`,
  `weight_base/beats_per_cycle/num_cycles`, `pixel_*`, `bias_*`, and 10
  `spike_count_<i>` readback CSRs. Owns a Wishbone master (`self.wb`).
- `icepi_zero_mnist.py` — SoC target wrapping `BaseSoC` with the `SNNMLP`
  peripheral. `N_MAC = 2` constant at the top (must match the packer's
  `--n-mac`), plus LED status indicators.

### Firmware

- `software/snn_mnist_demo/main.c` — Standalone firmware running from SDRAM
  under the LiteX BIOS. Binary UART command protocol (N_MAC-agnostic — the
  weight blob is opaque bytes):
  - `P` → `p` (ping)
  - `C` + u32 base + u32 beats + u32 cycles → `c` (configure loader CSRs)
  - `B` + 74 × int16 → `b` (load biases via CSR write port)
  - `W` + u32 length + length bytes → `.` per 256-byte chunk + final `w`
    (chunked-ack weight blob upload to SDRAM @ `MAIN_RAM_BASE + 0x100000`)
  - `I` + 784 × int16 → `i` + 1 B class + 10 B spike counts (inference)
- `software/snn_mnist_demo/{Makefile, linker.ld, crt0.d}` — Build scaffolding.
  Builds against the LiteX-generated headers in
  `build/icepi_zero/software/include/generated/`.

### Host driver

- `tools/stream_snn_mnist_uart.py` — Sends biases + chunked weight blob over
  UART, then streams MNIST test images, comparing the DUT classification + spike
  vector against `sim/snn_mlp.py`. Derives `beats_per_cycle = len(blob) // 4`,
  which is correct for **N_MAC≤2** (one 32-bit word per beat). A multi-word-beat
  scheme (N_MAC≥4) would need this changed.

### Cocotb verification

- `verilog/snn_mlp_core.v` tested at two scales, both N_MAC-parameterized:
  - **tiny** (4→2→2, T=3, hand-crafted weights) — fast smoke test
  - **MNIST** (784→64→10, T=25, trained checkpoint) — full-scale bit-exact
- `verilog/snn_weight_loader.v` has its own cocotb tests with a mocked Wishbone
  slave (sequential reads + backpressure).
- `sim/cocotb/run.sh` — single entry point; forwards args to `make`. Uses a
  custom `vvp-shim/` so cocotb's Python 3.12 and system glibc load correctly.
- Targets via `TARGET=core` (default) or `TARGET=loader`; `N_MAC=<n>` overrides
  MAC parallelism.

## End-to-end workflow

```bash
# 1. Train the model (~3 min on the RTX 3080 Ti, CUDA-accelerated)
.venv/bin/python tools/train_snn_mnist.py --epochs 15
# → build/snn_mnist.pt at ~95.67% test accuracy

# 2. Sanity-check via the numpy simulator on the full 10K MNIST test set (~2 s)
.venv/bin/python sim/snn_mlp.py
# → hw-schedule accuracy: 95.67% (9567/10000)

# 3. (Optional) cocotb tests. N_MAC=2 is the shipping config.
sim/cocotb/run.sh N_MAC=2                                # tiny core
MNIST_IMAGES=2 sim/cocotb/run.sh IN_SIZE=784 HIDDEN=64 OUT_SIZE=10 TIMESTEPS=25 N_MAC=2
sim/cocotb/run.sh TARGET=loader                          # loader unit tests

# 4. Build + flash gateware (~3 min PnR). N_MAC set by the constant in the target.
.venv/bin/python icepi_zero_mnist.py --build --load
# Output dir is build/icepi_zero/ (NOT build/snn_mnist_soc/).

# 5. Build firmware against the generated headers
PATH=/path/to/oss-cad-suite/bin:$PATH make -C software/snn_mnist_demo

# 6. Pack the weight blob — MUST match the gateware's N_MAC
.venv/bin/python tools/pack_snn_mnist_weights.py --n-mac 2
# → build/snn_mnist_weights.bin (101 KB, 25408 beats × 25 cycles)

# 7. Boot the firmware over UART
.venv/bin/python -m litex.tools.litex_term /dev/ttyUSB0 \
    --speed 1000000 --kernel software/snn_mnist_demo/snn_mnist_demo.bin

# 8. Run inference (host pings instead of waiting for the boot banner)
.venv/bin/python tools/stream_snn_mnist_uart.py --port /dev/ttyUSB0 --n-images 1000
```

## FPGA resource budget (LFE5U-25, post-route, N_MAC=2)

Captured from the 2026-05-26 `icepi_zero_mnist.py --build`:

| Resource | Used | Capacity | % | Notes |
|---|---:|---:|---:|---|
| TRELLIS_COMB | 9,387 | 24,288 | 38% | room (+~2.2k vs N_MAC=1, the 2nd MAC lane + adders) |
| TRELLIS_FF | 4,734 | 24,288 | 19% | tons of room |
| DP16KD (BRAM) | 44 | 56 | **78%** | tight — vexriscv caches, BIOS ROM, LiteDRAM L2 |
| MULT18X18D (DSP) | 6 | 28 | 21% | +1 vs N_MAC=1; 22 free |
| EHXPLLL | 1 | 2 | 50% | one PLL free (relevant if you ever add a mem clock domain) |

**Timing:** closes at **50.97 MHz** (PASS at 50). The critical path is in the
**LiteDRAM L2 cache tag memory** (`tag_mem...`), *not* the SNN core — so the FIN
split has headroom at N_MAC=2, and the memory subsystem is what limits Fmax.
Margin is thin (~2%); the SoC is already memory-bound in timing too, which
reinforces the bandwidth story above.

## Memory map (current SoC)

- `0x00000000` — integrated ROM (BIOS, 128 KB)
- `0x10000000` — integrated SRAM (8 KB)
- `0x40000000` — SDRAM (32 MB)
  - `0x40000000` → firmware (loaded by BIOS, ~5.6 KB)
  - `0x40100000` → weight blob (101 KB at N_MAC=2; `WEIGHT_BLOB_OFFSET` in firmware)
- `0xf0000000` — CSR space (SNN CSRs at offset 0)

## What carried over from prior work

- **From the 8-neuron SNN tracker PoC** (`software/snn_demo`,
  `gateware/snn_estimator.py`, `verilog/lif_bank_debug.v`,
  `sim/lif_reservoir_1d.py`, `tools/train_snn_torch.py`): the whole Q4.12 numeric
  convention, the snntorch QAT + STE quantizer, the schedule-bit-exact simulator,
  the LiteX peripheral wrapping idiom, the standalone-firmware Makefile/linker.ld,
  the UART command-protocol idiom, and the FIN_A/FIN_B stage split idea.
- **From the LCD project**: confidence that LiteDRAM + L2 cache + custom Wishbone
  masters work reliably — the deciding factor for SDRAM weight streaming over a
  BRAM-resident cache.
- **From the dead `mlp-testing-new` branch**: no surviving code, but it surfaced
  the "BRAM-too-tight for MNIST" constraint that drove the SDRAM-streaming choice
  — which, in hindsight, is exactly the blocker this project's substrate now
  removes (see the retrospective).

## Gotchas / things future-me should know

- **N_MAC must match in three places**: the gateware (`N_MAC` in
  `icepi_zero_mnist.py` + the `n_mac` default in `gateware/snn_mlp.py`), the
  packer (`--n-mac`), and implicitly the host (`beats_per_cycle = len(blob)//4`
  assumes one 32-bit word per beat, i.e. N_MAC≤2).
- **N_MAC>2 needs more than a parameter bump.** The loader slices one 32-bit
  word per beat and the packer masks each beat to 32 bits, so N_MAC≥4 silently
  truncates / over-slices. It also won't help throughput (see ceiling analysis).
- **LiteX default output dir is `build/icepi_zero/`**, not `build/snn_mnist_soc/`.
- **Don't pass `--no-compile-software`** for hardware deployment; it skips
  libc / picolibc.h generation and breaks the firmware build.
- **Cocotb 2.0 + Icarus glibc-conflict**: solved by `sim/cocotb/vvp-shim/`. Don't
  source `oss-cad-suite/environment` for cocotb runs.
- **Cocotb signal reads in a stable phase**: use `FallingEdge` (or `ReadOnly`) so
  NBAs have applied before sampling `w_ready`.
- **The 1 MiB SDRAM offset for the weight blob** is a firmware constant
  (`WEIGHT_BLOB_OFFSET = 0x100000`). Firmware lives below it.
- **LCD backlight tie-down**: `BaseSoC` drives P1 low by default via
  `force_lcd_backlight_off=True`; the LCD project overrides to `False`.

## Where the bodies are buried

- **The `FIN_A`/`FIN_B` pipeline split** is correct and has headroom at N_MAC=2
  (critical path is in the L2 cache, not the core). If you ever go past N_MAC=2
  *and* widen the memory path to make it worthwhile, re-validate the split; a
  third stage may be needed.
- **The Wishbone byte→word address conversion** in `gateware/snn_mlp.py` drops
  the bottom 2 bits via `self.wb.adr.eq(loader_wb_adr_byte[2:])`. Assumes
  `WB_DATA_WIDTH=32`. Breaks if the bus widens.
- **Chunked-ack weight upload** assumes both ends agree on `WEIGHT_CHUNK_SIZE`
  (256). Change it in firmware → change it in the host driver.
- **LiteDRAM L2 coherence** between CPU writes and SNN reads relies on
  `flush_cpu_dcache()` + `flush_l2_cache()` after the `W` command. Don't remove
  them or weights become invisible to the SNN's Wishbone master.

## Where to go next (and where NOT to)

**Done — the clean stopping point.** N_MAC=2 is the capstone: the core
parallelizes, it's verified bit-exact, it closes timing, and it sits right at the
DRAM bandwidth wall. There is no cheap win past it on this board.

**If you want more out of *this* project** (diminishing returns, but legitimate):
- Full 10K-image hardware validation run (`--n-images 10000`, ~42 min at current
  UART speed) to confirm overall hardware accuracy matches the 95.67% ceiling.
- Retrain to push past 95.67% (`--full-qat`, bigger hidden, cosine LR + ~30
  epochs) — the substrate accepts any 784→N→10 with no Verilog change.

**The higher-leverage move is a fresh project on the right task.** The substrate
now unblocks two directions:
- **A classical MLP** — swap LIF→ReLU, drop the timestep loop; ~97% MNIST in one
  pass, ~25× cheaper, reusing the MAC + loader + SDRAM streaming. Closes the
  Perceptron A/B chapter for real.
- **An event-driven SNN on temporal data** — DVS gestures, audio keyword
  spotting, sensor anomaly — *with* event-driven sparsity (skip the weight fetch
  on no activation) as a hard requirement, so it can't degrade into "dense MLP,
  N× over" again. This is the path that makes the SNN substrate *mean* something.

**Do NOT** sink more effort into MNIST core-latency / N_MAC>2 work, or revive the
dead `mlp-testing*` branches.

## Test commands worth remembering

```bash
# Quick cocotb smoke (tiny network, <1 s)
sim/cocotb/run.sh N_MAC=2

# Bit-exact cocotb at MNIST scale (N_MAC=2, ~80 s for 2 images)
MNIST_IMAGES=2 sim/cocotb/run.sh IN_SIZE=784 HIDDEN=64 OUT_SIZE=10 TIMESTEPS=25 N_MAC=2

# Full simulator MNIST eval (~2 s)
.venv/bin/python sim/snn_mlp.py

# Hardware MNIST with 100 images
.venv/bin/python tools/stream_snn_mnist_uart.py --port /dev/ttyUSB0 --n-images 100

# Skip the upload phase on subsequent runs (firmware retains state)
.venv/bin/python tools/stream_snn_mnist_uart.py --port /dev/ttyUSB0 --n-images 100 --skip-setup
```
