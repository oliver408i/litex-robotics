# VexiiRiscv on the TURBO SoC

The `targets/icepi_zero/turbo.py` variant can be built with an alternate CPU via `--cpu`:

| `--cpu` value      | Core                                    | Use it for                          |
|--------------------|-----------------------------------------|-------------------------------------|
| `vexriscv` (default) | Production VexRiscv `standard`         | Highest clock, smallest — the winner for raw overclock |
| `vexii`            | VexiiRiscv `standard` (single-issue)    | Curiosity / baseline for the newer core |
| `vexii-superscalar`| VexiiRiscv 2-way superscalar + BTB/RAS/gshare + late-ALU | Trade clock for IPC (only wins if IPC beats the clock deficit) |

Raw `--cpu-type` / `--cpu-variant` / `--vexii-args` still work and override the `--cpu` preset.

## VexiiRiscv is build-on-demand (unlike VexRiscv)

VexRiscv ships pre-generated Verilog. **VexiiRiscv generates its netlist on first
build** by cloning `SpinalHDL/VexiiRiscv` (branch `dev`, pinned commit `dbcaa98`)
into `litex-setup/pythondata-cpu-vexiiriscv/.../ext/` and running SBT/SpinalHDL
(`SocGen`). So a VexiiRiscv build needs, on PATH:

- **sbt** — installed via Coursier: `~/.local/share/coursier/bin/sbt`
- **JDK 21** — `JAVA_HOME` → Coursier-managed Temurin 21 (system Java 25 is too new for SpinalHDL)

The build wrapper must source the FPGA toolchain too:

```bash
source oss-cad-suite/oss-cad-suite/environment          # yosys/nextpnr/ecppack
export PATH="$HOME/.local/share/coursier/bin:$HOME/.local/bin:$PATH"
export JAVA_HOME="$(cs java-home --jvm temurin:21)"
.venv/bin/python targets/icepi_zero/turbo.py --cpu vexii-superscalar --sys-clk-freq 85e6 --build
```

First build per unique `--vexii-args` runs SBT (~1–2 min); the netlist is then
cached by an args-hash, so repeat builds skip it.

## Measured comparison (2026-07-09, nextpnr reports on ECP5 25F, TURBO shape)

Ceiling numbers are from an **85 MHz target** (a failing constraint → nextpnr
optimizes maximally, so the reported max is the true ceiling). All same seed.

| CPU (`standard` base)     | `sys` Fmax ceiling | `sys2x` (SDRAM) ceiling | LUT4        | TRELLIS_FF  | BRAM (DP16KD) |
|---------------------------|--------------------|-------------------------|-------------|-------------|---------------|
| **VexRiscv**              | **70.78 MHz**      | **167.4 MHz**           | 8134 (33%)  | 4206 (17%)  | 32 (57%)      |
| VexiiRiscv standard       | 66.27 MHz          | 141.9 MHz               | 12321 (50%) | 7209 (29%)  | 30 (53%)      |
| VexiiRiscv superscalar    | 46.36 MHz          | 141.4 MHz               | 19389 (79%) | 9843 (40%)  | 36 (64%)      |

### What this means

- **For raw overclock, VexRiscv wins outright** — highest `sys` and `sys2x`
  ceilings, one-third the logic. Nothing here beats it on clock.
- **VexiiRiscv `standard`** is ~6% slower and ~1.5× the logic — no reason to use
  it on this board for clock.
- **VexiiRiscv superscalar** collapses the clock to **46 MHz** (66% of VexRiscv)
  and fills **79% of the 25F**. It only makes sense if IPC wins the trade:
  performance ≈ clock × IPC, so it must deliver **> 70.78 / 46.36 = 1.53×** the
  IPC of VexRiscv on the actual workload just to break even. A 2-way core with
  branch prediction can hit that on branchy integer code, but DOOM is also
  cache/framebuffer-bound where superscalar helps less — **genuinely borderline,
  only a hardware DOOM frame-time test settles it.** Note it also nearly fills
  the chip, leaving no room for the future pixel/palette accelerator.

All figures are static nextpnr timing/area, **not silicon**. Validate any target
clock with a BIOS memtest before trusting it, and settle the IPC question by
running DOOM on hardware.
