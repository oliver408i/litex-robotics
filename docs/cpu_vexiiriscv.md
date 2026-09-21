# VexiiRiscv on the TURBO SoC (IcePi Zero) and the Colorlight i9

`--cpu` selects the core on `targets/icepi_zero/turbo.py` and, since the port,
on every `targets/colorlight_i9/` top (the argument lives in that board's shared
`make_parser`, so `bringup`/`eth`/`lcd`/`psc18sr` all take it):

| `--cpu` value      | Core                                    | Use it for                          |
|--------------------|-----------------------------------------|-------------------------------------|
| `vexriscv` (default) | Production VexRiscv `standard`         | Highest clock, smallest — the winner for raw overclock |
| `vexii`            | VexiiRiscv `standard` (single-issue)    | Curiosity / baseline for the newer core |
| `vexii-superscalar`| VexiiRiscv 2-way superscalar + BTB/RAS/gshare + late-ALU | Trade clock for IPC. **On the i9 this wins: 1.45x on DOOM** (see below). On the 25F it loses. |

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

## Measured comparison (2026-09-17, ECP5 45F, `targets/colorlight_i9/lcd.py`)

Same design (sys 90 MHz, half-rate SDRAM, LCD + Ethernet), same gateware, one
build per core. Ceilings are the post-route report for the net carrying the
90.63 MHz sys constraint -- note nextpnr names it `crg_ecp5pll0_clkout0` on the
VexRiscv build and `etherbone_clk` on the VexiiRiscv ones; it is the same sys
domain, just a different net picked as representative.

> **`TRELLIS_COMB` and the `LUT4` column in the 25F table above are the same
> metric**, renamed between nextpnr versions -- one cell = one LUT4 site. The
> denominator here, 43848, is the 45F's LUT count (Lattice's "44K"); the 25F's
> is 24288. ECP5 packs 2 LUTs per slice and 4 slices per PFU, which is why
> `TRELLIS_RAMW` is out of 5481 = 43848/8. Note a COMB cell is not always doing
> logic -- carry chains and route-throughs occupy LUT sites too -- and
> `TRELLIS_FF` shares the same denominator because every LUT site has an
> associated flop, so the two are not independent budgets.

| CPU                     | `sys` ceiling | SDRAM ceiling | TRELLIS_COMB | TRELLIS_FF  | DP16KD   |
|-------------------------|---------------|---------------|--------------|-------------|----------|
| **VexRiscv**            | 43.81 MHz     | 143.53 MHz    | 12645 (28%)  | 7071 (16%)  | 55 (50%) |
| VexiiRiscv standard     | 42.62 MHz     | 133.30 MHz    | 18039 (41%)  | 10359 (23%) | 53 (49%) |
| VexiiRiscv superscalar  | 43.30 MHz     | 123.75 MHz    | 24509 (55%)  | 12802 (29%) | 59 (54%) |

### What this means -- and why it is the opposite of the 25F result

- **The CPU is not on the critical path here.** All three ceilings land within
  2.7% of each other (43.81 / 42.62 / 43.30). On the 25F the same swap moved the
  ceiling by 35%. Whatever limits `sys` in this design, it is not the core, so
  changing the core does not pay for itself in clock -- and does not cost
  anything either.
- **Break-even flips from 1.53x to 1.01x.** On the IcePi, superscalar had to beat
  VexRiscv's IPC by more than half again just to draw level against a 66% clock.
  Here it starts from a 1.2% clock deficit, so essentially *any* IPC gain is a
  net win. This is the single reason the core is worth revisiting on this board.
- **It fits, with room left.** 55% of the 45F's logic against 79% of the 25F's.
  The pixel/palette accelerator the 25F had no space for still has space here.
- **The SDRAM ceiling does fall with core size** (143.5 -> 133.3 -> 123.8). All
  three are reported as failing the 181.26 MHz constraint and the board runs at
  that clock regardless, which is the usual ~70% pessimism on this part -- but
  superscalar spends the most of that margin, so it is the build most likely to
  be the first to actually break. Re-run the BIOS memtest after the swap rather
  than assuming the 90/181 operating point carries over.

## What the two cores actually are

Same ISA -- both are **rv32i2p0_m** (RV32IM, ilp32). Everything below is
microarchitecture, not instruction set.

| | VexRiscv `standard` | VexiiRiscv `vexii-superscalar` |
|---|---|---|
| Issue width | 1 | **2** (`--decoders=2 --lanes=2`) |
| Branch prediction | STATIC | **BTB + RAS + gshare** |
| Late ALU | no | yes |
| I-cache | 4 KB, direct-mapped, 32 B line | **8 KB** -- 64 sets x 2 ways x 64 B |
| D-cache | 4 KB, direct-mapped, 32 B line | **8 KB** -- 64 sets x 2 ways x 64 B, + bypass |
| Netlist | pre-generated Verilog | generated on demand via SBT/SpinalHDL |

So VexiiRiscv has **2x the capacity, 2x the associativity and 2x the line
size** on both L1s -- 16 KB of L1 against 8 KB.

Cache figures are read off the generated netlist, not just the defaults: the tag
arrays are `FetchL1Plugin_logic_ways_{0,1}_mem [0:63]` (and the same for
`LsuL1Plugin`), I-cache data is 2 banks x 512 x 64 bit = 8192 B, D-cache data is
2 banks x 1024 x 32 bit = 8192 B. That matches `Param.scala`'s `fetchL1Sets = 64`
/ `lsuL1Sets = 64` with `lineSize = 64`, which apply because LiteX passes ways
but never overrides sets. VexRiscv's come from `GenCoreDefault.scala`
(`iCacheSize`/`dCacheSize = 4096`, `bytePerLine = 32`, `wayCount = 1`,
`prediction = STATIC`).

The full `vexii-superscalar` generator argument list, as actually passed --
LiteX's `standard` variant base plus this repo's preset:

```
--with-mul --with-div --allow-bypass-from=0 --performance-counters=0
--fetch-l1 --fetch-l1-ways=2
--lsu-l1  --lsu-l1-ways=2 --with-lsu-bypass
--relaxed-branch                                   <- LiteX adds this; see below
--decoders=2 --lanes=2 --with-btb --with-ras --with-gshare --with-late-alu
```

## Core area on its own, separate from the SoC

The utilisation tables above are **whole-SoC** numbers -- they include LiteDRAM,
the gigabit MAC + Etherbone, LiteSPI, the LCD engine and the CSR fabric, most of
which you pay for whatever the core. Synthesising each core standalone
(`synth_ecp5`, LUT sites = `LUT4 + 2*CCU2C + 4*DPR16X4`, a formula checked
against nextpnr on the full SoC to 0.25%):

| core alone | LUT sites | % of 45F | FF | BRAM |
|---|---|---|---|---|
| VexRiscv standard | **3120** | 7.1% | 1527 | 10 |
| VexiiRiscv standard | 7180 | 16.4% | 4548 | 8 |
| VexiiRiscv superscalar | **13239** | 30.2% | 6970 | 14 |

**The trade, stated plainly: superscalar costs 4.24x the LUTs of VexRiscv for
1.45x the performance -- about 2.9x worse performance-per-LUT.** Free if the
fabric has no other claimant; expensive if it does. For scale, the 10119-LUT
difference between them is almost exactly the *entire* free space the 25F had
(10287) at the point where adding one more peripheral stopped routing.

**`--cpu vexii` (single-issue) has no DOOM number yet.** At 7180 LUTs it is 2.3x
VexRiscv rather than 4.2x, so if it captures much of the 1.45x it is the better
point on the curve. Measuring it is one build plus one boot.

## `--relaxed-branch`: tested, makes no difference (2026-09-17)

LiteX adds `--relaxed-branch` to every VexiiRiscv variant unconditionally, and
VexiiRiscv has no negation for it -- it is a bare flag (its `Regression.scala`
dimensions it as `["", "--relaxed-branch"]`). So the trade was untestable from
outside. `patches/litex-vexii-no-relaxed-branch.patch` adds a
`--no-relaxed-branch` sentinel for `--vexii-args`, stripped before it reaches
the generator, and only active when asked for.

It was worth testing because the flag spends a cycle on branch resolution to
buy Fmax, and the sweep says the CPU is not on this design's critical path --
so it looked like a cycle bought with nothing.

Result: **no change.** Build: sys ceiling 44.10 vs 43.30 MHz, 23937 vs 24509
TRELLIS_COMB -- marginally better, within placement noise. DOOM `render`, six
scene-matched 50-frame windows (the attract demo is deterministic, and these
windows lined up):

| window | with | without |
|--------|--------|--------|
| 1 | 85076 | 84903 |
| 2 | 87576 | 87527 |
| 3 | 81370 | 81601 |
| 4 | 87853 | 87957 |
| 5 | 91266 | 91260 |
| 6 | 90165 | 89836 |
| **mean** | **87218 us** | **87181 us** (-0.04%) |

Why nothing moves: the flag delays branch *resolution*, and with BTB/RAS/gshare
on, correctly-predicted branches never wait on it -- only mispredicts pay, and
DOOM's rasterizer does not mispredict enough to show. **This conclusion is
specific to a build with branch prediction enabled.** On `--cpu vexii`
(no BTB/RAS/gshare) there is nothing hiding the latency and it would likely
cost real cycles; that case has not been measured.

**Leave the upstream default on.** It is free either way, so carrying a
divergence buys nothing.

## SETTLED ON HARDWARE (2026-09-17): superscalar wins by 1.45x

DOOM, `targets/colorlight_i9/lcd.py` at sys 90 MHz, WAD read in place from
flash XIP, both CPUs running the same `software/doom` build shape. Numbers are
the `render` phase -- game logic + software rasterizer -- which is 85-90% of
frame time here, so it is very close to the whole story.

| CPU @ 90 MHz            | render (steady, in-game) | total       | fps      |
|-------------------------|--------------------------|-------------|----------|
| VexRiscv                | 126.6 ms (103-136)       | 116-149 ms  | 6.7-8.6  |
| VexiiRiscv superscalar  | **87.2 ms (81-91)**      | 94-104 ms   | 9.6-10.6 |

**1.45x on render, ~1.43x on frame rate, with non-overlapping ranges.** The
static break-even was 1.01x, so this is a large net win and `vexii-superscalar`
is the right default for CPU-bound work on this board. `Memtest OK` at 90/180
on the superscalar build, so the SDRAM margin it spends (123.75 MHz reported)
does not bite at this operating point -- but that is one board at one
temperature, so re-check it if you raise the clock.

**The one place it loses: the title screen**, 75.9 ms against VexRiscv's
70.4 ms. That frame is a static bitmap blit -- memory-bound, almost no
branching -- so the wider core buys nothing and its higher miss cost shows.
The 1.45x is specifically the branchy integer rasterizer, which is exactly the
shape a 2-way core with BTB/RAS/gshare is built for. Do not expect it on
blit-heavy or DMA-bound work (it is worth nothing at all on `lcd_test`).

Caveat on method: the two runs were sampled from different points in DOOM's
attract demo, so the scenes are not frame-matched. With six consecutive
50-frame windows each and no overlap between the ranges, the conclusion is
safe, but treat 1.45x as the magnitude, not a precise figure.

Also measured on the same runs, and the reason these numbers are trustworthy:
`convert` is **0-2 us** (the LCD engine does XRGB8888 -> RGB565 in gateware
now; it was ~12 ms of CPU work) and `flush` is **12.66 ms constant to the
microsecond** on both CPUs, as a fixed-size DMA over a fixed-rate SPI link
should be. A single end-to-end frame time would have hidden both.
