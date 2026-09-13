# PSC18SR prototype — what exists, and what it is for

> **STATUS: PROTOTYPE against a DRAFT ISA.** `docs/psc18sr_isa_draft.md` is not
> frozen. There is **no golden model yet**, so `verilog/psc18sr.v` is currently
> the only executable statement of the encoding — which is exactly the
> situation the draft says must not become permanent ("the golden model gates
> the ISA"). Treat the RTL as a sketch that compiles, not as a contract.
>
> Renamed from `PSC16S` on 2026-09-12, when the shifts landed and the name
> question closed. `PSC16S`/`PSC16I` are dead spellings — see the draft's
> "Naming — settled".

Built to answer the question the paper sketch could not: does this encoding
survive contact with a real SoC bus and a real toolchain?

## Files

| File | What it is |
|---|---|
| `verilog/psc18sr.v` | the core. Multicycle FSM, same shape as `verilog/pisc.v` |
| `gateware/psc18sr.py` | LiteX wrapper — CSR peripheral **and** Wishbone master |
| `targets/icepi_zero/psc18sr.py` | bring-up top on `BaseSoC`. Sibling of `targets/icepi_zero/pisc.py`, which is untouched |
| `targets/colorlight_i9/psc18sr.py` | the same idea on the i9: core as a **non-boot side block**, CPU boots normally |
| `software/psc18sr_host/` | RV32 firmware that loads, runs and checks the programs from the CPU side |
| `tools/socboot.py` | non-interactive serial boot + console capture, with an exit status. Not PSC18SR-specific |
| `tools/psc18sr_asm.py` | v2 assembler. Separate from `tools/pisc_asm.py`, whose encoding is frozen |
| `sim/psc18sr_tb.v` | self-checking testbench: runs a program, checks `result`. **Not** the golden model |
| `software/psc18sr_test/*.s` | four programs, one per thing worth testing — the specification the firmware checks against |

v1 is entirely untouched: `pisc.v`, `gateware/pisc.py`, `tools/pisc_asm.py`,
`sim/pisc_model.py`, `targets/icepi_zero/pisc.py` and `docs/pisc_isa.md` all still
describe and implement the frozen v1 core.

## What has actually been checked

Verified 2026-08-27, no hardware:

| Check | Result |
|---|---|
| `yosys synth_ecp5` on `psc18sr.v` | clean, 0 problems |
| imem inference | **1 DP16KD**, `DATA_WIDTH_A/B = 0b10010` = 18 — native 18-bit |
| logic | 1437 LUT4, 394 TRELLIS_FF, 90 CCU2C |
| SoC elaboration | `psc18sr` allocated as CSR **and** registered as bus master |
| assembler | all three test programs assemble; encodings hand-checked against the draft table |

Added 2026-09-12, with the shifts and the rename, still no hardware:

| Check | Result |
|---|---|
| `yosys synth_ecp5` after the shifts | clean; 1457 LUT4 (+20), 394 TRELLIS_FF, 90 CCU2C, **1 DP16KD** unchanged |
| RTL simulation (`sim/psc18sr_tb.v`, iverilog) | `sum.s` → 55, `shift.s` → 0x5A, `seq.s` → 0xA5 |
| the 3-cycle property, measured | `shift.s` is 30 instructions, 5 of them `LD`: 25×3 + 5×4 = **95 cycles**, exactly what the bench reports. The shifts cost 3, like everything else |
| bitstream-resident entry | the same image loaded as `INIT_FILE` with `AUTOSTART=1` runs identically from pc=0 |
| i9 SoC build | `targets/colorlight_i9/psc18sr.py` places, routes and closes timing on the 45F — see "On hardware" below |
| **hardware** | **all ten firmware checks pass on a real i9**, and `DELAY` measures 429993 cycles against 429954 in simulation |

**Now checked:** three of the four programs execute correctly on the RTL,
end to end, through both entry paths.

**Still not checked:** anything on hardware; timing closure; `bus.s`, which
needs a Wishbone target and so a real SoC, not the bench; and every
instruction the four programs do not reach. There is still **no golden model**,
so nothing proves the *encoding* is right — only that the FSM does what this
file's encoding says. Post-synthesis gate-level simulation was attempted and
abandoned: the netlist would not initialise under `cells_sim.v` in iverilog, a
bench-setup problem rather than a finding, and not worth the chase before
hardware.

## Running it

```bash
source oss-cad-suite/oss-cad-suite/environment
.venv/bin/python targets/icepi_zero/psc18sr.py --build --load

python3 tools/psc18sr_asm.py software/psc18sr_test/sum.s --py
# -> load those words at addr >= 256, start_pc = 256, pulse run, poll halted
```

Load/start/poll is v1's flow and `gateware/snn_mlp.py`'s flow; the wrapper
docstring has the CSR sequence.

No board needed for the programs themselves:

```bash
python3 tools/psc18sr_asm.py software/psc18sr_test/shift.s --hex /tmp/p.hex --words 1024
iverilog -g2012 -o /tmp/tb.vvp sim/psc18sr_tb.v verilog/psc18sr.v
vvp /tmp/tb.vvp +hex=/tmp/p.hex +expect=0x5a          # -> PASS
```

`seq.s` parks in `WAIT`, so it needs the host poke the bench fakes with
`+in=1 +in_after=20000`.

### Deliberate defaults

**`autostart` is off.** The stage-0 role has the core running at pc=0 out of
configuration while holding the CPU in reset — on a bring-up board that means a
wedged program takes the SoC that is debugging it with it. Bring the ISA up as
a peripheral first.

**The watchdog is off** (`--psc18sr-wdt-ms 0`). It exists to force-release a CPU
reset line, and nothing in this top drives one. Turn it on together with
autostart, not before.

**`--psc18sr-init` is empty.** The read-only low region is all zeros, which
decodes as `JMP +0` — a blank imem parks the core rather than running garbage.

## On hardware: the i9 side-block rig

`targets/colorlight_i9/psc18sr.py` is where the core gets proven against real
silicon, and the arrangement is deliberately the safe half of its two roles:
**the CPU boots normally and the core hangs off the side**. `autostart` is off,
the watchdog is off, no reset line goes through it, nothing the SoC needs is
downstream of it. A wedged program costs a `control.abort` write from a CPU
that is still alive and still holding the console — which is the opposite of
the stage-0 role, where a wedged program takes the SoC with it. Debugging an
unproven ISA belongs on this side of that line; the boot role is a different
top, built when there is a golden model.

    .venv/bin/python targets/colorlight_i9/psc18sr.py --build --load
    cd software/psc18sr_host && make
    .venv/bin/python tools/socboot.py \
        --kernel software/psc18sr_host/psc18sr_host.bin \
        --expect "all passed" --fail FAILED

`tools/socboot.py` rather than `litex_term` on purpose: `litex_term` is
interactive and calls `termios.tcgetattr` on stdin at startup, so it cannot be
run from a script at all. `socboot` does the same serial boot without a tty,
exits on `--expect`/`--fail`, and returns an exit status, so "did the core
pass" is a shell conditional. It also owns the reset question -- `--reset soft`
is the BIOS `reboot` command (1.2 s measured, and it needs the BIOS to be the
thing listening, so it does nothing while a firmware owns the UART),
`--reset hard --bitstream <file>` reconfigures the FPGA through the probe
(7.3 s to a booted firmware), and `auto` tries the first then falls back to the
second (13.3 s).

One thing the hard path has to work around, because it cost an hour to find:
**the BIOS's boot-time serial-boot offer cannot be answered after a
reconfigure.** The BIOS sends its magic exactly once per boot with a short ack
window, and the probe does not bridge the target UART while it is shifting
JTAG -- so the magic emitted at 5.2 s is not readable by the host until
programming finishes at 6.4 s. It is not a race that can be won by listening
harder (threading it was tried); the bytes do not exist yet. `socboot` lets the
BIOS fall through to its prompt and sends `serialboot`, which re-offers it live.
The same investigation found that closing the console CDC around a reconfigure
leaves the probe's UART bridge at the wrong baud. Both are recorded in
`probe/icelink-fast/README.md`.

The firmware runs five tests and prints PASS/FAIL per line:

| Test | What a pass proves |
|---|---|
| `sum.s` → 55 | compute and control flow execute on real silicon |
| `shift.s` → 0x5A | `SLL`/`SRL`/`SRA`, or the index of the check that failed |
| `seq.s` | `BITOP` drives a real port; the `DELAY` pulse is measured against `415 × prescale` cycles and checked to ±10%; `WAIT` blocks and then releases on `gpio_in` |
| abort | the escape from an unbounded `WAIT` — sketch finding F5, reproduced deliberately and then escaped |
| `bus.s` → 42 | the Wishbone window: the core writes its own `gpio_in` CSR and reads it back through `IN`, and the CPU sees the same value |

Two things make that possible without hand-maintained duplication. The
programs are regenerated into C arrays from `software/psc18sr_test/*.s` by the
firmware's Makefile (`tools/psc18sr_asm.py --c`), so the `.s` files stay the
specification. And `add_psc18sr` exports `PSC18SR_RO_WORDS` /
`PSC18SR_DELAY_PRESCALE` as generated constants, so a build with those
overridden still tests itself rather than testing the defaults — the `--c`
output's label defines are what let the firmware patch `bus.s`'s constant pool
with a CSR offset that is only knowable at runtime.

Build results, 2026-09-12, LFE5U-45F at 48 MHz:

| | |
|---|---|
| place & route | fits; 8095/43848 TRELLIS_COMB (18%), 3386 FF (7%), 52/108 DP16KD (48%) |
| timing | **48.81 MHz vs 47.73 MHz required — PASS, but only ~1 MHz of margin** |
| firmware | builds clean against the generated CSR header |

The thin timing margin is the thing to watch: the plain `bringup.py` closes 48
MHz comfortably and this does not, so read the reported Fmax on every build
rather than treating a bitstream as proof. If it ever fails, the honest fixes
are a lower `--sys-clk-freq` or a different seed, not `--timing-allow-fail`,
which is already on in this flow and is why a failing build still produces a
bitstream.

### It runs. Verified on hardware 2026-09-12

All ten checks pass on a Colorlight i9 v7.2 at 48 MHz, reproducibly, with the
bitstream loaded through the probe and the firmware serial-booted into SDRAM:

```
PSC18SR side-block self-test:
  sum.s   (LI/ALU/ADDI/BNE): got 0x0037 want 0x0037   PASS
  shift.s (SLL/SRL/SRA): got 0x005a want 0x005a   PASS
  seq.s   reset-low pulse: 8958 us (429993 cycles)
  seq.s   DELAY within 2% of expected: got 0x0001 want 0x0001   PASS
  seq.s   (BITOP/DELAY/WAIT/JAL): got 0x00a5 want 0x00a5   PASS
  abort   stops a wedged WAIT: got 0x0000 want 0x0000   PASS
  abort   is distinguishable from HLT: got 0x0001 want 0x0001   PASS
  bus.s   gpio_in CSR at window offset 0x0028
  bus.s   BUS write -> IN readback: got 0x002a want 0x002a   PASS
  bus.s   CPU sees the same CSR: got 0x002a want 0x002a   PASS
all passed
```

The line worth keeping is the third: **429993 cycles on silicon against 429954
in simulation**, a 39-cycle difference which is the CPU's polling granularity
at each end of the measurement, not the core. `DELAY` is exact, which is the
one property the whole design is arranged around, and it is now measured
rather than argued.

So the instructions execute correctly on real hardware — including the shifts,
the sequencing instructions, the Wishbone window, and the abort escape from a
deliberately wedged `WAIT`. What that does **not** establish is that the
*encoding* is right: there is still no golden model, and these programs reach
only the instructions they reach. Four programs passing is not an ISA.

## The test programs

| Program | Covers | Expect |
|---|---|---|
| `sum.s` | `LI`, `ALU`, `ADDI`, `BNE`, `MOV`, `HLT` + result latch | `result == 55` |
| `bus.s` | `BUS` write + `IN` readback, constant pool, RMW half-store | `result == 42` |
| `seq.s` | `BITOP`, `DELAY`, `WAIT`, `JAL`/`RET` | `result == 0xA5`, after the host sets the ready bit |
| `shift.s` | `SLL`/`SRL`/`SRA`, `LD` from a pool, byte unpacking | `result == 0x5A`; any other value is the index of the check that failed |

`shift.s` leads with the case that caused the instructions to exist at all —
unpacking two ASCII characters from one word, draft open question 13 — rather
than with an abstract shift, so a failure says which property broke.

`bus.s` is the one v1 could not have: the core writes its own `gpio_in` CSR
through the Wishbone window and reads it back through `IN`. A pass exercises
the window base, the address arithmetic, the read-modify-write half-word store,
and the CSR landing where `IN` can see it — with no other peripheral having to
cooperate.

`seq.s` deliberately parks in `WAIT` until the host sets a bit. That stall is
the hazard from sketch F5, reproduced on purpose so the escape paths
(`control.abort`, and on a stage-0 build the watchdog) have something to
escape from.

## Prototype decisions that are NOT in the draft

Two things the RTL had to settle that the draft leaves loose. Both need folding
back into the draft, or overriding.

**1. `PORT` register field by direction.** `dir=0` (IN) uses the `rd` field as
destination; `dir=1` (OUT) uses the `rs` field as source. This follows the
draft's pseudo-op table (`IN rd,p` / `OUT p,rs`) but the draft never says it.

**2. `BUS` half-word writes are read-modify-write.** LiteX's `Wishbone2CSR`
ignores `wb_sel`, so a 16-bit write must read the 32-bit word back first — two
bus cycles per write, one per read. **This is wrong for any CSR with a read
side effect.** Every CSR stage-0 touches today is a plain `CSRStorage`, which
reads back what it holds, so it is safe there and nowhere promised beyond
that. The alternative — a staging register holding the other half across two
`BUS` instructions — is stateful in a way that is easy to get wrong in the
model, which is why RMW won for the prototype.

## What is owed

In rough order:

1. **`sim/psc18sr_model.py`** — but **not yet, and on purpose.** The draft is
   explicit that the model gates the ISA, and the RTL is running ahead of it;
   that is tolerable only while the encoding is still moving, which the
   2026-09-12 shifts demonstrate it is. Writing it now buys a model that gets
   rewritten. It is owed when the draft banner comes off. `sim/psc18sr_tb.v`
   covers the weaker question in the meantime — do the instructions execute —
   and that much is now answered for three programs.
2. **A differential test**, as v1 has: model vs RTL over the four programs and
   then random ones. Blocked on 1, by the same reasoning.
3. **The `.config` `.bram_init` patch loop.** The whole bitstream-resident-imem
   decision assumes updating stage-0 is an `ecppack` repack rather than a
   resynthesis. The readback direction is verified (draft "Toolchain facts");
   the patch-and-repack direction is not.
4. **Hardware.** Everything above is simulation-free static analysis.
