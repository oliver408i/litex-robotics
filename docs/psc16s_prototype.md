# PSC16S prototype — what exists, and what it is for

> **STATUS: PROTOTYPE against a DRAFT ISA.** `docs/psc16s_isa_draft.md` is not
> frozen. There is **no golden model yet**, so `verilog/psc16s.v` is currently
> the only executable statement of the encoding — which is exactly the
> situation the draft says must not become permanent ("the golden model gates
> the ISA"). Treat the RTL as a sketch that compiles, not as a contract.

Built to answer the question the paper sketch could not: does this encoding
survive contact with a real SoC bus and a real toolchain?

## Files

| File | What it is |
|---|---|
| `verilog/psc16s.v` | the core. Multicycle FSM, same shape as `verilog/pisc.v` |
| `gateware/psc16s.py` | LiteX wrapper — CSR peripheral **and** Wishbone master |
| `icepi_zero_psc16s.py` | bring-up top on `BaseSoC`. Sibling of `icepi_zero_pisc.py`, which is untouched |
| `tools/psc16s_asm.py` | v2 assembler. Separate from `tools/pisc_asm.py`, whose encoding is frozen |
| `software/psc16s_test/*.s` | three programs, one per thing worth testing |

v1 is entirely untouched: `pisc.v`, `gateware/pisc.py`, `tools/pisc_asm.py`,
`sim/pisc_model.py`, `icepi_zero_pisc.py` and `docs/pisc_isa.md` all still
describe and implement the frozen v1 core.

## What has actually been checked

Verified 2026-08-27, no hardware:

| Check | Result |
|---|---|
| `yosys synth_ecp5` on `psc16s.v` | clean, 0 problems |
| imem inference | **1 DP16KD**, `DATA_WIDTH_A/B = 0b10010` = 18 — native 18-bit |
| logic | 1437 LUT4, 394 TRELLIS_FF, 90 CCU2C |
| SoC elaboration | `psc16s` allocated as CSR **and** registered as bus master |
| assembler | all three test programs assemble; encodings hand-checked against the draft table |

**Not checked:** anything on hardware, timing closure, and every instruction's
runtime behaviour. Nothing has executed a single instruction — there is no
model and no testbench. The encodings are right; whether the FSM implements
them correctly is unproven.

## Running it

```bash
source oss-cad-suite/oss-cad-suite/environment
.venv/bin/python icepi_zero_psc16s.py --build --load

python3 tools/psc16s_asm.py software/psc16s_test/sum.s --py
# -> load those words at addr >= 256, start_pc = 256, pulse run, poll halted
```

Load/start/poll is v1's flow and `gateware/snn_mlp.py`'s flow; the wrapper
docstring has the CSR sequence.

### Deliberate defaults

**`autostart` is off.** The stage-0 role has the core running at pc=0 out of
configuration while holding the CPU in reset — on a bring-up board that means a
wedged program takes the SoC that is debugging it with it. Bring the ISA up as
a peripheral first.

**The watchdog is off** (`--psc16s-wdt-ms 0`). It exists to force-release a CPU
reset line, and nothing in this top drives one. Turn it on together with
autostart, not before.

**`--psc16s-init` is empty.** The read-only low region is all zeros, which
decodes as `JMP +0` — a blank imem parks the core rather than running garbage.

## The test programs

| Program | Covers | Expect |
|---|---|---|
| `sum.s` | `LI`, `ALU`, `ADDI`, `BNE`, `MOV`, `HLT` + result latch | `result == 55` |
| `bus.s` | `BUS` write + `IN` readback, constant pool, RMW half-store | `result == 42` |
| `seq.s` | `BITOP`, `DELAY`, `WAIT`, `JAL`/`RET` | `result == 0xA5`, after the host sets the ready bit |

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

1. **`sim/psc16s_model.py`.** The draft is explicit that the model gates the
   ISA, and right now the RTL is running ahead of it. Nothing above proves an
   instruction *executes* correctly.
2. **A differential test**, as v1 has: model vs RTL over the three programs and
   then random ones.
3. **The `.config` `.bram_init` patch loop.** The whole bitstream-resident-imem
   decision assumes updating stage-0 is an `ecppack` repack rather than a
   resynthesis. The readback direction is verified (draft "Toolchain facts");
   the patch-and-repack direction is not.
4. **Hardware.** Everything above is simulation-free static analysis.
