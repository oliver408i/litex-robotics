# PSC18SR — draft ISA (successor to PISC v1)

> **STATUS: DRAFT. Nothing here is frozen.**
>
> This is deliberately the opposite of `docs/pisc_isa.md`, which says "if any
> implementation disagrees with this table, this table wins." That freeze did
> its job for v1 and **still holds for v1** — `docs/pisc_isa.md` is untouched
> and remains the contract for the existing core.
>
> This document is a working sketch. Every table, field width, opcode
> assignment, and name below is provisional and expected to change. Do not
> implement against it yet. It becomes a contract only when it says so.

## Naming — settled 2026-09-12

**The core is `PSC18SR`.** `PSC16S` and `PSC16I` are dead spellings; nothing
in the tree should use them.

The earlier sketch's `PSC16I` → `PSC16S` was wrong in a way worth keeping on
record so it is not re-proposed. A trailing letter reads as an *extension*
marker, because RV32I / RV32IMAC trained everyone to parse it that way: `I`
looks like "base integer" and `S` like a supervisor extension. What is
actually being distinguished here is **generation and role**, not an
extension. The digit was separately ambiguous — 16 could be read as data or
instruction width.

`PSC18SR` fixes both halves:

- **18** is the **instruction** width, which is the thing that actually
  distinguishes this machine (and is not an accident — see "Why 18 bits is
  free"). Data stays 16-bit; that is stated in the machine model rather than
  smuggled into the name.
- **SR** is *sequencer*, the role. Two characters that read as one word, not
  as an extension letter.

The family name in the tree is still **PISC** (`gateware/pisc.py`,
`verilog/pisc.v`, `docs/pisc_isa.md`) and v1 keeps it; nothing is renamed
retroactively.

Independent of the name: the core's role is no longer purely IO-oriented. It
boots the SoC, touches the CSR bus, and holds the main CPU in reset until it
is ready.

## Why v2 exists

v1 was specced for peripheral choreography — reset ordering, pulse widths,
"wait for this pin, drive that one." Promoting it to stage-0 boot sequencer
(hold CPU in reset, configure flash, init panel, emit UART markers, release
CPU) exposed hard limits:

| v1 limit | Consequence at boot scale |
|---|---|
| No subroutines | The CSR-write pattern repeats 100+ times; inlining blows the imem |
| `BNE` reaches ±31 words | Constant hand-written trampolines in a 1000+ word program |
| Only `BNE`, no `BEQ` | Every equality test costs an inverted branch over a `JMP` |
| No 16-bit immediate | Constant pool works, but `imm6` only reaches ±31 words of it |
| No bus access | Cannot reach the flash divisor CSR or the LCD engine at t=0 |

The first three are the ones that actually hurt.

**Amended after the stage-0 sketch (`docs/psc18sr_stage0_sketch.md`):** the
"inlining blows the imem" line above is the weakest of the five. Written out,
the real stage-0 sequence is ~26 instructions — capacity was never the binding
constraint. What v1 actually lacks is `JAL`/`RET` and `BEQ` *existing at all*,
plus branch reach. That matters because "we need more imem" and "we need
subroutines" point at different designs, and only the second is supported by
evidence (sketch F7).

## The design line

The thing that justifies a custom core at all is `WAIT`, `DELAY`, and `BITOP`
— **deterministic timing as single instructions**. A RISC-V core does these as
loops; a bit-serial one (SERV, ~30–70 cycles/instruction) makes "assert reset
for exactly 500 µs" an exercise in cycle counting.

Everything else in this ISA is generic CPU.

> **Rule:** an instruction earns its place only if it serves deterministic
> sequencing, or boot duty requires it. "A normal CPU would have this" is a
> reason to leave it out.

If the generic half ever outgrows the sequencing half, SERV plus gcc wins on
every axis — real toolchain, no vasm backend, no golden model, no three-way
contract — and we should switch rather than keep going.

**The cost of an instruction is not gates.** It is that every defined
instruction must be implemented and tested in three places (`verilog/psc18sr.v`,
`sim/psc18sr_model.py`, the vasm backend) and agreed with this doc. Reserved
encoding space is free; defined encoding space is a permanent tax.

## Machine model (provisional)

- **Instruction width:** 18 bits.
- **Data width:** 16 bits, two's complement, wraps mod 2^16.
- **Memory:** unified, word-addressed, 18 bits wide. Data occupies the low 16
  bits; the top two lanes are unused for data. **Default 1024 words (= 1
  DP16KD)**, split by a synthesis-time write-protect boundary:

  | Region | Words | Contents | Host-writable |
  |---|---|---|---|
  | low | 0 – 255 | stage-0, `INITVAL` from the bitstream | **no** |
  | high | 256 – 1023 | post-boot programs, CPU-loaded over CSR | yes |

  2048 was the earlier default on an assumption that boot code would be large.
  The stage-0 sketch says ~26 instructions (F7), and v1's entire default memory
  was 256 words, so 1024 covers both jobs in **one** EBR — which matters on a
  board where the XIP BIOS exists to free EBR for the SNN builds.
- **Registers:** `r0`–`r7`. `r0` hardwired to 0. `r7` latched into the
  `result` CSR at `HLT`.
- **Link register:** `r6`, fixed (see open questions).
- **I/O ports:** up to 64 out, 64 in, 16 bits each — unchanged from v1.
  Output ports have a **synthesis-time reset value**, applied at configuration
  before the first instruction retires. This is new in v2 and it is
  safety-critical: the port bit holding the main CPU in reset must come out of
  configuration *asserted*, or there is a window where the CPU runs against an
  unconfigured SoC. v1 never specified a power-on value because nothing
  depended on it.
- **Bus window:** 64K of CSR space at a synthesis-time base. See below.
- **Execution:** one instruction at a time. No interrupts, no pipeline hazards
  visible to software.
- **Start:** two entry paths, which is what lets one core do both jobs.
  **Autostart at configuration/reset, entering at PC=0** — no CPU is involved
  because none is running yet. **Post-boot, the CPU writes `start_pc` and
  pulses `run`.** See "Autostart, HLT, and restart".
- **Clock:** `sys`. The core is held in reset until `pll.locked`, so it never
  executes against an unlocked clock and every `DELAY` in a program counts
  against one timebase.

### Why 18 bits is free

The ECP5 DP16KD is natively 18 bits wide (geometries: 16384×1, 8192×2,
4096×4, 2048×9, 1024×18, 512×36). A 16-bit imem already sits in a 1024×18
block wasting two lanes per word. Verified 2026-08-26 with oss-cad-suite
(yosys 0.66+2), see "Toolchain facts" below.

## Encoding (provisional)

```
 17  14 | 13  11 | 10   8 | 7                     0
 opcode |   rd   |   rs   |  imm8 / rs2+funct5 / port+bits
```

4-bit opcode, 16 slots. The ALU funct field keeps that from being tight.

| op | mnemonic | fields | effect |
|---|---|---|---|
| `0x0` | `JMP off` | imm14 | `PC += sext14(off)` — ±8191 |
| `0x1` | `ALU rd,rs,rs2` | rs2[7:5], funct[4:0] | see funct table (incl. shifts) |
| `0x2` | `ADDI rd,rs,imm8` | imm8 signed | `rd = rs + sext8(imm8)` |
| `0x3` | `LI rd,imm11` | rs+imm8 | `rd = sext11(imm11)` |
| `0x4` | `LD rd,rs,imm8` | | `rd = mem[rs + sext8]` (low 16 bits) |
| `0x5` | `ST rd,rs,imm8` | | `mem[rs + sext8] = rd` |
| `0x6` | `BEQ rd,rs,off` | imm8 | ±127 |
| `0x7` | `BNE rd,rs,off` | imm8 | ±127 |
| `0x8` | `JAL off` | imm14 | `r6 = PC+1; PC += sext14` |
| `0x9` | `JALR rs,imm8` | | `r6 = PC+1; PC = rs + sext8` |
| `0xA` | `PORT rd/rs,dir,port` | dir[7], port[5:0] | merged `IN`/`OUT` |
| `0xB` | `BITOP port,#bit,lvl` | bit[11:8], lvl[7], port[5:0] | merged `SETB`/`CLRB` |
| `0xC` | `WAIT port,#bit,lvl` | bit[11:8], lvl[7], port[5:0] | block until match |
| `0xD` | `BUS rd,rs,dir,half` | dir[7], half[6], imm6 | CSR access, see below |
| `0xE` | `DELAY imm14` | | stall imm14 counts × prescale |
| `0xF` | `HLT` | | latch `r7` → `status`, raise `halted`, stop fetching |

Branch and jump offsets are relative to the branching instruction's own PC,
as in v1 (`target = PC_of_branch + offset`).

`0x00000` still decodes as `JMP +0` — an infinite self-loop, so a blank imem
parks the core rather than running garbage. This property matters more in v2
than v1: the core holds the CPU reset line, so "blank program does nothing
safely" is a boot-safety guarantee, not just tidiness.

### ALU funct field — deliberately mostly reserved

| funct | op |
|---|---|
| `0x00` | `ADD` |
| `0x01` | `SUB` |
| `0x02` | `AND` |
| `0x03` | `OR` |
| `0x04` | `XOR` |
| `0x05` | `SLL` |
| `0x06` | `SRL` |
| `0x07` | `SRA` |
| `0x08`–`0x1F` | **reserved — do not assign** |

The field is 5 bits because reclaiming it was free, not because 32 operations
are wanted. Rotates, bit-count, min/max and the rest are still intentionally
*not* defined: adding one later costs nothing in encoding and everything in
contract surface; see the design line above.

### The shifts (added 2026-09-12) — and why they are combinational

`SLL`/`SRL`/`SRA` are ordinary ALU ops: `rd = rs <shift> rs2`. The **amount is
`rs2[3:0]`, masked, not saturated** — every amount 0–15 is defined and 16 or
more wraps, so a shift can never trap or stall. There is no shift-immediate
form; shifting by a literal is `LI` plus the shift, two instructions.

This is the one place the design line was argued and *lost* on purpose, so the
reasoning is recorded rather than the verdict alone:

- They were found the way the sketch says things should be found — by writing
  actual programs and hitting the wall, not by review. Open question 13 below
  predicted the exact wall (packing two bytes per word) and it arrived.
- The funct field was reserved for precisely this, which is why the cost is
  three encodings rather than an opcode.
- They are **general**, which is the point. A fixed-purpose "unpack a byte"
  instruction would have served the same program and been dead weight for the
  next one.

**Combinational, resolved inside `S_EXEC` — never iterative.** Every
instruction usable inside a bit-bang loop costs exactly **3 cycles**, branches
taken *and* not-taken included (they share the FSM path). `LD` is the lone
4-cycle outlier; `BUS`, `WAIT` and `DELAY` are variable by design and by name.
That uniformity is what makes IO timing computable by hand, and it is the only
reason this core exists instead of SERV. A data-dependent shifter — a cheaper
implementation that shifts one place per cycle — would silently make every
loop containing a shift untimeable. **Any future ALU op must hold the same
line: if it cannot complete in one `S_EXEC` cycle, it does not go in.**

Cost, measured on the prototype (`yosys synth_ecp5`, LFE5U-25F): 1437 → 1457
LUT4, one DP16KD unchanged, and the mux primitives fall enough that total
logic cells land slightly *below* the pre-shift build. The shifter is not what
makes this core big.

### Pseudo-ops

Each assembles to exactly one real instruction.

| pseudo | expands to |
|---|---|
| `NOP` | `ALU r0, r0, r0` (funct `ADD`) |
| `MOV rd, rs` | `ALU rd, rs, r0` (funct `ADD`) |
| `RET` | `JALR r6, 0` |
| `SETB p,#b` | `BITOP p, #b, 1` |
| `CLRB p,#b` | `BITOP p, #b, 0` |
| `IN rd,p` | `PORT rd, dir=0, p` |
| `OUT p,rs` | `PORT rs, dir=1, p` |

## Autostart, `HLT`, and restart

In v1 the core was purely a **peripheral**: the host CPU loaded imem over CSRs,
pulsed `run`, and polled `halted`. `HLT` was a *reply* — the core answering
upward to its master.

v2 keeps that relationship and adds one it cannot have: at t=0 there is no
upward, because this core is holding the CPU in reset. So stage-0 is the core
**leaving a note for the successor it is about to start**, and everything after
that is v1's peripheral model unchanged.

One core serves both because it has two entry paths and a `HLT` that is
**terminal for the run, not for the core**:

| | Entry | Loaded from | Ends at |
|---|---|---|---|
| stage-0 | autostart, PC=0 | bitstream `INITVAL`, read-only | `HLT` → CPU reads `status` |
| post-boot | CPU writes `start_pc`, pulses `run` | CSR writes into the high region | `HLT`, or CPU pulses `abort` |

`HLT` latches `r7` into `status`, raises `halted`, and stops fetch. It does not
lock anything out and does not need reconfiguration to undo — the CPU can start
the core again at a different `start_pc`.

**Why the low region is read-only:** autostart always enters at PC=0. If the
CPU could write there, a program it loaded for a post-boot job would silently
become the *boot* sequence on the next warm reset, executed by a core that
holds the CPU reset line. The boundary is what makes reuse safe; it is not a
security control (see the bus window, below) but a "you cannot accidentally
redefine boot" control.

Control interface — essentially v1's, plus `start_pc`, and the same
load/start/poll shape already working in `gateware/snn_mlp.py`:

`start_pc`, `run` (strobe), `abort` (strobe, kills a hung post-boot program),
`halted` (status), `status` (`r7`), imem write window.

`r7` is therefore a general return value, as in v1 — for stage-0 the convention
is 0 = success, nonzero = the stage that failed, read by the very CPU this core
just released. Note this means the boot-attempt counter cannot live in
`status`; it still wants a `reset_less` CSR.

### The bricking hazard, and the one invariant that covers it

This core holds the CPU reset line. Three different roads end with stage-0
stopped and the CPU never released: `BUS` with no ack, `WAIT` on a pin that
never arrives, and an ordinary status-poll loop that never exits (sketch F5).
A fourth is a program that simply executes `HLT` without releasing.

Rather than a timeout field on `BUS`, another on `WAIT`, and a rule about what
`HLT` does to a port bit, state the property once and enforce it outside the
ISA:

> **The CPU is released within N ms of configuration, no matter what the
> program does.**

A watchdog armed at reset, firing unconditionally at N ms unless the CPU-reset
port bit has already been deasserted. On expiry: deassert it, latch a
distinguished `r7` (`0xFFFF`), raise `halted`. N ≈ 500 ms — well above the
~130 ms real sequence, well below "the human thinks the board is dead".

Three properties worth noting:

- It covers all four roads uniformly, including the ones software cannot fix.
- **The ISA never learns which port bit is the CPU reset.** Only the watchdog
  does, as a synthesis parameter. That is the whole reason to put it here
  rather than in `HLT`.
- **It disarms permanently once the CPU is released**, so post-boot programs —
  which may legitimately run for a long time — are never watched. If one hangs,
  the CPU is alive and pulses `abort`.

No kick instruction: a whole-program deadline is a stronger guarantee than a
refreshable one and adds no contract surface.

**The invariant is load-bearing, not belt-and-braces.** It reads like a
backstop for buggy programs, but the extension mechanism below makes it the
primary safety property: the intended idiom for a port-mapped peripheral is
`WAIT busy,0`, and `WAIT` is unbounded by design. A wrong clock divider, a
peripheral held in reset, a block that never deasserts BUSY — each of those
hangs stage-0 in a *correct* program. Nothing in the ISA can bound it, because
"how long should this take" is not knowable from the encoding. The watchdog is
what makes that idiom safe to write, so it must be **on** for any autostart
build that waits on a peripheral, not just for ones with suspect software.

## The bus window

`BUS` addresses a **64K window** whose base is a **synthesis-time parameter**
(the SoC CSR base, typically `0xf0000000`). Consequences, all intentional:

- No registers are burned staging a 32-bit address.
- The core **structurally cannot** address SDRAM, the flash mmap, or anything
  outside the window. Reach is limited by the encoding, not by convention.

`half` selects the low or high 16 bits of a 32-bit CSR, so a full CSR write is
two instructions.

**There is no lockout latch.** An earlier draft had one — set at `HLT`, not
clearable until reconfiguration — so the core went from "full CSR window"
during boot to "pins only" afterward. It is dropped, for two reasons:

1. It buys **only** privilege separation, which is not a goal here. The
   structural limit above is the half that is free, and it stays.
2. It breaks a post-boot job we actually want. On the `mnist_lcd` build
   `LCD_RST` lives on the MCP23S17 on the aux SPI bus
   (`docs/reset_sidebands.md`), so driving it means writing `aux_spi` CSRs —
   `BUS` access *after* boot. A lockout would make the core's own original job
   unreachable on that build.

What remains bounded is *reach* (the window) and *when boot can be redefined*
(the read-only low imem region), not *when the bus is available*.

## Extending the machine: ports, not opcodes

**Decided 2026-09-12, before any of it was built.** As soon as stage-0 has to
push real traffic — an SPI-attached panel, a flash divisor, an expander — the
question is how the core gets faster at it than bit-banging. Two roads were
considered and rejected, and one taken.

**Rejected: a dedicated engine reached by new opcodes.** An SPI-shaped
instruction (or a family of them) would be the fastest thing to write and the
most expensive thing to own. It spends ISA surface on one peripheral, has to
be modelled and agreed in three places forever, and is exactly the
fixed-purpose shape the design line exists to refuse.

**Rejected: reaching such an engine through `BUS`.** `BUS` already reaches
every CSR, so this costs no encoding at all — but `BUS` is the one instruction
whose timing is not uniform (variable on `wb_ack`, and a half-word write is
read-modify-write, so two bus round trips). Putting the hot path through it
gives up the property the shifts were just kept combinational to protect.

**Taken: hang blocks off the existing 16-bit IO ports.** `OUT`/`IN`/`SETB`/
`CLRB`/`WAIT` already exist, so a new peripheral costs **zero ISA surface**.
A byte transfer becomes

    OUT   tx, rX          ; data
    SETB  ctl, START      ; go
    WAIT  sts, BUSY, 0    ; block until the engine is done
    IN    rY, rx          ; result

— roughly 9 core cycles plus the engine's own time, against ~72 to bit-bang
the same byte. `NUM_OUT`/`NUM_IN` are synthesis parameters, so ports can be
allocated for a new block without touching this document. **The encoding can
freeze while the hardware keeps changing**, which is the whole argument: a
frozen ISA and a growing SoC stop fighting each other.

Two consequences, both already stated above but worth naming here: the
watchdog becomes the safety property that makes `WAIT busy,0` writable, and
everything on that path stays 3 cycles except `WAIT` itself.

**Measure before building any of it.** The SoC already has SPI masters
(`gateware/aux_spi.py`, `AuxSPIMaster`, CSR at `0xf0000800` — inside the `BUS`
window), so the first question is whether driving *that* over `BUS` is simply
good enough. Nobody knows yet, because no real stage-0 program exists and so
nothing has measured what it has to push. The non-boot side instantiation is
the rig for answering it; building an engine before that measurement would be
building to a guess.

## Toolchain facts (verified 2026-08-26, no hardware)

Run against oss-cad-suite 2026-06-02 (yosys 0.66+2), target LFE5U-25F-6CABGA256.
Test files in scratch, reproducible from the walking-1 pattern described below.

| Check | Result |
|---|---|
| `1024×18` inference | 1 DP16KD (1/56 on the 25F) |
| `DATA_WIDTH_A` | `0b10010` = 18 — native 18-bit mode, not 9+9 |
| `2048×18` inference | 2 DP16KD exactly |
| INITVAL after synthesis | 1024/1024 words bit-exact, all 18 lanes |
| `.bram_init` after nextpnr | 1024/1024 words bit-exact, all 18 lanes |
| `ecppack` | clean |

Method: words 0–17 held a walking `1` (so each of the 18 lanes carries a lone
set bit in some word), remainder pseudorandom. Words 16 and 17 (`0x10000`,
`0x20000`) specifically prove the two lanes that would be "parity" in 9-bit
modes. Post-P&R `.bram_init` stores 9+9 splits, which decodes correctly.

**Not yet verified:** silicon readback. Requires the board connected. Risk is
low — `ecppack` is a deterministic transform of a `.config` verified
word-for-word — but it is unproven. The worthwhile version of that test is a
minimal LiteX SoC with `add_jtagbone()` plus the 18-bit ROM, which also brings
up jtagbone end-to-end with the CH347.

## Open questions

### Resolved 2026-08-27 — folded into the text above

Kept as a record, since these were the ones blocking everything else.

1. **How does imem get loaded?** Both ways, split by region: low 256 words
   bitstream-resident (`INITVAL`, not host-writable), high region CSR-written
   exactly as in v1. **One core serves both jobs.** A second core dedicated to
   stage-0 was considered and rejected: it would sit idle from ~130 ms onward
   and cost an EBR plus its own decode/ALU, where reuse costs three CSRs and a
   comparator on the imem write port.
2. **Reset domain.** Soft-reset domain, same as the CPU. A warm reset re-runs
   stage-0, which is *correct* — the panel needs re-init and the flash divisor
   re-established before the CPU re-XIPs. The mutation hazard that made this
   look dangerous is handled by the read-only low region, not by the reset
   domain. Boundary value: **256 words**.
3. **Which clock.** `sys`, core held in reset until `pll.locked`. Nothing
   stage-0 touches lives outside `sys`, so there is nothing to do before lock —
   and this deletes the `WAIT`-on-PLL-lock stall road at the source rather than
   catching it. The ECP5 internal oscillator was the alternative, disqualified
   by its tolerance, which is the one thing this core cannot trade away.
4. **`DELAY` prescale.** Default 1024, synthesis parameter, model takes it as a
   constructor argument. 12.05 µs/count, 197.4 ms max against `sys` at 85 MHz;
   the real sequence needs 5 ms and 120 ms (sketch F4).
5. **What bounds a stalled stage-0?** One watchdog enforcing one invariant, not
   per-instruction timeouts. See "The bricking hazard".
6. **Should `HLT` force the CPU-reset port to deassert?** No. The watchdog
   invariant supersedes it and keeps the ISA from having to know which port bit
   is the CPU reset.

Also settled, outside the numbering: **no bus lockout latch** (see "The bus
window"), and **`HLT` is restartable**, not terminal.

### Resolved 2026-09-12

10. **`BITOP`/`WAIT` bit field is 4 bits, reaching all 16 bits of a port.**
    This was *already true in the RTL* — `bsel = ir[11:8]` — and the draft
    simply had not caught up; the `rd` field `[13:11]` is unused in both
    instructions, so the width was free. Folded back here rather than left as
    a question the code had quietly answered. The port-mapped extension
    mechanism makes it load-bearing: control and status bits for a new block
    land wherever they land in a 16-bit port, not politely in the low half.
13. **Shifts exist: `SLL`/`SRL`/`SRA` at funct `0x05`–`0x07`.** The predicted
    pressure (packing two bytes per word) arrived, from writing programs
    rather than from review, which is the sketch's own method working. See
    "The shifts" above for the reasoning and for the constraint the
    implementation has to hold. The naming question above is resolved in the
    same breath: this is what the `18` in `PSC18SR` was waiting on.

### Still open

All of the following have a leaning recorded; none blocks the model.

7. **`LD` width — 16 or 18?** Truncating to 16 is assumed above. Exposing all
   18 lanes gives two free flag bits per data word but invites one very
   confusing bug in a nominally 16-bit machine. Leaning 16; leave the lanes for
   a future tag use that can be specced deliberately.
8. **Does `r0 = 0` survive?** Costs one of eight registers. With `LI` reaching
   ±1024 the cheap-zero argument weakens, but `MOV`/`NOP` depend on it and
   `BEQ rd,r0` is the natural zero test. Leaning keep.
9. **Fixed `r6` link vs `JAL rd`.** Fixed is assumed above; it frees three bits
   so `JAL` reaches ±8191 instead of ±1024. Costs nested calls, which boot code
   is unlikely to need. Leaning fixed — it is the less general, more
   sequencer-shaped choice, which under the design line is a point in favour.
   Note that `RET` = `JALR r6, 0` rewrites `r6` as it returns, so a frame
   cannot be returned from twice.
11. **Assembler: extend `tools/pisc_asm.py`, or a vasm backend?** The draft
    assumes vasm in "What must stay true", but nothing vasm exists in the tree
    and v1 ships a working homegrown assembler. This is an unstated decision
    that changes how much work "four implementations must agree" actually is.
12. **`BUS` `imm6` reaches ±31 bytes = 7 LiteX CSR registers.** Fine for the
    UART pair, not enough to reach into the LiteSPI register file, which costs
    an extra `ADDI` and a scratch register (sketch F2). Cost is one `ADDI` per
    *block* if the base register is kept live, so leaning accept rather than
    widen — but it is a field that mostly does not pay for itself.

## What must stay true

- **The golden model gates the ISA.** `sim/psc18sr_model.py` is what makes the
  boot sequence testable before hardware exists. If the model cannot run it,
  it does not go in.

  **It is deliberately not written yet, and that is sequencing rather than
  debt.** The encoding is still moving — the shifts above are the proof — and
  a model written against a moving encoding is a model that gets rewritten
  instead of a model that catches anything. It is owed the moment the draft
  banner comes off, and not before. `sim/psc18sr_tb.v` is the smaller thing
  that is useful in the meantime: it runs an assembled program on the RTL and
  checks the result, which proves instructions execute but says nothing about
  whether the encoding is right.
- **Four implementations must agree** once this is frozen: this doc, the
  model, the assembler (whichever question 11 picks), and the Verilog. This doc
  is the arbiter — but only once the draft banner comes off.
- **v1 is not broken.** `docs/pisc_isa.md` stays as-is; PSC16I and PSC18SR are
  versioned siblings. The DTB node for an instantiated core should carry a
  `compatible` string and its `imem_words` so tooling picks the right encoding
  from what the hardware reports rather than a build-time assumption.

## Deliberately out of scope

Interrupts, a stack, condition flags, multiple addressing modes, memory
protection, and anything else that would make this a general-purpose CPU. If
those become wanted, that is the signal to drop in SERV and use gcc instead.

## Companion documents

- `docs/psc18sr_stage0_sketch.md` — the stage-0 boot sequence written out in
  this encoding against the real CSR map. Paper exercise, nothing assembled or
  run, but it is where findings F1–F7 cited above come from. Rewrite it
  whenever the encoding changes; it is the cheapest test the ISA has.
- `docs/psc18sr_prototype.md` — the RTL/LiteX/assembler prototype. Synthesizes
  to 1 DP16KD in native 18-bit mode and elaborates into a SoC as both a CSR
  peripheral and a bus master. **It has no golden model behind it**, so it
  currently violates "the golden model gates the ISA" below; that is a debt,
  not a change of policy. It also settles two things this doc leaves loose —
  the `PORT` register field per direction, and `BUS` half-writes being
  read-modify-write — which need folding back here or overriding.

## Related, not yet written

The boot-chain redesign this ISA serves is still only in conversation:
stage-0 sequencer duties and handoff, SoC discovery (pinned identity anchor,
DTB-in-BRAM, flash write-protect, boot-attempt counter), the shrunken SBI-style
runtime, and the stage-1 replacement for the LiteX BIOS.
