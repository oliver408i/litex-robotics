# PISC — Programmable I/O Sequencer Core

PISC is a tiny 16-bit soft CPU whose job is **peripheral choreography**, not
general compute. VexRiscv already runs C; PISC exists to offload deterministic
bit-level sequencing — power-up / reset ordering, pulse-width timing, "wait for
this pin, then drive that one" — onto a small reprogrammable engine that lives
in the SoC as a host-loaded peripheral. Think RP2040 PIO, but a real little CPU.

First intended job: drive the slow reset/enable sidebands (today on direct FPGA
pins, headed for a SPI GPIO expander on the aux bus — see
`docs/reset_sidebands.md`) from a few hundred bytes of microcode the host loads
into the core's instruction memory.

This document is the **frozen contract** between three implementations that must
agree exactly:

- `sim/pisc_model.py`   — the golden reference interpreter
- `tools/pisc_asm.py`   — the assembler
- `verilog/pisc.v`      — the hardware core (Phase 1)

If any of them disagrees with this table, this table wins.

## Machine model

- **Word size:** 16-bit, two's complement. All arithmetic wraps mod 2^16.
- **Registers:** `r0`–`r7`. `r0` is hardwired to 0 (writes to it are discarded).
  `r7` is latched into the `result` CSR when the core executes `HLT`.
- **Memory:** one unified word-addressed RAM (default 256 × 16-bit), holding both
  program and data. `PC` starts at 0. Load/store address wraps to the RAM size.
- **I/O ports:** up to 64 output ports and 64 input ports, each 16-bit. In
  hardware only a handful are wired (reset_n, chip_en, an SPI byte port, …); the
  rest read 0 / go nowhere. Output ports hold their value until rewritten.
- **Execution:** one instruction at a time, run to `HLT`. No interrupts, no
  pipeline hazards visible to software (the hardware is multicycle).

## Instruction encoding

Fixed 16-bit width. Common field layout:

```
 15  12 | 11   9 | 8    6 | 5         0
 opcode |   rd   |   rs   |   imm6 / rs2 / port
```

`rs2` occupies bits `[5:3]`; `imm6` occupies `[5:0]` (signed); `port` occupies
`[5:0]` (unsigned, 0–63); `imm12` occupies `[11:0]`.

| op  | mnemonic            | fields                          | effect |
|-----|---------------------|---------------------------------|--------|
| 0x0 | `JMP  off`          | imm12 = off                     | `PC += sext12(off)` (relative to this instruction) |
| 0x1 | `ADD  rd,rs,rs2`    | rd, rs, rs2[5:3]                | `rd = rs + rs2` |
| 0x2 | `SUB  rd,rs,rs2`    | rd, rs, rs2[5:3]                | `rd = rs - rs2` |
| 0x3 | `ADDI rd,rs,imm6`   | rd, rs, imm6                    | `rd = rs + sext6(imm6)` |
| 0x4 | `AND  rd,rs,rs2`    | rd, rs, rs2[5:3]                | `rd = rs & rs2` |
| 0x5 | `OR   rd,rs,rs2`    | rd, rs, rs2[5:3]                | `rd = rs \| rs2` |
| 0x6 | `LD   rd,rs,imm6`   | rd, rs, imm6                    | `rd = mem[rs + sext6(imm6)]` |
| 0x7 | `ST   rd,rs,imm6`   | rd=data, rs=base, imm6          | `mem[rs + sext6(imm6)] = rd` |
| 0x8 | `BNE  ra,rb,off`    | rs=ra, rd=rb, imm6=off          | `if ra != rb: PC += sext6(off)` |
| 0x9 | `OUT  port,rs`      | rs[8:6], port[5:0]              | `io_out[port] = rs` |
| 0xA | `IN   rd,port`      | rd[11:9], port[5:0]             | `rd = io_in[port]` |
| 0xB | `SETB port,#bit`    | bit[11:9], port[5:0]            | `io_out[port] \|= (1 << bit)` |
| 0xC | `CLRB port,#bit`    | bit[11:9], port[5:0]            | `io_out[port] &= ~(1 << bit)` |
| 0xD | `WAIT port,#bit,lvl`| bit[11:9], lvl[8], port[5:0]    | block until `io_in[port][bit] == lvl` |
| 0xE | `DELAY imm12`       | imm12                           | stall `imm12` core cycles (× hardware prescale) |
| 0xF | `HLT`               | —                               | stop; latch `r7` → `result`, raise `halted` |

Notes:

- **Branch/jump targets are relative to the branching instruction's own PC**
  (`target = PC_of_branch + offset`). `BNE`/`JMP` to self is offset 0. Ranges:
  `BNE` ±31 words, `JMP` ±2047 words.
- `bit` for `SETB`/`CLRB`/`WAIT` is 0–7 (3-bit field), selecting a bit of the
  16-bit port — enough for the low control lines that matter; use `OUT` to set a
  whole word.
- **`0x0000` decodes as `JMP +0` — an infinite self-loop.** This is deliberate:
  an uninitialized (all-zero) instruction memory parks the core rather than
  running garbage. A real program ends in `HLT` (`0xF000`). `NOP` is **not**
  all-zeros — see pseudo-ops.

## Assembler syntax (`tools/pisc_asm.py`)

- One instruction per line. `;` begins a comment.
- Labels: `name:` at the start of a line.
- Registers: `r0`–`r7` (case-insensitive).
- Immediates: decimal, `0x` hex, or a `.def` name. An optional leading `#` is
  ignored (`#5` == `5`), so `SETB`/`DELAY` read naturally.
- `LD`/`ST` accept either `rd, rs, imm` or bracket form `rd, [rs+imm]` — brackets
  and `+` are cosmetic.
- Constants: `.def NAME value` (e.g. `.def LCD_RST 0`) for port/bit names.
- Branch/jump operand: a label (assembler computes the relative offset) or a
  literal signed offset.

Pseudo-ops (each assembles to exactly one real instruction):

| pseudo            | expands to            |
|-------------------|-----------------------|
| `NOP`             | `ADD r0, r0, r0`      |
| `MOV  rd, rs`     | `ADD rd, rs, r0`      |
| `LDI  rd, imm6`   | `ADDI rd, r0, imm6`   |

## Example — sum 1..10 (the Phase-2 "hello world")

```asm
        LDI  r1, 0        ; sum = 0
        LDI  r2, 1        ; i   = 1
        LDI  r3, 11       ; limit
loop:   ADD  r1, r1, r2   ; sum += i
        ADDI r2, r2, 1    ; i++
        BNE  r2, r3, loop ; while i != 11
        MOV  r7, r1       ; result = sum  (-> result CSR on HLT)
        HLT               ; r7 == 55
```

## Example — an LCD-style reset pulse (the actual job)

```asm
        .def RST   0      ; output port 0, bit 0 = active-low reset_n
        .def READY 0      ; input port 0, bit 0 = device ready

        SETB  RST, 0      ; idle high
        DELAY 100         ; settle
        CLRB  RST, 0      ; assert reset (low)
        DELAY 500         ; min reset pulse width
        SETB  RST, 0      ; release
        WAIT  READY, 0, 1 ; block until the device reports ready
        HLT
```

The host (VexRiscv) writes the assembled words into PISC's instruction memory
over CSRs, pulses `run`, and polls `halted` — exactly the load/start/poll flow
the SNN-MLP peripheral already uses (`gateware/snn_mlp.py`).
