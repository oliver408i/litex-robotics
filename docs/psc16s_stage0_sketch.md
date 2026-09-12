# Stage-0 boot sequence — paper sketch against the v2 draft ISA

> **STATUS: PAPER EXERCISE. Nothing here has been assembled or run.**
>
> There is no assembler for the v2 encoding and no golden model, so every
> instruction below is hand-encoded against the table in
> `docs/psc16s_isa_draft.md` and every cycle count is arithmetic, not
> measurement. The point is not the program — it is the list of things that
> broke while writing it, at the end.

Companion to `docs/psc16s_isa_draft.md`. Written to test the ISA empirically
rather than by inspection: take the actual stage-0 duties, write them in the
proposed encoding against the real CSR map, and see what the encoding fights.

## Target

The duties named in the draft's "Why v2 exists": hold the CPU in reset,
configure flash, init the panel, emit UART markers, release the CPU. Against
the real SoC (`docs/soc_layout.md`, `docs/boot_chain.md`), stage-0 inserts
itself between ECP5 self-configuration and the CPU's first XIP fetch of the
BIOS:

```
power-on
  └─ ECP5 self-config from flash @0x000000
       └─ PISC v2 autostarts at PC=0, CPU held in reset   <-- new
            └─ ... this program ...
                 └─ CPU released, XIP BIOS @0x100000
```

## Assumed bindings

Ports, with **reset values** (draft machine model — safety-critical):

| Port | Bit | Line | Reset value |
|---|---|---|---|
| `OUT0` | 0 | `cpu_rst_n` | **0 — CPU held in reset** |
| `OUT0` | 1 | `lcd_rst_n` | 0 — panel held in reset |
| `IN0` | 0 | `pll_locked` | — |

CSR byte offsets within the 64K `BUS` window (base `0xf000_0000`, from
`docs/soc_layout.md`):

| Constant | Value | Block |
|---|---|---|
| `SPIFLASH` | `0x6800` | LiteSPI |
| `UART` | `0x7800` | UART |
| `LCD` | `0x5800` | LCD engine |

Constant pool in data memory at `POOL`, one 16-bit constant per word.

## The program

```asm
; ---------------------------------------------------------------- setup
start:  LI    r5, POOL           ; pool base pointer (POOL <= 1023, see F1)
        WAIT  IN0, #0, 1         ; block until PLL locked          (see F5)

; ------------------------------------------------- flash clock divisor
; BIOS XIP fetches through the mmap at the build-time divisor; stage-0
; sets it before the CPU ever fetches. 25 MHz = sys/2 = divisor 1.
        LD    r1, r5, #0         ; r1 = 0x6800   SPIFLASH block base
        ADDI  r1, r1, #CLKDIV    ; + register offset within block   (see F2)
        LI    r2, 1
        BUS   r2, r1, w, lo      ; divisor low half
        BUS   r0, r1, w, hi      ; divisor high half = 0

; -------------------------------------------------------- panel reset
        CLRB  OUT0, #1           ; assert lcd_rst_n low
        DELAY 415                ; 5 ms   @ 12.05 us/count
        SETB  OUT0, #1           ; release
        DELAY 9958               ; 120 ms panel wake              (see F4)

; -------------------------------------------------------- UART marker
        LI    r1, MSG            ; pointer to string, one char per word
puts:   LD    r2, r1, #0
        BEQ   r2, r0, done       ; NUL terminator
        JAL   putc
        ADDI  r1, r1, #1
        JMP   puts

done:   SETB  OUT0, #0           ; release the CPU
        LI    r7, 0              ; exit code 0 = success
        HLT

; -------------------------------------------------- putc(r2 = char)
; clobbers r3, r4. r6 is the link and survives: putc calls nothing.
putc:   LD    r3, r5, #1         ; r3 = 0x7800   UART block base
1:      BUS   r4, r3, r, lo, #TXFULL
        BNE   r4, r0, 1b         ; spin while TX full             (see F5)
        BUS   r2, r3, w, lo, #RXTX
        RET                      ; JALR r6, 0

; ------------------------------------------------------------- data
POOL:   .word 0x6800             ; SPIFLASH
        .word 0x7800             ; UART
MSG:    .word 'P','I','S','C',' ','o','k',10,0    ; 9 words        (see F3)
```

Roughly **26 instructions** plus 11 data words.

## Findings

Numbered so the draft's open questions can cite them.

### F1 — `LI` cannot reach a CSR offset

`LI` is `sext11`, so it reaches −1024..+1023. Every CSR block offset in the
window (`0x5800`, `0x6800`, `0x7800`) is far outside that. **Every single bus
access must load its address from a constant pool first.** That works, and
`LD rd, rs, imm8` with a base register is a fine idiom, but it means:

- one register (`r5`) is permanently a pool pointer, out of eight;
- the pool base itself must be `<= 1023` to be loadable with one `LI`, or it
  needs its own two-instruction dance;
- in a **unified** memory where `PC` starts at 0, the pool cannot live at low
  addresses — that is the program. So the pool sits above the code, and
  `POOL <= 1023` collides with any program longer than 1023 words.

Not fatal at 26 instructions. It becomes real if stage-0 ever grows toward the
"CSR-write pattern repeats 100+ times" scale that motivated v2.

### F2 — `BUS` `imm6` is nearly useless for LiteX CSR blocks

The draft gives `BUS` a 6-bit offset, ±31. LiteX CSR registers are 4 bytes
apart, so ±31 bytes reaches **7 registers** from the base. The UART block is
fine (`rxtx` and `txfull` are adjacent). LiteSPI is not — its register file is
much wider than 32 bytes, so reaching the divisor costs an extra `ADDI` and a
scratch register, and the `imm6` field goes unused on that access.

Two obvious responses, neither free: widen the offset by stealing from
somewhere, or accept `ADDI` as the idiom and note that `imm6` only pays off
for tight register pairs. Leaning accept — the sketch shows the `ADDI` cost is
one instruction per *block*, not per access, as long as the base register is
kept live.

### F3 — no shifts means one 16-bit word per ASCII character

The draft deliberately leaves shifts undefined. The consequence shows up
immediately: a UART string cannot pack two bytes per word, because unpacking
needs a shift. A 9-character boot marker costs 9 words of an 18-bit-wide
memory to carry 9 bytes — using 9 of 144 available bits.

This is survivable for a handful of markers and genuinely fatal for anything
string-heavy. It is the sharpest test of the draft's design line so far: the
honest reading is not "add shifts", it is **stage-0 should not be formatting
text**. If boot markers grow past a few fixed strings, that is the signal the
draft says to listen for.

Worth noting the cheap escape hatch if it ever bites: a single `SRL` would fix
it, and the ALU funct field has 27 reserved slots. The draft is right that the
cost is contract surface, not gates — but the pressure is real and should be
recorded rather than rediscovered.

### F4 — the prescale number works, and 200 ms was the wrong target

At prescale 1024 and 85 MHz, one count is 12.05 µs and `imm14` reaches
**197.4 ms**. The panel sequence needs 5 ms and 120 ms; both fit in a single
`DELAY` with room to spare. The 200 ms figure in the draft's open question 4
appears to have been a worst-case guess rather than a real requirement —
nothing in the actual panel bring-up needs it.

So prescale 1024 is fine, and open question 4's discomfort resolves. Keeping
it a synthesis parameter is still right, because the model must match the
build, but the default is defensible.

### F5 — `WAIT` has the same unbounded-stall problem as `BUS`

Draft open question 5 asks what `BUS` does when the bus does not answer. The
sketch shows the identical hazard in two more places, and one of them is
`WAIT` — the instruction the whole core exists for:

- `WAIT IN0, #0, 1` hangs forever if the PLL never locks;
- the `putc` spin loop hangs forever if the UART never drains.

All three leave stage-0 stalled **with the CPU held in reset** — the bricking
hazard from the draft, arriving by three different roads. The `putc` loop is
software and can be given its own bounded counter. `WAIT` and `BUS` are
instructions, and cannot be, without a decision in the ISA.

This argues the timeout question is not `BUS`-specific. Either both blocking
instructions get a bounded form, or the machine gets one watchdog that fires
independently of what the program is doing. A watchdog is the smaller contract
surface: one synthesis-time counter and one defined action on expiry (release
the CPU, latch a nonzero `r7`, halt), rather than a timeout field on every
blocking instruction and a matching escape path in the model.

**Recommend folding open question 5 into a single "what bounds a stalled
stage-0" question**, with the watchdog as the leading answer.

### F6 — `RET` clobbers `r6`, and the sketch got lucky

`putc` is called from a loop and returns via `JALR r6, 0`, which rewrites
`r6` with `PC+1` as it jumps. That is harmless here because the caller
re-establishes the link with a fresh `JAL` on every iteration. It would not be
harmless in any nested call, and it means `r6` cannot be treated as a
callee-saved anything. Consistent with the draft's "leaning fixed `r6`" — just
worth writing down before someone assumes otherwise.

### F7 — 2048 words is generous, and the imem pressure was overstated

The full sequence is ~26 instructions. Even allowing for SDRAM bring-up, a
larger marker vocabulary, and a real error path, stage-0 looks like low
hundreds of words, not thousands.

The draft justifies v2 partly on "inlining blows the imem". That is true of
the *v1* encoding — no subroutines and a ±31 branch — but the sketch suggests
the binding constraint was never capacity. It was `JAL`/`RET` and `BEQ`
existing at all. Worth being honest about in the draft, because "we need more
imem" and "we need subroutines" point at different designs, and only the
second one is supported by this exercise.

## What this does not test

- **SDRAM bring-up.** Left out entirely. If stage-0 ever takes over LiteDRAM
  initialisation from the BIOS, that is where the 100+ CSR writes actually
  live, and F1/F2 get much sharper. This sketch cannot say whether that is
  wise.
- **The expander path.** `LCD_RST` is on an MCP23S17 over the aux SPI bus in
  the `mnist_lcd` build (`docs/reset_sidebands.md`), not a direct pin. Doing
  that from stage-0 means driving `aux_spi` CSRs through `BUS` — plausible,
  but a longer sequence than the direct-pin version sketched here.
- **The boot-attempt counter.** Needs storage that survives a warm reset.
  Neither imem (write-protected, `INITVAL` at configuration) nor registers
  qualify; it wants a `reset_less` CSR, which is a SoC question, not an ISA
  one.
- **Anything about timing accuracy**, since no model exists to run this.
