# RREF demo — Gauss-Jordan on PSC18SR + a row-operation engine

A demo, not a product. The point is the split, not the speed:

- **PSC18SR runs the algorithm.** `software/psc18sr_rref/rref.s` is textbook
  Gauss-Jordan — pivot search, swap, normalize, eliminate — in ~80 words of
  PSC ASM. It never sees a matrix value, only a "zero / not zero" bit.
- **The engine does the arithmetic.** `verilog/rref_engine.v` performs one
  elementary row operation per command, in **exact rationals**, so the output
  is `[1 0 -1/3]`, not `0.3333`.

It is the first real use of the draft ISA's "ports, not opcodes" extension
(`docs/psc18sr_isa_draft.md`): the engine hangs off the core's IO ports and
costs no new instructions. Every command is

    OUT  CMD, r5        ; command word
    SETB CTL, #0        ; go
    CLRB CTL, #0
    WAIT STS, #0, 0     ; until not busy

## The engine

| op | command | effect |
|---|---|---|
| 1 | `TEST a,c` | zero flag ← (M[a][c] = 0) |
| 2 | `SWAP a,b` | Ra ↔ Rb |
| 3 | `GETK a,c,[neg],[inv]` | K ← M[a][c], optionally negated and/or inverted |
| 4 | `SCALE a` | Ra ← K·Ra |
| 5 | `ADDMUL a,b` | Ra ← Ra + K·Rb |

Command word: `[13:11] a  [10:8] b  [7:5] c  [4] neg  [3] inv  [2:0] op`.
Status word: `[15:12] rows  [11:8] cols  [3] err  [2] ovf  [1] zero  [0] busy`.

`GETK` is what keeps division out of the core. Normalizing the pivot row is
`GETK r,c,inv` + `SCALE r`; clearing row i is `GETK i,c,neg` + `ADDMUL i,r`.

Two files, each doing one thing:

- `verilog/rref_rat.v` — computes `r = x + k·y` for fractions and reduces the
  result by its gcd. SCALE is `0 + K·a` and ADDMUL is `a + K·b`, so this one
  operation covers both. Products are plain Verilog `*`; the gcd (binary) and
  the exact divide (restoring) are small iterative modules.
- `verilog/rref_engine.v` — an 8×8 memory of num/den pairs and a state machine
  that walks a row, column by column, feeding `rref_rat`.

Values are 32-bit signed numerator / 32-bit positive denominator, always
reduced. Anything that does not fit sets the sticky `ovf` flag. Integer
matrices up to about 6×6 with single-digit entries stay inside 32 bits; 7×7
and 8×8 ones often do not, and those come back flagged rather than wrong.

## Running it

Simulation — the real core, the real engine, the real program, checked
against Python `fractions.Fraction`:

    source oss-cad-suite/oss-cad-suite/environment
    python3 sim/test_rref.py -n 200

On the i9, driven entirely over Etherbone (no firmware):

    .venv/bin/python targets/colorlight_i9/rref.py --build --load
    .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &
    .venv/bin/python tools/rref_demo.py '[[2,4,-2],[4,9,-3],[-2,-3,7]]'
    .venv/bin/python tools/rref_demo.py '[["1/2", 3], [5, "-2/3"]]'
    .venv/bin/python tools/rref_demo.py --random 50

## Port map (i9 top)

| PSC18SR port | direction | use |
|---|---|---|
| 0–3 | out/in | CSR gpio, as on the plain side-block top |
| out 4 | out | engine command |
| out 5 bit 0 | out | engine go |
| in 4 | in | engine status |
