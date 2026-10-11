; Reduced row echelon form, sequenced by PSC18SR, arithmetic by rref_engine.
;
; DEMO.  The core never touches a matrix value: it only decides *which*
; elementary row operation happens next, and the engine (verilog/rref_engine.v,
; on out ports 4-5 / in port 4) does the arithmetic in exact rationals.
; This file is textbook Gauss-Jordan elimination, nothing more:
;
;     r = 0
;     for c in 0 .. cols-1, while r < rows:
;         find the first i >= r with M[i][c] != 0      (TEST)
;         none: next column
;         swap rows r and i                            (SWAP)
;         Rr <- (1 / M[r][c]) * Rr                     (GETK inv, SCALE)
;         for every other row i:
;             Ri <- Ri + (-M[i][c]) * Rr                (GETK neg, ADDMUL)
;         r = r + 1
;
; Result: r7 = rank.  The reduced matrix stays in the engine for the host.
;
; Registers:  r1 = r (pivot row)   r2 = c (column)   r3 = i (other row)
;             r4, r5, r7 scratch   r6 link
;
;   python3 tools/psc18sr_asm.py software/psc18sr_rref/rref.s --list

        .org 256

        .def CMD    4           ; out port: engine command word
        .def CTL    5           ; out port: bit 0 = go (rising edge)
        .def STS    4           ; in port:  bit 0 busy, bit 1 zero,
                                ;           [11:8] cols, [15:12] rows

        .def TEST   1           ; engine ops, command bits [2:0]
        .def SWAP   2
        .def GETK   3
        .def SCALE  4
        .def ADDMUL 5
        .def INV    8           ; command bit 3
        .def NEG    16          ; command bit 4

        ; command fields: a << 11, b << 8, c << 5

start:  LI    r1, 0             ; r = 0
        LI    r2, 0             ; c = 0

next_col:
        JAL   dims              ; r4 = rows, r5 = cols
        BEQ   r2, r5, done      ; out of columns
        BEQ   r1, r4, done      ; out of rows

        ; ---- look for a pivot in column c, at or below row r ------------
        MOV   r3, r1            ; i = r
find:   LI    r7, 11            ; TEST i, c
        SLL   r5, r3, r7
        LI    r7, 5
        SLL   r4, r2, r7
        OR    r5, r5, r4
        ADDI  r5, r5, TEST
        JAL   issue
        IN    r4, STS           ; zero flag clear -> found a pivot
        LI    r7, 2
        AND   r4, r4, r7
        BEQ   r4, r0, found

        ADDI  r3, r3, 1         ; try the next row
        JAL   dims
        BNE   r3, r4, find
        ADDI  r2, r2, 1         ; the whole column is zero: move on
        JMP   next_col

        ; ---- bring the pivot to row r -----------------------------------
found:  BEQ   r3, r1, normalize
        LI    r7, 11            ; SWAP r, i
        SLL   r5, r1, r7
        LI    r7, 8
        SLL   r4, r3, r7
        OR    r5, r5, r4
        ADDI  r5, r5, SWAP
        JAL   issue

        ; ---- make the pivot 1 -------------------------------------------
normalize:
        LI    r7, 11            ; GETK r, c, inv:  K = 1 / M[r][c]
        SLL   r5, r1, r7
        LI    r7, 5
        SLL   r4, r2, r7
        OR    r5, r5, r4
        ADDI  r5, r5, GETK
        ADDI  r5, r5, INV
        JAL   issue

        LI    r7, 11            ; SCALE r:  Rr = K * Rr
        SLL   r5, r1, r7
        ADDI  r5, r5, SCALE
        JAL   issue

        ; ---- clear column c in every other row --------------------------
        LI    r3, 0             ; i = 0
elim:   BEQ   r3, r1, elim_next ; skip the pivot row itself

        LI    r7, 11            ; GETK i, c, neg:  K = -M[i][c]
        SLL   r5, r3, r7
        LI    r7, 5
        SLL   r4, r2, r7
        OR    r5, r5, r4
        ADDI  r5, r5, GETK
        ADDI  r5, r5, NEG
        JAL   issue

        LI    r7, 11            ; ADDMUL i, r:  Ri = Ri + K * Rr
        SLL   r5, r3, r7
        LI    r7, 8
        SLL   r4, r1, r7
        OR    r5, r5, r4
        ADDI  r5, r5, ADDMUL
        JAL   issue

elim_next:
        ADDI  r3, r3, 1
        JAL   dims
        BNE   r3, r4, elim

        ADDI  r1, r1, 1         ; next pivot row
        ADDI  r2, r2, 1         ; next column
        JMP   next_col

done:   MOV   r7, r1            ; rank
        HLT

; dims() -> r4 = rows, r5 = cols, read from the engine's status port.
; Clobbers r7.
dims:   IN    r5, STS
        LI    r7, 8
        SRL   r5, r5, r7
        LI    r7, 15
        AND   r5, r5, r7
        IN    r4, STS
        LI    r7, 12
        SRL   r4, r4, r7
        RET

; issue(r5 = command) -- hand one command to the engine and wait for it.
issue:  OUT   CMD, r5
        SETB  CTL, #0
        CLRB  CTL, #0
        WAIT  STS, #0, 0
        RET
