; Shift smoke test -- SLL / SRL / SRA, the instructions PSC18SR adds.
;
; The motivating case is first: unpacking two ASCII characters from one 16-bit
; word. Draft open question 13 recorded that as the thing a missing shift made
; impossible, so it is the thing the shift has to make possible.
;
; The amount comes from rs2 and is masked to its low 4 bits. There is no
; shift-immediate form -- shifting by a literal is LI + the shift, as written
; below -- because an assembler pseudo-op here must be exactly one instruction.
;
;   python3 tools/psc18sr_asm.py software/psc18sr_test/shift.s --py
;
; Expected: result == 0x5A. Any other value is the index of the check that
; failed, because r7 carries the check number while the check runs.

        .org 256

start:  LI    r5, pool                  ; pool pointer (F1: one register, forever)

        LI    r7, 1                     ; -- check 1: SRL unpacks the high byte
        LD    r1, r5, 0                 ; r1 = 0x4869 = 'H','i' packed
        LI    r2, 8
        SRL   r3, r1, r2                ; r3 = 0x0048 = 'H'
        LI    r4, 'H'
        BNE   r3, r4, fail

        LI    r7, 2                     ; -- check 2: the low byte, for symmetry
        LI    r4, 0xFF
        AND   r3, r1, r4
        LI    r4, 'i'
        BNE   r3, r4, fail

        LI    r7, 3                     ; -- check 3: SLL to the sign bit
        LI    r1, 1
        LI    r2, 15
        SLL   r3, r1, r2                ; r3 = 0x8000
        LD    r4, r5, 1
        BNE   r3, r4, fail

        LI    r7, 4                     ; -- check 4: SRA sign-extends
        LI    r1, -256                  ; 0xFF00
        LI    r2, 4
        SRA   r3, r1, r2                ; r3 = 0xFFF0
        LD    r4, r5, 2
        BNE   r3, r4, fail

        LI    r7, 5                     ; -- check 5: SRL of the same does not
        SRL   r3, r1, r2                ; r3 = 0x0FF0
        LD    r4, r5, 3
        BNE   r3, r4, fail

        LI    r7, 0x5A                  ; all five passed
fail:   HLT                             ; r7 = 0x5A, or the failing check index

pool:   .word 0x4869, 0x8000, 0xFFF0, 0x0FF0
