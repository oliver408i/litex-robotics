; sum 1..10 -> 55.  The v1 "hello world" (docs/pisc_isa.md) in the v2 encoding.
; Exercises: LI, ALU/ADD, ADDI, BNE (backward), MOV pseudo, HLT + result latch.
;
;   python3 tools/psc18sr_asm.py software/psc18sr_test/sum.s --py
;
; Assembled at 256 = the default ro_words, i.e. the first host-writable word.

        .org 256

        LI    r1, 0             ; sum = 0
        LI    r2, 1             ; i   = 1
        LI    r3, 11            ; limit
loop:   ADD   r1, r1, r2        ; sum += i
        ADDI  r2, r2, 1         ; i++
        BNE   r2, r3, loop      ; while i != 11
        MOV   r7, r1            ; result = sum
        HLT                     ; r7 == 55
