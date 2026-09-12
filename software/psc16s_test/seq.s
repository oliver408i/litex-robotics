; Sequencing test -- the instructions that justify a custom core at all.
; Exercises: BITOP (SETB/CLRB), DELAY, WAIT, and JAL/RET (which v1 has no
; equivalent of, and which were the actual reason for v2 -- see the draft's
; "Why v2 exists", amended).
;
; Shape is the v1 doc's LCD reset pulse, restructured around a subroutine so
; the call path is under test too.
;
;   python3 tools/psc16s_asm.py software/psc16s_test/seq.s --py
;
; Host procedure (autostart=0):
;   1. load, start_pc = 256, pulse run
;   2. watch gpio_out port 0 bit 0: high, then low for ~5 ms, then high
;   3. the core parks in WAIT -- gpio_out is stable, status.halted still 0
;   4. write gpio_in port 0 bit 0 = 1  -> the core proceeds and halts
;   5. result == 0xA5
;
; Step 3 is the interesting one: it is a deliberate unbounded stall, and the
; only ways out are the host's ready bit or control.abort. On a stage-0 build
; this is exactly the hazard the watchdog exists for (sketch F5).
;
; Tick note: one DELAY tick is DELAY_PRESCALE core cycles = 12.05 us at
; 85 MHz, but 20.48 us at the 50 MHz this top defaults to. The counts below
; are quoted for 85 MHz; scale them for whatever --sys-clk-freq you build.

        .org 256

        .def RST    0                   ; output port 0
        .def RSTBIT 0                   ; bit 0 = active-low reset_n
        .def READY  0                   ; input port 0
        .def RDYBIT 0

start:  SETB  RST, RSTBIT               ; idle high
        LI    r1, 8                     ; 8 ticks (see tick note below)
        JAL   settle

        CLRB  RST, RSTBIT               ; assert reset (low)
        LI    r1, 415                   ; 415 ticks ~ 5 ms at 85 MHz
        JAL   settle

        SETB  RST, RSTBIT               ; release
        WAIT  READY, RDYBIT, 1          ; block until the host says ready

        LI    r7, 0xA5
        HLT

; settle(r1 = ticks) -- DELAY takes an immediate, so a variable-length wait is
; a loop. Clobbers r1. r6 is the link and survives: this calls nothing.
settle: BEQ   r1, r0, settle_done
        DELAY 1
        ADDI  r1, r1, -1
        JMP   settle
settle_done:
        RET
