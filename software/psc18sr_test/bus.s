; BUS smoke test -- the one instruction v1 does not have.
;
; The core writes a value into its OWN gpio_in CSR over the Wishbone window,
; then reads it back through the IN instruction. Self-contained: no other
; peripheral has to cooperate, and a pass proves the whole path (window base,
; address arithmetic, the read-modify-write half-word store, and the CSR
; actually landing where IN can see it).
;
; GPIO_IN_OFF is the byte offset of psc18sr_gpio_in within the CSR window --
; from build/icepi_zero/csr.csv, minus the 0xf0000000 base. gpio_in is 64 bits
; = two CSR words, and LiteX orders them big-endian, so bits [31:0] live in the
; SECOND word (0x0024 + 4). Port 0 is bits [15:0], hence 0x0028 + half=lo.
;
; The constant pool looks like overkill here, and on THIS SoC it is: psc18sr is
; the only peripheral, so its block lands at window offset 0 and a bare
; `LI r1, 0x28` would do. Keep the pool anyway -- it is the idiom every real
; SoC forces (uart is already at 0x2800 in this build, well outside LI's
; +-1023), and that is finding F1 in the stage-0 sketch.
;
;   python3 tools/psc18sr_asm.py software/psc18sr_test/bus.s --py
;
; The offset below is the IcePi Zero value and is only right where psc18sr is
; the sole peripheral. software/psc18sr_host patches the pool word at load time
; from CSR_PSC18SR_GPIO_IN_ADDR instead -- that is what the `--c` output's
; label defines are for -- so on the i9 the constant here is unused.
;
; Expected: result == 42.

        .org 256
        .def GPIO_IN_OFF 0x0028         ; verify against csr.csv after any rebuild

start:  LI    r5, pool                  ; pool pointer (F1: one register, forever)
        LD    r1, r5, 0                 ; r1 = CSR offset of gpio_in
        LI    r2, 42
        BUSW  r2, r1, lo                ; gpio_in[15:0] = 42   (RMW: read+write)
        IN    r3, 0                     ; read input port 0 back
        MOV   r7, r3
        HLT

pool:   .word GPIO_IN_OFF
