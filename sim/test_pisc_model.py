#!/usr/bin/env python3
"""Self-checking tests for the PISC assembler + golden model (Phase 0).

Runs under pytest or standalone (`python sim/test_pisc_model.py`). Crucially,
every test assembles with tools/pisc_asm.py and runs with sim/pisc_model.py, so
a passing run also proves the two agree on the encoding -- the property the
Phase-1 cocotb differential test will lean on.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.pisc_asm import assemble                     # noqa: E402
from sim.pisc_model import PISC                          # noqa: E402


def test_sum_1_to_10():
    src = """
            LDI  r1, 0
            LDI  r2, 1
            LDI  r3, 11
    loop:   ADD  r1, r1, r2
            ADDI r2, r2, 1
            BNE  r2, r3, loop
            MOV  r7, r1
            HLT
    """
    cpu = PISC(assemble(src))
    assert cpu.run() == 55
    assert cpu.halted


def test_alu_and_sext():
    # 16-bit wraparound and signed immediates.
    src = """
            LDI  r1, 5
            ADDI r1, r1, -8     ; 5 + (-8) = -3 = 0xfffd
            MOV  r7, r1
            HLT
    """
    cpu = PISC(assemble(src))
    assert cpu.run() == 0xFFFD


def test_load_store_roundtrip():
    src = """
            LDI  r1, 7          ; value
            LDI  r2, 20         ; base address (data region, past the program)
            ST   r1, r2, 5      ; mem[25] = 7   (exercises base+imm6)
            LD   r7, r2, 5      ; r7 = mem[25]
            HLT
    """
    cpu = PISC(assemble(src))
    assert cpu.run() == 7
    assert cpu.mem[25] == 7


def test_pulse_sequence():
    # SETB/CLRB/SETB on port 0 bit 0, with a DELAY in the low phase.
    src = """
            .def RST 0
            SETB  RST, 0        ; idle high
            CLRB  RST, 0        ; assert low
            DELAY 100
            SETB  RST, 0        ; release high
            HLT
    """
    cpu = PISC(assemble(src))
    cpu.run()
    assert cpu.io_out[0] & 1 == 1          # ends high
    assert cpu.cycles >= 100               # the DELAY was accounted for


def test_wait_on_input_pin():
    # io_in port 0 bit 0 is low for the first 5 polls, then goes high.
    state = {"polls": 0}

    def io_in(port):
        if port != 0:
            return 0
        state["polls"] += 1
        return 1 if state["polls"] > 5 else 0

    src = """
            WAIT 0, 0, 1        ; block until input goes high
            LDI  r7, 21
            HLT
    """
    cpu = PISC(assemble(src), io_in=io_in)
    assert cpu.run() == 21
    assert state["polls"] > 5              # it actually polled until the flip


def test_zero_word_is_infinite_spin():
    # An all-zero (uninitialized) imem must park, not run garbage.
    cpu = PISC([0x0000])
    try:
        cpu.run(max_steps=1000)
    except RuntimeError:
        pass
    else:
        raise AssertionError("0x0000 should be an infinite JMP self-loop")


def test_r0_is_hardwired_zero():
    src = """
            ADDI r0, r0, 5      ; attempt to write r0
            MOV  r7, r0         ; r7 should still be 0
            HLT
    """
    cpu = PISC(assemble(src))
    assert cpu.run() == 0


def _main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"\n{len(tests)} passed")


if __name__ == "__main__":
    _main()
