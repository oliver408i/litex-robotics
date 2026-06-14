#!/usr/bin/env python3
"""Golden reference interpreter for the PISC sequencer core.

This is the authority the Verilog core (verilog/pisc.v, Phase 1) gets
differentially tested against in cocotb: run the same program through both and
diff architectural state (registers, memory, io_out) after every retired
instruction. See docs/pisc_isa.md for the frozen ISA contract.

The model tracks an informational `cycles` count (DELAY/WAIT consume cycles) but
makes no claim about exact hardware cycle counts per instruction -- only the
*architectural* state transitions are normative. The hardware is multicycle;
its DELAY prescale is a hardware parameter.

    from sim.pisc_model import PISC
    cpu = PISC(words)                 # words = assembled program (list[int])
    cpu.run()
    assert cpu.regs[7] == 55
"""
from __future__ import annotations

from typing import Callable, Optional


def _sext(value: int, bits: int) -> int:
    """Sign-extend a `bits`-wide two's-complement value to a Python int."""
    sign = 1 << (bits - 1)
    return (value & (sign - 1)) - (value & sign)


class Halt(Exception):
    """Raised internally when HLT executes; caught by run()."""


class PISC:
    """Cycle-faithful (architecturally) PISC interpreter.

    io_in: optional callable(port:int) -> int (16-bit). Lets a test make an
    input pin change over time (it is re-called on every WAIT poll and every
    IN). Defaults to all-zero inputs.
    """

    MASK = 0xFFFF

    def __init__(self, program, mem_size: int = 256,
                 io_in: Optional[Callable[[int], int]] = None,
                 wait_timeout: int = 1_000_000):
        if len(program) > mem_size:
            raise ValueError(f"program ({len(program)} words) exceeds mem_size {mem_size}")
        self.mem_size = mem_size
        self.mem = [0] * mem_size
        for i, w in enumerate(program):
            self.mem[i] = w & self.MASK
        self.regs = [0] * 8
        self.pc = 0
        self.io_out: dict[int, int] = {}
        self._io_in = io_in or (lambda port: 0)
        self.cycles = 0
        self.halted = False
        self.result = 0
        self._wait_timeout = wait_timeout

    # -- register file (r0 hardwired to 0) -------------------------------
    def _set(self, rd: int, value: int) -> None:
        if rd != 0:
            self.regs[rd] = value & self.MASK

    def io_in(self, port: int) -> int:
        return self._io_in(port) & self.MASK

    # -- one instruction -------------------------------------------------
    def step(self) -> dict:
        """Execute one instruction. Returns a small trace record. Raises Halt
        on HLT (run() catches it)."""
        pc = self.pc
        instr = self.mem[pc % self.mem_size]
        op = (instr >> 12) & 0xF
        rd = (instr >> 9) & 0x7
        rs = (instr >> 6) & 0x7
        rs2 = (instr >> 3) & 0x7
        imm6 = _sext(instr & 0x3F, 6)
        imm12 = instr & 0xFFF
        port = instr & 0x3F
        bit = (instr >> 9) & 0x7
        lvl = (instr >> 8) & 0x1

        next_pc = pc + 1
        self.cycles += 1

        if op == 0x0:                                   # JMP
            next_pc = pc + _sext(imm12, 12)
        elif op == 0x1:                                 # ADD
            self._set(rd, self.regs[rs] + self.regs[rs2])
        elif op == 0x2:                                 # SUB
            self._set(rd, self.regs[rs] - self.regs[rs2])
        elif op == 0x3:                                 # ADDI
            self._set(rd, self.regs[rs] + imm6)
        elif op == 0x4:                                 # AND
            self._set(rd, self.regs[rs] & self.regs[rs2])
        elif op == 0x5:                                 # OR
            self._set(rd, self.regs[rs] | self.regs[rs2])
        elif op == 0x6:                                 # LD
            addr = (self.regs[rs] + imm6) % self.mem_size
            self._set(rd, self.mem[addr])
        elif op == 0x7:                                 # ST
            addr = (self.regs[rs] + imm6) % self.mem_size
            self.mem[addr] = self.regs[rd] & self.MASK
        elif op == 0x8:                                 # BNE ra(rs), rb(rd)
            if self.regs[rs] != self.regs[rd]:
                next_pc = pc + imm6
        elif op == 0x9:                                 # OUT port, rs
            self.io_out[port] = self.regs[rs] & self.MASK
        elif op == 0xA:                                 # IN rd, port
            self._set(rd, self.io_in(port))
        elif op == 0xB:                                 # SETB port, #bit
            self.io_out[port] = (self.io_out.get(port, 0) | (1 << bit)) & self.MASK
        elif op == 0xC:                                 # CLRB port, #bit
            self.io_out[port] = (self.io_out.get(port, 0) & ~(1 << bit)) & self.MASK
        elif op == 0xD:                                 # WAIT port, #bit, lvl
            polls = 0
            while ((self.io_in(port) >> bit) & 1) != lvl:
                self.cycles += 1
                polls += 1
                if polls > self._wait_timeout:
                    raise TimeoutError(
                        f"WAIT on port {port} bit {bit}=={lvl} never satisfied "
                        f"(pc={pc})")
        elif op == 0xE:                                 # DELAY imm12
            self.cycles += imm12
        elif op == 0xF:                                 # HLT
            self.halted = True
            self.result = self.regs[7]
            self.pc = pc
            raise Halt()

        self.pc = next_pc % self.mem_size
        return {"pc": pc, "instr": instr, "op": op}

    def run(self, max_steps: int = 1_000_000) -> int:
        """Run until HLT (returns result = r7) or max_steps (raises)."""
        for _ in range(max_steps):
            try:
                self.step()
            except Halt:
                return self.result
        raise RuntimeError(f"PISC did not halt within {max_steps} steps "
                           f"(pc={self.pc}, runaway program?)")


if __name__ == "__main__":
    # Tiny demo: assemble + run the sum-1..10 program, print the trace.
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from tools.pisc_asm import assemble

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
    print(f"result = {cpu.run()}  (expected 55), cycles = {cpu.cycles}")
