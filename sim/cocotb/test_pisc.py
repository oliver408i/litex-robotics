"""Cocotb differential test for verilog/pisc.v.

sim/pisc_model.py is the golden reference. Each program is loaded into the DUT's
instruction memory, `run` is pulsed, and the DUT is single-stepped at its
`retire` strobe -- after every committed instruction we compare the full
architectural state (r0-r7, next PC, all output ports) against the model.
Memory is compared at HLT. Any divergence is a Verilog bug.

Coverage = directed programs (one per opcode / addressing mode, plus a backward
loop and a wait-on-pin) + a randomized straight-line fuzz.

Run via sim/cocotb/run.sh from the repo root:
    ./run.sh TARGET=pisc
"""
from __future__ import annotations

import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO))

from tools.pisc_asm import assemble                       # noqa: E402
from sim.pisc_model import PISC, Halt                     # noqa: E402

CLOCK_PERIOD_NS = 10


def _param(dut, name, default):
    """Read a Verilog parameter from the DUT, falling back to a default."""
    try:
        return int(getattr(dut, name).value)
    except Exception:
        return default


# ----------------------------------------------------------------------------
# DUT helpers
# ----------------------------------------------------------------------------
async def reset_dut(dut):
    dut.rst.value = 1
    dut.run.value = 0
    dut.clr.value = 0
    dut.imem_we.value = 0
    dut.imem_addr.value = 0
    dut.imem_data.value = 0
    dut.io_in.value = 0
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def load_program(dut, words, imem_words):
    # Zero-fill the whole memory first: reset_dut clears regs/io/pc but not the
    # BRAM, so without this a shorter program would leave stale words from a
    # previous program past its end (the model always starts memory-clean).
    padded = list(words) + [0] * (imem_words - len(words))
    for addr, w in enumerate(padded):
        dut.imem_addr.value = addr
        dut.imem_data.value = int(w) & 0xFFFF
        dut.imem_we.value = 1
        await RisingEdge(dut.clk)
    dut.imem_we.value = 0
    dut.imem_addr.value = 0
    dut.imem_data.value = 0
    await RisingEdge(dut.clk)


async def pulse_run(dut):
    dut.run.value = 1
    await RisingEdge(dut.clk)
    dut.run.value = 0


def _io_out_port(dut, p):
    return (int(dut.io_out.value) >> (16 * p)) & 0xFFFF


def _compare_state(dut, model, num_out, where):
    for i in range(8):
        got = int(dut.regs[i].value)
        exp = model.regs[i]
        assert got == exp, f"{where}: r{i} = {got:#06x}, model {exp:#06x}"
    for p in range(num_out):
        got = _io_out_port(dut, p)
        exp = model.io_out.get(p, 0)
        assert got == exp, f"{where}: io_out[{p}] = {got:#06x}, model {exp:#06x}"


async def run_and_diff(dut, words, io_in_const, imem_words, num_out, num_in,
                       max_cycles=20000):
    """Load + run a program on both DUT and model, diffing at every retire."""
    model = PISC(words, mem_size=imem_words,
                 io_in=lambda p: (io_in_const >> (16 * p)) & 0xFFFF)

    await reset_dut(dut)
    await load_program(dut, words, imem_words)
    dut.io_in.value = io_in_const
    await pulse_run(dut)

    retired = 0
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.retire.value) != 1:
            continue
        halted = False
        try:
            model.step()
        except Halt:
            halted = True
        _compare_state(dut, model, num_out, f"after instr #{retired}")
        if not halted:
            assert int(dut.dbg_pc.value) == model.pc, (
                f"after instr #{retired}: dbg_pc={int(dut.dbg_pc.value)}, "
                f"model.pc={model.pc}")
        retired += 1
        if int(dut.halted.value) == 1:
            assert halted, f"DUT halted but model did not (instr #{retired})"
            assert int(dut.result.value) == model.result, (
                f"result {int(dut.result.value):#06x} != model "
                f"{model.result:#06x}")
            # Full memory check (covers ST / self-modifying programs).
            for a in range(imem_words):
                got = int(dut.mem[a].value)
                assert got == model.mem[a], (
                    f"mem[{a}] = {got:#06x}, model {model.mem[a]:#06x}")
            return retired
    raise TimeoutError(f"DUT did not halt within {max_cycles} cycles "
                       f"({retired} instrs retired)")


# ----------------------------------------------------------------------------
# Directed tests -- one program per opcode / addressing mode
# ----------------------------------------------------------------------------
DIRECTED = {
    "sum_1_to_10": """
            LDI  r1, 0
            LDI  r2, 1
            LDI  r3, 11
    loop:   ADD  r1, r1, r2
            ADDI r2, r2, 1
            BNE  r2, r3, loop
            MOV  r7, r1
            HLT
    """,
    "alu_ops": """
            LDI  r1, 12
            LDI  r2, 10
            SUB  r3, r1, r2
            AND  r4, r1, r2
            OR   r5, r1, r2
            ADD  r7, r3, r4
            HLT
    """,
    "sext_wrap": """
            LDI  r1, 5
            ADDI r1, r1, -8
            MOV  r7, r1
            HLT
    """,
    "load_store": """
            LDI  r1, 7
            LDI  r2, 20
            ST   r1, r2, 5
            LD   r7, r2, 5
            HLT
    """,
    "jmp_skip": """
            LDI  r7, 1
            JMP  2
            LDI  r7, 9
            HLT
    """,
    "io_pulse": """
            .def RST 0
            SETB  RST, 0
            CLRB  RST, 0
            OUT   1, r0
            SETB  RST, 0
            HLT
    """,
    "r0_hardwired": """
            ADDI r0, r0, 5
            MOV  r7, r0
            HLT
    """,
    "delay": """
            LDI   r1, 3
            DELAY 5
            MOV   r7, r1
            HLT
    """,
}


@cocotb.test()
async def directed(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    imem_words = _param(dut, "IMEM_WORDS", 256)
    num_out = _param(dut, "NUM_OUT", 8)
    num_in = _param(dut, "NUM_IN", 8)

    for name, src in DIRECTED.items():
        words = assemble(src)
        n = await run_and_diff(dut, words, 0, imem_words, num_out, num_in)
        dut._log.info(f"directed '{name}': {n} instrs OK")


@cocotb.test()
async def reads_inputs(dut):
    """IN reads a fixed input word; diff covers it against the model."""
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    imem_words = _param(dut, "IMEM_WORDS", 256)
    num_out = _param(dut, "NUM_OUT", 8)
    num_in = _param(dut, "NUM_IN", 8)

    src = """
            IN   r1, 0
            IN   r2, 1
            OR   r7, r1, r2
            OUT  0, r1
            HLT
    """
    words = assemble(src)
    io_in_const = (0x00AB) | (0x0C00 << 16)        # port0=0x00AB, port1=0x0C00
    await run_and_diff(dut, words, io_in_const, imem_words, num_out, num_in)


@cocotb.test()
async def wait_on_changing_pin(dut):
    """WAIT must block until the input pin reaches the level, then proceed."""
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    src = """
            WAIT 0, 0, 1
            LDI  r7, 21
            HLT
    """
    words = assemble(src)
    imem_words = _param(dut, "IMEM_WORDS", 256)
    await reset_dut(dut)
    await load_program(dut, words, imem_words)
    dut.io_in.value = 0                 # pin low: WAIT should block
    await pulse_run(dut)

    # Let it spin a while with the pin low; it must NOT have halted.
    for _ in range(40):
        await RisingEdge(dut.clk)
    assert int(dut.halted.value) == 0, "WAIT did not block while pin was low"

    dut.io_in.value = 1                 # drive port0 bit0 high
    for _ in range(20):
        await RisingEdge(dut.clk)
        if int(dut.halted.value) == 1:
            break
    assert int(dut.halted.value) == 1, "WAIT never released after pin went high"
    assert int(dut.result.value) == 21


# ----------------------------------------------------------------------------
# Randomized straight-line fuzz (guaranteed to terminate: forward-only branches)
# ----------------------------------------------------------------------------
def _rand_program(rng, num_in):
    """Generate straight-line PISC asm (occasional forward branch), ends HLT."""
    n = rng.randint(8, 24)
    lines = []
    # Seed a couple registers so ALU ops have non-trivial inputs.
    for r in range(1, 8):
        lines.append(f"LDI r{r}, {rng.randint(-31, 31)}")
    body_len = n
    for idx in range(body_len):
        remaining = body_len - idx
        choices = ["ADD", "SUB", "AND", "OR", "ADDI", "MOV", "OUT", "SETB",
                   "CLRB", "IN", "DELAY", "NOP"]
        if remaining > 4:
            choices += ["JMP", "BNE"]      # forward only, bounded
        op = rng.choice(choices)
        rd = rng.randint(0, 7)
        rs = rng.randint(0, 7)
        rs2 = rng.randint(0, 7)
        if op in ("ADD", "SUB", "AND", "OR"):
            lines.append(f"{op} r{rd}, r{rs}, r{rs2}")
        elif op == "ADDI":
            lines.append(f"ADDI r{rd}, r{rs}, {rng.randint(-31, 31)}")
        elif op == "MOV":
            lines.append(f"MOV r{rd}, r{rs}")
        elif op == "OUT":
            lines.append(f"OUT {rng.randint(0, 7)}, r{rs}")
        elif op in ("SETB", "CLRB"):
            lines.append(f"{op} {rng.randint(0, 7)}, {rng.randint(0, 7)}")
        elif op == "IN":
            lines.append(f"IN r{rd}, {rng.randint(0, 7)}")
        elif op == "DELAY":
            lines.append(f"DELAY {rng.randint(0, 6)}")
        elif op == "NOP":
            lines.append("NOP")
        elif op == "JMP":
            lines.append(f"JMP {rng.randint(1, min(3, remaining - 1))}")
        elif op == "BNE":
            lines.append(f"BNE r{rs}, r{rd}, {rng.randint(1, min(3, remaining - 1))}")
    lines.append("HLT")
    return "\n".join(lines)


@cocotb.test()
async def fuzz(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    imem_words = _param(dut, "IMEM_WORDS", 256)
    num_out = _param(dut, "NUM_OUT", 8)
    num_in = _param(dut, "NUM_IN", 8)

    n_programs = int(os.environ.get("PISC_FUZZ_N", "60"))
    rng = random.Random(12345)
    for k in range(n_programs):
        src = _rand_program(rng, num_in)
        words = assemble(src)
        io_in_const = rng.getrandbits(16 * num_in)
        try:
            await run_and_diff(dut, words, io_in_const, imem_words,
                               num_out, num_in)
        except Exception:
            dut._log.error(f"fuzz program #{k} failed:\n{src}")
            raise
    dut._log.info(f"fuzz: {n_programs} random programs OK")
