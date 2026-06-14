#!/usr/bin/env python3
"""Assembler for the PISC sequencer core. See docs/pisc_isa.md (the frozen
ISA contract this file and sim/pisc_model.py must agree on).

Two passes: pass 1 fixes label addresses (every instruction is exactly one
16-bit word, pseudo-ops included); pass 2 encodes with labels resolved.

Use as a library:
    from tools.pisc_asm import assemble
    words = assemble(source_text)          # -> list[int], one 16-bit word each

Or as a CLI:
    python tools/pisc_asm.py prog.pisc            # $readmemh hex to stdout
    python tools/pisc_asm.py prog.pisc -o prog.hex
    python tools/pisc_asm.py prog.pisc -f c        # C array
"""
from __future__ import annotations

import argparse
import sys

# Opcodes (must match docs/pisc_isa.md and sim/pisc_model.py).
OP = {
    "JMP": 0x0, "ADD": 0x1, "SUB": 0x2, "ADDI": 0x3, "AND": 0x4, "OR": 0x5,
    "LD": 0x6, "ST": 0x7, "BNE": 0x8, "OUT": 0x9, "IN": 0xA, "SETB": 0xB,
    "CLRB": 0xC, "WAIT": 0xD, "DELAY": 0xE, "HLT": 0xF,
}


class AsmError(Exception):
    """Assembly error carrying the offending source line number (1-based)."""


def _word(op, rd=0, rs=0, low=0):
    return ((op & 0xF) << 12) | ((rd & 0x7) << 9) | ((rs & 0x7) << 6) | (low & 0x3F)


def _split(line):
    """Tokenize one source line: drop comments, treat , [ ] + as separators."""
    line = line.split(";", 1)[0]
    for ch in ",[]+":
        line = line.replace(ch, " ")
    return line.split()


def assemble(source: str) -> list[int]:
    """Assemble PISC source text into a list of 16-bit instruction words."""
    defs: dict[str, int] = {}
    labels: dict[str, int] = {}

    # Normalize into (lineno, tokens) and strip labels/.def, assigning addresses.
    program = []   # list of (lineno, tokens) for real instructions
    addr = 0
    for lineno, raw in enumerate(source.splitlines(), 1):
        toks = _split(raw)
        while toks and toks[0].endswith(":"):       # one or more labels on a line
            labels[toks[0][:-1]] = addr
            toks = toks[1:]
        if not toks:
            continue
        if toks[0] == ".def":
            if len(toks) != 3:
                raise AsmError(f"line {lineno}: .def NAME VALUE")
            defs[toks[1]] = _imm(toks[2], defs, lineno)
            continue
        program.append((lineno, toks))
        addr += 1                                   # every instruction is 1 word

    # Pass 2: encode.
    words = []
    for pc, (lineno, toks) in enumerate(program):
        mnem = toks[0].upper()
        args = toks[1:]
        try:
            words.append(_encode(mnem, args, pc, defs, labels, lineno))
        except AsmError:
            raise
        except Exception as e:                       # noqa: BLE001 - reframe as AsmError
            raise AsmError(f"line {lineno}: {mnem}: {e}") from e
    return words


def _reg(tok, lineno):
    t = tok.lower()
    if not (t.startswith("r") and t[1:].isdigit() and 0 <= int(t[1:]) <= 7):
        raise AsmError(f"line {lineno}: expected register r0-r7, got '{tok}'")
    return int(t[1:])


def _imm(tok, defs, lineno):
    t = tok.lstrip("#")
    if t in defs:
        return defs[t]
    try:
        return int(t, 0)
    except ValueError:
        raise AsmError(f"line {lineno}: expected immediate, got '{tok}'") from None


def _rel(tok, pc, defs, labels, lineno, bits):
    """Resolve a branch/jump target to a signed self-relative offset."""
    t = tok.lstrip("#")
    off = (labels[t] - pc) if t in labels else _imm(tok, defs, lineno)
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    if not (lo <= off <= hi):
        raise AsmError(f"line {lineno}: branch/jump offset {off} out of range "
                       f"[{lo},{hi}] (target too far)")
    return off & ((1 << bits) - 1)


def _check_imm(val, bits, signed, lineno):
    if signed:
        lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    else:
        lo, hi = 0, (1 << bits) - 1
    if not (lo <= val <= hi):
        raise AsmError(f"line {lineno}: immediate {val} out of range [{lo},{hi}]")
    return val & ((1 << bits) - 1)


def _encode(mnem, args, pc, defs, labels, lineno):
    R = lambda i: _reg(args[i], lineno)                       # noqa: E731
    I = lambda i: _imm(args[i], defs, lineno)                 # noqa: E731

    def need(n):
        if len(args) != n:
            raise AsmError(f"line {lineno}: {mnem} takes {n} operand(s), got {len(args)}")

    # --- pseudo-ops -----------------------------------------------------
    if mnem == "NOP":
        need(0); return _word(OP["ADD"])
    if mnem == "MOV":
        need(2); return _word(OP["ADD"], rd=R(0), rs=R(1))    # rd = rs + r0
    if mnem == "LDI":
        need(2); return _word(OP["ADDI"], rd=R(0), rs=0,
                              low=_check_imm(I(1), 6, True, lineno))

    if mnem not in OP:
        raise AsmError(f"line {lineno}: unknown mnemonic '{mnem}'")
    op = OP[mnem]

    # --- real instructions ----------------------------------------------
    if mnem in ("ADD", "SUB", "AND", "OR"):                   # rd, rs, rs2
        need(3); return _word(op, rd=R(0), rs=R(1), low=(R(2) << 3))
    if mnem in ("ADDI", "LD", "ST"):                          # rd, rs, imm6
        need(3); return _word(op, rd=R(0), rs=R(1),
                              low=_check_imm(I(2), 6, True, lineno))
    if mnem == "BNE":                                         # ra, rb, target
        need(3); return _word(op, rd=R(1), rs=R(0),
                              low=_rel(args[2], pc, defs, labels, lineno, 6))
    if mnem == "JMP":                                         # target
        need(1)
        off = _rel(args[0], pc, defs, labels, lineno, 12)
        return (op << 12) | off
    if mnem == "OUT":                                         # port, rs
        need(2); return _word(op, rs=R(1), low=_check_imm(I(0), 6, False, lineno))
    if mnem == "IN":                                          # rd, port
        need(2); return _word(op, rd=R(0), low=_check_imm(I(1), 6, False, lineno))
    if mnem in ("SETB", "CLRB"):                              # port, bit
        need(2)
        return _word(op, rd=_check_imm(I(1), 3, False, lineno),
                     low=_check_imm(I(0), 6, False, lineno))
    if mnem == "WAIT":                                        # port, bit, lvl
        need(3)
        bit = _check_imm(I(1), 3, False, lineno)
        lvl = _check_imm(I(2), 1, False, lineno)
        port = _check_imm(I(0), 6, False, lineno)
        return (op << 12) | (bit << 9) | (lvl << 8) | port
    if mnem == "DELAY":                                       # imm12
        need(1); return (op << 12) | _check_imm(I(0), 12, False, lineno)
    if mnem == "HLT":
        need(0); return _word(op)
    raise AsmError(f"line {lineno}: unhandled mnemonic '{mnem}'")


def _emit(words, fmt):
    if fmt == "hex":                                          # $readmemh
        return "\n".join(f"{w:04x}" for w in words) + "\n"
    if fmt == "c":
        body = ",\n    ".join(f"0x{w:04x}" for w in words)
        return (f"static const unsigned short pisc_prog[{len(words)}] = {{\n"
                f"    {body}\n}};\n")
    if fmt == "list":
        return repr(words) + "\n"
    raise ValueError(f"unknown format {fmt}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Assemble PISC source.")
    ap.add_argument("source", help="input .pisc file ('-' for stdin)")
    ap.add_argument("-o", "--output", help="output file (default stdout)")
    ap.add_argument("-f", "--format", choices=("hex", "c", "list"), default="hex")
    a = ap.parse_args(argv)

    text = sys.stdin.read() if a.source == "-" else open(a.source).read()
    try:
        words = assemble(text)
    except AsmError as e:
        print(f"pisc_asm: {e}", file=sys.stderr)
        return 1
    out = _emit(words, a.format)
    if a.output:
        with open(a.output, "w") as f:
            f.write(out)
    else:
        sys.stdout.write(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
