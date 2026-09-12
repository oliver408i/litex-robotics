#!/usr/bin/env python3
"""Assembler for the PSC16S (PISC v2) draft encoding.

PROTOTYPE against a DRAFT ISA -- docs/psc16s_isa_draft.md. Separate file from
tools/pisc_asm.py on purpose: v1's encoding is frozen and its assembler must
keep working unchanged.

    18-bit instruction word
     17  14 | 13  11 | 10   8 | 7                     0
     opcode |   rd   |   rs   |  imm8 / rs2+funct5 / port+bits

Usage:
    python3 tools/psc16s_asm.py prog.s --hex prog.hex     # $readmemh, for INITVAL
    python3 tools/psc16s_asm.py prog.s --list             # address/word/source
    python3 tools/psc16s_asm.py prog.s --py               # python list, for CSR loads

Syntax notes (differences from v1's assembler are marked):
    ; comment                    labels as `name:`
    .def NAME value              constant
    .org N                       set the assembly base address (NEW: the
                                 writable imem region starts at ro_words, so a
                                 post-boot program is assembled with --org 256)
    .word v[, v...]              raw data words
    registers r0-r7; r0 reads 0, r6 is the link register, r7 -> result
    immediates: decimal, 0xhex, 'c' char literal, a .def name, or a label
    a leading # is ignored, so `SETB p, #3` reads naturally
"""
from __future__ import annotations

import argparse
import re
import sys

# --- opcodes -----------------------------------------------------------------
OP = dict(JMP=0x0, ALU=0x1, ADDI=0x2, LI=0x3, LD=0x4, ST=0x5, BEQ=0x6, BNE=0x7,
          JAL=0x8, JALR=0x9, PORT=0xA, BITOP=0xB, WAIT=0xC, BUS=0xD,
          DELAY=0xE, HLT=0xF)

# ALU funct field. 0x05-0x1F are reserved and deliberately unassigned; the
# assembler refuses them so a program cannot depend on decode-to-ADD.
FUNCT = dict(ADD=0x00, SUB=0x01, AND=0x02, OR=0x03, XOR=0x04)


class AsmError(Exception):
    pass


def _mask(v: int, bits: int) -> int:
    return v & ((1 << bits) - 1)


def _fits_signed(v: int, bits: int) -> bool:
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return lo <= v <= hi


class Assembler:
    def __init__(self):
        self.defs: dict[str, int] = {}
        self.labels: dict[str, int] = {}
        self.org = 0

    # -- operand parsing ------------------------------------------------------
    def reg(self, tok: str) -> int:
        t = tok.strip().lower()
        if not re.fullmatch(r"r[0-7]", t):
            raise AsmError(f"expected register r0-r7, got {tok!r}")
        return int(t[1])

    def val(self, tok: str, pass2: bool) -> int:
        t = tok.strip().lstrip("#").strip()
        if not t:
            raise AsmError("empty operand")
        if re.fullmatch(r"'.'", t):
            return ord(t[1])
        try:
            return int(t, 0)
        except ValueError:
            pass
        if t in self.defs:
            return self.defs[t]
        if t in self.labels:
            return self.labels[t]
        if pass2:
            raise AsmError(f"undefined symbol {t!r}")
        return 0        # pass 1 placeholder; sizes are fixed at 1 word anyway

    def half(self, tok: str) -> int:
        t = tok.strip().lower()
        if t in ("lo", "low", "l"):
            return 0
        if t in ("hi", "high", "h"):
            return 1
        raise AsmError(f"expected lo|hi, got {tok!r}")

    # -- encoding -------------------------------------------------------------
    def encode(self, mnem: str, ops: list[str], pc: int, pass2: bool) -> int:
        m = mnem.upper()
        V = lambda t: self.val(t, pass2)

        def rel(tok: str, bits: int) -> int:
            # Branch/jump targets are relative to the branch's own PC, as in v1.
            target = V(tok)
            off = target - pc if (tok.strip() in self.labels) else target
            if pass2 and not _fits_signed(off, bits):
                raise AsmError(f"{m} target out of range: offset {off} "
                               f"needs {bits} signed bits")
            return _mask(off, bits)

        # ---- pseudo-ops (each exactly one real instruction) -----------------
        if m == "NOP":
            return self.encode("ALU", ["r0", "r0", "r0", "ADD"], pc, pass2)
        if m == "MOV":
            return self.encode("ALU", [ops[0], ops[1], "r0", "ADD"], pc, pass2)
        if m == "RET":
            return self.encode("JALR", ["r6", "0"], pc, pass2)
        if m == "SETB":
            return self.encode("BITOP", [ops[0], ops[1], "1"], pc, pass2)
        if m == "CLRB":
            return self.encode("BITOP", [ops[0], ops[1], "0"], pc, pass2)
        if m == "IN":                                   # IN rd, port
            rd = self.reg(ops[0]); port = V(ops[1])
            return (OP["PORT"] << 14) | (rd << 11) | (0 << 7) | _mask(port, 6)
        if m == "OUT":                                  # OUT port, rs
            port = V(ops[0]); rs = self.reg(ops[1])
            return (OP["PORT"] << 14) | (rs << 8) | (1 << 7) | _mask(port, 6)
        if m in FUNCT:                                  # ADD/SUB/AND/OR/XOR rd,rs,rs2
            return self.encode("ALU", [ops[0], ops[1], ops[2], m], pc, pass2)
        if m == "BUSR":                                 # BUSR rd, rs, half[, imm6]
            return self.encode("BUS", [ops[0], ops[1], "r",
                                       ops[2], ops[3] if len(ops) > 3 else "0"],
                               pc, pass2)
        if m == "BUSW":                                 # BUSW rd, rs, half[, imm6]
            return self.encode("BUS", [ops[0], ops[1], "w",
                                       ops[2], ops[3] if len(ops) > 3 else "0"],
                               pc, pass2)

        # ---- real instructions ----------------------------------------------
        if m == "JMP":
            return (OP["JMP"] << 14) | rel(ops[0], 14)
        if m == "JAL":
            return (OP["JAL"] << 14) | rel(ops[0], 14)
        if m == "JALR":
            rs = self.reg(ops[0]); imm = V(ops[1]) if len(ops) > 1 else 0
            return (OP["JALR"] << 14) | (rs << 8) | _mask(imm, 8)
        if m == "ALU":
            rd, rs, rs2 = self.reg(ops[0]), self.reg(ops[1]), self.reg(ops[2])
            fn = ops[3].strip().upper() if len(ops) > 3 else "ADD"
            if fn not in FUNCT:
                raise AsmError(f"unknown/reserved ALU funct {fn!r}; "
                               f"defined: {', '.join(sorted(FUNCT))}")
            return (OP["ALU"] << 14) | (rd << 11) | (rs << 8) | (rs2 << 5) | FUNCT[fn]
        if m == "ADDI":
            rd, rs, imm = self.reg(ops[0]), self.reg(ops[1]), V(ops[2])
            if pass2 and not _fits_signed(imm, 8):
                raise AsmError(f"ADDI immediate {imm} does not fit in 8 signed bits")
            return (OP["ADDI"] << 14) | (rd << 11) | (rs << 8) | _mask(imm, 8)
        if m == "LI":
            rd, imm = self.reg(ops[0]), V(ops[1])
            if pass2 and not _fits_signed(imm, 11):
                raise AsmError(f"LI immediate {imm} does not fit in 11 signed bits "
                               f"(-1024..1023). CSR offsets need a constant pool "
                               f"-- see sketch finding F1.")
            return (OP["LI"] << 14) | (rd << 11) | _mask(imm, 11)
        if m in ("LD", "ST"):
            rd, rs, imm = self.reg(ops[0]), self.reg(ops[1]), \
                          (V(ops[2]) if len(ops) > 2 else 0)
            if pass2 and not _fits_signed(imm, 8):
                raise AsmError(f"{m} offset {imm} does not fit in 8 signed bits")
            return (OP[m] << 14) | (rd << 11) | (rs << 8) | _mask(imm, 8)
        if m in ("BEQ", "BNE"):
            rd, rs = self.reg(ops[0]), self.reg(ops[1])
            return (OP[m] << 14) | (rd << 11) | (rs << 8) | rel(ops[2], 8)
        if m == "PORT":
            raise AsmError("write PORT as the IN/OUT pseudo-ops")
        if m in ("BITOP", "WAIT"):
            port, bit, lvl = V(ops[0]), V(ops[1]), V(ops[2])
            if pass2 and not (0 <= bit <= 15):
                raise AsmError(f"{m} bit index {bit} out of range 0-15")
            return (OP[m] << 14) | (_mask(bit, 4) << 8) | ((lvl & 1) << 7) | _mask(port, 6)
        if m == "BUS":
            rd, rs = self.reg(ops[0]), self.reg(ops[1])
            dtok = ops[2].strip().lower()
            if dtok not in ("r", "w", "rd", "wr", "read", "write"):
                raise AsmError(f"BUS direction must be r|w, got {ops[2]!r}")
            dirbit = 0 if dtok.startswith("r") else 1
            hb = self.half(ops[3])
            imm = V(ops[4]) if len(ops) > 4 else 0
            if pass2 and not _fits_signed(imm, 6):
                raise AsmError(f"BUS offset {imm} does not fit in 6 signed bits "
                               f"(+-31 bytes = 7 LiteX CSRs) -- see sketch F2")
            return (OP["BUS"] << 14) | (rd << 11) | (rs << 8) | (dirbit << 7) \
                   | (hb << 6) | _mask(imm, 6)
        if m == "DELAY":
            imm = V(ops[0])
            if pass2 and not (0 <= imm <= 0x3FFF):
                raise AsmError(f"DELAY count {imm} out of range 0-16383")
            return (OP["DELAY"] << 14) | _mask(imm, 14)
        if m == "HLT":
            return OP["HLT"] << 14

        raise AsmError(f"unknown mnemonic {mnem!r}")

    # -- driver ---------------------------------------------------------------
    def assemble(self, text: str):
        lines = []
        for lineno, raw in enumerate(text.splitlines(), 1):
            line = raw.split(";", 1)[0].strip()
            if line:
                lines.append((lineno, line, raw))

        # pass 1: labels, .def, .org, sizes
        pc = self.org
        work = []
        for lineno, line, raw in lines:
            while True:
                mlab = re.match(r"^([A-Za-z_.$][\w.$]*)\s*:\s*(.*)$", line)
                if not mlab:
                    break
                self.labels[mlab.group(1)] = pc
                line = mlab.group(2).strip()
            if not line:
                continue
            parts = line.split(None, 1)
            mnem = parts[0]
            rest = parts[1] if len(parts) > 1 else ""
            if mnem.lower() == ".def":
                name, value = rest.split(None, 1)
                self.defs[name] = self.val(value, True)
                continue
            if mnem.lower() == ".org":
                if work:
                    raise AsmError(f"line {lineno}: .org must precede all code")
                self.org = pc = self.val(rest, True)
                continue
            ops = [o for o in rest.split(",")] if rest.strip() else []
            n = len(ops) if mnem.lower() == ".word" else 1
            work.append((lineno, pc, mnem, ops, raw))
            pc += n

        # pass 2: encode
        out: list[tuple[int, int, str]] = []
        for lineno, addr, mnem, ops, raw in work:
            try:
                if mnem.lower() == ".word":
                    for i, o in enumerate(ops):
                        out.append((addr + i, _mask(self.val(o, True), 18), raw))
                else:
                    out.append((addr, _mask(self.encode(mnem, ops, addr, True), 18), raw))
            except AsmError as e:
                raise AsmError(f"line {lineno}: {e}") from None
            except IndexError:
                raise AsmError(f"line {lineno}: too few operands for {mnem}") from None
        return out


def main() -> int:
    ap = argparse.ArgumentParser(description="PSC16S (PISC v2) assembler -- DRAFT ISA.")
    ap.add_argument("source")
    ap.add_argument("--hex", metavar="FILE", help="write $readmemh output (INITVAL).")
    ap.add_argument("--py", action="store_true", help="print a python list of words.")
    ap.add_argument("--list", action="store_true", help="print an address/word listing.")
    ap.add_argument("--words", type=int, default=None,
                    help="pad the hex output to this many words.")
    args = ap.parse_args()

    with open(args.source) as f:
        text = f.read()

    asm = Assembler()
    try:
        out = asm.assemble(text)
    except AsmError as e:
        print(f"{args.source}: {e}", file=sys.stderr)
        return 1

    if args.list or not (args.hex or args.py):
        for addr, word, raw in out:
            print(f"{addr:04x}  {word:05x}  {raw.strip()}")
    if args.py:
        print("[" + ", ".join(f"0x{w:05x}" for _, w, _ in out) + "]")
    if args.hex:
        top = max(a for a, _, _ in out) + 1 if out else 0
        size = args.words or top
        img = [0] * size
        for addr, word, _ in out:
            if addr >= size:
                print(f"{args.source}: word at {addr} exceeds --words {size}",
                      file=sys.stderr)
                return 1
            img[addr] = word
        with open(args.hex, "w") as f:
            for w in img:
                f.write(f"{w:05x}\n")
        print(f"wrote {args.hex}: {size} words ({len(out)} emitted)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
