#!/usr/bin/env python3
"""RREF on the i9: PSC18SR sequences it, the row-op engine does the arithmetic.

Needs targets/colorlight_i9/rref.py loaded and litex_server running:

    .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &
    .venv/bin/python tools/rref_demo.py '[[2,4,-2],[4,9,-3],[-2,-3,7]]'
    .venv/bin/python tools/rref_demo.py '[[1,2,3],[4,5,6]]' '[["1/2", 3], [5, "-2/3"]]'
    .venv/bin/python tools/rref_demo.py --random 50      # check against Python

Each run: assemble software/psc18sr_rref/rref.s, load it into PSC18SR imem,
write the matrix into the engine, pulse run, wait for HLT, read the matrix
back. The result is checked against a plain fractions.Fraction Gauss-Jordan.
"""
from __future__ import annotations

import argparse
import ast
import random
import sys
import time
from fractions import Fraction
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
for _p in ["litex", "litex-boards", "migen"]:
    _cand = REPO / "litex-setup" / _p
    if _cand.is_dir() and str(_cand) not in sys.path:
        sys.path.insert(0, str(_cand))
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "sim"))

from litex import RemoteClient          # noqa: E402
from psc18sr_asm import Assembler       # noqa: E402
from test_rref import rref as reference  # noqa: E402

PROGRAM = REPO / "software/psc18sr_rref/rref.s"
ST_HALTED = 1 << 1


def to_u32(x: int) -> int:
    return x & 0xFFFFFFFF


def to_s32(x: int) -> int:
    return x - (1 << 32) if x & 0x80000000 else x


class Board:
    def __init__(self, csr_csv: str):
        self.bus = RemoteClient(csr_csv=csr_csv)
        self.bus.open()
        self.r = self.bus.regs
        if not hasattr(self.r, "rref_status"):
            raise SystemExit("no rref block in this csr.csv -- wrong bitstream?")

    def load_program(self):
        words = Assembler().assemble(PROGRAM.read_text())
        self.r.psc18sr_control.write(2)                 # abort: idle the core
        for addr, word, _ in words:
            self.r.psc18sr_imem_addr.write(addr)
            self.r.psc18sr_imem_data.write(word)
            self.r.psc18sr_imem_ctl.write(1)
        self.start = min(a for a, _, _ in words)
        self.r.psc18sr_start_pc.write(self.start)
        return len(words)

    def solve(self, m):
        rows, cols = len(m), len(m[0])
        r = self.r
        r.rref_ctl.write(2)                              # clear ovf/err
        r.rref_dims.write(rows | (cols << 4))
        for i, row in enumerate(m):
            for j, x in enumerate(row):
                x = Fraction(x)
                r.rref_addr.write(i * 8 + j)
                r.rref_num.write(to_u32(x.numerator))
                r.rref_den.write(x.denominator)
                r.rref_ctl.write(1)
        ops0 = r.rref_ops_done.read()
        t0 = time.monotonic()
        r.psc18sr_control.write(1)                       # run
        while not (r.psc18sr_status.read() & ST_HALTED):
            if time.monotonic() - t0 > 5:
                r.psc18sr_control.write(2)
                raise RuntimeError("PSC18SR did not halt in 5 s")
        rank = r.psc18sr_result.read()
        sts = r.rref_status.read()
        out = []
        for i in range(rows):
            row = []
            for j in range(cols):
                r.rref_addr.write(i * 8 + j)
                row.append(Fraction(to_s32(r.rref_rnum.read()), r.rref_rden.read() or 1))
            out.append(row)
        return out, rank, dict(ovf=bool(sts & 4), err=bool(sts & 8),
                               ops=r.rref_ops_done.read() - ops0)


def fmt(m):
    w = max(len(str(x)) for row in m for x in row)
    return "\n".join("  [ " + "  ".join(f"{str(x):>{w}}" for x in row) + " ]" for row in m)


def parse_matrix(s: str):
    m = ast.literal_eval(s)
    m = [[Fraction(x) for x in row] for row in m]
    if not m or not (1 <= len(m) <= 8) or any(not (1 <= len(r) <= 8) for r in m) \
            or len({len(r) for r in m}) != 1:
        raise SystemExit(f"need a rectangular matrix, 1-8 rows and columns: {s}")
    return m


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("matrix", nargs="*", help="python-style list of rows; "
                    "entries may be ints or 'p/q' strings")
    ap.add_argument("--random", type=int, default=0, metavar="N",
                    help="also run N random integer matrices and check them")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--csr-csv", default=str(REPO / "build/colorlight_i5/csr.csv"))
    args = ap.parse_args()

    b = Board(args.csr_csv)
    n = b.load_program()
    print(f"loaded {PROGRAM.relative_to(REPO)}: {n} words at {b.start}")

    fails = 0
    for s in args.matrix:
        m = parse_matrix(s)
        got, rank, info = b.solve(m)
        want, wrank, fits = reference(m)
        ok = got == want and rank == wrank and not info["ovf"]
        print(f"\ninput {len(m)}x{len(m[0])}:\n{fmt(m)}")
        print(f"rref (rank {rank}, {info['ops']} engine ops)"
              + ("  OVERFLOW" if info["ovf"] else "") + ":")
        print(fmt(got))
        print("matches Python" if ok else f"MISMATCH, expected:\n{fmt(want)}")
        fails += not ok

    rng = random.Random(args.seed)
    n_ovf = rfails = 0
    for k in range(args.random):
        rows, cols = rng.randint(1, 8), rng.randint(1, 8)
        m = [[rng.randint(-9, 9) if rng.random() > 0.2 else 0 for _ in range(cols)]
             for _ in range(rows)]
        got, rank, info = b.solve(m)
        want, wrank, fits = reference(m)
        if fits:
            ok = got == want and rank == wrank and not info["ovf"] and not info["err"]
        else:
            ok = info["ovf"]
            n_ovf += 1
        if not ok:
            print(f"random #{k} FAIL:\n{fmt(m)}\ngot:\n{fmt(got)}\nwant:\n{fmt(want)}")
        rfails += not ok
    if args.random:
        print(f"\nrandom: {args.random - rfails}/{args.random} ok "
              f"({n_ovf} overflowed 32 bits and were flagged)")
    return 1 if fails or rfails else 0


if __name__ == "__main__":
    raise SystemExit(main())
