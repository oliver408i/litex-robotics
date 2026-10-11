#!/usr/bin/env python3
"""Check the RREF demo in simulation: PSC18SR + rref_engine vs fractions.Fraction.

Runs sim/rref_tb.v (the real core, the real engine, software/psc18sr_rref/rref.s)
on a few hand-picked matrices and then random ones, and compares every element
against a plain-Python Gauss-Jordan.

    source oss-cad-suite/oss-cad-suite/environment
    python3 sim/test_rref.py            # default: fixed cases + 30 random
    python3 sim/test_rref.py -n 200 --seed 7
"""
from __future__ import annotations

import argparse
import random
import subprocess
import sys
import tempfile
from fractions import Fraction
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


LIMIT = 2**31 - 1


def rref(m):
    """Reference: textbook Gauss-Jordan over Fractions.

    Returns (matrix, rank, fits): `fits` is False if any value the engine would
    have to hold -- every product K*x and every sum -- needs more than 32 bits,
    i.e. exactly when the engine should raise OVF.
    """
    fits = True

    def chk(x):
        nonlocal fits
        if abs(x.numerator) > LIMIT or x.denominator > LIMIT:
            fits = False
        return x

    m = [[Fraction(x) for x in row] for row in m]
    rows, cols = len(m), len(m[0])
    r = 0
    for c in range(cols):
        if r == rows:
            break
        p = next((i for i in range(r, rows) if m[i][c] != 0), None)
        if p is None:
            continue
        m[r], m[p] = m[p], m[r]
        k = 1 / m[r][c]
        m[r] = [chk(k * x) for x in m[r]]
        for i in range(rows):
            if i != r and m[i][c] != 0:
                f = m[i][c]
                m[i] = [chk(a + chk(-f * b)) for a, b in zip(m[i], m[r])]
        r += 1
    return m, r, fits


def build(tmp: Path) -> tuple[Path, Path]:
    prog = tmp / "rref.hex"
    subprocess.run([sys.executable, str(REPO / "tools/psc18sr_asm.py"),
                    str(REPO / "software/psc18sr_rref/rref.s"),
                    "--hex", str(prog), "--words", "1024"],
                   check=True, capture_output=True)
    vvp = tmp / "rref_tb.vvp"
    subprocess.run(["iverilog", "-g2012", "-o", str(vvp),
                    str(REPO / "sim/rref_tb.v"), str(REPO / "verilog/psc18sr.v"),
                    str(REPO / "verilog/rref_engine.v"), str(REPO / "verilog/rref_rat.v")],
                   check=True)
    return prog, vvp


def run_one(m, tmp: Path, prog: Path, vvp: Path):
    rows, cols = len(m), len(m[0])
    words = [rows, cols]
    for row in m:
        for x in row:
            x = Fraction(x)
            words += [x.numerator & 0xFFFFFFFF, x.denominator]
    matf = tmp / "mat.hex"
    matf.write_text("".join(f"{w:08x}\n" for w in words))
    out = subprocess.run(["vvp", "-n", str(vvp), f"+prog={prog}", f"+mat={matf}"],
                         check=True, capture_output=True, text=True).stdout
    got = [[None] * cols for _ in range(rows)]
    info = {}
    for line in out.splitlines():
        f = line.split()
        if not f:
            continue
        if f[0] == "M":
            r, c, n, d = map(int, f[1:])
            got[r][c] = Fraction(n, d) if d else None
            if d and Fraction(n, d).denominator != d:
                got[r][c] = "unreduced"
        elif f[0] in ("RANK", "CYCLES", "OPS"):
            info[f[0]] = int(f[1])
        elif f[0] == "FLAGS":
            info["ovf"], info["err"] = int(f[1]), int(f[2])
        elif f[0] == "TIMEOUT":
            raise RuntimeError(line)
    return got, info


def fmt(m):
    w = max(len(str(x)) for row in m for x in row)
    return "\n".join("  [" + "  ".join(f"{str(x):>{w}}" for x in row) + "]" for row in m)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=30, help="random matrices")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("-v", action="store_true", help="print every result")
    args = ap.parse_args()

    fixed = [
        [[2, 4, -2], [4, 9, -3], [-2, -3, 7]],                  # full rank 3x3
        [[1, 2, 3], [2, 4, 6], [1, 1, 1]],                      # rank 2
        [[0, 0, 1], [0, 1, 0], [1, 0, 0]],                      # needs swaps
        [[0, 0], [0, 0]],                                       # zero matrix
        [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12]],          # wide, rank 2
        [[Fraction(1, 2), Fraction(-1, 3)], [Fraction(2, 5), 7]],  # rational input
        [[3, 1, 4, 1, 5, 9, 2, 6]] * 1 + [[5, 3, 5, 8, 9, 7, 9, 3]],  # 2x8
    ]
    rng = random.Random(args.seed)
    cases = list(fixed)
    for _ in range(args.n):
        rows, cols = rng.randint(1, 8), rng.randint(1, 8)
        cases.append([[rng.randint(-9, 9) if rng.random() > 0.2 else 0
                       for _ in range(cols)] for _ in range(rows)])

    fails = n_ovf = 0
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        prog, vvp = build(tmp)
        for idx, m in enumerate(cases):
            want, rank, fits = rref(m)
            got, info = run_one(m, tmp, prog, vvp)
            if fits:
                ok = (got == want and info["RANK"] == rank
                      and not info["ovf"] and not info["err"])
                tag = "PASS" if ok else "FAIL"
            else:
                # Needs more than 32 bits: all that is promised is that the
                # engine noticed.
                ok = bool(info["ovf"])
                tag = "OVF " if ok else "FAIL"
                n_ovf += 1
            print(f"{tag} #{idx:<3} {len(m)}x{len(m[0])} rank {info['RANK']}  "
                  f"{info['OPS']:>3} engine ops  {info['CYCLES']:>7} cycles"
                  + ("  OVF" if info["ovf"] else "") + ("  ERR" if info["err"] else ""))
            if (not ok or args.v) and fits:
                print(" input:\n" + fmt(m))
                print(" engine:\n" + fmt(got))
                if not ok:
                    print(" expected:\n" + fmt(want))
            fails += not ok
    print(f"{len(cases) - fails}/{len(cases)} passed "
          f"({n_ovf} of them overflow 32 bits and were correctly flagged OVF)")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
