#!/usr/bin/env python3
"""Regenerate the Q15 twiddle tables embedded in main.c.

Not part of the build -- main.c carries the literal arrays so this directory
builds the same dependency-free way as rv32_test (no codegen step). Re-run
this and paste the output back into main.c only if NMAX changes.
"""
import math

NMAX = 1024
HALF = NMAX // 2


def table(fn):
    vals = [round(fn(-2.0 * math.pi * k / NMAX) * 32767) for k in range(HALF)]
    lines = []
    for i in range(0, len(vals), 12):
        lines.append("    " + ", ".join(str(v) for v in vals[i:i + 12]) + ",")
    return "\n".join(lines)


if __name__ == "__main__":
    print(f"static const int16_t TW_COS[{HALF}] = {{")
    print(table(math.cos))
    print("};\n")
    print(f"static const int16_t TW_SIN[{HALF}] = {{")
    print(table(math.sin))
    print("};")
