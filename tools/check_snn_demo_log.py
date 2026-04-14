#!/usr/bin/env python3
"""Check the standalone SNN demo UART log against the validated reference."""

from __future__ import annotations

import argparse
import re
import sys


EXPECTED = [
    (0.483, 0.321, 0.293, 13),
    (0.444, 0.513, 0.227, 13),
    (0.417, 0.481, 0.185, 13),
    (0.364, 0.433, 0.140, 13),
    (0.295, 0.361, 0.094, 13),
    (0.239, 0.292, 0.055, 13),
    (0.186, 0.229, 0.020, 13),
    (0.149, 0.180, -0.008, 13),
    (0.127, 0.150, -0.029, 13),
    (0.149, 0.159, -0.034, 13),
    (0.193, 0.203, -0.032, 13),
    (0.264, 0.277, -0.020, 13),
    (0.334, 0.363, -0.008, 13),
    (0.400, 0.445, 0.004, 13),
    (0.457, 0.517, 0.013, 13),
    (0.498, 0.574, 0.019, 13),
    (0.483, 0.582, 0.008, 13),
    (0.430, 0.536, -0.014, 13),
    (0.369, 0.467, -0.036, 13),
    (0.308, 0.395, -0.057, 13),
    (0.247, 0.323, -0.079, 13),
    (0.205, 0.264, -0.093, 13),
    (0.186, 0.232, -0.100, 13),
    (0.198, 0.233, -0.098, 13),
]

LINE_RE = re.compile(
    r"sample\s+(?P<idx>\d+)\s+in=(?P<inp>-?\d+\.\d+)\s+"
    r"pos=(?P<pos>-?\d+\.\d+)\s+vel=(?P<vel>-?\d+\.\d+)\s+cyc=(?P<cyc>\d+)"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logfile", help="UART log captured from the standalone SNN demo")
    parser.add_argument(
        "--tol",
        type=float,
        default=0.01,
        help="allowed absolute error for input/position/velocity values",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    seen = {}

    with open(args.logfile, "r", encoding="ascii", errors="ignore") as infile:
        for line in infile:
            match = LINE_RE.search(line)
            if not match:
                continue
            idx = int(match.group("idx"))
            if idx >= len(EXPECTED) or idx in seen:
                continue
            seen[idx] = (
                float(match.group("inp")),
                float(match.group("pos")),
                float(match.group("vel")),
                int(match.group("cyc")),
            )

    missing = [idx for idx in range(len(EXPECTED)) if idx not in seen]
    if missing:
        print(f"FAIL: missing sample lines: {missing}")
        return 1

    for idx, expected in enumerate(EXPECTED):
        observed = seen[idx]
        labels = ("in", "pos", "vel")
        for field_idx, label in enumerate(labels):
            if abs(observed[field_idx] - expected[field_idx]) > args.tol:
                print(
                    f"FAIL: sample {idx} {label} expected {expected[field_idx]:.3f} "
                    f"observed {observed[field_idx]:.3f}"
                )
                return 1
        if observed[3] != expected[3]:
            print(
                f"FAIL: sample {idx} cyc expected {expected[3]} observed {observed[3]}"
            )
            return 1

    print(f"PASS: validated {len(EXPECTED)} samples within +/-{args.tol:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
