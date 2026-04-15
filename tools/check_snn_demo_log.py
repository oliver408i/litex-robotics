#!/usr/bin/env python3
"""Check the standalone SNN demo UART log against the validated reference."""

from __future__ import annotations

import argparse
import re
import sys


EXPECTED = [
    (0.483, 0.319, 0.285, 13),
    (0.444, 0.513, 0.218, 13),
    (0.417, 0.480, 0.178, 13),
    (0.364, 0.432, 0.133, 13),
    (0.295, 0.361, 0.087, 13),
    (0.239, 0.292, 0.049, 13),
    (0.186, 0.230, 0.014, 13),
    (0.149, 0.181, -0.012, 13),
    (0.127, 0.151, -0.033, 13),
    (0.149, 0.159, -0.038, 13),
    (0.193, 0.203, -0.035, 13),
    (0.264, 0.277, -0.024, 13),
    (0.334, 0.363, -0.011, 13),
    (0.400, 0.445, 0.001, 13),
    (0.457, 0.517, 0.010, 13),
    (0.498, 0.574, 0.016, 13),
    (0.483, 0.583, 0.005, 13),
    (0.430, 0.537, -0.017, 13),
    (0.369, 0.468, -0.039, 13),
    (0.308, 0.396, -0.061, 13),
    (0.247, 0.323, -0.081, 13),
    (0.205, 0.265, -0.095, 13),
    (0.186, 0.233, -0.103, 13),
    (0.198, 0.233, -0.099, 13),
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
