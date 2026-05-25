#!/usr/bin/env python3
"""Small helper to convert SNN PoC fixed-point values for hardware poking."""

from __future__ import annotations

import argparse


FRAC_BITS = 12
WIDTH = 16


def to_fixed(value: float) -> int:
    scale = 1 << FRAC_BITS
    raw = int(round(value * scale))
    if raw < 0:
        raw = (1 << WIDTH) + raw
    return raw & ((1 << WIDTH) - 1)


def from_fixed(value: int) -> float:
    if value & (1 << (WIDTH - 1)):
        value -= 1 << WIDTH
    return value / float(1 << FRAC_BITS)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("value", help="float to encode or integer to decode")
    parser.add_argument("--decode", action="store_true", help="decode an integer instead of encoding a float")
    args = parser.parse_args()

    if args.decode:
        raw = int(args.value, 0)
        print(from_fixed(raw))
    else:
        value = float(args.value)
        raw = to_fixed(value)
        print(f"{raw} (0x{raw:04x})")


if __name__ == "__main__":
    main()
