#!/usr/bin/env python3
"""Regenerate and verify the current SNN hardware coefficients."""

from __future__ import annotations

import argparse
import re
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sim.generate_tracking_data import generate_dataset
from sim.lif_reservoir_1d import ReservoirConfig, create_random_reservoir, train_readout


def _to_fixed(value: float, frac_bits: int) -> int:
    return int(round(value * (1 << frac_bits)))


def build_model(train_count: int, length: int, seed: int):
    cfg = ReservoirConfig(recurrent_scale=0.5)
    model = create_random_reservoir(cfg)
    train_set = generate_dataset(train_count, length=length, seed=seed)
    train_readout(model, train_set)
    return model


def quantized_tables(model) -> dict[str, object]:
    cfg = model.config
    return {
        "input": [[_to_fixed(v, cfg.frac_bits) for v in row] for row in model.input_weights],
        "recurrent": [[_to_fixed(v, cfg.frac_bits) for v in row] for row in model.recurrent_weights],
        "readout": [[_to_fixed(v, cfg.frac_bits) for v in row] for row in model.readout_weights],
    }


def _parse_nested_case_function(verilog_path: Path, function_name: str, row_count: int, col_count: int) -> list[list[int]]:
    lines = verilog_path.read_text(encoding="ascii").splitlines()
    in_function = False
    current_row = None
    table = [[0 for _ in range(col_count)] for _ in range(row_count)]
    row_re = re.compile(r"^\s*(\d+|default):\s*case\s*\(j\)")
    pair_re = re.compile(r"(\d+):\s*" + re.escape(function_name) + r"\s*=\s*(-?)16'sd(\d+)")

    for line in lines:
        if line.strip().startswith(f"function signed [15:0] {function_name};"):
            in_function = True
            continue
        if in_function and line.strip() == "endfunction":
            break
        if not in_function:
            continue

        row_match = row_re.match(line)
        if row_match:
            token = row_match.group(1)
            current_row = row_count - 1 if token == "default" else int(token)

        if current_row is None:
            continue

        for col, sign, digits in pair_re.findall(line):
            value = int(digits)
            if sign == "-":
                value = -value
            table[current_row][int(col)] = value

    return table


def _parse_readout_function(verilog_path: Path) -> list[list[int]]:
    lines = verilog_path.read_text(encoding="ascii").splitlines()
    in_function = False
    current_output = None
    table = [[0 for _ in range(11)] for _ in range(2)]
    if_re = re.compile(r"if\s*\(out_idx == 0\)")
    val_re = re.compile(r"^\s*(\d+|default):\s*readout_weight\s*=\s*(-?)16'sd(\d+);")

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("function signed [15:0] readout_weight;"):
            in_function = True
            continue
        if in_function and stripped == "endfunction":
            break
        if not in_function:
            continue

        if if_re.search(line):
            current_output = 0
            continue
        if stripped == "end else begin":
            current_output = 1
            continue

        match = val_re.match(line)
        if match and current_output is not None:
            token, sign, digits = match.groups()
            idx = 10 if token == "default" else int(token)
            value = int(digits)
            if sign == "-":
                value = -value
            table[current_output][idx] = value

    return table


def load_verilog_tables(verilog_path: Path) -> dict[str, object]:
    return {
        "input": _parse_nested_case_function(verilog_path, "weight", 8, 4),
        "recurrent": _parse_nested_case_function(verilog_path, "recurrent_weight", 8, 8),
        "readout": _parse_readout_function(verilog_path),
    }


def print_table(name: str, table: list[list[int]]) -> None:
    print(f"{name}:")
    for idx, row in enumerate(table):
        print(f"  {idx}: {row}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-count", type=int, default=24)
    parser.add_argument("--length", type=int, default=300)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument(
        "--verilog",
        default="verilog/lif_bank_debug.v",
        help="Verilog core to compare against",
    )
    parser.add_argument(
        "--print-tables",
        action="store_true",
        help="Print the regenerated quantized tables",
    )
    args = parser.parse_args()

    model = build_model(args.train_count, args.length, args.seed)
    generated = quantized_tables(model)
    verilog = load_verilog_tables(Path(args.verilog))

    mismatch = False
    for name in ("input", "recurrent", "readout"):
        if generated[name] != verilog[name]:
            mismatch = True
            print(f"MISMATCH in {name} coefficients")
            print_table("generated", generated[name])  # type: ignore[arg-type]
            print_table("verilog", verilog[name])  # type: ignore[arg-type]

    if args.print_tables and not mismatch:
        for name in ("input", "recurrent", "readout"):
            print_table(name, generated[name])  # type: ignore[arg-type]

    if mismatch:
        return 1

    print("PASS: Verilog coefficients match the regenerated recurrent Python model")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
