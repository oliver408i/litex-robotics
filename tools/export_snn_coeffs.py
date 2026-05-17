#!/usr/bin/env python3
"""Generate a loadable SNN model image for the hardware UART protocol."""

from __future__ import annotations

import argparse
import json
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

def linear_model_image(tables: dict[str, object]) -> list[int]:
    """Return the hardware load order: input[8][4], recurrent[8][8], readout[2][11]."""
    image: list[int] = []
    for name in ("input", "recurrent", "readout"):
        table = tables[name]
        for row in table:  # type: ignore[union-attr]
            image.extend(int(value) for value in row)
    return image


def print_table(name: str, table: list[list[int]]) -> None:
    print(f"{name}:")
    for idx, row in enumerate(table):
        print(f"  {idx}: {row}")


def emit_uart(image: list[int]) -> None:
    print("# snn_demo protocol v2 model image")
    print("# order: input[8][4], recurrent[8][8], readout[2][11]")
    print("X")
    for addr, value in enumerate(image):
        print(f"W {addr} {value}")
    print("L")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-count", type=int, default=24)
    parser.add_argument("--length", type=int, default=300)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument(
        "--format",
        choices=("uart", "json", "tables"),
        default="uart",
        help="Output format. uart emits commands accepted by software/snn_demo.",
    )
    parser.add_argument(
        "--print-tables",
        action="store_true",
        help="Alias for --format tables.",
    )
    args = parser.parse_args()

    model = build_model(args.train_count, args.length, args.seed)
    generated = quantized_tables(model)
    image = linear_model_image(generated)
    output_format = "tables" if args.print_tables else args.format

    if output_format == "tables":
        for name in ("input", "recurrent", "readout"):
            print_table(name, generated[name])  # type: ignore[arg-type]
    elif output_format == "json":
        print(json.dumps({
            "format": "snn-model-v1",
            "frac_bits": model.config.frac_bits,
            "order": ["input[8][4]", "recurrent[8][8]", "readout[2][11]"],
            "word_count": len(image),
            "tables": generated,
            "image": image,
        }, indent=2))
    else:
        emit_uart(image)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
