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
from sim.lif_reservoir_1d import (
    ReservoirConfig,
    ReservoirModel,
    create_random_reservoir,
    train_readout,
)


def _to_fixed(value: float, frac_bits: int) -> int:
    return int(round(value * (1 << frac_bits)))


def build_model(train_count: int, length: int, seed: int, dataset_kwargs: dict[str, object]):
    cfg = ReservoirConfig(recurrent_scale=0.5)
    model = create_random_reservoir(cfg)
    train_set = generate_dataset(train_count, length=length, seed=seed, **dataset_kwargs)
    train_readout(model, train_set)
    return model


def load_from_pt(pt_path: Path) -> ReservoirModel:
    """Build a ReservoirModel from a snntorch-trained checkpoint."""
    import torch  # lazy: only required for --from-pt
    state = torch.load(pt_path, map_location="cpu", weights_only=False)
    if state.get("format") != "snn-tracker-v1":
        raise ValueError(f"unrecognized checkpoint format: {state.get('format')!r}")
    cfg = ReservoirConfig(
        input_size=int(state["n_in"]),
        neuron_count=int(state["n_neurons"]),
        beta=float(state["beta"]),
        threshold=float(state["threshold"]),
        membrane_clip=float(state["mem_clip"]),
        weight_clip=float(state["weight_clip"]),
        feature_clip=float(state["feature_clip"]),
        readout_acc_clip=float(state["readout_acc_clip"]),
        measurement_scale=float(state["meas_scale"]),
        delta_scale=float(state["delta_scale"]),
        frac_bits=int(state["frac_bits"]),
    )
    return ReservoirModel(
        config=cfg,
        input_weights=state["W_in"].tolist(),
        recurrent_weights=state["W_rec"].tolist(),
        readout_weights=state["W_read"].tolist(),
    )


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
    parser.add_argument("--dt", type=float, default=0.02, help="dataset timestep")
    parser.add_argument(
        "--mode",
        choices=("tracking", "sine", "square", "ramp", "walk"),
        default="tracking",
        help="training dataset mode",
    )
    parser.add_argument("--accel-limit", type=float, default=1.0, help="tracking-mode acceleration limit")
    parser.add_argument("--segment-min", type=int, default=12, help="tracking-mode minimum segment length")
    parser.add_argument("--segment-max", type=int, default=48, help="tracking-mode maximum segment length")
    parser.add_argument("--noise", type=float, default=0.08, help="measurement noise sigma")
    parser.add_argument("--bias-walk-sigma", type=float, default=0.002, help="tracking-mode bias random-walk sigma")
    parser.add_argument("--initial-position-span", type=float, default=1.0, help="tracking-mode initial position span")
    parser.add_argument("--initial-velocity-span", type=float, default=0.8, help="tracking-mode initial velocity span")
    parser.add_argument("--offset", type=float, default=0.0, help="waveform offset")
    parser.add_argument("--amplitude", type=float, default=0.5, help="waveform amplitude, or walk step sigma")
    parser.add_argument("--period", type=float, default=48.0, help="waveform period in samples")
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
    parser.add_argument(
        "--from-pt",
        type=Path,
        default=None,
        help="Load weights from a tools/train_snn_torch.py checkpoint instead of training a random reservoir.",
    )
    args = parser.parse_args()

    if args.from_pt is not None:
        model = load_from_pt(args.from_pt)
    else:
        dataset_kwargs = {
            "dt": args.dt,
            "mode": args.mode,
            "accel_limit": args.accel_limit,
            "segment_min": args.segment_min,
            "segment_max": args.segment_max,
            "measurement_sigma": args.noise,
            "bias_walk_sigma": args.bias_walk_sigma,
            "initial_position_span": args.initial_position_span,
            "initial_velocity_span": args.initial_velocity_span,
            "signal_offset": args.offset,
            "signal_amplitude": args.amplitude,
            "signal_period": args.period,
        }
        model = build_model(args.train_count, args.length, args.seed, dataset_kwargs)
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
