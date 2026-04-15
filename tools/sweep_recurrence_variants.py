#!/usr/bin/env python3
"""Sweep a few small recurrent SNN variants against the hardware-style model."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sim.generate_tracking_data import generate_dataset
from sim.lif_reservoir_1d import (
    ReservoirConfig,
    create_random_reservoir,
    evaluate_hardware_schedule_model,
    train_readout,
)


def avg_rmse(model, sequences) -> tuple[float, float]:
    pos = 0.0
    vel = 0.0
    count = 0
    for seq in sequences:
        metrics = evaluate_hardware_schedule_model(model, seq)
        pos += metrics["position_rmse"]
        vel += metrics["velocity_rmse"]
        count += 1
    return pos / count, vel / count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-count", type=int, default=24)
    parser.add_argument("--test-count", type=int, default=8)
    parser.add_argument("--length", type=int, default=300)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    train_set = generate_dataset(args.train_count, length=args.length, seed=args.seed)
    test_set = generate_dataset(args.test_count, length=args.length, seed=args.seed + 1000)

    variants = [
        ("baseline", ReservoirConfig()),
        ("denser", ReservoirConfig(recurrent_density=0.5)),
        ("stronger", ReservoirConfig(recurrent_scale=0.5)),
        ("denser_stronger", ReservoirConfig(recurrent_density=0.5, recurrent_scale=0.5)),
        ("slower_leak", ReservoirConfig(beta=0.9375)),
        ("slower_dense", ReservoirConfig(beta=0.9375, recurrent_density=0.5, recurrent_scale=0.5)),
    ]

    rows: list[tuple[str, ReservoirConfig, float, float]] = []
    for name, cfg in variants:
        model = create_random_reservoir(cfg)
        train_readout(model, train_set)
        pos_rmse, vel_rmse = avg_rmse(model, test_set)
        rows.append((name, cfg, pos_rmse, vel_rmse))

    baseline_pos = rows[0][2]
    baseline_vel = rows[0][3]

    print("Recurrent variant sweep")
    print(f"train sequences: {args.train_count}")
    print(f"test sequences:  {args.test_count}")
    print(f"sequence length: {args.length}")
    print("")
    print("name              pos_rmse  vel_rmse  d_pos    d_vel    beta    rec_scale  rec_density")
    for name, cfg, pos_rmse, vel_rmse in rows:
        print(
            f"{name:17} {pos_rmse:8.5f}  {vel_rmse:8.5f}  "
            f"{pos_rmse - baseline_pos:+.5f}  {vel_rmse - baseline_vel:+.5f}  "
            f"{cfg.beta:6.4f}  {cfg.recurrent_scale:9.4f}  {cfg.recurrent_density:11.4f}"
        )

    best = min(rows, key=lambda item: item[2] + item[3])
    print("")
    print(
        "Best combined RMSE:"
        f" {best[0]} (pos={best[2]:.5f}, vel={best[3]:.5f}, "
        f"beta={best[1].beta:.4f}, rec_scale={best[1].recurrent_scale:.4f}, "
        f"rec_density={best[1].recurrent_density:.4f})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
