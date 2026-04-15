#!/usr/bin/env python3
"""Compare feedforward and recurrent hardware-style SNN variants."""

from __future__ import annotations

import argparse
from copy import deepcopy
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


def build_models(train_count: int, test_count: int, length: int, seed: int):
    cfg = ReservoirConfig()
    train_set = generate_dataset(train_count, length=length, seed=seed)
    test_set = generate_dataset(test_count, length=length, seed=seed + 1000)

    recurrent = create_random_reservoir(cfg)
    train_readout(recurrent, train_set)

    feedforward = deepcopy(recurrent)
    feedforward.recurrent_weights = [
        [0.0 for _ in row] for row in feedforward.recurrent_weights
    ]
    train_readout(feedforward, train_set)
    return feedforward, recurrent, test_set


def avg_rmse(model, sequences):
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

    feedforward, recurrent, test_set = build_models(
        args.train_count, args.test_count, args.length, args.seed
    )

    ff_pos, ff_vel = avg_rmse(feedforward, test_set)
    rec_pos, rec_vel = avg_rmse(recurrent, test_set)

    print("SNN variant comparison")
    print(f"train sequences: {args.train_count}")
    print(f"test sequences:  {args.test_count}")
    print(f"sequence length: {args.length}")
    print("")
    print("Hardware-style RMSE")
    print(f"feedforward position: {ff_pos:.5f}")
    print(f"feedforward velocity: {ff_vel:.5f}")
    print(f"recurrent position:   {rec_pos:.5f}")
    print(f"recurrent velocity:   {rec_vel:.5f}")
    print("")
    print("Recurrent - feedforward delta")
    print(f"position delta: {rec_pos - ff_pos:+.5f}")
    print(f"velocity delta: {rec_vel - ff_vel:+.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
