#!/usr/bin/env python3
"""Synthetic 1D tracking and waveform dataset generation."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
import random
import sys
from typing import List


@dataclass
class TrackingSequence:
    dt: float
    true_position: List[float]
    true_velocity: List[float]
    true_acceleration: List[float]
    measurement: List[float]
    measurement_bias: List[float]


def _draw_segment_acceleration(rng: random.Random, accel_limit: float) -> float:
    return rng.uniform(-accel_limit, accel_limit)


def _generate_waveform_sequence(
    mode: str,
    length: int,
    dt: float,
    offset: float,
    amplitude: float,
    period: float,
    measurement_sigma: float,
    seed: int | None,
) -> TrackingSequence:
    if period <= 0.0:
        raise ValueError("period must be positive")

    rng = random.Random(seed)
    value = offset
    prev_position = offset
    prev_velocity = 0.0

    true_position: List[float] = []
    true_velocity: List[float] = []
    true_acceleration: List[float] = []
    measurement: List[float] = []
    measurement_bias: List[float] = []

    for t in range(length):
        if mode == "sine":
            position = offset + amplitude * math.sin((2.0 * math.pi * t) / period)
        elif mode == "square":
            phase = (t % int(max(1, period))) / max(1.0, period)
            position = offset + (amplitude if phase < 0.5 else -amplitude)
        elif mode == "ramp":
            phase = (t % int(max(1, period))) / max(1.0, period)
            position = offset + amplitude * ((2.0 * phase) - 1.0)
        elif mode == "walk":
            value += rng.gauss(0.0, amplitude)
            position = value
        else:
            raise ValueError(f"unsupported sequence mode: {mode}")

        velocity = position - prev_position
        acceleration = velocity - prev_velocity
        z = position + rng.gauss(0.0, measurement_sigma)

        true_position.append(position)
        true_velocity.append(velocity)
        true_acceleration.append(acceleration)
        measurement.append(z)
        measurement_bias.append(0.0)

        prev_position = position
        prev_velocity = velocity

    return TrackingSequence(
        dt=dt,
        true_position=true_position,
        true_velocity=true_velocity,
        true_acceleration=true_acceleration,
        measurement=measurement,
        measurement_bias=measurement_bias,
    )


def generate_tracking_sequence(
    length: int = 300,
    dt: float = 0.02,
    mode: str = "tracking",
    accel_limit: float = 1.0,
    segment_min: int = 12,
    segment_max: int = 48,
    measurement_sigma: float = 0.08,
    bias_walk_sigma: float = 0.002,
    initial_position_span: float = 1.0,
    initial_velocity_span: float = 0.8,
    signal_offset: float = 0.0,
    signal_amplitude: float = 0.5,
    signal_period: float = 48.0,
    seed: int | None = None,
) -> TrackingSequence:
    """Generate a small motion-tracking sequence with noisy measurements.

    The default ``tracking`` mode preserves the original piecewise-acceleration
    dataset. The waveform modes are intended for repeatable hardware streams.
    """
    if length <= 0:
        raise ValueError("length must be positive")
    if mode != "tracking":
        return _generate_waveform_sequence(
            mode=mode,
            length=length,
            dt=dt,
            offset=signal_offset,
            amplitude=signal_amplitude,
            period=signal_period,
            measurement_sigma=measurement_sigma,
            seed=seed,
        )
    if segment_min <= 0 or segment_max < segment_min:
        raise ValueError("invalid segment length bounds")

    rng = random.Random(seed)
    position = rng.uniform(-initial_position_span, initial_position_span)
    velocity = rng.uniform(-initial_velocity_span, initial_velocity_span)
    bias = 0.0
    accel = _draw_segment_acceleration(rng, accel_limit)
    segment_left = rng.randint(segment_min, segment_max)

    true_position: List[float] = []
    true_velocity: List[float] = []
    true_acceleration: List[float] = []
    measurement: List[float] = []
    measurement_bias: List[float] = []

    for _ in range(length):
        if segment_left == 0:
            accel = _draw_segment_acceleration(rng, accel_limit)
            segment_left = rng.randint(segment_min, segment_max)

        velocity += accel * dt
        position += velocity * dt
        bias += rng.gauss(0.0, bias_walk_sigma)
        z = position + bias + rng.gauss(0.0, measurement_sigma)

        true_position.append(position)
        true_velocity.append(velocity)
        true_acceleration.append(accel)
        measurement.append(z)
        measurement_bias.append(bias)

        segment_left -= 1

    return TrackingSequence(
        dt=dt,
        true_position=true_position,
        true_velocity=true_velocity,
        true_acceleration=true_acceleration,
        measurement=measurement,
        measurement_bias=measurement_bias,
    )


def generate_dataset(
    count: int,
    length: int = 300,
    dt: float = 0.02,
    seed: int = 0,
    **kwargs: object,
) -> List[TrackingSequence]:
    """Create a deterministic set of sequences from a root seed."""
    rng = random.Random(seed)
    dataset: List[TrackingSequence] = []
    for _ in range(count):
        dataset.append(
            generate_tracking_sequence(
                length=length,
                dt=dt,
                seed=rng.randrange(1 << 30),
                **kwargs,
            )
        )
    return dataset


def write_dataset_csv(dataset: List[TrackingSequence], output) -> None:
    writer = csv.DictWriter(
        output,
        fieldnames=[
            "sequence",
            "t",
            "dt",
            "true_position",
            "true_velocity",
            "true_acceleration",
            "measurement",
            "measurement_bias",
        ],
    )
    writer.writeheader()
    for sequence_idx, sequence in enumerate(dataset):
        for t, (position, velocity, acceleration, measurement, bias) in enumerate(
            zip(
                sequence.true_position,
                sequence.true_velocity,
                sequence.true_acceleration,
                sequence.measurement,
                sequence.measurement_bias,
            )
        ):
            writer.writerow(
                {
                    "sequence": sequence_idx,
                    "t": t,
                    "dt": sequence.dt,
                    "true_position": position,
                    "true_velocity": velocity,
                    "true_acceleration": acceleration,
                    "measurement": measurement,
                    "measurement_bias": bias,
                }
            )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=1, help="number of sequences")
    parser.add_argument("--length", type=int, default=300, help="timesteps per sequence")
    parser.add_argument("--dt", type=float, default=0.02, help="sequence timestep")
    parser.add_argument("--seed", type=int, default=0, help="root random seed")
    parser.add_argument(
        "--mode",
        choices=("tracking", "sine", "square", "ramp", "walk"),
        default="tracking",
        help="dataset mode",
    )
    parser.add_argument("--output", help="CSV output path; default stdout")
    parser.add_argument("--accel-limit", type=float, default=1.0, help="tracking acceleration limit")
    parser.add_argument("--segment-min", type=int, default=12, help="tracking minimum segment length")
    parser.add_argument("--segment-max", type=int, default=48, help="tracking maximum segment length")
    parser.add_argument("--noise", type=float, default=0.08, help="measurement noise sigma")
    parser.add_argument("--bias-walk-sigma", type=float, default=0.002, help="tracking bias random-walk sigma")
    parser.add_argument("--initial-position-span", type=float, default=1.0, help="tracking initial position span")
    parser.add_argument("--initial-velocity-span", type=float, default=0.8, help="tracking initial velocity span")
    parser.add_argument("--offset", type=float, default=0.0, help="waveform offset")
    parser.add_argument("--amplitude", type=float, default=0.5, help="waveform amplitude, or walk step sigma")
    parser.add_argument("--period", type=float, default=48.0, help="waveform period in samples")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    dataset = generate_dataset(
        args.count,
        length=args.length,
        dt=args.dt,
        seed=args.seed,
        mode=args.mode,
        accel_limit=args.accel_limit,
        segment_min=args.segment_min,
        segment_max=args.segment_max,
        measurement_sigma=args.noise,
        bias_walk_sigma=args.bias_walk_sigma,
        initial_position_span=args.initial_position_span,
        initial_velocity_span=args.initial_velocity_span,
        signal_offset=args.offset,
        signal_amplitude=args.amplitude,
        signal_period=args.period,
    )

    if args.output:
        with open(args.output, "w", newline="", encoding="ascii") as outfile:
            write_dataset_csv(dataset, outfile)
    else:
        write_dataset_csv(dataset, sys.stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
