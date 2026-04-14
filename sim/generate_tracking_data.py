#!/usr/bin/env python3
"""Synthetic 1D constant-velocity tracking dataset generation."""

from __future__ import annotations

from dataclasses import dataclass
import random
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


def generate_tracking_sequence(
    length: int = 300,
    dt: float = 0.02,
    accel_limit: float = 1.0,
    segment_min: int = 12,
    segment_max: int = 48,
    measurement_sigma: float = 0.08,
    bias_walk_sigma: float = 0.002,
    initial_position_span: float = 1.0,
    initial_velocity_span: float = 0.8,
    seed: int | None = None,
) -> TrackingSequence:
    """Generate a small motion-tracking sequence with noisy measurements."""
    if length <= 0:
        raise ValueError("length must be positive")
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
    **kwargs: float,
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
