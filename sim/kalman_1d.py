#!/usr/bin/env python3
"""Small constant-velocity Kalman filter baseline."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Iterable, List

try:
    from .generate_tracking_data import TrackingSequence
except ImportError:
    from generate_tracking_data import TrackingSequence


@dataclass
class KalmanResult:
    position: List[float]
    velocity: List[float]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def quantize(value: float, frac_bits: int = 12, clip: float = 7.999) -> float:
    value = clamp(value, -clip, clip)
    scale = 1 << frac_bits
    return round(value * scale) / scale


def run_constant_velocity_kalman(
    measurements: Iterable[float],
    dt: float,
    process_accel_sigma: float = 0.35,
    measurement_sigma: float = 0.08,
    quantized: bool = False,
    frac_bits: int = 12,
) -> KalmanResult:
    """Run a 2-state constant-velocity Kalman filter."""
    z_list = list(measurements)
    if not z_list:
        return KalmanResult(position=[], velocity=[])

    x_pos = z_list[0]
    x_vel = 0.0

    p00 = 1.0
    p01 = 0.0
    p10 = 0.0
    p11 = 1.0

    q = process_accel_sigma * process_accel_sigma
    q00 = 0.25 * (dt ** 4) * q
    q01 = 0.5 * (dt ** 3) * q
    q10 = q01
    q11 = (dt ** 2) * q
    r = measurement_sigma * measurement_sigma

    if quantized:
        dt = quantize(dt, frac_bits=frac_bits)
        q00 = quantize(q00, frac_bits=frac_bits)
        q01 = quantize(q01, frac_bits=frac_bits)
        q10 = quantize(q10, frac_bits=frac_bits)
        q11 = quantize(q11, frac_bits=frac_bits)
        r = quantize(r, frac_bits=frac_bits)

    out_pos: List[float] = []
    out_vel: List[float] = []

    for z in z_list:
        if quantized:
            z = quantize(z, frac_bits=frac_bits)

        x_pos_pred = x_pos + dt * x_vel
        x_vel_pred = x_vel

        p00_pred = p00 + dt * (p10 + p01) + (dt ** 2) * p11 + q00
        p01_pred = p01 + dt * p11 + q01
        p10_pred = p10 + dt * p11 + q10
        p11_pred = p11 + q11

        if quantized:
            x_pos_pred = quantize(x_pos_pred, frac_bits=frac_bits)
            x_vel_pred = quantize(x_vel_pred, frac_bits=frac_bits)
            p00_pred = quantize(p00_pred, frac_bits=frac_bits)
            p01_pred = quantize(p01_pred, frac_bits=frac_bits)
            p10_pred = quantize(p10_pred, frac_bits=frac_bits)
            p11_pred = quantize(p11_pred, frac_bits=frac_bits)

        innovation = z - x_pos_pred
        s = p00_pred + r
        k0 = p00_pred / s
        k1 = p10_pred / s

        if quantized:
            innovation = quantize(innovation, frac_bits=frac_bits)
            s = quantize(s, frac_bits=frac_bits)
            k0 = quantize(k0, frac_bits=frac_bits)
            k1 = quantize(k1, frac_bits=frac_bits)

        x_pos = x_pos_pred + k0 * innovation
        x_vel = x_vel_pred + k1 * innovation

        if quantized:
            x_pos = quantize(x_pos, frac_bits=frac_bits)
            x_vel = quantize(x_vel, frac_bits=frac_bits)

        p00 = (1.0 - k0) * p00_pred
        p01 = (1.0 - k0) * p01_pred
        p10 = p10_pred - k1 * p00_pred
        p11 = p11_pred - k1 * p01_pred

        if quantized:
            p00 = quantize(p00, frac_bits=frac_bits)
            p01 = quantize(p01, frac_bits=frac_bits)
            p10 = quantize(p10, frac_bits=frac_bits)
            p11 = quantize(p11, frac_bits=frac_bits)

        out_pos.append(x_pos)
        out_vel.append(x_vel)

    return KalmanResult(position=out_pos, velocity=out_vel)


def rmse(reference: Iterable[float], estimate: Iterable[float]) -> float:
    ref_list = list(reference)
    est_list = list(estimate)
    if len(ref_list) != len(est_list):
        raise ValueError("reference and estimate lengths must match")
    if not ref_list:
        return 0.0
    err_sum = 0.0
    for ref, est in zip(ref_list, est_list):
        diff = ref - est
        err_sum += diff * diff
    return sqrt(err_sum / len(ref_list))


def evaluate_kalman(sequence: TrackingSequence) -> dict[str, float]:
    result = run_constant_velocity_kalman(
        sequence.measurement,
        dt=sequence.dt,
    )
    return {
        "position_rmse": rmse(sequence.true_position, result.position),
        "velocity_rmse": rmse(sequence.true_velocity, result.velocity),
    }


def evaluate_quantized_kalman(
    sequence: TrackingSequence,
    frac_bits: int = 12,
) -> dict[str, float]:
    result = run_constant_velocity_kalman(
        sequence.measurement,
        dt=sequence.dt,
        quantized=True,
        frac_bits=frac_bits,
    )
    return {
        "position_rmse": rmse(sequence.true_position, result.position),
        "velocity_rmse": rmse(sequence.true_velocity, result.velocity),
    }
