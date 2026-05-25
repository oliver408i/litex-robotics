#!/usr/bin/env python3
"""Tiny LIF reservoir estimator for 1D tracking."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
import random
from typing import Iterable, List, Sequence

try:
    from .generate_tracking_data import TrackingSequence
except ImportError:
    from generate_tracking_data import TrackingSequence


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


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


def solve_linear_system(matrix: List[List[float]], vector: List[float]) -> List[float]:
    """Solve A x = b with Gaussian elimination."""
    n = len(vector)
    aug = [row[:] + [vector[idx]] for idx, row in enumerate(matrix)]

    for pivot_idx in range(n):
        pivot_row = max(range(pivot_idx, n), key=lambda row: abs(aug[row][pivot_idx]))
        if abs(aug[pivot_row][pivot_idx]) < 1e-12:
            raise ValueError("singular system in linear solve")
        if pivot_row != pivot_idx:
            aug[pivot_idx], aug[pivot_row] = aug[pivot_row], aug[pivot_idx]

        pivot = aug[pivot_idx][pivot_idx]
        scale = 1.0 / pivot
        for col in range(pivot_idx, n + 1):
            aug[pivot_idx][col] *= scale

        for row in range(n):
            if row == pivot_idx:
                continue
            factor = aug[row][pivot_idx]
            if factor == 0.0:
                continue
            for col in range(pivot_idx, n + 1):
                aug[row][col] -= factor * aug[pivot_idx][col]

    return [aug[idx][n] for idx in range(n)]


def fit_ridge_regression(
    features: Sequence[Sequence[float]],
    targets: Sequence[Sequence[float]],
    ridge_lambda: float = 1e-3,
) -> List[List[float]]:
    """Fit weights for y = W x using normal equations."""
    if not features or not targets:
        raise ValueError("features and targets must be non-empty")
    feature_dim = len(features[0])
    output_dim = len(targets[0])

    gram = [[0.0 for _ in range(feature_dim)] for _ in range(feature_dim)]
    for i in range(feature_dim):
        gram[i][i] = ridge_lambda

    rhs = [[0.0 for _ in range(feature_dim)] for _ in range(output_dim)]
    for feat, target in zip(features, targets):
        for row in range(feature_dim):
            for col in range(feature_dim):
                gram[row][col] += feat[row] * feat[col]
        for out_idx in range(output_dim):
            for feat_idx in range(feature_dim):
                rhs[out_idx][feat_idx] += feat[feat_idx] * target[out_idx]

    weights: List[List[float]] = []
    for out_idx in range(output_dim):
        weights.append(solve_linear_system(gram, rhs[out_idx]))
    return weights


def quantize(value: float, frac_bits: int = 12, clip: float = 7.999) -> float:
    value = clamp(value, -clip, clip)
    scale = 1 << frac_bits
    return round(value * scale) / scale


@dataclass
class ReservoirConfig:
    input_size: int = 4
    neuron_count: int = 8
    beta: float = 0.875
    threshold: float = 1.0
    membrane_clip: float = 3.999
    weight_clip: float = 3.999
    feature_clip: float = 1.999
    readout_acc_clip: float = 7.999
    input_scale: float = 0.6
    recurrent_scale: float = 0.35
    recurrent_density: float = 0.25
    frac_bits: int = 12
    seed: int = 7
    measurement_scale: float = 2.5
    delta_scale: float = 5.0


@dataclass
class ReservoirState:
    membrane: List[float]
    spikes: List[int]
    prev_measurement: float = 0.0


@dataclass
class ReservoirModel:
    config: ReservoirConfig
    input_weights: List[List[float]]
    recurrent_weights: List[List[float]]
    readout_weights: List[List[float]] | None = None


@dataclass
class HardwareStepTrace:
    timestep: int
    input_spikes: List[int]
    neuron_cycles: List[dict[str, float | int]]
    readout_cycles: List[dict[str, float | int]]
    membrane: List[float]
    spikes: List[int]
    position_estimate: float
    velocity_estimate: float


@dataclass
class HardwareExecutionResult:
    position: List[float]
    velocity: List[float]
    cycles_per_sample: int
    total_cycles: int
    trace: List[HardwareStepTrace] | None = None


def create_random_reservoir(config: ReservoirConfig) -> ReservoirModel:
    rng = random.Random(config.seed)
    input_weights: List[List[float]] = []
    recurrent_weights: List[List[float]] = []

    for _ in range(config.neuron_count):
        in_row = []
        for _ in range(config.input_size):
            value = rng.uniform(-config.input_scale, config.input_scale)
            in_row.append(
                quantize(value, frac_bits=config.frac_bits, clip=config.weight_clip)
            )
        input_weights.append(in_row)

    for row_idx in range(config.neuron_count):
        row: List[float] = []
        for col_idx in range(config.neuron_count):
            if row_idx == col_idx or rng.random() > config.recurrent_density:
                row.append(0.0)
                continue
            value = rng.uniform(-config.recurrent_scale, config.recurrent_scale)
            row.append(
                quantize(value, frac_bits=config.frac_bits, clip=config.weight_clip)
            )
        recurrent_weights.append(row)

    return ReservoirModel(
        config=config,
        input_weights=input_weights,
        recurrent_weights=recurrent_weights,
    )


def initial_state(config: ReservoirConfig) -> ReservoirState:
    return ReservoirState(
        membrane=[0.0] * config.neuron_count,
        spikes=[0] * config.neuron_count,
        prev_measurement=0.0,
    )


def encode_measurement(config: ReservoirConfig, measurement: float, prev_measurement: float) -> List[int]:
    z = clamp(measurement / config.measurement_scale, -1.5, 1.5)
    dz = clamp((measurement - prev_measurement) / config.delta_scale, -1.5, 1.5)
    return [
        1 if z > 0.20 else 0,
        1 if z < -0.20 else 0,
        1 if dz > 0.04 else 0,
        1 if dz < -0.04 else 0,
    ]


def step_reservoir(
    model: ReservoirModel,
    state: ReservoirState,
    input_spikes: Sequence[int],
) -> ReservoirState:
    cfg = model.config
    next_membrane: List[float] = []
    next_spikes: List[int] = []

    for neuron_idx in range(cfg.neuron_count):
        current = quantize(
            cfg.beta * state.membrane[neuron_idx],
            frac_bits=cfg.frac_bits,
            clip=cfg.membrane_clip,
        )

        input_sum = 0.0
        recurrent_sum = 0.0

        for input_idx, spike in enumerate(input_spikes):
            if spike:
                input_sum = quantize(
                    input_sum + model.input_weights[neuron_idx][input_idx],
                    frac_bits=cfg.frac_bits,
                    clip=cfg.readout_acc_clip,
                )

        for src_idx, spike in enumerate(state.spikes):
            if spike:
                recurrent_sum = quantize(
                    recurrent_sum + model.recurrent_weights[neuron_idx][src_idx],
                    frac_bits=cfg.frac_bits,
                    clip=cfg.readout_acc_clip,
                )

        current = quantize(
            current + input_sum,
            frac_bits=cfg.frac_bits,
            clip=cfg.membrane_clip,
        )
        current = quantize(
            current + recurrent_sum,
            frac_bits=cfg.frac_bits,
            clip=cfg.membrane_clip,
        )

        if current >= cfg.threshold:
            next_spikes.append(1)
            current = quantize(
                current - cfg.threshold,
                frac_bits=cfg.frac_bits,
                clip=cfg.membrane_clip,
            )
        else:
            next_spikes.append(0)

        next_membrane.append(current)

    return ReservoirState(
        membrane=next_membrane,
        spikes=next_spikes,
        prev_measurement=state.prev_measurement,
    )


def collect_features(model: ReservoirModel, sequence: TrackingSequence) -> List[List[float]]:
    state = initial_state(model.config)
    features: List[List[float]] = []

    for measurement in sequence.measurement:
        delta = measurement - state.prev_measurement
        encoded = encode_measurement(model.config, measurement, state.prev_measurement)
        state = step_reservoir(model, state, encoded)
        state.prev_measurement = measurement
        features.append(feature_vector(model, state, measurement, delta))

    return features


def feature_vector(
    model: ReservoirModel,
    state: ReservoirState,
    measurement: float,
    delta: float,
) -> List[float]:
    cfg = model.config
    return state.membrane[:] + [
        quantize(
            clamp(measurement / cfg.measurement_scale, -cfg.feature_clip, cfg.feature_clip),
            frac_bits=cfg.frac_bits,
            clip=cfg.feature_clip,
        ),
        quantize(
            clamp(delta / cfg.delta_scale, -cfg.feature_clip, cfg.feature_clip),
            frac_bits=cfg.frac_bits,
            clip=cfg.feature_clip,
        ),
        1.0,
    ]


def _readout_feature_count(model: ReservoirModel) -> int:
    return model.config.neuron_count + 3


def run_estimator_hardware_schedule(
    model: ReservoirModel,
    sequence: TrackingSequence,
    capture_trace: bool = False,
) -> HardwareExecutionResult:
    if model.readout_weights is None:
        raise ValueError("model has no trained readout_weights")

    cfg = model.config
    state = initial_state(cfg)
    position: List[float] = []
    velocity: List[float] = []
    hw_trace: List[HardwareStepTrace] | None = [] if capture_trace else None
    cycles_per_sample = cfg.neuron_count + 2 * _readout_feature_count(model)

    for timestep, measurement in enumerate(sequence.measurement):
        delta = measurement - state.prev_measurement
        input_spikes = encode_measurement(cfg, measurement, state.prev_measurement)
        prev_spikes = state.spikes[:]
        next_membrane: List[float] = [0.0] * cfg.neuron_count
        next_spikes: List[int] = [0] * cfg.neuron_count
        neuron_cycles: List[dict[str, float | int]] = []

        for neuron_idx in range(cfg.neuron_count):
            leak_term = quantize(
                cfg.beta * state.membrane[neuron_idx],
                frac_bits=cfg.frac_bits,
                clip=cfg.membrane_clip,
            )
            current = leak_term

            input_sum = 0.0
            for input_idx, spike in enumerate(input_spikes):
                if spike:
                    input_sum = quantize(
                        input_sum + model.input_weights[neuron_idx][input_idx],
                        frac_bits=cfg.frac_bits,
                        clip=cfg.readout_acc_clip,
                    )

            recurrent_sum = 0.0
            for src_idx, spike in enumerate(prev_spikes):
                if spike:
                    recurrent_sum = quantize(
                        recurrent_sum + model.recurrent_weights[neuron_idx][src_idx],
                        frac_bits=cfg.frac_bits,
                        clip=cfg.readout_acc_clip,
                    )

            current = quantize(
                current + input_sum,
                frac_bits=cfg.frac_bits,
                clip=cfg.membrane_clip,
            )
            current = quantize(
                current + recurrent_sum,
                frac_bits=cfg.frac_bits,
                clip=cfg.membrane_clip,
            )

            spike_out = 1 if current >= cfg.threshold else 0
            if spike_out:
                current = quantize(
                    current - cfg.threshold,
                    frac_bits=cfg.frac_bits,
                    clip=cfg.membrane_clip,
                )

            next_membrane[neuron_idx] = current
            next_spikes[neuron_idx] = spike_out

            if capture_trace:
                neuron_cycles.append(
                    {
                        "neuron_idx": neuron_idx,
                        "leak_term": leak_term,
                        "input_sum": input_sum,
                        "recurrent_sum": recurrent_sum,
                        "membrane_out": current,
                        "spike_out": spike_out,
                    }
                )

        state = ReservoirState(
            membrane=next_membrane,
            spikes=next_spikes,
            prev_measurement=measurement,
        )
        feat = feature_vector(model, state, measurement, delta)

        outputs: List[float] = []
        readout_cycles: List[dict[str, float | int]] = []
        for out_idx in range(2):
            acc = 0.0
            for feat_idx, value in enumerate(feat):
                product = quantize(
                    model.readout_weights[out_idx][feat_idx] * value,
                    frac_bits=cfg.frac_bits,
                    clip=cfg.readout_acc_clip,
                )
                acc = quantize(
                    acc + product,
                    frac_bits=cfg.frac_bits,
                    clip=cfg.readout_acc_clip,
                )
                if capture_trace:
                    readout_cycles.append(
                        {
                            "output_idx": out_idx,
                            "feature_idx": feat_idx,
                            "feature_value": value,
                            "weight": model.readout_weights[out_idx][feat_idx],
                            "product": product,
                            "accumulator": acc,
                        }
                    )
            outputs.append(acc)

        position.append(outputs[0])
        velocity.append(outputs[1])

        if capture_trace and hw_trace is not None:
            hw_trace.append(
                HardwareStepTrace(
                    timestep=timestep,
                    input_spikes=input_spikes[:],
                    neuron_cycles=neuron_cycles,
                    readout_cycles=readout_cycles,
                    membrane=state.membrane[:],
                    spikes=state.spikes[:],
                    position_estimate=outputs[0],
                    velocity_estimate=outputs[1],
                )
            )

    return HardwareExecutionResult(
        position=position,
        velocity=velocity,
        cycles_per_sample=cycles_per_sample,
        total_cycles=cycles_per_sample * len(sequence.measurement),
        trace=hw_trace,
    )


def train_readout(
    model: ReservoirModel,
    sequences: Sequence[TrackingSequence],
    ridge_lambda: float = 1e-3,
) -> ReservoirModel:
    all_features: List[List[float]] = []
    all_targets: List[List[float]] = []

    for sequence in sequences:
        seq_features = collect_features(model, sequence)
        all_features.extend(seq_features)
        all_targets.extend(
            [[pos, vel] for pos, vel in zip(sequence.true_position, sequence.true_velocity)]
        )

    readout = fit_ridge_regression(all_features, all_targets, ridge_lambda=ridge_lambda)
    model.readout_weights = [
        [
            quantize(
                value,
                frac_bits=model.config.frac_bits,
                clip=model.config.weight_clip,
            )
            for value in row
        ]
        for row in readout
    ]
    return model


def run_estimator(model: ReservoirModel, sequence: TrackingSequence) -> dict[str, List[float]]:
    if model.readout_weights is None:
        raise ValueError("model has no trained readout_weights")

    state = initial_state(model.config)
    pos_est: List[float] = []
    vel_est: List[float] = []

    for measurement in sequence.measurement:
        delta = measurement - state.prev_measurement
        encoded = encode_measurement(model.config, measurement, state.prev_measurement)
        state = step_reservoir(model, state, encoded)
        state.prev_measurement = measurement

        feat = feature_vector(model, state, measurement, delta)
        pos_value = 0.0
        vel_value = 0.0
        for idx, value in enumerate(feat):
            pos_product = quantize(
                model.readout_weights[0][idx] * value,
                frac_bits=model.config.frac_bits,
                clip=model.config.readout_acc_clip,
            )
            vel_product = quantize(
                model.readout_weights[1][idx] * value,
                frac_bits=model.config.frac_bits,
                clip=model.config.readout_acc_clip,
            )
            pos_value = quantize(
                pos_value + pos_product,
                frac_bits=model.config.frac_bits,
                clip=model.config.readout_acc_clip,
            )
            vel_value = quantize(
                vel_value + vel_product,
                frac_bits=model.config.frac_bits,
                clip=model.config.readout_acc_clip,
            )
        pos_est.append(pos_value)
        vel_est.append(vel_value)

    return {"position": pos_est, "velocity": vel_est}


def evaluate_model(model: ReservoirModel, sequence: TrackingSequence) -> dict[str, float]:
    result = run_estimator(model, sequence)
    return {
        "position_rmse": rmse(sequence.true_position, result["position"]),
        "velocity_rmse": rmse(sequence.true_velocity, result["velocity"]),
    }


def evaluate_hardware_schedule_model(
    model: ReservoirModel,
    sequence: TrackingSequence,
) -> dict[str, float]:
    result = run_estimator_hardware_schedule(model, sequence)
    return {
        "position_rmse": rmse(sequence.true_position, result.position),
        "velocity_rmse": rmse(sequence.true_velocity, result.velocity),
    }


def compare_execution_paths(
    model: ReservoirModel,
    sequence: TrackingSequence,
) -> dict[str, float]:
    high_level = run_estimator(model, sequence)
    hardware = run_estimator_hardware_schedule(model, sequence)
    return {
        "position_diff_rmse": rmse(high_level["position"], hardware.position),
        "velocity_diff_rmse": rmse(high_level["velocity"], hardware.velocity),
        "cycles_per_sample": float(hardware.cycles_per_sample),
        "total_cycles": float(hardware.total_cycles),
    }


def generate_trace(
    model: ReservoirModel,
    sequence: TrackingSequence,
) -> List[dict[str, float | int | list[float] | list[int]]]:
    if model.readout_weights is None:
        raise ValueError("model has no trained readout_weights")

    state = initial_state(model.config)
    trace: List[dict[str, float | int | list[float] | list[int]]] = []

    for timestep, measurement in enumerate(sequence.measurement):
        delta = measurement - state.prev_measurement
        encoded = encode_measurement(model.config, measurement, state.prev_measurement)
        state = step_reservoir(model, state, encoded)
        state.prev_measurement = measurement
        feat = feature_vector(model, state, measurement, delta)

        pos_value = 0.0
        vel_value = 0.0
        for idx, value in enumerate(feat):
            pos_product = quantize(
                model.readout_weights[0][idx] * value,
                frac_bits=model.config.frac_bits,
                clip=model.config.readout_acc_clip,
            )
            vel_product = quantize(
                model.readout_weights[1][idx] * value,
                frac_bits=model.config.frac_bits,
                clip=model.config.readout_acc_clip,
            )
            pos_value = quantize(
                pos_value + pos_product,
                frac_bits=model.config.frac_bits,
                clip=model.config.readout_acc_clip,
            )
            vel_value = quantize(
                vel_value + vel_product,
                frac_bits=model.config.frac_bits,
                clip=model.config.readout_acc_clip,
            )

        trace.append(
            {
                "t": timestep,
                "measurement": measurement,
                "delta": delta,
                "true_position": sequence.true_position[timestep],
                "true_velocity": sequence.true_velocity[timestep],
                "input_spikes": encoded[:],
                "membrane": state.membrane[:],
                "reservoir_spikes": state.spikes[:],
                "position_estimate": pos_value,
                "velocity_estimate": vel_value,
            }
        )

    return trace
