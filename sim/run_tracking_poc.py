#!/usr/bin/env python3
"""Run the tiny SNN tracking PoC against a Kalman baseline."""

from __future__ import annotations

import argparse
import csv
from statistics import mean
from typing import Sequence

try:
    from .generate_tracking_data import generate_dataset
    from .kalman_1d import evaluate_kalman, evaluate_quantized_kalman, run_constant_velocity_kalman
    from .lif_reservoir_1d import (
        compare_execution_paths,
        ReservoirConfig,
        create_random_reservoir,
        evaluate_hardware_schedule_model,
        evaluate_model,
        generate_trace,
        run_estimator_hardware_schedule,
        train_readout,
    )
except ImportError:
    from generate_tracking_data import generate_dataset
    from kalman_1d import evaluate_kalman, evaluate_quantized_kalman, run_constant_velocity_kalman
    from lif_reservoir_1d import (
        compare_execution_paths,
        ReservoirConfig,
        create_random_reservoir,
        evaluate_hardware_schedule_model,
        evaluate_model,
        generate_trace,
        run_estimator_hardware_schedule,
        train_readout,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-count", type=int, default=24, help="number of training sequences")
    parser.add_argument("--test-count", type=int, default=8, help="number of test sequences")
    parser.add_argument("--length", type=int, default=300, help="timesteps per sequence")
    parser.add_argument("--seed", type=int, default=1, help="dataset seed")
    parser.add_argument("--trace-out", type=str, default="", help="optional CSV file for one trace")
    parser.add_argument("--trace-index", type=int, default=0, help="test sequence index for trace")
    parser.add_argument(
        "--hw-trace-out",
        type=str,
        default="",
        help="optional detailed hardware-cycle CSV file for one trace",
    )
    return parser


def summarize(results: list[dict[str, float]]) -> dict[str, float]:
    return {
        "position_rmse": mean(item["position_rmse"] for item in results),
        "velocity_rmse": mean(item["velocity_rmse"] for item in results),
    }


def summarize_path_match(results: list[dict[str, float]]) -> dict[str, float]:
    return {
        "position_diff_rmse": mean(item["position_diff_rmse"] for item in results),
        "velocity_diff_rmse": mean(item["velocity_diff_rmse"] for item in results),
        "cycles_per_sample": mean(item["cycles_per_sample"] for item in results),
        "total_cycles": mean(item["total_cycles"] for item in results),
    }


def write_trace_csv(
    path: str,
    trace_rows: Sequence[dict[str, float | int | list[float] | list[int]]],
    kalman_position: Sequence[float],
    kalman_velocity: Sequence[float],
    qkalman_position: Sequence[float],
    qkalman_velocity: Sequence[float],
    hwsnn_position: Sequence[float] | None = None,
    hwsnn_velocity: Sequence[float] | None = None,
) -> None:
    if not trace_rows:
        return

    membrane_len = len(trace_rows[0]["membrane"])  # type: ignore[index]
    spike_len = len(trace_rows[0]["reservoir_spikes"])  # type: ignore[index]
    input_len = len(trace_rows[0]["input_spikes"])  # type: ignore[index]

    fieldnames = [
        "t",
        "measurement",
        "delta",
        "true_position",
        "true_velocity",
        "snn_position",
        "snn_velocity",
        "kalman_position",
        "kalman_velocity",
        "qkalman_position",
        "qkalman_velocity",
        "hwsnn_position",
        "hwsnn_velocity",
    ]
    fieldnames.extend(f"input_spike_{idx}" for idx in range(input_len))
    fieldnames.extend(f"reservoir_spike_{idx}" for idx in range(spike_len))
    fieldnames.extend(f"membrane_{idx}" for idx in range(membrane_len))

    with open(path, "w", newline="", encoding="ascii") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for idx, row in enumerate(trace_rows):
            out_row = {
                "t": row["t"],
                "measurement": row["measurement"],
                "delta": row["delta"],
                "true_position": row["true_position"],
                "true_velocity": row["true_velocity"],
                "snn_position": row["position_estimate"],
                "snn_velocity": row["velocity_estimate"],
                "kalman_position": kalman_position[idx],
                "kalman_velocity": kalman_velocity[idx],
                "qkalman_position": qkalman_position[idx],
                "qkalman_velocity": qkalman_velocity[idx],
                "hwsnn_position": "" if hwsnn_position is None else hwsnn_position[idx],
                "hwsnn_velocity": "" if hwsnn_velocity is None else hwsnn_velocity[idx],
            }
            for input_idx, value in enumerate(row["input_spikes"]):  # type: ignore[index]
                out_row[f"input_spike_{input_idx}"] = value
            for spike_idx, value in enumerate(row["reservoir_spikes"]):  # type: ignore[index]
                out_row[f"reservoir_spike_{spike_idx}"] = value
            for mem_idx, value in enumerate(row["membrane"]):  # type: ignore[index]
                out_row[f"membrane_{mem_idx}"] = value
            writer.writerow(out_row)


def write_hardware_trace_csv(path: str, hardware_trace: Sequence[object]) -> None:
    fieldnames = [
        "timestep",
        "phase",
        "slot_idx",
        "output_idx",
        "feature_idx",
        "input_spikes",
        "leak_term",
        "input_sum",
        "recurrent_sum",
        "membrane_out",
        "spike_out",
        "feature_value",
        "weight",
        "product",
        "accumulator",
        "position_estimate",
        "velocity_estimate",
    ]

    with open(path, "w", newline="", encoding="ascii") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for step in hardware_trace:
            input_spikes = ",".join(str(bit) for bit in step.input_spikes)
            for cycle in step.neuron_cycles:
                writer.writerow(
                    {
                        "timestep": step.timestep,
                        "phase": "neuron",
                        "slot_idx": cycle["neuron_idx"],
                        "output_idx": "",
                        "feature_idx": "",
                        "input_spikes": input_spikes,
                        "leak_term": cycle["leak_term"],
                        "input_sum": cycle["input_sum"],
                        "recurrent_sum": cycle["recurrent_sum"],
                        "membrane_out": cycle["membrane_out"],
                        "spike_out": cycle["spike_out"],
                        "feature_value": "",
                        "weight": "",
                        "product": "",
                        "accumulator": "",
                        "position_estimate": "",
                        "velocity_estimate": "",
                    }
                )
            for cycle in step.readout_cycles:
                writer.writerow(
                    {
                        "timestep": step.timestep,
                        "phase": "readout",
                        "slot_idx": "",
                        "output_idx": cycle["output_idx"],
                        "feature_idx": cycle["feature_idx"],
                        "input_spikes": input_spikes,
                        "leak_term": "",
                        "input_sum": "",
                        "recurrent_sum": "",
                        "membrane_out": "",
                        "spike_out": "",
                        "feature_value": cycle["feature_value"],
                        "weight": cycle["weight"],
                        "product": cycle["product"],
                        "accumulator": cycle["accumulator"],
                        "position_estimate": step.position_estimate,
                        "velocity_estimate": step.velocity_estimate,
                    }
                )


def main() -> None:
    args = build_arg_parser().parse_args()

    train_set = generate_dataset(args.train_count, length=args.length, seed=args.seed)
    test_set = generate_dataset(args.test_count, length=args.length, seed=args.seed + 1000)

    reservoir = create_random_reservoir(ReservoirConfig())
    train_readout(reservoir, train_set)

    kalman_scores = summarize([evaluate_kalman(seq) for seq in test_set])
    qkalman_scores = summarize([evaluate_quantized_kalman(seq) for seq in test_set])
    snn_scores = summarize([evaluate_model(reservoir, seq) for seq in test_set])
    hwsnn_scores = summarize([evaluate_hardware_schedule_model(reservoir, seq) for seq in test_set])
    path_match = summarize_path_match([compare_execution_paths(reservoir, seq) for seq in test_set])

    print("Tracking PoC results")
    print(f"train sequences: {args.train_count}")
    print(f"test sequences:  {args.test_count}")
    print(f"sequence length: {args.length}")
    print(f"frac bits:       {reservoir.config.frac_bits}")
    print(f"mem clip:        +/-{reservoir.config.membrane_clip}")
    print(f"weight clip:     +/-{reservoir.config.weight_clip}")
    print(f"feature clip:    +/-{reservoir.config.feature_clip}")
    print(f"acc clip:        +/-{reservoir.config.readout_acc_clip}")
    print("")
    print("Average RMSE over test set")
    print(f"Kalman position:  {kalman_scores['position_rmse']:.5f}")
    print(f"Kalman velocity:  {kalman_scores['velocity_rmse']:.5f}")
    print(f"QKalman position: {qkalman_scores['position_rmse']:.5f}")
    print(f"QKalman velocity: {qkalman_scores['velocity_rmse']:.5f}")
    print(f"SNN position:     {snn_scores['position_rmse']:.5f}")
    print(f"SNN velocity:     {snn_scores['velocity_rmse']:.5f}")
    print(f"HW-SNN position:  {hwsnn_scores['position_rmse']:.5f}")
    print(f"HW-SNN velocity:  {hwsnn_scores['velocity_rmse']:.5f}")
    print("")
    print("Execution-path match")
    print(f"SNN/HW pos diff:  {path_match['position_diff_rmse']:.7f}")
    print(f"SNN/HW vel diff:  {path_match['velocity_diff_rmse']:.7f}")
    print(f"cycles/sample:    {path_match['cycles_per_sample']:.0f}")
    print(f"total cycles:     {path_match['total_cycles']:.0f}")

    if args.trace_out:
        if args.trace_index < 0 or args.trace_index >= len(test_set):
            raise IndexError("--trace-index is out of range for the generated test set")
        trace_sequence = test_set[args.trace_index]
        trace_rows = generate_trace(reservoir, trace_sequence)
        kalman = run_constant_velocity_kalman(trace_sequence.measurement, trace_sequence.dt)
        qkalman = run_constant_velocity_kalman(
            trace_sequence.measurement,
            trace_sequence.dt,
            quantized=True,
            frac_bits=reservoir.config.frac_bits,
        )
        hardware_estimates = run_estimator_hardware_schedule(
            reservoir,
            trace_sequence,
        )
        write_trace_csv(
            args.trace_out,
            trace_rows,
            kalman.position,
            kalman.velocity,
            qkalman.position,
            qkalman.velocity,
            hardware_estimates.position,
            hardware_estimates.velocity,
        )
        print(f"trace written:    {args.trace_out}")

    if args.hw_trace_out:
        if args.trace_index < 0 or args.trace_index >= len(test_set):
            raise IndexError("--trace-index is out of range for the generated test set")
        trace_sequence = test_set[args.trace_index]
        hardware_trace = run_estimator_hardware_schedule(
            reservoir,
            trace_sequence,
            capture_trace=True,
        )
        if hardware_trace.trace is None:
            raise ValueError("hardware trace capture failed")
        write_hardware_trace_csv(args.hw_trace_out, hardware_trace.trace)
        print(f"hw trace written: {args.hw_trace_out}")


if __name__ == "__main__":
    main()
