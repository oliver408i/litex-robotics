#!/usr/bin/env python3
"""Read, log, plot, and optionally drive the SNN demo UART protocol."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
import random
import sys
import time
from typing import Iterable, TextIO


FRAC_BITS = 12
FIELDNAMES = [
    "type",
    "t",
    "measurement",
    "delta",
    "input_spikes",
    "position",
    "velocity",
    "cycles",
    "raw",
    "draw",
    "feat",
    "dfeat",
    "m0",
    "m1",
    "m2",
    "m3",
    "m4",
    "beta",
    "isum",
    "rsum",
    "mclip",
    "status",
]
Q_FIELDS = {
    "measurement",
    "delta",
    "position",
    "velocity",
    "raw",
    "draw",
    "feat",
    "dfeat",
    "m0",
    "m1",
    "m2",
    "m3",
    "m4",
    "beta",
    "isum",
    "rsum",
    "mclip",
}


@dataclass
class SNNRow:
    values: dict[str, int | str]

    def raw(self, name: str) -> int:
        value = self.values[name]
        if isinstance(value, str):
            raise TypeError(f"{name} is not numeric")
        return value

    def q(self, name: str) -> float:
        return self.raw(name) / float(1 << FRAC_BITS)

    @property
    def row_type(self) -> str:
        value = self.values["type"]
        if not isinstance(value, str):
            raise TypeError("type is not a string")
        return value


@dataclass
class GeneratedSample:
    raw: int
    clean_position: float
    clean_velocity: float


@dataclass
class ErrorStats:
    label: str
    count: int = 0
    sum_error: float = 0.0
    sum_abs_error: float = 0.0
    sum_sq_error: float = 0.0
    max_abs_error: float = 0.0

    def add(self, estimate: float, truth: float) -> None:
        error = estimate - truth
        abs_error = abs(error)
        self.count += 1
        self.sum_error += error
        self.sum_abs_error += abs_error
        self.sum_sq_error += error * error
        self.max_abs_error = max(self.max_abs_error, abs_error)

    def line(self) -> str:
        if self.count == 0:
            return f"{self.label}: no samples"
        return (
            f"{self.label}: n={self.count} "
            f"rmse={math.sqrt(self.sum_sq_error / self.count):.6f} "
            f"mae={self.sum_abs_error / self.count:.6f} "
            f"bias={self.sum_error / self.count:+.6f} "
            f"max_abs={self.max_abs_error:.6f}"
        )


@dataclass
class GeneratedMetrics:
    position: ErrorStats
    velocity: ErrorStats
    input_noise: ErrorStats

    @classmethod
    def create(cls) -> "GeneratedMetrics":
        return cls(
            position=ErrorStats("position_vs_clean"),
            velocity=ErrorStats("velocity_vs_clean_delta"),
            input_noise=ErrorStats("input_noise"),
        )

    def add(self, row: SNNRow, sample: GeneratedSample) -> None:
        self.position.add(row.q("position"), sample.clean_position)
        self.velocity.add(row.q("velocity"), sample.clean_velocity)
        self.input_noise.add(row.q("measurement"), sample.clean_position)

    def print(self) -> None:
        if self.position.count == 0:
            return
        print("\nGenerated-mode accuracy vs clean signal")
        print(self.position.line())
        print(self.velocity.line())
        print(self.input_noise.line())


class LivePlot:
    def __init__(self, max_points: int, plot_types: set[str]):
        try:
            import matplotlib.pyplot as plt  # type: ignore[import-not-found]
        except ImportError as exc:
            raise SystemExit("matplotlib is required for --plot") from exc

        self.plt = plt
        self.max_points = max_points
        self.plot_types = plot_types
        self.rows: list[SNNRow] = []
        self.fig, self.axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        type_label = ",".join(sorted(plot_types))
        self.fig.canvas.manager.set_window_title(f"SNN UART Stream ({type_label})")
        self.lines = {
            "measurement": self.axes[0].plot([], [], label="measurement")[0],
            "position": self.axes[0].plot([], [], label="position")[0],
            "velocity": self.axes[1].plot([], [], label="velocity")[0],
            "m0": self.axes[2].plot([], [], label="m0")[0],
            "m1": self.axes[2].plot([], [], label="m1")[0],
            "m2": self.axes[2].plot([], [], label="m2")[0],
            "m3": self.axes[2].plot([], [], label="m3")[0],
            "m4": self.axes[2].plot([], [], label="m4")[0],
        }
        self.axes[0].set_ylabel("Q4.12")
        self.axes[1].set_ylabel("Q4.12")
        self.axes[2].set_ylabel("membrane")
        self.axes[2].set_xlabel("sample")
        for axis in self.axes:
            axis.grid(True, alpha=0.25)
            axis.legend(loc="upper left")
        plt.tight_layout()

    def add(self, row: SNNRow) -> None:
        if row.row_type not in self.plot_types:
            return
        self.rows.append(row)
        self.rows = self.rows[-self.max_points :]

        for name, line in self.lines.items():
            line.set_data(*self._series(name))

        for axis in self.axes:
            axis.relim()
            axis.autoscale_view()
        self.plt.pause(0.001)

    def _series(self, name: str) -> tuple[list[int], list[float]]:
        x_values: list[int] = []
        y_values: list[float] = []
        previous_type: str | None = None
        previous_t: int | None = None

        for idx, row in enumerate(self.rows):
            row_type = row.row_type
            row_t = row.raw("t")
            if (
                previous_type is not None
                and (row_type != previous_type or (previous_t is not None and row_t <= previous_t))
            ):
                x_values.append(idx)
                y_values.append(float("nan"))
            x_values.append(idx)
            y_values.append(row.q(name))
            previous_type = row_type
            previous_t = row_t
        return x_values, y_values


def fixed_to_float(value: int) -> float:
    return value / float(1 << FRAC_BITS)


def float_to_fixed(value: float) -> int:
    raw = int(round(value * (1 << FRAC_BITS)))
    return max(-32768, min(32767, raw))


def parse_snn_row(line: str) -> SNNRow | None:
    line = line.strip()
    if not line or line.startswith("#") or line.startswith("type,"):
        return None

    parts = [part.strip() for part in line.split(",")]
    if len(parts) < len(FIELDNAMES):
        return None
    if parts[0] not in {"S", "P", "H"}:
        return None

    values: dict[str, int | str] = {"type": parts[0]}
    for name, token in zip(FIELDNAMES[1:], parts[1:]):
        values[name] = int(token, 0)
    return SNNRow(values)


def iter_text_lines(source: TextIO) -> Iterable[str]:
    for line in source:
        yield line


def iter_serial_lines(port: str, baud: int, timeout: float) -> Iterable[str]:
    serial = import_serial()
    with serial.Serial(port, baudrate=baud, timeout=timeout) as ser:
        yield from iter_serial_device(ser)


def iter_serial_device(ser) -> Iterable[str]:
    while True:
        raw = ser.readline()
        if not raw:
            yield ""
            continue
        yield raw.decode("ascii", errors="ignore")


def open_writer(path: str | None) -> tuple[TextIO | None, csv.DictWriter | None]:
    if path is None:
        return None, None
    outfile = open(path, "w", newline="", encoding="ascii")
    fields = FIELDNAMES + [f"{name}_float" for name in sorted(Q_FIELDS)]
    writer = csv.DictWriter(outfile, fieldnames=fields)
    writer.writeheader()
    return outfile, writer


def write_csv_row(writer: csv.DictWriter | None, row: SNNRow) -> None:
    if writer is None:
        return
    out: dict[str, int | str | float] = dict(row.values)
    for name in sorted(Q_FIELDS):
        out[f"{name}_float"] = row.q(name)
    writer.writerow(out)


def print_summary(row: SNNRow) -> None:
    print(
        f"{row.values['type']} t={row.raw('t'):04d} "
        f"in={row.q('measurement'):+.3f} pos={row.q('position'):+.3f} "
        f"vel={row.q('velocity'):+.3f} spk=0x{row.raw('input_spikes'):x} "
        f"cyc={row.raw('cycles')}"
    )


def import_serial():
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise SystemExit("pyserial is required when using --port or --replay") from exc
    return serial


def emit_command(ser, command: str) -> None:
    ser.write(command.encode("ascii") + b"\n")
    ser.flush()


def clear_serial_input(ser) -> None:
    try:
        ser.reset_input_buffer()
    except AttributeError:
        pass


def handle_row(
    row: SNNRow,
    writer: csv.DictWriter | None,
    plot: LivePlot | None,
    quiet: bool,
) -> None:
    write_csv_row(writer, row)
    if not quiet:
        print_summary(row)
    if plot is not None:
        plot.add(row)


def send_replay(
    port: str,
    baud: int,
    timeout: float,
    samples: Iterable[int | GeneratedSample],
    sample_count: int | None,
    delay: float,
    override_spikes: int | None,
    writer: csv.DictWriter | None,
    plot: LivePlot | None,
    quiet: bool,
    raw: bool,
    response_timeout: float,
) -> int:
    serial = import_serial()
    expected_t = 0
    received = 0
    sent = 0
    metrics = GeneratedMetrics.create()
    saw_generated = False

    with serial.Serial(port, baudrate=baud, timeout=timeout) as ser:
        clear_serial_input(ser)
        emit_command(ser, "C")
        try:
            for sample in samples:
                sent += 1
                truth = sample if isinstance(sample, GeneratedSample) else None
                sample_raw = sample.raw if isinstance(sample, GeneratedSample) else sample
                saw_generated = saw_generated or truth is not None

                if override_spikes is None:
                    emit_command(ser, f"M {sample_raw}")
                else:
                    emit_command(ser, f"S {sample_raw} {override_spikes}")

                deadline = time.monotonic() + response_timeout
                got_response = False
                while time.monotonic() < deadline:
                    raw_line = ser.readline()
                    if not raw_line:
                        continue
                    line = raw_line.decode("ascii", errors="ignore")
                    row = parse_snn_row(line)
                    if row is None:
                        if raw:
                            print(line.rstrip())
                        continue
                    handle_row(row, writer, plot, quiet)
                    if row.row_type == "H" and row.raw("t") == expected_t:
                        if truth is not None:
                            metrics.add(row, truth)
                        got_response = True
                        received += 1
                        expected_t += 1
                        break

                if not got_response:
                    print(f"warning: no H response for replay sample {expected_t}", file=sys.stderr)
                    expected_t += 1

                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            pass
        finally:
            if saw_generated:
                metrics.print()

    expected = sample_count if sample_count is not None else sent
    return 0 if received == expected else 1


def load_replay_samples(path: str) -> list[int]:
    samples: list[int] = []
    with open(path, "r", encoding="ascii", errors="ignore") as infile:
        reader = csv.DictReader(row for row in infile if not row.startswith("#"))
        for row in reader:
            token = row.get("measurement") or row.get("measurement_raw")
            if token is None:
                raise SystemExit("replay CSV needs a measurement or measurement_raw column")
            samples.append(int(token, 0))
    return samples


def generate_samples(
    mode: str,
    count: int,
    offset: float,
    amplitude: float,
    period: float,
    noise: float,
    seed: int,
) -> Iterable[GeneratedSample]:
    if count < 0:
        raise SystemExit("--generate-count must be >= 0")
    if period <= 0.0:
        raise SystemExit("--generate-period must be > 0")

    rng = random.Random(seed)
    t = 0
    value = offset
    prev_clean = 0.0

    while count == 0 or t < count:
        if mode == "sine":
            clean = offset + amplitude * math.sin((2.0 * math.pi * t) / period)
        elif mode == "square":
            phase = (t % int(max(1, period))) / max(1.0, period)
            clean = offset + (amplitude if phase < 0.5 else -amplitude)
        elif mode == "ramp":
            phase = (t % int(max(1, period))) / max(1.0, period)
            clean = offset + amplitude * ((2.0 * phase) - 1.0)
        elif mode == "walk":
            value += rng.gauss(0.0, amplitude)
            clean = value
        else:
            raise SystemExit(f"unsupported generator mode: {mode}")

        noisy = clean + rng.gauss(0.0, noise)
        yield GeneratedSample(
            raw=float_to_fixed(noisy),
            clean_position=clean,
            clean_velocity=clean - prev_clean,
        )
        prev_clean = clean
        t += 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1_000_000, help="UART baud rate")
    parser.add_argument("--input", help="read an existing text log instead of a serial port")
    parser.add_argument("--output", help="write decoded rows to CSV")
    parser.add_argument("--plot", action="store_true", help="show live matplotlib plots")
    parser.add_argument(
        "--plot-types",
        default=None,
        help="comma-separated row types to plot: S=demo, P=probe, H=host; default S, or H with --replay",
    )
    parser.add_argument("--max-points", type=int, default=300, help="points retained in live plot")
    parser.add_argument("--quiet", action="store_true", help="do not print decoded summaries")
    parser.add_argument("--raw", action="store_true", help="echo non-protocol UART lines")
    parser.add_argument("--timeout", type=float, default=0.1, help="serial read timeout in seconds")
    parser.add_argument(
        "--host-mode",
        action="store_true",
        help="send C on connect, clearing SNN state and stopping demo autorun",
    )
    parser.add_argument("--replay", help="CSV file of raw measurement samples to send to firmware")
    parser.add_argument(
        "--generate",
        choices=("sine", "square", "ramp", "walk"),
        help="live-generate host samples instead of reading a replay CSV",
    )
    parser.add_argument("--generate-count", type=int, default=300, help="generated sample count; 0 streams until interrupted")
    parser.add_argument("--generate-offset", type=float, default=0.0, help="generated signal offset in Q4.12 float units")
    parser.add_argument("--generate-amplitude", type=float, default=0.5, help="generated signal amplitude in Q4.12 float units")
    parser.add_argument("--generate-period", type=float, default=48.0, help="generated signal period in samples")
    parser.add_argument("--generate-noise", type=float, default=0.0, help="Gaussian noise sigma in Q4.12 float units")
    parser.add_argument("--generate-seed", type=int, default=1, help="random seed for generated noise")
    parser.add_argument("--replay-delay", type=float, default=0.02, help="delay between replay commands")
    parser.add_argument("--response-timeout", type=float, default=1.0, help="seconds to wait for each replay H row")
    parser.add_argument("--override-spikes", type=lambda value: int(value, 0), help="send S commands with this spike pattern")
    return parser.parse_args()


def parse_plot_types(value: str | None, default: str) -> set[str]:
    if value is None:
        value = default
    selected = {part.strip().upper() for part in value.split(",") if part.strip()}
    valid = {"S", "P", "H"}
    invalid = selected - valid
    if invalid:
        raise SystemExit(f"unknown --plot-types value(s): {', '.join(sorted(invalid))}")
    if not selected:
        raise SystemExit("--plot-types must include at least one of S,P,H")
    return selected


def main() -> int:
    args = parse_args()
    if args.replay and args.generate:
        raise SystemExit("--replay and --generate are mutually exclusive")

    drive_host = args.replay or args.generate
    plot_types = parse_plot_types(args.plot_types, "H" if drive_host else "S")
    outfile, writer = open_writer(args.output)
    plot = LivePlot(args.max_points, plot_types) if args.plot else None
    infile: TextIO | None = None

    try:
        if drive_host:
            if not args.port:
                raise SystemExit("--replay/--generate requires --port")
            if args.replay:
                samples = load_replay_samples(args.replay)
                sample_count: int | None = len(samples)
            else:
                sample_count = None if args.generate_count == 0 else args.generate_count
                samples = generate_samples(
                    args.generate,
                    args.generate_count,
                    args.generate_offset,
                    args.generate_amplitude,
                    args.generate_period,
                    args.generate_noise,
                    args.generate_seed,
                )
            return send_replay(
                args.port,
                args.baud,
                args.timeout,
                samples,
                sample_count,
                args.replay_delay,
                args.override_spikes,
                writer,
                plot,
                args.quiet,
                args.raw,
                args.response_timeout,
            )

        if args.input:
            infile = open(args.input, "r", encoding="ascii", errors="ignore")
            lines = iter_text_lines(infile)
        elif args.port:
            serial = import_serial()
            ser = serial.Serial(args.port, baudrate=args.baud, timeout=args.timeout)
            if args.host_mode:
                clear_serial_input(ser)
                emit_command(ser, "C")
            lines = iter_serial_device(ser)
        else:
            lines = iter_text_lines(sys.stdin)

        if args.host_mode and not args.port:
            raise SystemExit("--host-mode requires --port")

        for line in lines:
            if not line:
                continue
            row = parse_snn_row(line)
            if row is None:
                if args.raw:
                    print(line.rstrip())
                continue
            handle_row(row, writer, plot, args.quiet)
    except KeyboardInterrupt:
        return 0
    finally:
        if outfile is not None:
            outfile.close()
        if infile is not None:
            infile.close()
        if "ser" in locals():
            ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
