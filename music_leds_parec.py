#!/usr/bin/env python3
import argparse
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np

from uart_client import UartProtocol


# ----------------------------
# Pulse monitor source listing
# ----------------------------

@dataclass
class PulseSource:
    index: int
    name: str
    description: str


def _run(cmd: List[str]) -> str:
    return subprocess.check_output(cmd, text=True, stderr=subprocess.DEVNULL)


def list_pulse_monitor_sources() -> List[PulseSource]:
    short = _run(["pactl", "list", "sources", "short"])
    full = _run(["pactl", "list", "sources"])

    desc_by_name = {}
    current_name = None
    for line in full.splitlines():
        line = line.strip()
        if line.startswith("Name:"):
            current_name = line.split("Name:", 1)[1].strip()
        elif line.startswith("Description:") and current_name is not None:
            desc_by_name[current_name] = line.split("Description:", 1)[1].strip()

    sources: List[PulseSource] = []
    for row in short.splitlines():
        parts = row.split("\t")
        if len(parts) < 2:
            continue
        idx = int(parts[0])
        name = parts[1]
        if not name.endswith(".monitor"):
            continue
        sources.append(PulseSource(idx, name, desc_by_name.get(name, "(no description)")))
    return sources


def choose_index(n: int, prompt: str) -> int:
    while True:
        s = input(prompt).strip()
        if s.lower() in ("q", "quit", "exit"):
            raise SystemExit(0)
        try:
            i = int(s)
            if 0 <= i < n:
                return i
        except ValueError:
            pass
        print(f"Enter 0..{n-1} or 'q'.")


def pick_monitor_source(preferred: Optional[str] = None) -> str:
    sources = list_pulse_monitor_sources()
    if not sources:
        raise RuntimeError("No .monitor sources found.")

    if preferred:
        for s in sources:
            if preferred in s.name or preferred in s.description:
                return s.name

    print("\nAvailable monitor sources:")
    for i, s in enumerate(sources):
        print(f"  [{i}] {s.description}  |  {s.name}")
    print()
    idx = choose_index(len(sources), "Select source index: ")
    return sources[idx].name


# ----------------------------
# LED helpers (your protocol)
# ----------------------------

def bulk_fill(client: UartProtocol, count: int, colors_grb: List[Tuple[int, int, int]], chunk: int = 80):
    for start in range(0, count, chunk):
        end = min(count, start + chunk)
        client.set_strip_bulk(start, colors_grb[start:end])


def wheel_grb(pos: int) -> Tuple[int, int, int]:
    pos &= 0xFF
    if pos < 85:
        return (255 - pos * 3, 0, pos * 3)
    if pos < 170:
        pos -= 85
        return (0, pos * 3, 255 - pos * 3)
    pos -= 170
    return (pos * 3, 255 - pos * 3, 0)


def render_center_vu(count: int, level: float, peak: float, hue: int, max_brightness: int) -> List[Tuple[int, int, int]]:
    leds = [(0, 0, 0)] * count
    half = count // 2

    bar = int(round(level * half))
    peak_pos = int(round(peak * half))

    g, r, b = wheel_grb(hue)
    scale = max_brightness / 255.0
    base = (int(round(g * scale)), int(round(r * scale)), int(round(b * scale)))
    peakc = (max_brightness, max_brightness, max_brightness)

    for i in range(bar):
        left = half - 1 - i
        right = half + i
        if 0 <= left < count:
            leds[left] = base
        if 0 <= right < count:
            leds[right] = base

    if 0 <= (half - 1 - peak_pos) < count:
        leds[half - 1 - peak_pos] = peakc
    if 0 <= (half + peak_pos) < count:
        leds[half + peak_pos] = peakc

    return leds


def make_log_bands(freqs: np.ndarray, n_bands: int, fmin: float, fmax: float) -> List[Tuple[int, int]]:
    edges = np.logspace(np.log10(fmin), np.log10(fmax), num=n_bands + 1)
    bands: List[Tuple[int, int]] = []
    last = 1
    for i in range(n_bands):
        start = int(np.searchsorted(freqs, edges[i], side="left"))
        end = int(np.searchsorted(freqs, edges[i + 1], side="right"))
        start = max(start, last)
        end = max(end, start + 1)
        end = min(end, len(freqs))
        bands.append((start, end))
        last = end
    return bands


def render_spectrum(
    count: int,
    band_levels: List[float],
    band_peaks: List[float],
    max_brightness: int,
    hue_offset: int,
) -> List[Tuple[int, int, int]]:
    leds = [(0, 0, 0)] * count
    n = len(band_levels)
    if n == 0:
        return leds

    seg_base = count // n
    remainder = count % n
    cursor = 0
    for i, level in enumerate(band_levels):
        seg_len = seg_base + (1 if i < remainder else 0)
        start = cursor
        end = min(count, start + seg_len)
        cursor = end
        if start >= end:
            continue

        hue = (hue_offset + int(255 * i / max(1, n - 1))) & 0xFF
        g, r, b = wheel_grb(hue)
        bright = max(0, min(max_brightness, int(round(max_brightness * level))))
        scale = bright / 255.0
        color = (int(round(g * scale)), int(round(r * scale)), int(round(b * scale)))

        lit = int(round(level * (end - start)))
        for p in range(start, min(end, start + lit)):
            leds[p] = color

        peak = max(0.0, min(1.0, band_peaks[i]))
        peak_pos = start + int(round(peak * (end - start - 1)))
        if start <= peak_pos < end:
            leds[peak_pos] = (max_brightness, max_brightness, max_brightness)

    return leds


# ----------------------------
# Audio capture via parec
# ----------------------------

def start_parec(source_name: str, samplerate: int) -> subprocess.Popen:
    """
    Stream float32 mono from a Pulse/PipeWire source to stdout.
    """
    cmd = [
        "parec",
        "-d", source_name,
        "--format=float32le",
        "--rate", str(samplerate),
        "--channels=1",
        "--latency-msec=10",
        "--process-time-msec=10",
    ]

    try:
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise RuntimeError("parec not found. Install pulseaudio-utils.")


def read_exact(pipe, nbytes: int) -> bytes:
    buf = b""
    while len(buf) < nbytes:
        chunk = pipe.read(nbytes - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return buf


def main():
    ap = argparse.ArgumentParser(description="Music-reactive LEDs via UartProtocol using Pulse monitor source (parec).")
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", default=750000, type=int)
    ap.add_argument("--count", default=299, type=int)
    ap.add_argument("--fps", default=40, type=int)
    ap.add_argument("--max-brightness", default=120, type=int)
    ap.add_argument("--samplerate", default=48000, type=int)
    ap.add_argument("--source", default="", help="Substring to auto-pick (e.g. Speaker, HDMI, Logitech)")
    ap.add_argument("--agc", default=20.0, type=float)
    ap.add_argument("--attack", default=0.35, type=float)
    ap.add_argument("--decay", default=0.08, type=float)
    ap.add_argument("--peak-decay", default=0.02, type=float)
    ap.add_argument("--bands", default=12, type=int)
    ap.add_argument("--fmin", default=40.0, type=float)
    ap.add_argument("--fmax", default=8000.0, type=float)
    args = ap.parse_args()

    source = pick_monitor_source(args.source if args.source else None)

    print(f"Using audio source: {source}")
    print(f"UART: {args.port} @ {args.baud}, LEDs={args.count}, FPS={args.fps}, max_brightness={args.max_brightness}")
    print("Effect: spectrum_bands")
    print("Ctrl+C to stop.\n")

    client = UartProtocol(args.port, args.baud)

    block = int(args.samplerate / args.fps)
    block = max(block, 256)
    window = np.hanning(block).astype(np.float32)
    freqs = np.fft.rfftfreq(block, d=1.0 / args.samplerate)
    bands = make_log_bands(freqs, args.bands, args.fmin, args.fmax)

    proc = start_parec(source, args.samplerate)
    assert proc.stdout is not None

    level_smoothed = 0.0
    hue = 0
    band_levels = [0.0] * args.bands
    band_peaks = [0.0] * args.bands

    bytes_per_frame = block * 4  # float32 mono

    try:
        while True:
            raw = read_exact(proc.stdout, bytes_per_frame)
            if not raw:
                raise RuntimeError("Audio stream ended (parec produced no data).")

            x = np.frombuffer(raw, dtype=np.float32)

            # FFT magnitude
            X = np.fft.rfft(x * window)
            mag = np.abs(X).astype(np.float32)

            # energy proxy
            mag = np.log1p(mag)
            low = float(mag[1: max(2, len(mag)//8)].mean())
            overall = float(np.mean(mag))
            level = low / (overall * args.agc + 1e-9)
            level = max(0.0, min(1.0, level))

            # smoothing
            if level > level_smoothed:
                level_smoothed = (1 - args.attack) * level_smoothed + args.attack * level
            else:
                level_smoothed = (1 - args.decay) * level_smoothed + args.decay * level

            for i, (start, end) in enumerate(bands):
                if end <= start:
                    band_level = 0.0
                else:
                    band_level = float(np.mean(mag[start:end])) / (overall * args.agc + 1e-9)
                band_level = max(0.0, min(1.0, band_level))
                if band_level > band_levels[i]:
                    band_levels[i] = (1 - args.attack) * band_levels[i] + args.attack * band_level
                else:
                    band_levels[i] = (1 - args.decay) * band_levels[i] + args.decay * band_level

                if band_levels[i] > band_peaks[i]:
                    band_peaks[i] = band_levels[i]
                else:
                    band_peaks[i] = max(0.0, band_peaks[i] - args.peak_decay)

            if level_smoothed > 0.05:
                hue = (hue + 2) & 0xFF

            leds = render_spectrum(args.count, band_levels, band_peaks, args.max_brightness, hue)
            bulk_fill(client, args.count, leds, chunk=80)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            bulk_fill(client, args.count, [(0, 0, 0)] * args.count, chunk=80)
        except Exception:
            pass
        client.close()
        if proc.poll() is None:
            proc.terminate()


if __name__ == "__main__":
    main()
