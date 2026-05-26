#!/usr/bin/env python3
"""Host driver for software/snn_mnist_demo running on the IcePi Zero SoC.

Sends biases, weight blob, and MNIST test images over UART. For each image,
collects the classification + 10 spike counts from the firmware and compares
to sim/snn_mlp.py (the bit-exact reference) and to the ground-truth label.
Reports per-image agreement and overall accuracy.

UART protocol (see software/snn_mnist_demo/main.c):
    'P'                                           -> 'p'
    'C' + u32 weight_base + u32 beats + u32 cyc   -> 'c'
    'B' + 74*int16 biases (HIDDEN then OUT_SIZE)  -> 'b'
    'W' + u32 length + length bytes               -> 'w'
    'I' + 784*int16 pixels                        -> 'i' + 1B class + 10B spk
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from sim.snn_mlp import _to_fixed, hardware_inference, load_checkpoint

# Memory map must match icepi_zero_base.py / generated mem.h.
MAIN_RAM_BASE       = 0x40000000
WEIGHT_BLOB_OFFSET  = 0x00100000  # 1 MiB into SDRAM; matches firmware
DEFAULT_WEIGHT_BASE = MAIN_RAM_BASE + WEIGHT_BLOB_OFFSET


def _open_serial(port: str, baud: int):
    import serial  # pyserial
    s = serial.Serial(port, baud, timeout=10)
    return s


def _expect(ser, byte: bytes, ctx: str) -> None:
    got = ser.read(1)
    if got != byte:
        raise IOError(f"{ctx}: expected {byte!r}, got {got!r}")


def _handshake(ser, attempts: int = 4) -> None:
    """Ping the firmware until it responds.

    The boot banner is printed once and is gone by the time we attach (litex_term
    consumed it while loading the firmware), so we don't wait for it. Instead we
    drain any stale RX bytes and send 'P' until we get 'p' back. Retries help
    when the firmware is mid-command from a previous (interrupted) session: our
    extra 'P' bytes become payload bytes, the stalled command eventually
    completes, and the next 'P' lands in the main loop.
    """
    ser.reset_input_buffer()
    for attempt in range(attempts):
        ser.write(b"P")
        got = ser.read(1)
        if got == b"p":
            return
        if got:
            # Unexpected byte — likely a stale ack from a half-finished command.
            # Drain whatever else is sitting there and try again.
            ser.reset_input_buffer()
    raise IOError(
        f"handshake failed after {attempts} pings — "
        f"is the firmware running on {ser.port}?"
    )


def configure(ser, weight_base: int, beats_per_cycle: int, num_cycles: int) -> None:
    ser.write(b"C" + struct.pack("<III", weight_base, beats_per_cycle, num_cycles))
    _expect(ser, b"c", "configure")


def load_biases(ser, biases_q412: list[int]) -> None:
    assert len(biases_q412) == 74, f"expected 74 biases, got {len(biases_q412)}"
    payload = struct.pack(f"<{len(biases_q412)}h", *biases_q412)
    ser.write(b"B" + payload)
    _expect(ser, b"b", "load_biases")


WEIGHT_CHUNK_SIZE = 256  # must match the firmware's WEIGHT_CHUNK_SIZE


def load_weights(ser, blob: bytes, verbose: bool = False) -> None:
    """Chunked upload to keep the firmware's UART RX FIFO from overflowing.

    Firmware ack pattern: '.' after each chunk consumed, 'w' after the final
    cache flush. This caps in-flight bytes to one chunk worth so the host can
    never outrun the firmware, regardless of cache eviction stalls on the
    bytewise SDRAM writes.
    """
    ser.write(b"W" + struct.pack("<I", len(blob)))
    cursor = 0
    chunks_sent = 0
    t0 = time.perf_counter()
    while cursor < len(blob):
        end = min(cursor + WEIGHT_CHUNK_SIZE, len(blob))
        ser.write(blob[cursor:end])
        got = ser.read(1)
        if got != b".":
            raise IOError(
                f"weight chunk at offset {cursor}: expected '.', got {got!r}"
            )
        cursor = end
        chunks_sent += 1
        if verbose and (chunks_sent % 200 == 0 or cursor == len(blob)):
            print(f"  {cursor}/{len(blob)} bytes "
                  f"({chunks_sent} chunks, {time.perf_counter() - t0:.1f}s)")
    _expect(ser, b"w", "load_weights final ack")


def run_inference(ser, image_q412: np.ndarray) -> tuple[int, list[int]]:
    """Returns (classification, spike_counts[10])."""
    assert image_q412.shape == (784,), f"image must be 784 Q4.12 ints, got {image_q412.shape}"
    payload = struct.pack(f"<{784}h", *[int(v) for v in image_q412])
    ser.write(b"I" + payload)
    _expect(ser, b"i", "inference")
    resp = ser.read(1 + 10)
    if len(resp) != 11:
        raise IOError(f"inference: short response, got {len(resp)} bytes")
    classification = resp[0]
    spk = list(resp[1:11])
    return classification, spk


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial device (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument(
        "--checkpoint", type=Path, default=REPO / "build" / "snn_mnist.pt"
    )
    parser.add_argument(
        "--weights-bin",
        type=Path,
        default=REPO / "build" / "snn_mnist_weights.bin",
        help="Pre-packed weight blob (tools/pack_snn_mnist_weights.py output).",
    )
    parser.add_argument(
        "--weight-base",
        type=lambda v: int(v, 0),
        default=DEFAULT_WEIGHT_BASE,
        help=f"SDRAM byte address for weight blob. Default {DEFAULT_WEIGHT_BASE:#x}.",
    )
    parser.add_argument(
        "--n-images", type=int, default=100, help="Number of MNIST test images to run."
    )
    parser.add_argument(
        "--mnist-dir", type=Path, default=REPO / "build" / "mnist"
    )
    parser.add_argument(
        "--skip-setup",
        action="store_true",
        help="Skip biases/weights/config load (firmware already configured).",
    )
    args = parser.parse_args()

    if not args.checkpoint.exists():
        raise FileNotFoundError(f"checkpoint missing: {args.checkpoint}")
    if not args.weights_bin.exists():
        raise FileNotFoundError(
            f"weights blob missing: {args.weights_bin}\n"
            f"Run tools/pack_snn_mnist_weights.py first."
        )

    model = load_checkpoint(args.checkpoint)
    cfg = model.config
    biases = [int(v) for v in list(model.b1_q) + list(model.b2_q)]
    blob = args.weights_bin.read_bytes()
    beats_per_cycle = len(blob) // 4   # each beat is one 32-bit word
    print(f"checkpoint:       {args.checkpoint}")
    print(f"weight blob:      {args.weights_bin}  ({len(blob)} bytes, {beats_per_cycle} beats)")
    print(f"target:           {args.port} @ {args.baud} baud")
    print(f"weight_base:      {args.weight_base:#010x}")

    ser = _open_serial(args.port, args.baud)
    try:
        _handshake(ser)
        print("firmware ready:   ping ok")

        if not args.skip_setup:
            print("configuring loader CSRs...")
            configure(ser, args.weight_base, beats_per_cycle, cfg.timesteps)
            print(f"  beats_per_cycle={beats_per_cycle}, num_cycles={cfg.timesteps}")

            print("loading biases...")
            load_biases(ser, biases)

            print(f"uploading {len(blob)} bytes of weights to SDRAM "
                  f"(chunked, {WEIGHT_CHUNK_SIZE}B/chunk)...")
            t0 = time.perf_counter()
            load_weights(ser, blob, verbose=True)
            print(f"  uploaded in {time.perf_counter() - t0:.1f}s")

        from torchvision import datasets, transforms

        test_ds = datasets.MNIST(
            args.mnist_dir, train=False, download=False, transform=transforms.ToTensor()
        )

        n = min(args.n_images, len(test_ds))
        print(f"running {n} images...")

        ref_matches = 0
        label_matches = 0
        t0 = time.perf_counter()
        for i in range(n):
            img, label = test_ds[i]
            image_q = _to_fixed(
                img.view(-1).numpy() * cfg.input_scale, cfg.frac_bits
            ).astype(np.int32)

            ref_spk = hardware_inference(model, image_q)
            ref_class = int(np.argmax(ref_spk))

            dut_class, dut_spk = run_inference(ser, image_q)
            spk_match = dut_spk == ref_spk.tolist()

            if spk_match and dut_class == ref_class:
                ref_matches += 1
            if dut_class == label:
                label_matches += 1

            if i < 10 or i % 25 == 0:
                marker = "ok" if spk_match else "DIFF"
                print(
                    f"img {i:4d}  label={label}  ref={ref_class}  dut={dut_class}  "
                    f"spk={spk_match} [{marker}]  ref_spk={ref_spk.tolist()}  "
                    f"dut_spk={dut_spk}"
                )

        elapsed = time.perf_counter() - t0
        print()
        print(f"images processed:           {n}")
        print(f"hardware == simulator:      {ref_matches}/{n} ({100 * ref_matches / n:.2f}%)")
        print(f"hardware classification ok: {label_matches}/{n} ({100 * label_matches / n:.2f}%)")
        print(f"throughput:                 {n / elapsed:.1f} img/s ({elapsed * 1000 / n:.1f} ms/img)")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
