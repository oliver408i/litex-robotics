#!/usr/bin/env python3
"""Pack a trained snn_mnist checkpoint into the SDRAM weight blob.

Layout matches what verilog/snn_weight_loader.v consumes per timestep — the
loader replays this block T times. Each beat is one 32-bit little-endian
word holding N_MAC Q4.12 weights in the low N_MAC*16 bits.

Within one timestep:
  for tile in 0..ceil(HIDDEN/N_MAC)-1:    # layer 1 tiles
    for i in 0..IN_SIZE-1:                # input pixel index
      word[m=0..N_MAC-1] = W1_q[tile*N_MAC+m, i]   (0 for partial-tile pad)
  for tile in 0..ceil(OUT_SIZE/N_MAC)-1:  # layer 2 tiles
    for h in 0..HIDDEN-1:                 # hidden spike index
      word[m=0..N_MAC-1] = W2_q[tile*N_MAC+m, h]   (0 for partial-tile pad)

The script also prints the loader CSR values to load (beats_per_cycle,
num_cycles) and the bias write sequence (HIDDEN biases then OUT_SIZE biases)
so the host firmware knows what to push.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct
import sys

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from sim.snn_mlp import load_checkpoint


WEIGHT_BITS = 16          # Q4.12 word in the low bits of each beat
WORD_BYTES = 4            # 32-bit Wishbone


def pack_one_step(model, n_mac: int) -> tuple[bytes, int, int, int]:
    """Returns (blob, total_beats, w1_beats, w2_beats).

    Blob layout is unchanged: the W1 block (w1_beats) followed by the W2 block
    (w2_beats). The loader streams the W1 prefix once (preamble) and replays the
    W2 suffix per timestep, so w1_beats / w2_beats become the loader's
    preamble_beats / beats_per_cycle.
    """
    cfg = model.config
    W1 = model.W1_q
    W2 = model.W2_q

    l1_tiles = (cfg.hidden + n_mac - 1) // n_mac
    l2_tiles = (cfg.out_size + n_mac - 1) // n_mac
    w1_beats = l1_tiles * cfg.in_size
    w2_beats = l2_tiles * cfg.hidden
    beats = w1_beats + w2_beats

    blob = bytearray()
    weight_mask = (1 << WEIGHT_BITS) - 1

    def emit(words):
        val = 0
        for m, w in enumerate(words):
            val |= (int(w) & weight_mask) << (m * WEIGHT_BITS)
        blob.extend(struct.pack("<I", val & 0xFFFFFFFF))

    for tile in range(l1_tiles):
        for i in range(cfg.in_size):
            beat = []
            for m in range(n_mac):
                n = tile * n_mac + m
                beat.append(int(W1[n, i]) if n < cfg.hidden else 0)
            emit(beat)

    for tile in range(l2_tiles):
        for h in range(cfg.hidden):
            beat = []
            for m in range(n_mac):
                n = tile * n_mac + m
                beat.append(int(W2[n, h]) if n < cfg.out_size else 0)
            emit(beat)

    assert len(blob) == beats * WORD_BYTES, "blob size mismatch"
    return bytes(blob), beats, w1_beats, w2_beats


def pack_biases(model) -> list[int]:
    """Return [HIDDEN biases ... OUT_SIZE biases] in Q4.12 int order."""
    return [int(v) for v in list(model.b1_q) + list(model.b2_q)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint", type=Path, default=REPO / "build" / "snn_mnist.pt"
    )
    parser.add_argument(
        "--n-mac",
        type=int,
        default=1,
        help="Must match the SNNMLP wrapper's N_MAC (default 1).",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=REPO / "build" / "snn_mnist_weights.bin",
    )
    parser.add_argument(
        "--meta-out",
        type=Path,
        default=None,
        help="Optional JSON file capturing loader-CSR values and bias list.",
    )
    args = parser.parse_args()

    model = load_checkpoint(args.checkpoint)
    cfg = model.config
    blob, beats, w1_beats, w2_beats = pack_one_step(model, args.n_mac)
    biases = pack_biases(model)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(blob)

    print(f"checkpoint:       {args.checkpoint}")
    print(f"shape:            {cfg.in_size}->{cfg.hidden}->{cfg.out_size}  T={cfg.timesteps}")
    print(f"n_mac:            {args.n_mac}")
    print(f"blob path:        {args.out}")
    print(f"blob size:        {len(blob)} bytes ({beats} beats * {WORD_BYTES} bytes)")
    print(f"  W1 prefix:      {w1_beats} beats (streamed once)")
    print(f"  W2 block:       {w2_beats} beats (streamed per timestep x{cfg.timesteps})")
    print()
    print("Loader CSR values to write before each inference:")
    print(f"  weight_base             = <SDRAM byte address where the blob was loaded>")
    print(f"  weight_preamble_beats   = {w1_beats}")
    print(f"  weight_beats_per_cycle  = {w2_beats}")
    print(f"  weight_num_cycles       = {cfg.timesteps}")
    print()
    print(f"Bias load sequence (HIDDEN={cfg.hidden} then OUT_SIZE={cfg.out_size}, "
          f"{len(biases)} entries total):")
    print(f"  first 8 = {biases[:8]}  ...  last 4 = {biases[-4:]}")

    if args.meta_out:
        args.meta_out.parent.mkdir(parents=True, exist_ok=True)
        args.meta_out.write_text(json.dumps({
            "format": "snn-mnist-weights-v1",
            "checkpoint": str(args.checkpoint),
            "blob_path": str(args.out),
            "blob_size_bytes": len(blob),
            "n_mac": args.n_mac,
            "in_size": cfg.in_size,
            "hidden": cfg.hidden,
            "out_size": cfg.out_size,
            "timesteps": cfg.timesteps,
            "total_beats": beats,
            "preamble_beats": w1_beats,
            "beats_per_cycle": w2_beats,
            "num_cycles": cfg.timesteps,
            "biases_q412": biases,
        }, indent=2))
        print(f"meta written to:  {args.meta_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
