#!/usr/bin/env python3
"""Schedule-bit-exact Python simulator for the 784->hidden->10 SNN-MLP.

Mirrors what the planned hardware datapath will do:

- Q4.12 signed integers throughout (no float in the inference path)
- Sequential MAC per layer per timestep with int accumulator
- Leak implemented as `mem - (mem >> beta_shift)` (exact 1 - 2^-k decay,
  matches `lif_bank_debug.v:112`)
- Membrane clip at +/-3.999, threshold-subtract reset at 1.0
- Per-output spike-count accumulator over T timesteps; argmax classifies

This is the validation gate before any Verilog: a snntorch-trained checkpoint
should produce essentially the same MNIST accuracy when run through this
schedule. If accuracy diverges noticeably from the snntorch forward, training
or quantization choices need adjustment before going to hardware.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time

import numpy as np


REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


@dataclass
class MLPConfig:
    in_size: int = 784
    hidden: int = 64
    out_size: int = 10
    beta_shift: int = 3          # beta = 1 - 2^(-beta_shift); 3 -> 0.875
    threshold: float = 1.0
    mem_clip: float = 3.999
    weight_clip: float = 3.999
    input_scale: float = 1.0
    timesteps: int = 25
    frac_bits: int = 12


@dataclass
class MLPModel:
    config: MLPConfig
    W1_q: np.ndarray   # int32 [hidden, in_size]
    b1_q: np.ndarray   # int32 [hidden]
    W2_q: np.ndarray   # int32 [out_size, hidden]
    b2_q: np.ndarray   # int32 [out_size]


def _to_fixed(value, frac_bits: int) -> np.ndarray:
    return np.round(np.asarray(value) * (1 << frac_bits)).astype(np.int32)


_KNOWN_BETAS = {3: 0.875, 4: 0.9375, 2: 0.75, 5: 0.96875}


def _beta_to_shift(beta: float) -> int:
    for shift, value in _KNOWN_BETAS.items():
        if abs(beta - value) < 1e-6:
            return shift
    raise ValueError(
        f"beta {beta} is not a power-of-two leak (1 - 2^-k); hardware requires that form"
    )


def load_checkpoint(pt_path: Path) -> MLPModel:
    import torch
    state = torch.load(pt_path, map_location="cpu", weights_only=False)
    if state.get("format") != "snn-mnist-v1":
        raise ValueError(f"unrecognized checkpoint format: {state.get('format')!r}")
    cfg = MLPConfig(
        in_size=int(state["in_size"]),
        hidden=int(state["hidden"]),
        out_size=int(state["out_size"]),
        beta_shift=_beta_to_shift(float(state["beta"])),
        threshold=float(state["threshold"]),
        mem_clip=float(state["mem_clip"]),
        weight_clip=float(state["weight_clip"]),
        input_scale=float(state["input_scale"]),
        timesteps=int(state["timesteps"]),
        frac_bits=int(state["frac_bits"]),
    )
    return MLPModel(
        config=cfg,
        W1_q=_to_fixed(state["W1"].numpy(), cfg.frac_bits),
        b1_q=_to_fixed(state["b1"].numpy(), cfg.frac_bits),
        W2_q=_to_fixed(state["W2"].numpy(), cfg.frac_bits),
        b2_q=_to_fixed(state["b2"].numpy(), cfg.frac_bits),
    )


def hardware_inference_batch(model: MLPModel, images_q: np.ndarray) -> np.ndarray:
    """Run schedule-bit-exact inference on a batch of images.

    Batching is purely an evaluation accelerator: each sample is processed with
    identical per-image state (no cross-sample sharing), so the per-image result
    is the same as a sequential run.

    images_q: int32 array [B, in_size], Q4.12 pixel values.
    Returns: int32 array [B, out_size] of spike counts over T timesteps.
    """
    cfg = model.config
    frac = cfg.frac_bits
    mem_clip_q = int(round(cfg.mem_clip * (1 << frac)))
    thr_q = int(round(cfg.threshold * (1 << frac)))
    beta_shift = cfg.beta_shift

    B = images_q.shape[0]
    mem1 = np.zeros((B, cfg.hidden), dtype=np.int32)
    mem2 = np.zeros((B, cfg.out_size), dtype=np.int32)
    spk_count = np.zeros((B, cfg.out_size), dtype=np.int32)

    # Layer-1 MAC needs int64 headroom: 784 products of Q4.12 * Q4.12 (raw +/- ~2^28)
    # can exceed int32 in the sum prior to the post-shift.
    W1_64 = model.W1_q.astype(np.int64)
    img_64 = images_q.astype(np.int64)

    # Layer-1 contribution is constant across timesteps under direct-current encoding.
    # Compute once outside the loop.
    acc1_const = ((img_64 @ W1_64.T) >> frac).astype(np.int32) + model.b1_q

    for _ in range(cfg.timesteps):
        leak1 = mem1 - (mem1 >> beta_shift)
        pre1 = leak1 + acc1_const
        np.clip(pre1, -mem_clip_q, mem_clip_q, out=pre1)
        spk1 = (pre1 >= thr_q).astype(np.int32)
        mem1 = pre1 - thr_q * spk1

        # Layer 2: spikes are unitless 0/1, so no shift after the MAC.
        acc2 = spk1 @ model.W2_q.T + model.b2_q
        leak2 = mem2 - (mem2 >> beta_shift)
        pre2 = leak2 + acc2
        np.clip(pre2, -mem_clip_q, mem_clip_q, out=pre2)
        spk2 = (pre2 >= thr_q).astype(np.int32)
        mem2 = pre2 - thr_q * spk2

        spk_count += spk2

    return spk_count


def hardware_inference(model: MLPModel, image_q: np.ndarray) -> np.ndarray:
    """Single-image convenience wrapper."""
    return hardware_inference_batch(model, image_q[None, :])[0]


def evaluate_mnist(
    model: MLPModel,
    data_dir: Path,
    limit: int = 0,
    batch_size: int = 256,
) -> dict:
    from torchvision import datasets, transforms

    test_ds = datasets.MNIST(
        data_dir, train=False, download=True, transform=transforms.ToTensor()
    )
    n_total = len(test_ds)
    n = n_total if limit <= 0 else min(limit, n_total)
    cfg = model.config

    correct = 0
    t0 = time.perf_counter()
    for start in range(0, n, batch_size):
        end = min(n, start + batch_size)
        imgs = np.stack(
            [test_ds[i][0].view(-1).numpy() for i in range(start, end)], axis=0
        )
        labels = np.array([test_ds[i][1] for i in range(start, end)], dtype=np.int32)
        imgs_q = _to_fixed(imgs * cfg.input_scale, cfg.frac_bits)
        spk = hardware_inference_batch(model, imgs_q)
        pred = np.argmax(spk, axis=1)
        correct += int((pred == labels).sum())
    dt = time.perf_counter() - t0
    return {
        "total": n,
        "correct": correct,
        "accuracy": correct / max(1, n),
        "elapsed_s": dt,
        "imgs_per_s": n / max(dt, 1e-9),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint", type=Path, default=REPO_ROOT / "build" / "snn_mnist.pt"
    )
    parser.add_argument("--data", type=Path, default=REPO_ROOT / "build" / "mnist")
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Optional cap on the number of test images (0 = full 10000).",
    )
    parser.add_argument("--batch-size", type=int, default=256)
    args = parser.parse_args()

    model = load_checkpoint(args.checkpoint)
    cfg = model.config
    print(
        f"loaded {args.checkpoint}  "
        f"shape: {cfg.in_size}->{cfg.hidden}->{cfg.out_size}  "
        f"T={cfg.timesteps}  beta=1-2^-{cfg.beta_shift}  "
        f"Q{4}.{cfg.frac_bits}"
    )

    metrics = evaluate_mnist(model, args.data, limit=args.limit, batch_size=args.batch_size)
    print(
        f"hw-schedule accuracy: {metrics['accuracy']*100:.2f}%  "
        f"({metrics['correct']}/{metrics['total']})  "
        f"{metrics['elapsed_s']:.2f}s  {metrics['imgs_per_s']:.1f} img/s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
