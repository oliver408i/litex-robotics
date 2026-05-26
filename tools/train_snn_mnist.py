#!/usr/bin/env python3
"""Train a Q4.12-quantized SNN-MLP on MNIST with snntorch + BPTT.

Architecture: 784 -> hidden -> 10 LIF layers, direct-current input injection,
spike-rate readout. Numerics mirror lif_bank_debug.v:

- beta = 0.875 (leak), threshold = 1.0, subtract-reset
- Q4.12 fixed-point with mem_clip = 3.999, weight_clip = 3.999

Saves a checkpoint future hardware export tooling can consume.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time
from typing import Iterable

import torch
import torch.nn as nn
import torch.nn.functional as F
from snntorch import surrogate
from torchvision import datasets, transforms

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


FRAC_BITS = 12
SCALE = float(1 << FRAC_BITS)


class _Quantize(torch.autograd.Function):
    """Q4.12 quantize-then-clip with clip-aware straight-through gradient."""

    @staticmethod
    def forward(ctx, x: torch.Tensor, clip: float) -> torch.Tensor:
        ctx.save_for_backward(x)
        ctx.clip = clip
        q = torch.round(x * SCALE) / SCALE
        return torch.clamp(q, -clip, clip)

    @staticmethod
    def backward(ctx, grad_out: torch.Tensor):
        (x,) = ctx.saved_tensors
        mask = (x.abs() <= ctx.clip).to(grad_out.dtype)
        return grad_out * mask, None


def quantize(x: torch.Tensor, clip: float) -> torch.Tensor:
    return _Quantize.apply(x, clip)


class SNNMLP(nn.Module):
    def __init__(
        self,
        in_size: int = 784,
        hidden: int = 64,
        out_size: int = 10,
        beta: float = 0.875,
        threshold: float = 1.0,
        mem_clip: float = 3.999,
        weight_clip: float = 3.999,
        input_scale: float = 1.0,
        slope: float = 25.0,
        timesteps: int = 25,
        full_qat: bool = False,
    ) -> None:
        super().__init__()
        self.in_size = in_size
        self.hidden = hidden
        self.out_size = out_size
        self.beta = beta
        self.threshold = threshold
        self.mem_clip = mem_clip
        self.weight_clip = weight_clip
        self.input_scale = input_scale
        self.timesteps = timesteps
        self.full_qat = full_qat

        # He init keeps initial pre-activations near unit variance.
        self.W1 = nn.Parameter(torch.empty(hidden, in_size).normal_(0.0, math.sqrt(2.0 / in_size)))
        self.b1 = nn.Parameter(torch.zeros(hidden))
        self.W2 = nn.Parameter(torch.empty(out_size, hidden).normal_(0.0, math.sqrt(2.0 / hidden)))
        self.b2 = nn.Parameter(torch.zeros(out_size))

        self.spike_fn = surrogate.fast_sigmoid(slope=slope)

    def quantized_weights(self) -> tuple[torch.Tensor, ...]:
        return (
            quantize(self.W1, self.weight_clip),
            quantize(self.b1, self.weight_clip),
            quantize(self.W2, self.weight_clip),
            quantize(self.b2, self.weight_clip),
        )

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        """images: [B, 784] in [0, 1]; returns spike-count tensor [B, 10]."""
        B = images.size(0)
        W1, b1, W2, b2 = self.quantized_weights()
        drive_in = images * self.input_scale

        mem1 = torch.zeros(B, self.hidden, device=images.device)
        mem2 = torch.zeros(B, self.out_size, device=images.device)
        spk_sum = torch.zeros(B, self.out_size, device=images.device)

        for _ in range(self.timesteps):
            i1 = drive_in @ W1.T + b1
            if self.full_qat:
                i1 = quantize(i1, self.mem_clip)
            pre1 = torch.clamp(self.beta * mem1 + i1, -self.mem_clip, self.mem_clip)
            if self.full_qat:
                pre1 = quantize(pre1, self.mem_clip)
            spk1 = self.spike_fn(pre1 - self.threshold)
            mem1 = pre1 - self.threshold * spk1

            i2 = spk1 @ W2.T + b2
            if self.full_qat:
                i2 = quantize(i2, self.mem_clip)
            pre2 = torch.clamp(self.beta * mem2 + i2, -self.mem_clip, self.mem_clip)
            if self.full_qat:
                pre2 = quantize(pre2, self.mem_clip)
            spk2 = self.spike_fn(pre2 - self.threshold)
            mem2 = pre2 - self.threshold * spk2

            spk_sum = spk_sum + spk2

        return spk_sum


def make_loaders(
    data_dir: Path,
    batch_size: int,
    smoke: bool = False,
) -> tuple[torch.utils.data.DataLoader, torch.utils.data.DataLoader]:
    transform = transforms.ToTensor()
    train_ds = datasets.MNIST(data_dir, train=True, download=True, transform=transform)
    test_ds = datasets.MNIST(data_dir, train=False, download=True, transform=transform)
    if smoke:
        train_ds = torch.utils.data.Subset(train_ds, range(1024))
        test_ds = torch.utils.data.Subset(test_ds, range(1024))
    train_loader = torch.utils.data.DataLoader(
        train_ds, batch_size=batch_size, shuffle=True, num_workers=2, pin_memory=True
    )
    test_loader = torch.utils.data.DataLoader(
        test_ds, batch_size=batch_size, shuffle=False, num_workers=2, pin_memory=True
    )
    return train_loader, test_loader


def evaluate(
    model: SNNMLP,
    loader: Iterable,
    device: torch.device,
) -> float:
    model.eval()
    correct = total = 0
    with torch.no_grad():
        for x, y in loader:
            x = x.view(x.size(0), -1).to(device, non_blocking=True)
            y = y.to(device, non_blocking=True)
            spk_sum = model(x)
            pred = spk_sum.argmax(dim=1)
            correct += int((pred == y).sum().item())
            total += y.size(0)
    return correct / max(1, total)


def train_epoch(
    model: SNNMLP,
    loader: Iterable,
    opt: torch.optim.Optimizer,
    device: torch.device,
) -> tuple[float, float]:
    model.train()
    total_loss = 0.0
    correct = total = 0
    for x, y in loader:
        x = x.view(x.size(0), -1).to(device, non_blocking=True)
        y = y.to(device, non_blocking=True)
        spk_sum = model(x)
        loss = F.cross_entropy(spk_sum, y)
        opt.zero_grad()
        loss.backward()
        opt.step()
        total_loss += loss.item() * y.size(0)
        correct += int((spk_sum.argmax(dim=1) == y).sum().item())
        total += y.size(0)
    return total_loss / max(1, total), correct / max(1, total)


def save_checkpoint(model: SNNMLP, path: Path) -> None:
    W1, b1, W2, b2 = model.quantized_weights()
    torch.save(
        {
            "format": "snn-mnist-v1",
            "frac_bits": FRAC_BITS,
            "in_size": model.in_size,
            "hidden": model.hidden,
            "out_size": model.out_size,
            "beta": model.beta,
            "threshold": model.threshold,
            "mem_clip": model.mem_clip,
            "weight_clip": model.weight_clip,
            "input_scale": model.input_scale,
            "timesteps": model.timesteps,
            "W1": W1.detach().cpu(),
            "b1": b1.detach().cpu(),
            "W2": W2.detach().cpu(),
            "b2": b2.detach().cpu(),
        },
        path,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=REPO_ROOT / "build" / "mnist")
    parser.add_argument("--hidden", type=int, default=64)
    parser.add_argument("--timesteps", type=int, default=25)
    parser.add_argument("--epochs", type=int, default=15)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--input-scale", type=float, default=1.0)
    parser.add_argument(
        "--full-qat",
        action="store_true",
        help="Also quantize membrane/accumulator state during training.",
    )
    parser.add_argument(
        "--device",
        choices=("auto", "cuda", "cpu"),
        default="auto",
        help="Auto picks CUDA when available. Unlike the tracker, this model "
             "(50K MACs/timestep) has enough parallel work for the GPU to win.",
    )
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "build" / "snn_mnist.pt")
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="Truncate train/test to 1024 samples each for a fast pipeline smoke test.",
    )
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)
    print(f"device: {device}  hidden: {args.hidden}  T: {args.timesteps}")

    torch.manual_seed(args.seed)
    if device.type == "cuda":
        torch.cuda.manual_seed_all(args.seed)

    args.data.mkdir(parents=True, exist_ok=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)

    train_loader, test_loader = make_loaders(args.data, args.batch_size, smoke=args.smoke)

    model = SNNMLP(
        hidden=args.hidden,
        timesteps=args.timesteps,
        input_scale=args.input_scale,
        full_qat=args.full_qat,
    ).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)

    best_acc = 0.0
    for epoch in range(args.epochs):
        t0 = time.perf_counter()
        train_loss, train_acc = train_epoch(model, train_loader, opt, device)
        test_acc = evaluate(model, test_loader, device)
        dt = time.perf_counter() - t0
        tag = ""
        if test_acc > best_acc:
            best_acc = test_acc
            save_checkpoint(model, args.out)
            tag = "  *saved*"
        print(
            f"epoch {epoch+1:3d}  {dt:5.1f}s  loss {train_loss:.4f}  "
            f"train_acc {train_acc:.4f}  test_acc {test_acc:.4f}{tag}"
        )
    print(f"best test_acc: {best_acc:.4f}  saved to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
