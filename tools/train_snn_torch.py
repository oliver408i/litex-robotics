#!/usr/bin/env python3
"""Train the 8-neuron LIF tracking estimator with snntorch + BPTT.

Architecture and numeric format are pinned to the existing hardware:

- 4-bit firmware-side threshold encoder (fixed; non-trainable)
- 8 LIF neurons, beta=0.875, threshold=1.0, subtract reset
- 8x8 recurrent path with zero diagonal (no self-loops)
- 11-element feature vector: [m0..m7, feat_z, feat_dz, 1.0]
- 2-output linear readout
- Q4.12 fixed-point with mem_clip=3.999, weight_clip=3.999,
  feature_clip=1.999, readout_acc_clip=7.999

The trained checkpoint is consumed by tools/export_snn_coeffs.py --from-pt to
emit the same 118-entry UART weight image the firmware already accepts.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
from typing import Sequence

import torch
import torch.nn as nn
from snntorch import surrogate

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sim.generate_tracking_data import TrackingSequence, generate_dataset
from sim.lif_reservoir_1d import (
    ReservoirConfig,
    ReservoirModel,
    evaluate_hardware_schedule_model,
)


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


def encode_spikes(z: torch.Tensor, dz: torch.Tensor) -> torch.Tensor:
    zn = torch.clamp(z / 2.5, -1.5, 1.5)
    dn = torch.clamp(dz / 5.0, -1.5, 1.5)
    return torch.stack(
        [
            (zn > 0.20).float(),
            (zn < -0.20).float(),
            (dn > 0.04).float(),
            (dn < -0.04).float(),
        ],
        dim=-1,
    )


class TinyReservoir(nn.Module):
    def __init__(
        self,
        n_in: int = 4,
        n_neurons: int = 8,
        n_out: int = 2,
        beta: float = 0.875,
        threshold: float = 1.0,
        mem_clip: float = 3.999,
        weight_clip: float = 3.999,
        feature_clip: float = 1.999,
        readout_acc_clip: float = 7.999,
        meas_scale: float = 2.5,
        delta_scale: float = 5.0,
        slope: float = 25.0,
        full_qat: bool = False,
    ) -> None:
        super().__init__()
        self.n_in = n_in
        self.n_neurons = n_neurons
        self.n_out = n_out
        self.beta = beta
        self.threshold = threshold
        self.mem_clip = mem_clip
        self.weight_clip = weight_clip
        self.feature_clip = feature_clip
        self.readout_acc_clip = readout_acc_clip
        self.meas_scale = meas_scale
        self.delta_scale = delta_scale
        self.full_qat = full_qat

        self.W_in = nn.Parameter(torch.empty(n_neurons, n_in).uniform_(-0.6, 0.6))
        self.W_rec = nn.Parameter(torch.empty(n_neurons, n_neurons).uniform_(-0.35, 0.35))
        self.W_read = nn.Parameter(torch.empty(n_out, n_neurons + 3).uniform_(-0.3, 0.3))
        self.register_buffer("rec_mask", 1.0 - torch.eye(n_neurons))
        self.spike_fn = surrogate.fast_sigmoid(slope=slope)

    def quantized_weights(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        return (
            quantize(self.W_in, self.weight_clip),
            quantize(self.W_rec * self.rec_mask, self.weight_clip),
            quantize(self.W_read, self.weight_clip),
        )

    def forward(self, meas: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        B, T = meas.shape
        Win, Wrec, Wread = self.quantized_weights()
        mem = torch.zeros(B, self.n_neurons, device=meas.device)
        spk = torch.zeros_like(mem)
        prev = torch.zeros(B, device=meas.device)
        pos_list: list[torch.Tensor] = []
        vel_list: list[torch.Tensor] = []

        for t in range(T):
            z = meas[:, t]
            dz = z - prev
            inp = encode_spikes(z, dz)
            drive = inp @ Win.T + spk @ Wrec.T
            if self.full_qat:
                drive = quantize(drive, self.readout_acc_clip)
            pre = torch.clamp(self.beta * mem + drive, -self.mem_clip, self.mem_clip)
            if self.full_qat:
                pre = quantize(pre, self.mem_clip)
            spk = self.spike_fn(pre - self.threshold)
            mem = pre - self.threshold * spk

            fz = torch.clamp(z / self.meas_scale, -self.feature_clip, self.feature_clip)
            fdz = torch.clamp(dz / self.delta_scale, -self.feature_clip, self.feature_clip)
            if self.full_qat:
                fz = quantize(fz, self.feature_clip)
                fdz = quantize(fdz, self.feature_clip)
            one = torch.ones_like(fz)
            feat = torch.cat(
                [mem, fz.unsqueeze(-1), fdz.unsqueeze(-1), one.unsqueeze(-1)], dim=-1
            )
            out = feat @ Wread.T
            if self.full_qat:
                out = quantize(out, self.readout_acc_clip)
            pos_list.append(out[:, 0])
            vel_list.append(out[:, 1])
            prev = z

        return torch.stack(pos_list, dim=1), torch.stack(vel_list, dim=1)


def sequences_to_tensors(
    seqs: Sequence[TrackingSequence],
    device: torch.device,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    meas = torch.tensor([s.measurement for s in seqs], dtype=torch.float32, device=device)
    pos = torch.tensor([s.true_position for s in seqs], dtype=torch.float32, device=device)
    vel = torch.tensor([s.true_velocity for s in seqs], dtype=torch.float32, device=device)
    return meas, pos, vel


def model_to_reservoir_model(model: TinyReservoir) -> ReservoirModel:
    Win, Wrec, Wread = model.quantized_weights()
    cfg = ReservoirConfig(
        input_size=model.n_in,
        neuron_count=model.n_neurons,
        beta=model.beta,
        threshold=model.threshold,
        membrane_clip=model.mem_clip,
        weight_clip=model.weight_clip,
        feature_clip=model.feature_clip,
        readout_acc_clip=model.readout_acc_clip,
        measurement_scale=model.meas_scale,
        delta_scale=model.delta_scale,
        frac_bits=FRAC_BITS,
    )
    return ReservoirModel(
        config=cfg,
        input_weights=Win.detach().cpu().tolist(),
        recurrent_weights=Wrec.detach().cpu().tolist(),
        readout_weights=Wread.detach().cpu().tolist(),
    )


def evaluate_hardware(
    model: TinyReservoir,
    sequences: Sequence[TrackingSequence],
) -> dict[str, float]:
    rm = model_to_reservoir_model(model)
    pos_sq = 0.0
    vel_sq = 0.0
    total = 0
    for seq in sequences:
        m = evaluate_hardware_schedule_model(rm, seq)
        n = len(seq.measurement)
        pos_sq += (m["position_rmse"] ** 2) * n
        vel_sq += (m["velocity_rmse"] ** 2) * n
        total += n
    total = max(1, total)
    return {
        "position_rmse_hw": math.sqrt(pos_sq / total),
        "velocity_rmse_hw": math.sqrt(vel_sq / total),
    }


def save_checkpoint(model: TinyReservoir, path: Path) -> None:
    Win, Wrec, Wread = model.quantized_weights()
    torch.save(
        {
            "format": "snn-tracker-v1",
            "frac_bits": FRAC_BITS,
            "n_in": model.n_in,
            "n_neurons": model.n_neurons,
            "n_out": model.n_out,
            "beta": model.beta,
            "threshold": model.threshold,
            "mem_clip": model.mem_clip,
            "weight_clip": model.weight_clip,
            "feature_clip": model.feature_clip,
            "readout_acc_clip": model.readout_acc_clip,
            "meas_scale": model.meas_scale,
            "delta_scale": model.delta_scale,
            "W_in": Win.detach().cpu(),
            "W_rec": Wrec.detach().cpu(),
            "W_read": Wread.detach().cpu(),
        },
        path,
    )


def train(
    *,
    train_count: int,
    val_count: int,
    length: int,
    epochs: int,
    batch_size: int,
    lr: float,
    vel_weight: float,
    seed: int,
    noise: float,
    full_qat: bool,
    device: torch.device,
    out_path: Path,
) -> None:
    torch.manual_seed(seed)
    common = dict(length=length, measurement_sigma=noise, mode="tracking")
    train_seqs = generate_dataset(train_count, seed=seed, **common)
    val_seqs = generate_dataset(val_count, seed=seed + 10_000, **common)

    meas_t, pos_t, vel_t = sequences_to_tensors(train_seqs, device)

    model = TinyReservoir(full_qat=full_qat).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=lr)

    log_every = max(1, epochs // 20)
    best_pos = float("inf")
    for epoch in range(epochs):
        perm = torch.randperm(meas_t.size(0), device=device)
        epoch_loss = 0.0
        for start in range(0, meas_t.size(0), batch_size):
            idx = perm[start : start + batch_size]
            pos_hat, vel_hat = model(meas_t[idx])
            loss = (pos_hat - pos_t[idx]).pow(2).mean() + vel_weight * (
                vel_hat - vel_t[idx]
            ).pow(2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            epoch_loss += loss.item() * idx.size(0)
        epoch_loss /= meas_t.size(0)

        if (epoch + 1) % log_every == 0 or epoch == epochs - 1:
            metrics = evaluate_hardware(model, val_seqs)
            tag = ""
            if metrics["position_rmse_hw"] < best_pos:
                best_pos = metrics["position_rmse_hw"]
                save_checkpoint(model, out_path)
                tag = "  *saved*"
            print(
                f"epoch {epoch+1:4d}  loss {epoch_loss:.5f}  "
                f"hw_pos_rmse {metrics['position_rmse_hw']:.5f}  "
                f"hw_vel_rmse {metrics['velocity_rmse_hw']:.5f}{tag}"
            )
    print(f"best hw position RMSE: {best_pos:.5f}  saved to {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-count", type=int, default=64)
    parser.add_argument("--val-count", type=int, default=8)
    parser.add_argument("--length", type=int, default=300)
    parser.add_argument("--epochs", type=int, default=400)
    parser.add_argument(
        "--batch-size",
        type=int,
        default=0,
        help="Sequences per gradient step. 0 means full-batch (=train-count), which is the fastest "
             "setting for this model: the per-timestep matmuls are tiny, so the Python loop overhead "
             "dominates and larger batches amortize it.",
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=5e-3,
        help="Higher than the typical 2e-3 because the default full-batch setting takes one "
             "gradient step per epoch instead of many mini-batch steps.",
    )
    parser.add_argument("--vel-weight", type=float, default=0.3)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--noise", type=float, default=0.08)
    parser.add_argument(
        "--full-qat",
        action="store_true",
        help="Also quantize membrane/accumulator state during training.",
    )
    parser.add_argument(
        "--device",
        choices=("auto", "cuda", "cpu"),
        default="cpu",
        help="CPU is the default and faster here: with only ~130 parameters and a serial "
             "per-timestep loop, CUDA kernel-launch overhead dominates real compute. Use cuda only "
             "if the network is scaled up.",
    )
    parser.add_argument(
        "--out", type=Path, default=REPO_ROOT / "build" / "snn_trained.pt"
    )
    args = parser.parse_args()

    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)
    batch_size = args.batch_size if args.batch_size > 0 else args.train_count
    print(f"device: {device}  batch_size: {batch_size}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    train(
        train_count=args.train_count,
        val_count=args.val_count,
        length=args.length,
        epochs=args.epochs,
        batch_size=batch_size,
        lr=args.lr,
        vel_weight=args.vel_weight,
        seed=args.seed,
        noise=args.noise,
        full_qat=args.full_qat,
        device=device,
        out_path=args.out,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
