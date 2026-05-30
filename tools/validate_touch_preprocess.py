#!/usr/bin/env python3
"""Validate the touch-demo preprocessing pipeline against the SNN simulator.

The on-device firmware (software/mnist_lcd_demo/main.c) turns a finger drawing
into a 28x28 MNIST-style tensor via: crop to ink bbox -> area-average scale of
the longest side to 20 px -> place in 28x28 so the intensity centroid is
centered -> Q4.12 encode. If that pipeline doesn't match the training
distribution, the demo misclassifies even though the core is bit-exact.

We can't capture real finger input here, so we approximate it: take real MNIST
test images, upscale each into a CANVAS_DIM "drawing" (optionally binarized,
like ink), then run the *exact same integer preprocessing the C firmware uses*
(reimplemented here) and classify with sim/snn_mlp.py (the bit-exact reference
the hardware matches). If accuracy holds up, the geometry + encoding are sound.

This is a smoke test for the pipeline, not a claim about real handwriting.

    .venv/bin/python tools/validate_touch_preprocess.py --n 500
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from sim.snn_mlp import load_checkpoint, hardware_inference_batch

CANVAS_DIM = 280
INPUT_SCALE_Q = 4096       # input_scale(1.0) << frac_bits(12); matches snn_assets.h


def render_drawing(img28: np.ndarray, binarize: bool) -> np.ndarray:
    """Fake a finger drawing: nearest-neighbour upscale 28x28 -> CANVAS_DIM ink.

    Nearest-neighbour (not smooth) is deliberately crude so the downsample step
    in the pipeline has real work to do, closer to chunky finger strokes."""
    scale = CANVAS_DIM // 28
    big = np.kron(img28, np.ones((scale, scale), dtype=img28.dtype))
    pad = CANVAS_DIM - big.shape[0]
    lo = pad // 2
    canvas = np.zeros((CANVAS_DIM, CANVAS_DIM), dtype=np.uint8)
    canvas[lo:lo + big.shape[0], lo:lo + big.shape[1]] = big
    if binarize:
        canvas = np.where(canvas > 64, 255, 0).astype(np.uint8)
    return canvas


def preprocess_c_equivalent(ink: np.ndarray) -> np.ndarray | None:
    """Bit-for-bit port of preprocess_and_load() in main.c (integer math).

    Returns a length-784 int32 array of Q4.12 pixels, or None if empty."""
    ys, xs = np.nonzero(ink)
    if xs.size == 0:
        return None
    minx, maxx = int(xs.min()), int(xs.max())
    miny, maxy = int(ys.min()), int(ys.max())
    bw, bh = maxx - minx + 1, maxy - miny + 1
    lng = max(bw, bh)
    gw = min(max((bw * 20 + lng // 2) // lng, 1), 20)
    gh = min(max((bh * 20 + lng // 2) // lng, 1), 20)

    glyph = np.zeros((20, 20), dtype=np.int32)
    for gy in range(gh):
        sy0 = miny + (gy * bh) // gh
        sy1 = miny + ((gy + 1) * bh) // gh
        if sy1 <= sy0:
            sy1 = sy0 + 1
        for gx in range(gw):
            sx0 = minx + (gx * bw) // gw
            sx1 = minx + ((gx + 1) * bw) // gw
            if sx1 <= sx0:
                sx1 = sx0 + 1
            region = ink[sy0:min(sy1, CANVAS_DIM), sx0:min(sx1, CANVAS_DIM)]
            glyph[gy, gx] = int(region.sum()) // region.size  # integer mean

    s = int(glyph[:gh, :gw].sum())
    if s == 0:
        return None
    gxs = np.arange(gw)
    gys = np.arange(gh)
    comx = int((glyph[:gh, :gw].sum(axis=0) * gxs).sum() // s)
    comy = int((glyph[:gh, :gw].sum(axis=1) * gys).sum() // s)
    ox = min(max(14 - comx, 0), 28 - gw)
    oy = min(max(14 - comy, 0), 28 - gh)

    img28 = np.zeros((28, 28), dtype=np.int32)
    img28[oy:oy + gh, ox:ox + gw] = glyph[:gh, :gw]
    # pixel_q = (g * INPUT_SCALE_Q + 127) // 255
    pix = (img28 * INPUT_SCALE_Q + 127) // 255
    return pix.reshape(-1).astype(np.int32)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", type=Path, default=REPO / "build" / "snn_mnist.pt")
    ap.add_argument("--data", type=Path, default=REPO / "build" / "mnist")
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--binarize", action="store_true",
                    help="Threshold the rendered drawing to pure ink (harsher test).")
    args = ap.parse_args()

    from torchvision import datasets, transforms
    ds = datasets.MNIST(str(args.data), train=False, download=False,
                        transform=transforms.ToTensor())

    model = load_checkpoint(args.checkpoint)

    n = min(args.n, len(ds))
    # Baseline: encode the original centered 28x28 directly (the "ideal" path,
    # what stream_snn_mnist_uart.py feeds). Tells us the pipeline's ceiling.
    base_pix, proc_pix, labels = [], [], []
    skipped = 0
    for i in range(n):
        t, label = ds[i]
        img28 = (t.numpy()[0] * 255.0).round().astype(np.uint8)
        base_pix.append((img28.astype(np.int32) * INPUT_SCALE_Q + 127) // 255)
        ink = render_drawing(img28, args.binarize)
        p = preprocess_c_equivalent(ink)
        if p is None:
            skipped += 1
            continue
        proc_pix.append(p)
        labels.append(label)

    base_arr = np.stack([b.reshape(-1) for b in base_pix[:len(labels)]])
    proc_arr = np.stack(proc_pix)
    lab = np.array(labels)

    base_pred = hardware_inference_batch(model, base_arr).argmax(axis=1)
    proc_pred = hardware_inference_batch(model, proc_arr).argmax(axis=1)

    base_acc = (base_pred == lab).mean()
    proc_acc = (proc_pred == lab).mean()
    agree = (base_pred == proc_pred).mean()

    print(f"images:                 {len(lab)}  (skipped empty: {skipped})")
    print(f"binarize:               {args.binarize}")
    print(f"baseline (direct 28x28): {base_acc*100:.2f}%   <- pipeline ceiling")
    print(f"pipeline (render->prep): {proc_acc*100:.2f}%")
    print(f"pred agreement base/pipe:{agree*100:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
