"""Cocotb test for verilog/snn_mlp_core.v.

Tiny 4->2->2 network with hand-crafted weights:
    drive the same inputs into the DUT and into sim/snn_mlp.py,
    compare spike counts and the final classification.

The numpy simulator is the bit-exact reference. Any divergence is a Verilog
bug, which is exactly what this test is designed to catch.

Run via sim/cocotb/run.sh from the repo root.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge
import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO))

from sim.snn_mlp import (  # noqa: E402
    MLPConfig,
    MLPModel,
    hardware_inference,
    load_checkpoint,
    _to_fixed,
)


CLOCK_PERIOD_NS = 10
DATA_WIDTH = 16


async def reset_dut(dut) -> None:
    dut.rst.value = 1
    dut.start.value = 0
    dut.clear_state.value = 0
    dut.pixel_we.value = 0
    dut.pixel_addr.value = 0
    dut.pixel_data.value = 0
    dut.bias_we.value = 0
    dut.bias_addr.value = 0
    dut.bias_data.value = 0
    dut.w_valid.value = 0
    dut.w_data.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def write_pixels(dut, pixels_q) -> None:
    for i, v in enumerate(pixels_q):
        dut.pixel_addr.value = int(i)
        dut.pixel_data.value = int(v) & ((1 << DATA_WIDTH) - 1)
        dut.pixel_we.value = 1
        await RisingEdge(dut.clk)
    dut.pixel_we.value = 0
    dut.pixel_addr.value = 0
    dut.pixel_data.value = 0


async def write_biases(dut, biases_q) -> None:
    for i, v in enumerate(biases_q):
        dut.bias_addr.value = int(i)
        dut.bias_data.value = int(v) & ((1 << DATA_WIDTH) - 1)
        dut.bias_we.value = 1
        await RisingEdge(dut.clk)
    dut.bias_we.value = 0
    dut.bias_addr.value = 0
    dut.bias_data.value = 0


def pack_weight_stream(
    W1_q: np.ndarray,
    W2_q: np.ndarray,
    n_mac: int,
    in_size: int,
    hidden: int,
    out_size: int,
    timesteps: int,
) -> np.ndarray:
    """Build the weight beat sequence as a [n_beats, n_mac] int32 array.

    The DUT hoists the layer-1 MAC out of the timestep loop (the layer-1 current
    is time-invariant for a static image), so W1 is consumed ONCE up front and
    only the W2 block repeats per timestep. Layout matches the DUT's consumption
    order: [W1 block] + tile([W2 block], T).
    """
    L1_tiles = (hidden + n_mac - 1) // n_mac
    L2_tiles = (out_size + n_mac - 1) // n_mac

    # W1 block: consumed once (phase 1).
    w1_beats = L1_tiles * in_size
    w1_block = np.zeros((w1_beats, n_mac), dtype=np.int32)
    idx = 0
    for tile in range(L1_tiles):
        for i in range(in_size):
            for m in range(n_mac):
                n = tile * n_mac + m
                w1_block[idx, m] = int(W1_q[n, i]) if n < hidden else 0
            idx += 1

    # W2 block: consumed once per timestep (phase 2).
    w2_beats = L2_tiles * hidden
    w2_block = np.zeros((w2_beats, n_mac), dtype=np.int32)
    idx = 0
    for tile in range(L2_tiles):
        for i in range(hidden):
            for m in range(n_mac):
                n = tile * n_mac + m
                w2_block[idx, m] = int(W2_q[n, i]) if n < out_size else 0
            idx += 1

    return np.concatenate([w1_block, np.tile(w2_block, (timesteps, 1))], axis=0)


def pack_beat(beat, n_mac: int) -> int:
    mask = (1 << DATA_WIDTH) - 1
    val = 0
    for m in range(n_mac):
        val |= (int(beat[m]) & mask) << (m * DATA_WIDTH)
    return val


async def run_inference(dut, weight_stream, n_mac, max_cycles=10_000):
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    cursor = 0
    cycles = 0
    while True:
        # Drive on FallingEdge so the rising-edge NBAs have already applied
        # (combinational w_ready is settled) when we sample state. Cocotb 2.0's
        # `await RisingEdge(clk)` returns in the active region *before* NBAs
        # commit, so reads there would see the pre-edge w_ready and advance the
        # cursor on a phantom beat.
        await FallingEdge(dut.clk)

        if int(dut.done.value):
            break
        ready = bool(int(dut.w_ready.value))

        if ready and cursor < len(weight_stream):
            dut.w_data.value = pack_beat(weight_stream[cursor], n_mac)
            dut.w_valid.value = 1
            cursor += 1
        else:
            dut.w_valid.value = 0

        cycles += 1
        if cycles > max_cycles:
            raise TimeoutError(
                f"inference did not finish in {cycles} cycles; cursor={cursor}/{len(weight_stream)}"
            )

    dut.w_valid.value = 0
    return cycles


@cocotb.test()
async def tiny_network_matches_simulator(dut):
    """4->2->2 LIF MLP, hot-pixel-0 image. DUT spike counts must equal numpy sim."""
    n_mac = int(dut.N_MAC.value)
    in_size = int(dut.IN_SIZE.value)
    hidden = int(dut.HIDDEN.value)
    out_size = int(dut.OUT_SIZE.value)
    timesteps = int(dut.TIMESTEPS.value)
    frac = int(dut.FRAC_BITS.value)
    beta_shift = int(dut.BETA_SHIFT.value)
    spk_width = int(dut.SPK_WIDTH.value)

    if (in_size, hidden, out_size) != (4, 2, 2):
        dut._log.info(
            f"skipping tiny smoke test (DUT is {in_size}/{hidden}/{out_size}, not 4/2/2)"
        )
        return

    # Tiny model: pixel-i fires hidden-i; identity-ish readout.
    W1 = np.array(
        [[0.6, 0.0, 0.0, 0.0],
         [0.0, 0.6, 0.0, 0.0]],
        dtype=np.float32,
    )
    b1 = np.zeros(hidden, dtype=np.float32)
    W2 = np.array(
        [[1.5, 0.0],
         [0.0, 1.5]],
        dtype=np.float32,
    )
    b2 = np.zeros(out_size, dtype=np.float32)
    image = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    W1_q = _to_fixed(W1, frac).astype(np.int32)
    b1_q = _to_fixed(b1, frac).astype(np.int32)
    W2_q = _to_fixed(W2, frac).astype(np.int32)
    b2_q = _to_fixed(b2, frac).astype(np.int32)
    image_q = _to_fixed(image, frac).astype(np.int32)

    cfg = MLPConfig(
        in_size=in_size,
        hidden=hidden,
        out_size=out_size,
        beta_shift=beta_shift,
        timesteps=timesteps,
        frac_bits=frac,
    )
    model = MLPModel(config=cfg, W1_q=W1_q, b1_q=b1_q, W2_q=W2_q, b2_q=b2_q)
    expected_spk = hardware_inference(model, image_q)
    expected_class = int(np.argmax(expected_spk))
    dut._log.info(
        f"numpy sim spike counts: {expected_spk.tolist()}, expected class={expected_class}"
    )

    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)

    await write_pixels(dut, image_q)
    await write_biases(dut, list(b1_q) + list(b2_q))

    stream = pack_weight_stream(
        W1_q, W2_q, n_mac, in_size, hidden, out_size, timesteps
    )
    dut._log.info(f"weight stream: {len(stream)} beats; n_mac={n_mac}")

    cycles = await run_inference(dut, stream, n_mac)
    classification = int(dut.classification.value)
    spike_counts_packed = int(dut.spike_counts_packed.value)
    dut_spk = [
        (spike_counts_packed >> (i * spk_width)) & ((1 << spk_width) - 1)
        for i in range(out_size)
    ]
    dut._log.info(
        f"DUT spk_count: {dut_spk}, classification={classification}, cycles={cycles}"
    )

    assert dut_spk == expected_spk.tolist(), (
        f"spike counts diverged: DUT={dut_spk}, ref={expected_spk.tolist()}"
    )
    assert classification == expected_class, (
        f"classification diverged: DUT={classification}, ref={expected_class}"
    )


@cocotb.test()
async def mnist_matches_simulator(dut):
    """Real MNIST checkpoint through the DUT. Each image must produce the same
    spike count vector and classification as sim/snn_mlp.py.

    Skipped unless the DUT is built with the MNIST shape (784/64/10).
    Override `MNIST_IMAGES` env var to change the number of images (default 3).
    """
    n_mac = int(dut.N_MAC.value)
    in_size = int(dut.IN_SIZE.value)
    hidden = int(dut.HIDDEN.value)
    out_size = int(dut.OUT_SIZE.value)
    timesteps = int(dut.TIMESTEPS.value)
    frac = int(dut.FRAC_BITS.value)
    spk_width = int(dut.SPK_WIDTH.value)

    if (in_size, hidden, out_size) != (784, 64, 10):
        dut._log.info(
            f"skipping MNIST test (DUT is {in_size}/{hidden}/{out_size}, not 784/64/10)"
        )
        return

    ckpt_path = REPO / "build" / "snn_mnist.pt"
    if not ckpt_path.exists():
        raise FileNotFoundError(
            f"no checkpoint at {ckpt_path}; "
            f"run `.venv/bin/python tools/train_snn_mnist.py` first."
        )
    model = load_checkpoint(ckpt_path)
    cfg = model.config
    assert cfg.timesteps == timesteps, (
        f"checkpoint T={cfg.timesteps} != DUT T={timesteps}"
    )
    assert cfg.frac_bits == frac, (
        f"checkpoint frac_bits={cfg.frac_bits} != DUT FRAC_BITS={frac}"
    )

    n_images = int(os.environ.get("MNIST_IMAGES", "3"))
    dut._log.info(
        f"MNIST: {n_images} images, shape {in_size}->{hidden}->{out_size} T={timesteps}"
    )

    from torchvision import datasets, transforms

    test_ds = datasets.MNIST(
        REPO / "build" / "mnist",
        train=False,
        download=False,
        transform=transforms.ToTensor(),
    )

    biases_q = list(model.b1_q) + list(model.b2_q)
    stream = pack_weight_stream(
        model.W1_q, model.W2_q, n_mac, in_size, hidden, out_size, timesteps
    )
    dut._log.info(f"weight stream: {stream.shape[0]} beats, {stream.nbytes/1e6:.1f} MB")

    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)
    await write_biases(dut, biases_q)

    cycles_total = 0
    for i in range(n_images):
        img, label = test_ds[i]
        image_q = _to_fixed(
            img.view(-1).numpy() * cfg.input_scale, cfg.frac_bits
        ).astype(np.int32)

        ref_spk = hardware_inference(model, image_q)
        ref_class = int(np.argmax(ref_spk))

        # Clear membrane / spike state between images (start also resets spk_count).
        dut.clear_state.value = 1
        await RisingEdge(dut.clk)
        dut.clear_state.value = 0

        await write_pixels(dut, image_q)
        cycles = await run_inference(dut, stream, n_mac, max_cycles=5_000_000)
        cycles_total += cycles

        classification = int(dut.classification.value)
        spike_counts_packed = int(dut.spike_counts_packed.value)
        dut_spk = [
            (spike_counts_packed >> (j * spk_width)) & ((1 << spk_width) - 1)
            for j in range(out_size)
        ]

        match = dut_spk == ref_spk.tolist()
        dut._log.info(
            f"img {i:3d}  label={label}  ref_class={ref_class}  "
            f"dut_class={classification}  spk_match={match}  cycles={cycles}"
        )
        assert classification == ref_class, (
            f"img {i}: DUT class {classification} != ref {ref_class}"
        )
        assert dut_spk == ref_spk.tolist(), (
            f"img {i}: DUT spk {dut_spk} != ref {ref_spk.tolist()}"
        )

    dut._log.info(
        f"MNIST: all {n_images} images matched bit-exactly  "
        f"(avg {cycles_total / n_images:.0f} cycles/image)"
    )
