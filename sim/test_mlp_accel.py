#!/usr/bin/env python3
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from migen import *
from migen.sim import run_simulation

from gateware.mlp_accel import MLP2Accel


def _sign(val, bits):
    mask = (1 << bits) - 1
    val &= mask
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val


def _write_csr(dut, csr, value):
    yield csr.storage.eq(value)
    yield csr.re.eq(1)
    yield
    yield csr.re.eq(0)
    yield


def _read_csr(dut, csr):
    yield
    return (yield csr.status)


def tb(dut, in_size, hidden_size, out_size):
    inputs = [3, -2, 1, 4, -1, 2, 0, -3, 1, 1, -2, 2, -1, 0, 3, -2]

    w1 = [
        [1, -1, 0, 2, -2, 1, 1, 0, 1, -1, 2, 0, 0, 1, -1, 1],
        [0, 2, -1, 1, 1, -2, 0, 1, -1, 0, 1, 2, -2, 1, 0, -1],
        [2, 0, 1, -1, 0, 1, -2, 2, 1, 0, -1, 1, 2, -1, 0, 1],
        [-1, 1, 2, 0, 1, 0, -1, 1, 0, 2, 1, -2, 1, 0, -1, 2],
        [1, 0, -2, 1, 2, -1, 0, 1, -1, 1, 0, 2, -2, 1, 0, -1],
        [0, 1, 1, -2, 1, 0, 2, -1, 0, 1, -1, 2, 1, 0, -2, 1],
        [2, -1, 0, 1, -1, 2, 1, 0, 1, -2, 0, 1, 2, -1, 0, 1],
        [-1, 0, 2, 1, 0, -1, 1, 2, -2, 1, 0, 1, -1, 2, 1, 0],
    ]
    b1 = [1, -1, 0, 2, -2, 1, 0, -1]

    w2 = [
        [1, -1, 2, 0, 1, 0, -1, 1],
        [0, 2, -1, 1, -2, 1, 0, -1],
        [1, 0, 1, -1, 0, 1, -2, 2],
        [-1, 1, 0, 2, 1, -2, 1, 0],
    ]
    b2 = [0, 1, -1, 2]

    print(f"MLP2Accel sim: in={in_size} hidden={hidden_size} out={out_size} macs_per_cycle=2")
    print(f"Inputs:  {inputs}")
    print(f"L1 Bias: {b1}")
    print(f"L2 Bias: {b2}")

    # Load inputs.
    for i, v in enumerate(inputs):
        yield from _write_csr(dut, dut._in_addr, i)
        yield from _write_csr(dut, dut._in_data, v & 0xFF)
        yield from _write_csr(dut, dut._in_we, 1)
        print(f"Loaded IN[{i}]={v}")

    # Load L1 weights.
    for h in range(hidden_size):
        for i in range(in_size):
            addr = ((h & 0xFF) << 8) | (i & 0xFF)
            yield from _write_csr(dut, dut._w1_addr, addr)
            yield from _write_csr(dut, dut._w1_data, w1[h][i] & 0xFF)
            yield from _write_csr(dut, dut._w1_we, 1)
            print(f"Loaded W1[{h}][{i}]={w1[h][i]}")

    # Load L1 bias.
    for h, v in enumerate(b1):
        yield from _write_csr(dut, dut._b1_addr, h)
        yield from _write_csr(dut, dut._b1_data, v & 0xFFFF)
        yield from _write_csr(dut, dut._b1_we, 1)
        print(f"Loaded B1[{h}]={v}")

    # Load L2 weights.
    for o in range(out_size):
        for h in range(hidden_size):
            addr = ((o & 0xFF) << 8) | (h & 0xFF)
            yield from _write_csr(dut, dut._w2_addr, addr)
            yield from _write_csr(dut, dut._w2_data, w2[o][h] & 0xFF)
            yield from _write_csr(dut, dut._w2_we, 1)
            print(f"Loaded W2[{o}][{h}]={w2[o][h]}")

    # Load L2 bias.
    for o, v in enumerate(b2):
        yield from _write_csr(dut, dut._b2_addr, o)
        yield from _write_csr(dut, dut._b2_data, v & 0xFFFF)
        yield from _write_csr(dut, dut._b2_we, 1)
        print(f"Loaded B2[{o}]={v}")

    # Start.
    yield from _write_csr(dut, dut._ctrl, 0x1)
    print("Start asserted")

    # Wait for done.
    for cycle in range(400):
        status = (yield dut._status.status)
        done = (status >> 1) & 0x1
        busy = status & 0x1
        if cycle % 20 == 0:
            print(f"  cycle={cycle} busy={busy} done={done}")
        if done:
            break
        yield
    else:
        raise RuntimeError("Timeout waiting for done")

    # Software reference.
    hidden = []
    for h in range(hidden_size):
        acc = b1[h]
        for i in range(in_size):
            acc += inputs[i] * w1[h][i]
        hidden.append(max(0, acc) & 0xFFFF)

    expected = []
    for o in range(out_size):
        acc = b2[o]
        for h in range(hidden_size):
            acc += _sign(hidden[h], 16) * w2[o][h]
        expected.append(max(0, acc))

    # Read outputs and check.
    for o in range(out_size):
        yield from _write_csr(dut, dut._out_addr, o)
        raw = yield from _read_csr(dut, dut._out_data)
        got = _sign(raw, 32)
        exp = expected[o]
        print(f"OUT[{o}] got={got} expected={exp}")
        if got != exp:
            raise AssertionError(f"out[{o}] got {got} expected {exp}")

    print("MLP2Accel sim OK")


if __name__ == "__main__":
    in_size = 16
    hidden_size = 8
    out_size = 4
    dut = MLP2Accel(in_size=in_size, hidden_size=hidden_size, out_size=out_size, macs_per_cycle=2)
    run_simulation(dut, tb(dut, in_size, hidden_size, out_size), vcd_name="sim/build/mlp_accel.vcd")
