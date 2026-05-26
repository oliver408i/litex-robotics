"""Cocotb test for verilog/snn_weight_loader.v.

Mocks a Wishbone-classic slave (Python dict {byte_addr -> word}) and verifies
that the loader's emitted w_valid/w_data stream matches the expected sequence,
including wrap-around for num_cycles > 1 and downstream backpressure.

Run via sim/cocotb/run.sh TARGET=loader from the repo root.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge


REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO))

CLOCK_PERIOD_NS = 10


async def reset_dut(dut) -> None:
    dut.rst.value = 1
    dut.start.value = 0
    dut.base_addr.value = 0
    dut.beats_per_cycle.value = 0
    dut.num_cycles.value = 0
    dut.w_ready.value = 0
    dut.wb_ack.value = 0
    dut.wb_dat_r.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst.value = 0
    await RisingEdge(dut.clk)


async def wishbone_slave(dut, memory: dict[int, int]) -> None:
    """Single-cycle-ack Wishbone-classic read slave.

    Drives wb_ack high for one cycle after seeing cyc&stb (read), with
    wb_dat_r = memory[wb_adr]. memory addresses are byte-aligned to the bus
    width as far as the loader is concerned.
    """
    while True:
        await FallingEdge(dut.clk)
        cyc = bool(int(dut.wb_cyc.value))
        stb = bool(int(dut.wb_stb.value))
        we = bool(int(dut.wb_we.value))
        ack_was = bool(int(dut.wb_ack.value))

        if ack_was:
            # We acked last cycle; deassert now.
            dut.wb_ack.value = 0
        elif cyc and stb and not we:
            adr = int(dut.wb_adr.value)
            dut.wb_dat_r.value = memory.get(adr, 0)
            dut.wb_ack.value = 1


async def stream_consumer(
    dut,
    captured: list[int],
    n_total: int,
    backpressure_every: int = 0,
) -> None:
    """Consume beats from the loader. Optional backpressure: drop w_ready every Nth cycle."""
    iteration = 0
    while len(captured) < n_total:
        await FallingEdge(dut.clk)
        # Backpressure based on iteration count (not accepted) so the loader
        # never sees w_ready stuck low before producing the first beat.
        stall = backpressure_every > 0 and (iteration % backpressure_every) == (backpressure_every - 1)
        dut.w_ready.value = 0 if stall else 1
        await RisingEdge(dut.clk)
        if bool(int(dut.w_valid.value)) and bool(int(dut.w_ready.value)):
            captured.append(int(dut.w_data.value))
        iteration += 1
    await FallingEdge(dut.clk)
    dut.w_ready.value = 0


async def _drive_loader(dut, memory, base_addr, beats_per_cycle, num_cycles, backpressure=0):
    """Drive a single loader run and return (captured_beats, total_cycles, expected)."""
    wb_data_width = int(dut.WB_DATA_WIDTH.value)
    wb_bytes = wb_data_width // 8
    n_mac = int(dut.N_MAC.value)
    data_width = int(dut.DATA_WIDTH.value)
    beat_mask = (1 << (n_mac * data_width)) - 1

    expected = []
    for i in range(beats_per_cycle):
        word = memory.get(base_addr + i * wb_bytes, 0)
        expected.append(word & beat_mask)
    expected = expected * num_cycles

    captured: list[int] = []
    consumer = cocotb.start_soon(
        stream_consumer(dut, captured, len(expected), backpressure_every=backpressure)
    )

    dut.base_addr.value = base_addr
    dut.beats_per_cycle.value = beats_per_cycle
    dut.num_cycles.value = num_cycles
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    timeout = 1_000_000
    cycles = 0
    while True:
        await RisingEdge(dut.clk)
        if int(dut.done.value):
            break
        cycles += 1
        if cycles > timeout:
            raise TimeoutError(
                f"loader did not finish in {cycles} cycles "
                f"(captured {len(captured)}/{len(expected)})"
            )

    await consumer
    return captured, cycles, expected


@cocotb.test()
async def loader_streams_expected_sequence(dut):
    """Sequential reads from a small ROM, replayed num_cycles times."""
    wb_data_width = int(dut.WB_DATA_WIDTH.value)
    wb_bytes = wb_data_width // 8

    beats_per_cycle = 12
    num_cycles = 3
    base_addr = 0x1000

    memory = {
        base_addr + i * wb_bytes: (0x4000 + i) & ((1 << wb_data_width) - 1)
        for i in range(beats_per_cycle)
    }

    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)
    cocotb.start_soon(wishbone_slave(dut, memory))

    captured, cycles, expected = await _drive_loader(
        dut, memory, base_addr, beats_per_cycle, num_cycles
    )
    dut._log.info(
        f"sequential run: captured {len(captured)} beats in {cycles} cycles"
    )
    assert captured == expected, (
        f"mismatch: got {captured[:6]}... expected {expected[:6]}..."
    )


@cocotb.test()
async def loader_handles_backpressure(dut):
    """Same as above but the consumer stalls w_ready every other beat."""
    wb_data_width = int(dut.WB_DATA_WIDTH.value)
    wb_bytes = wb_data_width // 8

    beats_per_cycle = 8
    num_cycles = 2
    base_addr = 0x2000
    memory = {
        base_addr + i * wb_bytes: (0xa000 + i * 3) & ((1 << wb_data_width) - 1)
        for i in range(beats_per_cycle)
    }

    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, units="ns").start())
    await reset_dut(dut)
    cocotb.start_soon(wishbone_slave(dut, memory))

    captured, cycles, expected = await _drive_loader(
        dut, memory, base_addr, beats_per_cycle, num_cycles, backpressure=2
    )
    dut._log.info(
        f"backpressure run: captured {len(captured)} beats in {cycles} cycles"
    )
    assert captured == expected, (
        f"mismatch under backpressure: got {captured[:6]}... expected {expected[:6]}..."
    )
