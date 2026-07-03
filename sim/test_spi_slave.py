#!/usr/bin/env python3
"""Migen-simulation bench for gateware/spi_slave.py (SPISlave).

The core is the FPGA-slave half of the ESP32-C3 flash-loader link. This bench
plays the *master* (the C3): it drives CS#/SCLK/MOSI in SPI mode 0 and samples
MISO, then checks the firmware-facing CSR FIFOs.

Coverage:
  A  master->slave: clocked bytes land in the RX FIFO in order (MSB first)
  B  slave->master: bytes firmware pushes to the TX FIFO clock out on MISO
  C  full-duplex in one CS frame: simultaneous TX out / RX in
  D  the firmware-driven READY pad mirrors the ready CSR

Run from the repo root:
    .venv/bin/python sim/test_spi_slave.py
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LITEX_SETUP = REPO / "litex-setup"
for _p in ["litex", "litex-boards", "migen", "litedram", "litespi", "litesdcard"]:
    _cand = LITEX_SETUP / _p
    if _cand.is_dir() and str(_cand) not in sys.path:
        sys.path.insert(0, str(_cand))
sys.path.insert(0, str(REPO))

from migen import Record                                      # noqa: E402
from litex.gen.sim import run_simulation                      # noqa: E402

from gateware.spi_slave import SPISlave                       # noqa: E402

# sys cycles per SCLK half-bit. >2 so the input synchronizers settle between
# edges (SCLK must stay <= ~sys/4 in real hardware; this is the sim analogue).
HALF = 6


def _pads():
    return Record([("sclk", 1), ("mosi", 1), ("miso", 1), ("cs_n", 1), ("ready", 1)])


def idle(dut):
    """Drive the bus to its real idle state (CS# high, SCLK low) before a frame.
    A migen Record defaults every field to 0, but CS# idles HIGH in hardware."""
    yield dut.pads.cs_n.eq(1)
    yield dut.pads.sclk.eq(0)
    yield dut.pads.mosi.eq(0)
    for _ in range(4):
        yield


# --- master (the C3) -----------------------------------------------------------------------------
def spi_xfer(dut, tx_bytes):
    """Drive one CS frame clocking out tx_bytes; return the bytes read on MISO."""
    rx = []
    yield dut.pads.cs_n.eq(0)
    for _ in range(HALF):
        yield
    for b in tx_bytes:
        rb = 0
        for i in range(8):
            yield dut.pads.mosi.eq((b >> (7 - i)) & 1)
            for _ in range(HALF):          # SCLK low phase
                yield
            yield dut.pads.sclk.eq(1)       # rising edge: slave samples MOSI
            for _ in range(HALF):           # SCLK high phase
                yield
            rb = (rb << 1) | (yield dut.pads.miso)   # master samples MISO
            yield dut.pads.sclk.eq(0)       # falling edge: slave shifts MISO
        rx.append(rb)
    for _ in range(HALF):
        yield
    yield dut.pads.cs_n.eq(1)
    for _ in range(HALF):
        yield
    return rx


# --- firmware-side CSR helpers -------------------------------------------------------------------
def read_rx(dut, n):
    out = []
    for _ in range(n):
        assert not (yield dut._rxempty.status), "RX FIFO empty earlier than expected"
        out.append((yield dut._rxtx.w))     # current head
        yield dut._rxtx.re.eq(1)            # pop
        yield
        yield dut._rxtx.re.eq(0)
        yield
    return out


def push_tx(dut, vals):
    for b in vals:
        assert not (yield dut._txfull.status), "TX FIFO full"
        yield dut._rxtx.r.eq(b)
        yield dut._rxtx.we.eq(1)
        yield
        yield dut._rxtx.we.eq(0)
        yield


# --- tests ---------------------------------------------------------------------------------------
def test_rx():
    dut = SPISlave(_pads())
    sent = [0xA5, 0x3C, 0xFF, 0x00, 0x81]

    def main(dut):
        yield from idle(dut)
        yield from spi_xfer(dut, sent)
        for _ in range(8):
            yield
        got = yield from read_rx(dut, len(sent))
        assert got == sent, f"RX mismatch: {[hex(x) for x in got]} != {[hex(x) for x in sent]}"
        assert (yield dut._rxempty.status) == 1, "RX FIFO should be empty after draining"

    run_simulation(dut, [main(dut)])
    print(f"  A master->RX FIFO: {[hex(x) for x in sent]} received in order  OK")


def test_tx():
    dut = SPISlave(_pads())
    reply = [0xB7, 0x42, 0xDE]

    def main(dut):
        yield from idle(dut)
        yield from push_tx(dut, reply)
        for _ in range(4):
            yield
        got = yield from spi_xfer(dut, [0x00] * len(reply))   # dummy MOSI, read MISO
        assert got == reply, f"TX mismatch: {[hex(x) for x in got]} != {[hex(x) for x in reply]}"

    run_simulation(dut, [main(dut)])
    print(f"  B TX FIFO->master MISO: {[hex(x) for x in reply]} clocked out  OK")


def test_full_duplex():
    dut = SPISlave(_pads())
    mosi_bytes = [0x11, 0x22, 0x33]
    miso_bytes = [0xAA, 0xBB, 0xCC]

    def main(dut):
        yield from idle(dut)
        yield from push_tx(dut, miso_bytes)
        for _ in range(4):
            yield
        got = yield from spi_xfer(dut, mosi_bytes)
        assert got == miso_bytes, f"duplex MISO: {[hex(x) for x in got]} != {[hex(x) for x in miso_bytes]}"
        for _ in range(8):
            yield
        rx = yield from read_rx(dut, len(mosi_bytes))
        assert rx == mosi_bytes, f"duplex MOSI: {[hex(x) for x in rx]} != {[hex(x) for x in mosi_bytes]}"

    run_simulation(dut, [main(dut)])
    print("  C full-duplex one frame: TX out + RX in both correct  OK")


def test_ready_pad():
    dut = SPISlave(_pads())

    def main(dut):
        yield from idle(dut)
        assert (yield dut.pads.ready) == 0, "ready pad should reset low (busy)"
        yield dut._ready.storage.eq(1)
        yield
        assert (yield dut.pads.ready) == 1, "ready pad should follow ready CSR high"
        yield dut._ready.storage.eq(0)
        yield
        assert (yield dut.pads.ready) == 0, "ready pad should follow ready CSR low"

    run_simulation(dut, [main(dut)])
    print("  D READY pad mirrors the ready CSR  OK")


if __name__ == "__main__":
    print("SPISlave migen sim:")
    test_rx()
    test_tx()
    test_full_duplex()
    test_ready_pad()
    print("all SPISlave tests passed")
