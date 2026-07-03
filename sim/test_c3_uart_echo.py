#!/usr/bin/env python3
"""Simulate the C3 UART echo loopback gateware (add_c3_uart_echo's guts).

Verifies MY gateware independent of hardware: drive a UART frame into pads.rx,
confirm the RX PHY decodes it, the FIFO passes it, and the TX PHY re-transmits
the same byte on pads.tx. If this passes, the echo logic is correct and any
hardware failure is the link / C3 side, not this gateware.

Run: .venv/bin/python sim/test_c3_uart_echo.py
"""
from migen import *
from migen.genlib.record import Record

from litex.soc.cores.uart import RS232PHY
from litex.soc.interconnect import stream

CLK_FREQ = 1600      # sim clock
BAUD     = 100       # -> 16 clocks/bit (a clean, decodable divisor)
BIT      = CLK_FREQ // BAUD


class DUT(Module):
    def __init__(self):
        self.pads = Record([("tx", 1), ("rx", 1)])
        self.submodules.phy  = phy  = RS232PHY(self.pads, CLK_FREQ, BAUD)
        self.submodules.fifo = fifo = stream.SyncFIFO([("data", 8)], 16)
        self.comb += phy.source.connect(fifo.sink)   # RX -> FIFO
        self.comb += fifo.source.connect(phy.sink)   # FIFO -> TX (echo)


def send_uart_frame(pads, byte):
    """Drive one 8N1 frame onto pads.rx, LSB first (real UART order)."""
    yield pads.rx.eq(0)                       # start bit
    for _ in range(BIT): yield
    for i in range(8):                        # 8 data bits, LSB first
        yield pads.rx.eq((byte >> i) & 1)
        for _ in range(BIT): yield
    yield pads.rx.eq(1)                        # stop bit
    for _ in range(BIT): yield


rx_seen = []
tx_seen = []
tx_low  = []


def driver(dut):
    yield dut.pads.rx.eq(1)                    # idle high
    for _ in range(BIT * 2): yield
    yield from send_uart_frame(dut.pads, 0xA5)
    for _ in range(BIT * 24): yield            # let RX->FIFO->TX complete


def monitor(dut):
    for _ in range(BIT * 28):                  # capture the whole run continuously
        if (yield dut.phy.source.valid) and (yield dut.phy.source.ready):
            rx_seen.append((yield dut.phy.source.data))
        if (yield dut.phy.sink.valid) and (yield dut.phy.sink.ready):
            tx_seen.append((yield dut.phy.sink.data))
        tx_low.append((yield dut.pads.tx))
        yield


if __name__ == "__main__":
    dut = DUT()
    run_simulation(dut, [driver(dut), monitor(dut)], vcd_name=None)

    print("RX PHY decoded bytes :", [hex(b) for b in rx_seen])
    print("TX PHY consumed bytes:", [hex(b) for b in tx_seen])
    print("TX line produced a start bit:", 0 in tx_low)

    ok = (rx_seen == [0xA5]) and (tx_seen == [0xA5])
    print("\nRESULT:", "PASS -- echo gateware is correct" if ok
          else "FAIL -- echo gateware bug")
