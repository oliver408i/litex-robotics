"""Behavioral check of gateware/sr595.py against a software 74HC595 model.

Run: .venv/bin/python -m sim.sr595

Checks, in order: the forced post-reset shift latches the initial value,
a value change re-shifts (bit order: value[7] -> Qh), and a change landing
mid-shift converges on the final value with one extra shift.
"""
from migen import Signal, Module
from migen.sim import run_simulation

from gateware.sr595 import SR595


class _Pads:
    def __init__(self):
        self.ser   = Signal()
        self.rclk  = Signal()
        self.srclk = Signal()


class _DUT(Module):
    def __init__(self):
        self.pads = _Pads()
        self.value = Signal(8)
        # div = 2 sys cycles per SRCLK half-period.
        self.submodules.sr = SR595(self.pads, sys_clk_freq=8, sclk_freq=2)
        self.comb += self.sr.value.eq(self.value)


def main():
    dut = _DUT()
    state = {"shift": 0, "latched": None, "prev_srclk": 0, "prev_rclk": 0,
             "latch_log": []}

    def hc595(pads, s):
        """One sys-clock observation step of the 74HC595 model."""
        srclk = yield pads.srclk
        rclk  = yield pads.rclk
        ser   = yield pads.ser
        if srclk and not s["prev_srclk"]:      # SRCLK rising: shift in SER
            s["shift"] = ((s["shift"] << 1) | ser) & 0xFF
        if rclk and not s["prev_rclk"]:        # RCLK rising: latch outputs
            s["latched"] = s["shift"]
            s["latch_log"].append(s["shift"])
        s["prev_srclk"] = srclk
        s["prev_rclk"]  = rclk

    def stim():
        yield dut.value.eq(0x03)               # power-on: LCD+CTP reset_n high
        for _ in range(60):                    # forced post-reset shift
            yield from hc595(dut.pads, state)
            yield
        assert state["latched"] == 0x03, hex(state["latched"] or -1)

        yield dut.value.eq(0x87)               # winc_en+winc_rst high too
        for _ in range(60):
            yield from hc595(dut.pads, state)
            yield
        assert state["latched"] == 0x87, hex(state["latched"])

        yield dut.value.eq(0x86)               # change mid-idle then mid-shift
        for _ in range(8):                     # land a second change mid-shift
            yield from hc595(dut.pads, state)
            yield
        yield dut.value.eq(0xA5)
        for _ in range(120):
            yield from hc595(dut.pads, state)
            yield
        assert state["latched"] == 0xA5, hex(state["latched"])
        assert state["latch_log"][-1] == 0xA5
        print("sr595 sim ok, latches:", [hex(v) for v in state["latch_log"]])

    run_simulation(dut, stim())


if __name__ == "__main__":
    main()
