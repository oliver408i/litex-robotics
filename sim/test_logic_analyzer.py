#!/usr/bin/env python3
"""Migen-simulation bench for gateware/logic_analyzer.py (LogicAnalyzer).

The capture core is a migen/LiteX module (it wraps LiteDRAMDMAWriter), so it is
exercised with migen's own simulator + a behavioral DRAM write-port model --
the same approach as litedram/test/test_dma.py -- NOT the iverilog/cocotb flow
the hand-written Verilog cores use.

A passive DRAM model services the native write port into a Python list (the
"SDRAM"); a monitor records the value at every sample strobe (the ground truth
for what should land in the ring, in write order). Each test arms a capture,
drives the probe bus, runs to `done`, and checks the ring + status registers.

Coverage:
  A  immediate trigger, exact ring contents, no wrap, write count + trig_addr
  B  pattern/edge trigger placement with ring wrap (pre-trigger window)
  C  overrun latches when the sample rate outruns the DRAM model
  D  LCD-present guard (on-module pull-up sense)

Run from the repo root:
    .venv/bin/python sim/test_logic_analyzer.py
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

from migen import Module, Signal                              # noqa: E402
from litex.gen.sim import run_simulation, passive             # noqa: E402
from litedram.common import LiteDRAMNativeWritePort           # noqa: E402

from gateware.logic_analyzer import LogicAnalyzer             # noqa: E402

N_CH       = 8
DATA_WIDTH = 32
GUARD_MASK = (1 << 2) | (1 << 5)   # arbitrary two-channel guard for the bench


# --- behavioral SDRAM write-port model (passive) --------------------------------------------------
class DRAMWriteModel:
    """Minimal LiteDRAM native write-port slave -> Python list. Adapted from
    litedram/test/common.py DRAMMemory.write_handler. Multi-cycle latency, so
    it naturally caps throughput (which is what the overrun test relies on)."""
    def __init__(self, depth):
        self.mem = [0] * depth

    @passive
    def handler(self, port):
        address = 0
        pending = 0
        yield port.cmd.ready.eq(0)
        while True:
            yield port.wdata.ready.eq(0)
            if pending:
                while (yield port.wdata.valid) == 0:
                    yield
                yield port.wdata.ready.eq(1)
                yield
                self.mem[address % len(self.mem)] = (yield port.wdata.data)
                yield port.wdata.ready.eq(0)
                yield
                pending = 0
                yield
            elif (yield port.cmd.valid):
                pending = (yield port.cmd.we)
                address = (yield port.cmd.addr)
                if pending:
                    yield port.cmd.ready.eq(1)
                    yield
                    yield port.cmd.ready.eq(0)
            yield


class _DUT(Module):
    def __init__(self, ring_size, fifo_depth=8):
        self.port  = LiteDRAMNativeWritePort(address_width=32, data_width=DATA_WIDTH)
        self.probe = Signal(N_CH)
        self.submodules.la = LogicAnalyzer(self.port, self.probe, n_channels=N_CH,
                                           guard_mask=GUARD_MASK, fifo_depth=fifo_depth)
        self.ring_size = ring_size


# --- sim helpers ----------------------------------------------------------------------------------
def _fld(dut, name):
    return getattr(dut.la._status.fields, name)


def configure(dut, sample_div, ring_size, post_trig,
              trig_mask=0, trig_value=0, trig_edge=0, ring_base=0):
    yield dut.la._sample_div.storage.eq(sample_div)
    yield dut.la._ring_base.storage.eq(ring_base)
    yield dut.la._ring_size.storage.eq(ring_size)
    yield dut.la._post_trig.storage.eq(post_trig)
    yield dut.la._trig_mask.storage.eq(trig_mask)
    yield dut.la._trig_value.storage.eq(trig_value)
    yield dut.la._trig_edge.storage.eq(trig_edge)
    yield


def arm(dut):
    yield dut.la._arm.re.eq(1)
    yield
    yield dut.la._arm.re.eq(1)   # CSR re is 1-cycle; held over the eval edge
    yield dut.la._arm.re.eq(0)
    yield


def run_until_done(dut, recorded, max_cycles, drive_probe, flush_cycles=300):
    """Drive the probe each cycle, record value at each sample strobe, stop at done.

    After `done` we run `flush_cycles` more so the last samples -- which `done`
    only means were accepted into the DMA input FIFO -- finish draining through
    the DMA and into the DRAM model before the caller inspects memory.
    """
    for c in range(max_cycles):
        yield dut.probe.eq(drive_probe(c) & ((1 << N_CH) - 1))
        if (yield dut.la.sample_stb):
            recorded.append((yield dut.la._probe_live.status))
        if (yield _fld(dut, "done")):
            for _ in range(flush_cycles):
                yield
            return True
        yield
    return False


# --- tests ----------------------------------------------------------------------------------------
def test_immediate_no_wrap():
    """A: trig_mask=0 -> immediate trigger; ring holds exactly post_trig+2 samples."""
    POST, RING, DIV = 10, 64, 8
    dut = _DUT(ring_size=RING)
    model = DRAMWriteModel(depth=RING)
    rec = []

    def main(dut):
        yield from configure(dut, sample_div=DIV, ring_size=RING, post_trig=POST, trig_mask=0)
        yield from arm(dut)
        done = yield from run_until_done(dut, rec, max_cycles=4000,
                                         drive_probe=lambda c: c)
        assert done, "capture never reached done"
        assert (yield _fld(dut, "triggered")) == 1
        assert (yield _fld(dut, "wrapped")) == 0, "should not wrap with RING >> writes"
        assert (yield _fld(dut, "overrun")) == 0, "decimated enough to never overrun"
        n = yield dut.la._wr_count.status
        ta = yield dut.la._trig_addr.status
        assert n == POST + 2, f"wr_count {n} != {POST+2}"
        assert ta == 0, f"immediate trigger should fire at slot 0, got {ta}"
        # ring slots 0..n-1 must equal the recorded strobe values in order
        for i in range(n):
            assert model.mem[i] == rec[i], f"slot {i}: mem {model.mem[i]} != rec {rec[i]}"

    run_simulation(dut, [main(dut), model.handler(dut.port)])
    print(f"  A immediate/no-wrap: {POST+2} samples, trig@0, ring matches  OK")


def test_pattern_edge_trigger_wrap():
    """B: edge trigger on a single bit; verify trigger placement + wrap."""
    POST, RING, DIV = 6, 16, 6
    BIT = 1 << 4
    RAISE = 200   # hold probe=0 well past one full ring of strobes, then assert BIT
    dut = _DUT(ring_size=RING)
    model = DRAMWriteModel(depth=RING)
    rec = []

    def main(dut):
        yield from configure(dut, sample_div=DIV, ring_size=RING, post_trig=POST,
                             trig_mask=BIT, trig_value=BIT, trig_edge=1)
        yield from arm(dut)
        # 0 until RAISE (fills + wraps the ring with no match), then a clean 0->1 edge.
        done = yield from run_until_done(dut, rec, max_cycles=8000,
                                         drive_probe=lambda c: BIT if c >= RAISE else 0)
        assert done, "capture never reached done"
        assert (yield _fld(dut, "triggered")) == 1
        assert (yield _fld(dut, "wrapped")) == 1, "pre-trigger fill should wrap the ring"
        ta = yield dut.la._trig_addr.status
        # the sample stored at the trigger slot must itself satisfy the trigger
        assert model.mem[ta] & BIT, f"trigger slot {ta} value {model.mem[ta]:#x} lacks BIT"
        # and it must be a 0->1 edge: the chronologically previous sample lacked BIT.
        prev = (ta - 1) % RING
        assert not (model.mem[prev] & BIT), \
            f"edge trigger: prev slot {prev} value {model.mem[prev]:#x} already had BIT"

    run_simulation(dut, [main(dut), model.handler(dut.port)])
    print("  B pattern/edge trigger + wrap: trigger correctly placed on 0->1 edge  OK")


def test_overrun():
    """C: full-rate sampling (div=0) outruns the multi-cycle DRAM model -> overrun."""
    dut = _DUT(ring_size=32, fifo_depth=4)
    model = DRAMWriteModel(depth=32)

    def main(dut):
        yield from configure(dut, sample_div=0, ring_size=32, post_trig=10_000, trig_mask=0)
        yield from arm(dut)
        seen = False
        for c in range(3000):
            yield dut.probe.eq(c & ((1 << N_CH) - 1))
            if (yield _fld(dut, "overrun")):
                seen = True
                break
            yield
        assert seen, "overrun never latched despite full-rate sampling"

    run_simulation(dut, [main(dut), model.handler(dut.port)])
    print("  C overrun latches at full rate vs slow DRAM  OK")


def test_lcd_guard():
    """D: lcd_present reflects the guard-masked probe lines (on-module pull-ups)."""
    dut = _DUT(ring_size=16)
    model = DRAMWriteModel(depth=16)

    def main(dut):
        yield dut.probe.eq(0)
        for _ in range(4):
            yield
        assert (yield _fld(dut, "lcd_present")) == 0, "no guard lines high -> absent"
        yield dut.probe.eq(GUARD_MASK)      # both guard lines pulled high == module attached
        for _ in range(4):
            yield
        assert (yield _fld(dut, "lcd_present")) == 1, "guard lines high -> present"
        yield dut.probe.eq(1 << 2)          # only one guard line: still "present"
        for _ in range(4):
            yield
        assert (yield _fld(dut, "lcd_present")) == 1

    run_simulation(dut, [main(dut), model.handler(dut.port)])
    print("  D LCD-present guard tracks the touch pull-up lines  OK")


if __name__ == "__main__":
    print("LogicAnalyzer migen-sim bench:")
    test_lcd_guard()
    test_immediate_no_wrap()
    test_pattern_edge_trigger_wrap()
    test_overrun()
    print("ALL PASS")
