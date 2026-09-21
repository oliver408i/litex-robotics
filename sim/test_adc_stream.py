#!/usr/bin/env python3
"""Migen-simulation bench for gateware/adc_stream.py.

Same approach as sim/test_logic_analyzer.py: migen's own simulator plus a
behavioural model of the thing on the other end of the wires -- here an
MCP3008 rather than a DRAM port.

The model is the point of this file. On real hardware every grounded channel
reads ~0 whichever way the 10 bits are shifted, so the bring-up test in
software/mcp3008_test/ cannot tell a correct sampler from one that is off by a
bit or two. A model that returns a known awkward value (0x2A5 = 0101001001,
deliberately not a palindrome and not near a rail) turns bit alignment into a
pass/fail here, before the bitstream is built.

Coverage:
  A  bit alignment + the config byte the sampler actually transmits
  B  hardware timing: the conversion interval equals `period`, exactly
  C  packet framing: header fields, payload contents, `last` placement
  D  drop accounting when the UDP side backpressures

Run from the repo root:
    .venv/bin/python sim/test_adc_stream.py
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LITEX_SETUP = REPO / "litex-setup"
for _p in ["litex", "litex-boards", "migen", "litedram", "liteeth", "litespi", "litesdcard"]:
    _cand = LITEX_SETUP / _p
    if _cand.is_dir() and str(_cand) not in sys.path:
        sys.path.insert(0, str(_cand))
sys.path.insert(0, str(REPO))

from migen import Signal, Module                                   # noqa: E402
from litex.gen.sim import run_simulation, passive                  # noqa: E402

from gateware.adc_stream import (                                  # noqa: E402
    MCP3008Sampler, ADCStreamer, MAGIC, VERSION, HEADER_WORDS,
)

SCLK_DIV = 4          # keep the bench short; the real build uses 24
TEST_VALUE = 0x2A5    # 677, an awkward bit pattern on purpose


class FakePads:
    """Just the four wires, as a namespace. Not a Module -- nothing to elaborate."""
    def __init__(self):
        self.sclk = Signal()
        self.cs_n = Signal(reset=1)
        self.mosi = Signal()
        self.miso = Signal()


@passive
def mcp3008_model(pads, state):
    """Behavioural MCP3008, single-ended mode.

    Counts SCLK rising edges while CS is low -- those are the master's sample
    points -- and presents the next bit right after each one, which gives the
    master a full low half-period of setup. Received bit 14 through 23 carry
    the result MSB-first, which is what puts it in the low 10 bits of a 24-bit
    shift register.

    Records every config byte the master sends, so the test can check the
    channel was actually addressed rather than just that *some* number came
    back.
    """
    prev_sclk = 0
    prev_cs   = 1
    idx       = -1
    mosi_bits = []

    while True:
        cs   = yield pads.cs_n
        sclk = yield pads.sclk

        if prev_cs == 1 and cs == 0:            # start of transaction
            idx = -1
            mosi_bits = []
            yield pads.miso.eq(0)

        if cs == 0 and prev_sclk == 0 and sclk == 1:
            idx += 1
            mosi_bits.append((yield pads.mosi))
            nxt = idx + 1
            if 14 <= nxt <= 23:
                bit = (state["value"] >> (23 - nxt)) & 1
            else:
                bit = 0
            yield pads.miso.eq(bit)

        if prev_cs == 0 and cs == 1 and len(mosi_bits) == 24:
            word = 0
            for b in mosi_bits:
                word = (word << 1) | b
            state["tx_words"].append(word)

        prev_sclk = sclk
        prev_cs   = cs
        yield


# --- A + B: the sampler ---------------------------------------------------------------------------
def test_sampler_alignment_and_timing():
    pads  = FakePads()
    dut   = MCP3008Sampler(pads, sclk_div=SCLK_DIV)
    state = {"value": TEST_VALUE, "tx_words": []}
    got   = []
    stbs  = []

    def main():
        yield dut.channel.eq(5)
        yield dut.period.eq(400)
        yield dut.enable.eq(1)
        for cycle in range(4000):
            if (yield dut.sample_stb):
                got.append((yield dut.sample_data))
                stbs.append(cycle)
            yield

    run_simulation(dut, [main(), mcp3008_model(pads, state)])

    assert len(got) >= 3, f"expected several conversions, got {len(got)}"
    assert all(v == TEST_VALUE for v in got), \
        f"bit alignment wrong: got {[hex(v) for v in got]}, expected {hex(TEST_VALUE)}"

    # B: the interval between conversions is the programmed period, exactly.
    intervals = [b - a for a, b in zip(stbs, stbs[1:])]
    assert all(i == 400 for i in intervals), \
        f"sample interval jitters: {intervals} (must all be 400)"

    # A: and the master addressed channel 5 single-ended.
    assert state["tx_words"], "model saw no complete transaction"
    for w in state["tx_words"]:
        assert (w >> 16) & 0xff == 0x01, f"start byte wrong: {w:#08x}"
        cfg = (w >> 8) & 0xff
        assert cfg == 0xd0, f"config byte {cfg:#04x} != 0xd0 (single-ended, ch5)"

    print(f"A/B ok: {len(got)} conversions, all {hex(TEST_VALUE)}, interval exactly 400 clocks")


# --- C: packet framing ----------------------------------------------------------------------------
def collect_packets(dut, n_words, ready=True):
    """Drain dut.source into a flat word list, recording where `last` fell."""
    words, lasts = [], []

    def drain():
        yield dut.source.ready.eq(1 if ready else 0)
        for _ in range(n_words):
            yield
            if (yield dut.source.valid) and (yield dut.source.ready):
                words.append((yield dut.source.data))
                lasts.append((yield dut.source.last))
    return words, lasts, drain


def test_packet_framing():
    pads  = FakePads()
    spp   = 8
    dut   = ADCStreamer(pads, sys_clk_freq=48e6, sclk_div=SCLK_DIV,
                        samples_per_packet=spp, sample_rate=20e3, fifo_depth=16)
    state = {"value": TEST_VALUE, "tx_words": []}
    words, lasts = [], []

    def main():
        yield dut._channel.storage.eq(3)
        yield dut._period.storage.eq(300)
        yield dut._enable.storage.eq(1)
        yield dut.source.ready.eq(1)
        for _ in range(12000):
            if (yield dut.source.valid) and (yield dut.source.ready):
                words.append((yield dut.source.data))
                lasts.append((yield dut.source.last))
            yield

    run_simulation(dut, [main(), mcp3008_model(pads, state)])

    per_packet = HEADER_WORDS + spp // 2
    assert len(words) >= 2 * per_packet, \
        f"expected at least two packets ({2*per_packet} words), got {len(words)}"

    for pkt in range(2):
        base = pkt * per_packet
        w = words[base:base + per_packet]
        l = lasts[base:base + per_packet]

        assert w[0] == MAGIC, f"packet {pkt}: magic {w[0]:#010x} != {MAGIC:#010x}"
        assert w[1] == pkt, f"packet {pkt}: seq {w[1]} != {pkt}"
        assert w[2] == pkt * spp, f"packet {pkt}: emitted {w[2]} != {pkt*spp}"
        assert w[3] == 0, f"packet {pkt}: dropped {w[3]} != 0 on an un-backpressured run"
        assert w[4] & 0xffff == spp, f"packet {pkt}: n_samples {w[4] & 0xffff} != {spp}"
        assert (w[4] >> 16) & 0x7 == 3, f"packet {pkt}: channel {(w[4]>>16)&7} != 3"
        assert (w[4] >> 24) & 0xff == VERSION, f"packet {pkt}: version"
        assert w[5] == 300, f"packet {pkt}: period {w[5]} != 300"
        assert w[6] == 48000000, f"packet {pkt}: sys_clk_freq {w[6]}"
        assert w[7] == 0, f"packet {pkt}: reserved"

        # `last` must fall on the final word and nowhere else -- a misplaced
        # last splits or merges datagrams and desynchronises the host.
        assert l[-1] == 1, f"packet {pkt}: no last on the final word"
        assert not any(l[:-1]), f"packet {pkt}: last asserted mid-packet at {l.index(1)}"

        for i, word in enumerate(w[HEADER_WORDS:]):
            lo, hi = word & 0xffff, (word >> 16) & 0xffff
            assert lo == TEST_VALUE and hi == TEST_VALUE, \
                f"packet {pkt} word {i}: {lo:#x}/{hi:#x} != {TEST_VALUE:#x}"

    print(f"C ok: {len(words)//per_packet} packets, headers and payload exact, last placed correctly")


# --- D: drop accounting ---------------------------------------------------------------------------
def test_drop_accounting():
    """Hold the UDP side off, overflow the FIFO, and check the loss is counted.

    A stalled link must show up as a `dropped` count, never as a quiet stretch
    of waveform.
    """
    pads  = FakePads()
    spp   = 8
    dut   = ADCStreamer(pads, sys_clk_freq=48e6, sclk_div=SCLK_DIV,
                        samples_per_packet=spp, sample_rate=20e3, fifo_depth=8)
    state = {"value": TEST_VALUE, "tx_words": []}
    result = {}

    def main():
        yield dut._period.storage.eq(200)
        yield dut._enable.storage.eq(1)
        yield dut.source.ready.eq(0)          # UDP side wedged
        for _ in range(12000):
            yield
        result["dropped"] = (yield dut._dropped.status)
        result["overruns"] = (yield dut._overruns.status)

    run_simulation(dut, [main(), mcp3008_model(pads, state)])

    assert result["dropped"] > 0, "FIFO overflowed but dropped stayed 0 -- loss is invisible"
    assert result["dropped"] % 2 == 0, "drops are counted in sample pairs"
    assert result["overruns"] == 0, \
        f"unexpected overruns ({result['overruns']}): period 200 should outlast a conversion"
    print(f"D ok: {result['dropped']} samples dropped and counted, no false overruns")


if __name__ == "__main__":
    test_sampler_alignment_and_timing()
    test_packet_framing()
    test_drop_accounting()
    print("\nall passed")
