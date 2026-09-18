#!/usr/bin/env python3
"""Migen-simulation bench for gateware/lcd_engine.py (LCDEngine) DMA payloads.

Focus is the payload byte stream the panel actually sees, because that is the
one thing a hardware run does NOT check: the console reports a clean pass while
the screen stays white (see docs/, and the ST7796S SCK wall). So the bench
snoops the engine's TX stream (tx_we/tx_byte, upstream of the CDC FIFO) and
compares it byte-for-byte against a model computed in Python.

The SPI_Master Verilog core is replaced by LCDEngine(sim_spi=True)'s behavioral
shifter -- migen's simulator cannot evaluate an Instance, so the FIFO would
never drain otherwise. Everything else (payload FSM, Wishbone master, async
FIFO, pulse CDC) is the real gateware.

Coverage:
  A  raw DMA_RECT: source bytes reach the wire unchanged, in order
  B  cvt DMA_RECT: XRGB8888 -> RGB565, MSB first, 2 wire bytes per source word
  C  cvt with stride (row_count > 1): per-row source jumps land correctly
  D  cvt channel mapping: primaries and saturation round-trip as expected

Run from the repo root:
    .venv/bin/python sim/test_lcd_engine.py
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

from migen import Module, Signal, ClockDomain, Record          # noqa: E402
from litex.gen.sim import run_simulation, passive              # noqa: E402

from gateware.lcd_engine import LCDEngine                      # noqa: E402

KIND_DMA_RECT = 5
ST7796_RAMWR  = 0x2C


def rgb565_of(px):
    """XRGB8888 -> RGB565, the mapping the gateware implements."""
    r = (px >> 16) & 0xFF
    g = (px >>  8) & 0xFF
    b = (px >>  0) & 0xFF
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


class _Pads(Module):
    def __init__(self):
        self.clk    = Signal()
        self.mosi   = Signal()
        self.miso   = Signal()
        self.cs_n   = Signal(1)


class _CtrlPads(Module):
    def __init__(self):
        self.dc        = Signal()
        self.backlight = Signal()
        self.reset_n   = Signal()


class _Platform:
    def add_source(self, *a, **kw):
        pass


class DUT(Module):
    """LCDEngine + a behavioral Wishbone memory holding the source pixels."""
    def __init__(self, mem_words):
        self.clock_domains.cd_spi = ClockDomain("spi")
        self.submodules.eng = eng = LCDEngine(_Pads(), _CtrlPads(), _Platform(),
                                              sclk_div=1, sim_spi=True)
        self.mem = list(mem_words)
        self.wb  = eng.bus


@passive
def wb_mem_server(dut):
    """Single-cycle Wishbone slave over dut.mem (word-addressed)."""
    while True:
        if (yield dut.wb.cyc) and (yield dut.wb.stb) and not (yield dut.wb.ack):
            adr = (yield dut.wb.adr)
            idx = adr - (BASE_WORD)
            val = dut.mem[idx] if 0 <= idx < len(dut.mem) else 0
            yield dut.wb.dat_r.eq(val)
            yield dut.wb.ack.eq(1)
        else:
            yield dut.wb.ack.eq(0)
        yield


@passive
def tx_monitor(dut, out, dc_out):
    """Record every byte the engine pushes toward the panel, with its DC line."""
    while True:
        if (yield dut.eng.tx_we) and (yield dut.eng.tx_fifo.writable):
            out.append((yield dut.eng.tx_byte))
            dc_out.append((yield dut.eng.ctrl_dc_probe) if hasattr(dut.eng, "ctrl_dc_probe") else 0)
        yield


BASE_BYTE = 0x1000
BASE_WORD = BASE_BYTE >> 2


def run_case(name, mem_words, row_bytes, row_count, stride, cvt, expect_payload,
             x0=0, x1=None, y0=0, y1=None):
    dut = DUT(mem_words)
    got, dcs = [], []

    def bench():
        # Stage the op. CSRStorage.write() is LiteX's simulation helper: it
        # drives the field signals too, which matters because `start` is a
        # pulse field (comb-gated on .re) and a bare .storage poke never
        # asserts it.
        yield from dut.eng.dma_src.write(BASE_BYTE)
        yield from dut.eng.dma_row_bytes.write(row_bytes)
        yield from dut.eng.dma_row_count.write(row_count)
        yield from dut.eng.dma_stride.write(stride)
        yield from dut.eng.rect_x.write(((x1 if x1 is not None else 0) << 16) | x0)
        yield from dut.eng.rect_y.write(((y1 if y1 is not None else 0) << 16) | y0)
        # Fire: kind=DMA_RECT, cvt, start.
        yield from dut.eng.op.write(KIND_DMA_RECT | (cvt << 4) | (1 << 8))
        # Let the op actually start before watching busy.
        for _ in range(4):
            yield
        # Run until the engine reports idle (with a generous cap).
        for _ in range(200000):
            if not (yield dut.eng.status.fields.busy):
                break
            yield
        for _ in range(200):
            yield

    run_simulation(dut, [bench(), wb_mem_server(dut), tx_monitor(dut, got, dcs)],
                   clocks={"sys": 10, "spi": 10})

    # The stream is CASET(cmd+4) + RASET(cmd+4) + RAMWR(cmd) + payload.
    # Find the RAMWR command byte and take everything after it.
    try:
        i = len(got) - 1 - got[::-1].index(ST7796_RAMWR)
    except ValueError:
        print(f"  {name}: FAIL - no RAMWR in stream ({len(got)} bytes)")
        return False
    payload = got[i + 1:]

    ok = payload == expect_payload
    print(f"  {name}: {'ok' if ok else 'FAIL'} "
          f"({len(payload)} payload bytes, expected {len(expect_payload)})")
    if not ok:
        print(f"     got      {payload[:24]}")
        print(f"     expected {expect_payload[:24]}")
    return ok


def main():
    results = []

    # --- A: raw mode, 16 source bytes straight through -------------------
    words = [0x03020100, 0x07060504, 0x0B0A0908, 0x0F0E0D0C]
    expect = list(range(16))
    results.append(run_case("A raw DMA_RECT passthrough", words,
                            row_bytes=16, row_count=1, stride=0, cvt=0,
                            expect_payload=expect))

    # --- B: cvt mode, 4 pixels -> 8 wire bytes ---------------------------
    px = [0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF]
    expect = []
    for p in px:
        v = rgb565_of(p)
        expect += [(v >> 8) & 0xFF, v & 0xFF]
    results.append(run_case("B cvt XRGB8888->RGB565", px,
                            row_bytes=len(px) * 4, row_count=1, stride=0, cvt=1,
                            expect_payload=expect))

    # --- C: cvt with stride, 2 rows of 2 pixels, stride 4 words ----------
    #   row0 = words[0:2], row1 = words[4:6]
    words = [0x00FF0000, 0x0000FF00, 0xDEADBEEF, 0xDEADBEEF,
             0x000000FF, 0x00FFFFFF, 0xDEADBEEF, 0xDEADBEEF]
    expect = []
    for p in [words[0], words[1], words[4], words[5]]:
        v = rgb565_of(p)
        expect += [(v >> 8) & 0xFF, v & 0xFF]
    results.append(run_case("C cvt strided rows", words,
                            row_bytes=2 * 4, row_count=2, stride=4 * 4, cvt=1,
                            expect_payload=expect))

    # --- D: cvt channel mapping / truncation ------------------------------
    px = [0x00000000, 0x00FFFFFF, 0x00080402, 0x00F8FCF8, 0x00123456]
    expect = []
    for p in px:
        v = rgb565_of(p)
        expect += [(v >> 8) & 0xFF, v & 0xFF]
    results.append(run_case("D cvt channel mapping", px,
                            row_bytes=len(px) * 4, row_count=1, stride=0, cvt=1,
                            expect_payload=expect))

    print()
    if all(results):
        print("all passed")
        return 0
    print(f"{results.count(False)} of {len(results)} FAILED")
    return 1


if __name__ == "__main__":
    sys.exit(main())
