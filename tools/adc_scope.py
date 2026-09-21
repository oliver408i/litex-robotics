#!/usr/bin/env python3
"""Live Doppler scope for the radar stream -- a curses TUI.

Reads the UDP stream from targets/colorlight_i9/adc_stream.py and shows it as
a live spectrum plus a scrolling waterfall, with velocity (not frequency) as
the x axis. Point the radar at something moving and watch it paint.

    .venv/bin/python tools/adc_scope.py                 # TUI, 0 - 6.2 m/s
    .venv/bin/python tools/adc_scope.py --fmax 3000     # out to 18.6 m/s, cars
    .venv/bin/python tools/adc_scope.py --plain --log peaks.csv

Keys:  q quit   space pause   +/- velocity range   h peak-hold   c clear
       n/N FFT size (velocity resolution vs update rate)

**Velocity, not frequency.** A CDM324 at 24.125 GHz shifts 2*v*f0/c = 161 Hz
per m/s. A hand wave is 40-200 Hz (0.25-1.25 m/s), walking ~225 Hz, a car at
100 km/h ~4.5 kHz.

**An empty screen with nothing moving is correct.** CW Doppler produces no AC
at zero relative velocity -- the mixer product is DC. Idle means working.

**The magnitude scale is absolute, never auto-ranged** (-12 to +48 dB in ADC
counts; 1 count = 3.22 mV at VREF = 3V3). Auto-ranging would make a weak
target look exactly like a strong one, which defeats the purpose when what you
want to see is a target getting fainter as it moves away.

**It will not invent targets.** A velocity is only reported when the frame's
AC exceeds 0.35 counts and the peak exceeds 0.5 counts. Below that the loudest
bin is just quantization noise, and reporting it produces confident readings
of targets that do not exist.

Colour is a sequential ramp, dark to white in one blue-cyan hue family --
deliberately not a rainbow. Brightness alone carries magnitude, so it reads
correctly in greyscale and under any colour blindness, and it does not invent
banding where the spectrum is smooth.

`--plain` forces the line-per-frame renderer (used automatically when stdout
is not a terminal), which is what you want for `--log` or piping.

Only one process can hold the UDP port, so stop `adc_stream_rx.py` first.
"""
from __future__ import annotations

import argparse
import curses
import shutil
import socket
import sys
import time
from collections import deque
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from tools.adc_stream_rx import parse_all                  # noqa: E402

C_LIGHT = 299_792_458.0
F0_CDM324 = 24.125e9
HZ_PER_MS = 2 * F0_CDM324 / C_LIGHT            # 160.94 Hz per m/s

# Sequential ramp, dark -> light, one blue-cyan hue family. Monotonic
# luminance is the point: magnitude is legible from brightness alone, so
# colour vision is never load-bearing.
RAMP = [234, 17, 18, 19, 20, 26, 32, 38, 44, 87, 231]
DENSITY = " .:-=+*#%@#"          # monochrome fallback, same 11 steps

# Only call a peak real above these. Below them the loudest bin is noise.
AC_GATE = 0.35
AMP_GATE = 0.5


class Pipeline:
    """UDP payloads in, spectrum frames out.

    Owns the FFT geometry so that changing the velocity range or the FFT size
    at runtime is a single call, and the renderers never touch numpy.
    """

    def __init__(self, nfft, fmax, fs=20000.0, width=80):
        import numpy as np
        self.np = np
        self.fs = fs
        self.buf = np.empty(0, dtype=np.float64)
        self.width = width
        self.set_geometry(nfft, fmax)

    def set_geometry(self, nfft=None, fmax=None, width=None):
        np = self.np
        if nfft is not None:
            self.nfft = int(nfft)
        if fmax is not None:
            self.fmax = float(fmax)
        if width is not None:
            self.width = max(8, int(width))
        self.win = np.hanning(self.nfft)
        self.wsum = self.win.sum()
        self.freqs = np.fft.rfftfreq(self.nfft, 1.0 / self.fs)
        self.keep = self.freqs <= self.fmax
        self.kept = self.freqs[self.keep]
        n = max(2, int(self.keep.sum()))
        # Column j takes the MAX over its bins: a one-bin-wide peak must not be
        # averaged away by quiet neighbours.
        self.edges = np.linspace(0, n, self.width + 1).astype(int)

    @property
    def row_ms(self):
        return self.nfft / self.fs * 1000.0

    @property
    def bin_ms(self):
        return self.fs / self.nfft / HZ_PER_MS

    def feed(self, payload):
        """Append samples; yield one frame dict per complete FFT window."""
        np = self.np
        self.buf = np.concatenate(
            [self.buf, np.frombuffer(payload, dtype="<u2").astype(np.float64)])
        while self.buf.size >= self.nfft:
            frame = self.buf[:self.nfft]
            self.buf = self.buf[self.nfft:]
            dc = float(frame.mean())
            ac = float(frame.std())
            spec = np.abs(np.fft.rfft((frame - dc) * self.win)) * 2.0 / self.wsum
            s = spec[self.keep]
            cols = np.array([s[self.edges[j]:max(self.edges[j + 1], self.edges[j] + 1)].max()
                             for j in range(self.width)])
            k = int(np.argmax(s[1:]) + 1) if s.size > 1 else 0
            pk_hz, pk_amp = float(self.kept[k]), float(s[k])
            real = ac > AC_GATE and pk_amp > AMP_GATE
            yield {
                "cols": cols, "dc": dc, "ac": ac,
                "pk_hz": pk_hz, "pk_ms": pk_hz / HZ_PER_MS, "pk_amp": pk_amp,
                "real": real,
            }


def _levels(cols, floor, ceil, np):
    """Magnitudes -> ramp indices 0..len(RAMP)-1."""
    with np.errstate(divide="ignore"):
        db = 20.0 * np.log10(np.maximum(cols, 1e-6))
    idx = np.clip(((db - floor) / (ceil - floor) * (len(RAMP) - 1)).round(),
                  0, len(RAMP) - 1).astype(int)
    idx[db < floor] = 0
    return idx


def _ruler(width, fmax, use_velocity=True):
    ticks = max(2, min(6, width // 14))
    # Spaces, not "": an empty string joins to nothing and the labels collapse
    # into each other.
    lab = [" "] * width
    marks = [" "] * width
    for i in range(ticks):
        col = int(round(i * (width - 1) / (ticks - 1)))
        f = i * fmax / (ticks - 1)
        text = f"{f / HZ_PER_MS:.1f}" if use_velocity else f"{f:.0f}"
        marks[col] = "|"
        start = max(0, min(col - len(text) // 2, width - len(text)))
        for j, ch in enumerate(text):
            lab[start + j] = ch
    return "".join(marks), "".join(lab)


# --- plain renderer (non-tty, --plain, --log) -----------------------------------------------------
def run_plain(sock, args, pipe, logf):
    width = pipe.width
    printed_header = False
    def _plain_record(hdr, payload):
        nonlocal printed_header, rows, t0
        if t0 is None:
            t0 = time.monotonic()
            pipe.fs = hdr.sample_rate
            pipe.set_geometry()
        if not printed_header:
            printed_header = True
            m, l = _ruler(width, pipe.fmax, not args.hz)
            print(f"ch{hdr.channel}  {pipe.fs:,.0f} sps  {pipe.nfft}-point "
                  f"({pipe.row_ms:.1f} ms/row, {pipe.bin_ms:.2f} m/s bins)")
            print(f"{l}  {'Hz' if args.hz else 'm/s'}")
            print(m)
        for fr in pipe.feed(payload):
            idx = _levels(fr["cols"], args.floor, args.ceil, pipe.np)
            out, prev, run = [], None, 0
            for c in (RAMP[i] for i in idx):
                if c == prev:
                    run += 1
                else:
                    if prev is not None:
                        out.append(f"\x1b[48;5;{prev}m" + " " * run)
                    prev, run = c, 1
            out.append(f"\x1b[48;5;{prev}m" + " " * run + "\x1b[0m")
            tag = (f"{fr['pk_ms']:5.2f} m/s {fr['pk_amp']:6.1f}"
                   if fr["real"] else "    --       ")
            print("".join(out) + " " + tag, flush=True)
            rows += 1
            if logf:
                logf.write(f"{time.time():.3f},{fr['pk_hz']:.1f},{fr['pk_ms']:.3f},"
                           f"{fr['pk_amp']:.2f},{fr['ac']:.3f}\n")
            if rows % 40 == 0:
                m, l = _ruler(width, pipe.fmax, not args.hz)
                print(f"{l}  {'Hz' if args.hz else 'm/s'}")
                print(m)

    rows = n_pkt = lost = 0
    prev_seq = None
    t0 = None
    try:
        while True:
            if args.seconds and t0 and (time.monotonic() - t0) >= args.seconds:
                break
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                if t0 is None:
                    print("nothing received. Is the board streaming to this host?",
                          file=sys.stderr)
                    return 2
                continue
            try:
                records = list(parse_all(data))
            except ValueError:
                continue
            for hdr, payload in records:
                n_pkt += 1
                if prev_seq is not None:
                    lost += ((hdr.seq - prev_seq) & 0xffffffff) - 1
                prev_seq = hdr.seq
                _plain_record(hdr, payload)
    except KeyboardInterrupt:
        pass
    el = (time.monotonic() - t0) if t0 else 0.0
    print(f"\n{rows} rows, {n_pkt} packets, {el:.1f}s"
          + (f", {lost} packets LOST" if lost else ", no loss"))
    return 0


# --- curses TUI -----------------------------------------------------------------------------------
class Tui:
    MIN_W, MIN_H = 40, 14

    def __init__(self, scr, args, pipe, logf):
        self.scr, self.args, self.pipe, self.logf = scr, args, pipe, logf
        self.np = pipe.np
        self.paused = False
        self.hold = True
        self.rows = deque()
        self.peak_hold = None
        self.vmax_seen = 0.0
        self.amax_seen = 0.0
        self.last = None
        self.n_pkt = self.lost = self.n_rows = 0
        self.drops0 = None
        self.drops = 0
        self.t0 = None
        self.color = False
        self._init_colors()
        self.layout()

    def _init_colors(self):
        try:
            curses.start_color()
            curses.use_default_colors()
        except curses.error:
            return
        if curses.COLORS >= 256:
            for i, c in enumerate(RAMP):
                curses.init_pair(1 + i, -1, c)          # waterfall: background
                curses.init_pair(21 + i, c, -1)         # bars: foreground
            curses.init_pair(50, curses.COLOR_CYAN, -1)
            curses.init_pair(51, curses.COLOR_YELLOW, -1)
            self.color = True

    def cell_wf(self, i):
        if self.color:
            return " ", curses.color_pair(1 + i)
        return DENSITY[i], curses.A_BOLD if i > 6 else curses.A_NORMAL

    def cell_bar(self, i, ch):
        if self.color:
            return ch, curses.color_pair(21 + i)
        return ch, curses.A_BOLD if i > 6 else curses.A_NORMAL

    def layout(self):
        self.h, self.w = self.scr.getmaxyx()
        self.ok = self.h >= self.MIN_H and self.w >= self.MIN_W
        if not self.ok:
            return
        self.gutter = 12 if self.w >= 60 else 0
        self.pw = max(8, self.w - self.gutter - 1)
        self.sh = max(3, min(9, (self.h - 10) // 3))     # spectrum height
        self.y_spec = 3
        self.y_ruler = self.y_spec + self.sh
        self.y_wf = self.y_ruler + 2
        self.wf_h = max(1, self.h - self.y_wf - 1)
        self.pipe.set_geometry(width=self.pw)
        self.peak_hold = None
        while len(self.rows) > self.wf_h:
            self.rows.popleft()

    def push(self, fr):
        np = self.np
        idx = _levels(fr["cols"], self.args.floor, self.args.ceil, np)
        self.rows.append((idx, fr))
        while len(self.rows) > self.wf_h:
            self.rows.popleft()
        self.last = fr
        self.n_rows += 1
        if self.peak_hold is None or self.peak_hold.shape != fr["cols"].shape:
            self.peak_hold = fr["cols"].copy()
        else:
            self.peak_hold = np.maximum(self.peak_hold * 0.995, fr["cols"])
        if fr["real"]:
            self.vmax_seen = max(self.vmax_seen, fr["pk_ms"])
            self.amax_seen = max(self.amax_seen, fr["pk_amp"])
        if self.logf:
            self.logf.write(f"{time.time():.3f},{fr['pk_hz']:.1f},{fr['pk_ms']:.3f},"
                            f"{fr['pk_amp']:.2f},{fr['ac']:.3f}\n")

    def _put(self, y, x, text, attr=0):
        """addstr that tolerates the bottom-right cell and short terminals."""
        if y < 0 or y >= self.h or x >= self.w:
            return
        text = text[:max(0, self.w - x)]
        if y == self.h - 1 and x + len(text) >= self.w:
            text = text[:max(0, self.w - x - 1)]
        try:
            self.scr.addstr(y, x, text, attr)
        except curses.error:
            pass

    def draw(self, hdr):
        scr = self.scr
        scr.erase()
        if not self.ok:
            self._put(0, 0, f"terminal too small ({self.w}x{self.h}); "
                            f"need {self.MIN_W}x{self.MIN_H}")
            scr.refresh()
            return
        np = self.np
        fr = self.last
        el = (time.monotonic() - self.t0) if self.t0 else 0.0

        # header
        title = (f" Doppler scope  ch{hdr.channel}  {self.pipe.fs:,.0f} sps  "
                 f"{self.pipe.nfft}-pt  {self.pipe.row_ms:.0f} ms/row  "
                 f"{self.pipe.bin_ms:.2f} m/s bins ")
        self._put(0, 0, title.ljust(self.w - 1), curses.A_REVERSE)
        if self.paused:
            self._put(0, max(0, self.w - 10), " PAUSED ", curses.A_REVERSE | curses.A_BLINK)

        if fr:
            vel = f"{fr['pk_ms']:5.2f} m/s" if fr["real"] else "   --    "
            hz = f"{fr['pk_hz']:6.1f} Hz" if fr["real"] else "    --   "
            s1 = (f" now {vel} {hz}  amp {fr['pk_amp']:6.1f}   "
                  f"AC {fr['ac']:6.2f} cnt   DC {fr['dc']:6.1f} ({fr['dc'] * 3300 / 1023:4.0f} mV)")
            self._put(1, 0, s1, curses.color_pair(51) if (self.color and fr["real"]) else 0)
        s2 = (f" max {self.vmax_seen:5.2f} m/s / {self.amax_seen:6.1f} cnt   "
              f"pkts {self.n_pkt}  lost {self.lost}  board drops {self.drops}  {el:5.1f}s")
        self._put(2, 0, s2, curses.A_DIM)

        # spectrum bars, newest frame
        if fr is not None:
            cols = fr["cols"]
            lo, hi = self.args.floor, self.args.ceil
            with np.errstate(divide="ignore"):
                db = 20.0 * np.log10(np.maximum(cols, 1e-6))
            n = np.clip((db - lo) / (hi - lo), 0.0, 1.0)
            idx = _levels(cols, lo, hi, np)
            blocks = "▁▂▃▄▅▆▇█"
            for j in range(min(self.pw, self.w - 1)):
                filled = n[j] * self.sh
                for r in range(self.sh):
                    row_from_bottom = self.sh - r
                    y = self.y_spec + r
                    if filled >= row_from_bottom:
                        ch, at = self.cell_bar(int(idx[j]), "█")
                        self._put(y, j, ch, at)
                    elif filled > row_from_bottom - 1:
                        frac = filled - (row_from_bottom - 1)
                        b = blocks[max(0, min(len(blocks) - 1, int(frac * len(blocks))))]
                        ch, at = self.cell_bar(int(idx[j]), b)
                        self._put(y, j, ch, at)
            if self.hold and self.peak_hold is not None:
                with np.errstate(divide="ignore"):
                    hdb = 20.0 * np.log10(np.maximum(self.peak_hold, 1e-6))
                hn = np.clip((hdb - lo) / (hi - lo), 0.0, 1.0)
                for j in range(min(self.pw, self.w - 1)):
                    if hn[j] <= 0:
                        continue
                    y = self.y_spec + self.sh - 1 - int(min(self.sh - 1, hn[j] * self.sh))
                    self._put(y, j, "·", curses.color_pair(50) if self.color else curses.A_DIM)

        # shared velocity ruler
        marks, labs = _ruler(self.pw, self.pipe.fmax, not self.args.hz)
        self._put(self.y_ruler, 0, marks, curses.A_DIM)
        self._put(self.y_ruler + 1, 0, labs, curses.A_DIM)
        self._put(self.y_ruler + 1, min(self.pw + 1, self.w - 5),
                  "Hz" if self.args.hz else "m/s", curses.A_DIM)

        # waterfall, newest at the bottom
        base = self.y_wf + self.wf_h - len(self.rows)
        for r, (idx, rfr) in enumerate(self.rows):
            y = base + r
            if y < self.y_wf:
                continue
            x = 0
            prev_i, run = None, 0
            for i in idx:
                i = int(i)
                if i == prev_i:
                    run += 1
                else:
                    if prev_i is not None:
                        ch, at = self.cell_wf(prev_i)
                        self._put(y, x, ch * run, at)
                        x += run
                    prev_i, run = i, 1
            if prev_i is not None:
                ch, at = self.cell_wf(prev_i)
                self._put(y, x, ch * run, at)
            if self.gutter:
                tag = (f"{rfr['pk_ms']:5.2f} {rfr['pk_amp']:5.1f}"
                       if rfr["real"] else "   --      ")
                self._put(y, self.pw + 1, tag,
                          (curses.color_pair(51) if self.color else curses.A_BOLD)
                          if rfr["real"] else curses.A_DIM)

        foot = (" q quit  space pause  +/- range  h hold  c clear  n/N fft "
                f"  range 0-{self.pipe.fmax / HZ_PER_MS:.1f} m/s ")
        self._put(self.h - 1, 0, foot.ljust(self.w - 1), curses.A_REVERSE)
        scr.refresh()

    def key(self, ch):
        if ch in (ord("q"), ord("Q"), 27):
            return False
        if ch == ord(" "):
            self.paused = not self.paused
        elif ch in (ord("+"), ord("=")):
            self.pipe.set_geometry(fmax=min(self.pipe.fs / 2, self.pipe.fmax * 2))
            self.peak_hold = None
            self.rows.clear()
        elif ch in (ord("-"), ord("_")):
            self.pipe.set_geometry(fmax=max(50.0, self.pipe.fmax / 2))
            self.peak_hold = None
            self.rows.clear()
        elif ch == ord("h"):
            self.hold = not self.hold
            self.peak_hold = None
        elif ch == ord("c"):
            self.rows.clear()
            self.peak_hold = None
            self.vmax_seen = self.amax_seen = 0.0
        elif ch == ord("n"):
            self.pipe.set_geometry(nfft=min(8192, self.pipe.nfft * 2))
            self.rows.clear()
            self.peak_hold = None
        elif ch == ord("N"):
            self.pipe.set_geometry(nfft=max(128, self.pipe.nfft // 2))
            self.rows.clear()
            self.peak_hold = None
        elif ch == curses.KEY_RESIZE:
            self.layout()
        return True


def run_tui(scr, sock, args, pipe, logf):
    curses.curs_set(0)
    scr.nodelay(True)
    ui = Tui(scr, args, pipe, logf)
    hdr = None
    prev_seq = None
    sock.settimeout(0.05)
    last_draw = 0.0

    ui._put(0, 0, "waiting for packets on "
                  f"{args.bind}:{args.port} ... (q to quit)")
    scr.refresh()

    while True:
        ch = scr.getch()
        while ch != -1:
            if not ui.key(ch):
                return 0
            ch = scr.getch()

        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            if hdr is not None and time.monotonic() - last_draw > 0.5:
                ui.draw(hdr)
                last_draw = time.monotonic()
            continue
        try:
            records = list(parse_all(data))
        except ValueError:
            continue
        for h, payload in records:
            ui.n_pkt += 1
            if prev_seq is not None:
                ui.lost += ((h.seq - prev_seq) & 0xffffffff) - 1
            prev_seq = h.seq
            if ui.drops0 is None:
                ui.drops0 = h.dropped
            ui.drops = (h.dropped - ui.drops0) & 0xffffffff
            if hdr is None:
                ui.t0 = time.monotonic()
                pipe.fs = h.sample_rate
                pipe.set_geometry()
            hdr = h

            # Always drain the socket, even paused -- otherwise the kernel
            # buffer overflows and the loss counter blames the network for
            # our pause.
            for fr in pipe.feed(payload):
                if not ui.paused:
                    ui.push(fr)

        if args.seconds and (time.monotonic() - ui.t0) >= args.seconds:
            return 0
        now = time.monotonic()
        if now - last_draw >= 1.0 / 30:
            ui.draw(hdr)
            last_draw = now


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--fmax", type=float, default=1000.0,
                    help="top of the velocity axis in Hz (default 1000 = 6.2 m/s)")
    ap.add_argument("--nfft", type=int, default=1024,
                    help="FFT size; 1024 at 20 ksps = 51.2 ms rows, 0.12 m/s bins")
    ap.add_argument("--floor", type=float, default=-12.0, help="dB at the bottom of the ramp")
    ap.add_argument("--ceil", type=float, default=48.0, help="dB at the top of the ramp")
    ap.add_argument("--seconds", type=float, help="stop after this long")
    ap.add_argument("--hz", action="store_true", help="label the axis in Hz, not m/s")
    ap.add_argument("--log", help="append peak velocity/magnitude per frame to this CSV")
    ap.add_argument("--plain", action="store_true",
                    help="line-per-frame output instead of the TUI (automatic when "
                         "stdout is not a terminal)")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    try:
        import numpy  # noqa: F401
    except ImportError:
        print("adc_scope needs numpy", file=sys.stderr)
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    try:
        sock.bind((args.bind, args.port))
    except OSError as e:
        print(f"cannot bind {args.bind}:{args.port}: {e}", file=sys.stderr)
        print("  Another receiver (adc_stream_rx.py?) already holds the port.",
              file=sys.stderr)
        return 1

    logf = open(args.log, "a") if args.log else None
    if logf and logf.tell() == 0:
        logf.write("t,peak_hz,peak_ms,peak_counts,ac_rms_counts\n")

    width = max(24, min(shutil.get_terminal_size((80, 24)).columns, 200) - 13)
    pipe = Pipeline(args.nfft, args.fmax, width=width)

    plain = args.plain or not sys.stdout.isatty()
    try:
        if plain:
            sock.settimeout(args.timeout)
            return run_plain(sock, args, pipe, logf)
        return curses.wrapper(run_tui, sock, args, pipe, logf)
    finally:
        sock.close()
        if logf:
            logf.close()


if __name__ == "__main__":
    sys.exit(main())
