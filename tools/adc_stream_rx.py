#!/usr/bin/env python3
"""Receive the i9's ADC stream and write it to a file.

Host side of gateware/adc_stream.py. Opens a UDP socket, checks every packet's
bookkeeping, and writes the samples as raw little-endian int16 -- one channel,
no header -- so numpy reads a capture with

    x = np.fromfile("capture.raw", dtype="<u2")

Typical use, with targets/colorlight_i9/adc_stream.py loaded on the board:

    .venv/bin/python tools/adc_stream_rx.py --seconds 5 --out /tmp/capture.raw
    .venv/bin/python tools/adc_stream_rx.py --seconds 2 --fft     # find the tone

**Loss is reported, never papered over.** The board stamps every packet with a
sequence number, the count of samples sent before it, and the count it had to
throw away, so the two failure modes stay distinct:

  * *network loss* -- a gap in seq. The samples are gone; the file would be
    silently shorter than the elapsed time. `--pad-gaps` writes zeros in their
    place so the file's time axis stays honest, which is what you want before
    an FFT; without it the gap closes up and every later sample is at the wrong
    time.
  * *board-side drops* -- the `dropped` counter climbing, meaning the FPGA
    could not hand samples to the link fast enough. Nothing the host can fix.

Both are printed in the summary, and a capture with either is flagged rather
than quietly returned.

If nothing arrives, in order of likelihood: the board is streaming to a
different host (`--adc-dest-ip` at build time, or the CSR), this machine's
firewall is eating UDP on the port, or the board is not actually running the
adc_stream bitstream.
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

MAGIC        = 0x43444152      # "RADC"
HEADER_WORDS = 8
HEADER_BYTES = HEADER_WORDS * 4


class Header:
    __slots__ = ("seq", "emitted", "dropped", "n_samples", "channel", "version",
                 "period", "sys_clk_freq")

    def __init__(self, words):
        (_magic, self.seq, self.emitted, self.dropped,
         packed, self.period, self.sys_clk_freq, _rsvd) = words
        self.n_samples = packed & 0xffff
        self.channel   = (packed >> 16) & 0x7
        self.version   = (packed >> 24) & 0xff

    @property
    def sample_rate(self):
        return self.sys_clk_freq / self.period if self.period else float("nan")


def parse(data):
    """Return (Header, samples bytes) or raise ValueError with a useful reason."""
    if len(data) < HEADER_BYTES:
        raise ValueError(f"runt packet, {len(data)} bytes")

    magic_le = struct.unpack_from("<I", data, 0)[0]
    if magic_le != MAGIC:
        magic_be = struct.unpack_from(">I", data, 0)[0]
        if magic_be == MAGIC:
            # Worth saying precisely: it means the 32->8 stride converter in
            # LiteEth put our words on the wire big-endian, and every field
            # (and every sample) needs the other byte order, not just the magic.
            raise ValueError(
                "packet is BIG-endian on the wire (magic matches byte-reversed). "
                "The gateware and this tool both assume little-endian; one of "
                "them needs flipping -- see gateware/adc_stream.py."
            )
        raise ValueError(f"bad magic {magic_le:#010x}, first 4 bytes {data[:4].hex()}")

    hdr = Header(struct.unpack_from("<8I", data, 0))
    want = HEADER_BYTES + hdr.n_samples * 2
    if len(data) < want:
        raise ValueError(f"length {len(data)} < header's {want} "
                         f"({hdr.n_samples} samples)")
    return hdr, data[HEADER_BYTES:want]


def parse_all(data):
    """Yield every record in a datagram -- usually one, sometimes more.

    LiteEth's Stream2UDPTX flushes its FIFO either on our `last` or when the
    FIFO fills, and the second case does not respect packet boundaries. While
    ARP resolves at startup the UDP path stalls, the FIFO backs up, and two
    records come out in a single 2112-byte datagram. Treating that as a
    malformed packet throws away good samples, so walk the buffer instead.

    (The gateware-side fix is to size tx_fifo_depth to exactly one packet so
    the full-flush coincides with a boundary -- see gateware/adc_stream.py.)
    """
    off = 0
    n = 0
    while off < len(data):
        hdr, payload = parse(data[off:])
        yield hdr, payload
        off += HEADER_BYTES + hdr.n_samples * 2
        n += 1
        if n > 64:
            break


_MON_HEADER_DONE = [False]


def _monitor_line(raw, hdr):
    """One line per window: DC, AC content, and what frequency dominates.

    AC rms is the number to watch on an analog front end. The ADC's own
    quantization floor is 1/sqrt(12) = 0.289 LSB rms, so anything at or below
    ~0.3 counts means the input is DC as far as this ADC can tell -- the signal,
    if there is one, is under the 3.22 mV that one count is worth at VREF=3V3.
    """
    import numpy as np
    x = np.frombuffer(raw, dtype="<u2").astype(np.float64)
    if x.size < 32:
        return
    dc = x.mean()
    ac = x.std()
    pp = int(x.max()) - int(x.min())
    xc = x - dc
    spec = np.abs(np.fft.rfft(xc * np.hanning(x.size)))
    freqs = np.fft.rfftfreq(x.size, 1.0 / hdr.sample_rate)
    k = int(np.argmax(spec[1:]) + 1)
    # Only call a peak real if it stands clear of the rest of the spectrum.
    med = np.median(spec[1:]) or 1e-12
    snr = 20 * np.log10(spec[k] / med)
    verdict = f"{freqs[k]:7.1f} Hz {snr:4.0f} dB" if snr > 20 and ac > 0.35 else "   -- no signal --"

    if not _MON_HEADER_DONE[0]:
        print(f"{'DC cnt':>8} {'DC mV':>8} {'AC rms':>8} {'AC mV':>8} {'p-p':>5}  peak")
        _MON_HEADER_DONE[0] = True
    print(f"{dc:8.2f} {dc*3300/1023:8.1f} {ac:8.3f} {ac*3300/1023:8.2f} {pp:5d}  {verdict}",
          flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5005, help="UDP port to listen on")
    ap.add_argument("--bind", default="0.0.0.0", help="address to bind")
    ap.add_argument("--out", help="write raw little-endian int16 samples here")
    ap.add_argument("--seconds", type=float, default=5.0, help="capture duration")
    ap.add_argument("--packets", type=int, help="stop after this many packets instead")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="give up if nothing arrives for this long")
    ap.add_argument("--pad-gaps", action="store_true",
                    help="write zeros for samples lost in transit, so the file's "
                         "time axis stays correct")
    ap.add_argument("--fft", action="store_true",
                    help="report the dominant frequency of the capture (needs numpy)")
    ap.add_argument("--quiet", action="store_true", help="summary only")
    ap.add_argument("--monitor", action="store_true",
                    help="live one-line-per-window readout (DC, AC rms, peak-to-peak, "
                         "dominant frequency) instead of a silent capture. For poking "
                         "an analog front end and watching it respond.")
    ap.add_argument("--window", type=float, default=0.5,
                    help="--monitor averaging window in seconds (default 0.5)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # A big receive buffer is cheap insurance: the default can be a couple of
    # packets, and a scheduling hiccup then shows up as "network loss" that is
    # really this process being slow.
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    sock.bind((args.bind, args.port))
    sock.settimeout(args.timeout)

    out = open(args.out, "wb") if args.out else None
    chunks = []

    n_pkt = n_samples = 0
    done = False
    lost_pkts = lost_samples = 0
    bad = 0
    first_hdr = None
    prev = None
    t0 = None
    deadline = None

    mon_buf = []
    if args.monitor:
        try:
            import numpy  # noqa: F401
        except ImportError:
            print("--monitor needs numpy", file=sys.stderr)
            return 1

    if not args.quiet:
        print(f"listening on {args.bind}:{args.port} ...", flush=True)

    try:
        while True:
            try:
                data, peer = sock.recvfrom(65535)
            except socket.timeout:
                if n_pkt == 0:
                    print(f"nothing received for {args.timeout:.0f}s.", file=sys.stderr)
                    print("  Is the board running the adc_stream bitstream, and is its "
                          "destination IP this machine?", file=sys.stderr)
                    return 2
                break

            try:
                records = list(parse_all(data))
            except ValueError as e:
                bad += 1
                if bad <= 3:
                    print(f"  ignoring packet: {e}", file=sys.stderr)
                continue
            for hdr, payload in records:

                if first_hdr is None:
                    first_hdr = hdr
                    t0 = time.monotonic()
                    deadline = t0 + args.seconds
                    if not args.quiet:
                        print(f"streaming from {peer[0]}: ch{hdr.channel}, "
                              f"{hdr.sample_rate:,.1f} sps "
                              f"(sys {hdr.sys_clk_freq/1e6:.0f} MHz / period {hdr.period}), "
                              f"{hdr.n_samples} samples/packet, v{hdr.version}", flush=True)

                if prev is not None:
                    gap = (hdr.seq - prev.seq) & 0xffffffff
                    if gap != 1:
                        missing = (hdr.emitted - prev.emitted - prev.n_samples) & 0xffffffff
                        lost_pkts += gap - 1
                        lost_samples += missing
                        if not args.quiet:
                            print(f"  ! seq {prev.seq} -> {hdr.seq}: "
                                  f"{gap-1} packet(s), {missing} samples lost in transit",
                                  flush=True)
                        if args.pad_gaps and missing:
                            pad = b"\x00\x00" * missing
                            chunks.append(pad)
                            if out:
                                out.write(pad)

                chunks.append(payload)
                if out:
                    out.write(payload)
                n_pkt += 1
                n_samples += hdr.n_samples
                prev = hdr

                if args.monitor:
                    mon_buf.append(payload)
                    if (len(mon_buf) * hdr.n_samples) >= hdr.sample_rate * args.window:
                        _monitor_line(b"".join(mon_buf), hdr)
                        mon_buf = []

                # A datagram can hold several records, so the stop condition
                # has to break BOTH loops -- a bare break here would only end
                # the inner one and the capture would never finish.
                if args.packets is not None:
                    if n_pkt >= args.packets:
                        done = True
                elif time.monotonic() >= deadline:
                    done = True
                if done:
                    break
            if done:
                break
    finally:
        if out:
            out.close()
        sock.close()

    if n_pkt == 0:
        return 2

    elapsed = time.monotonic() - t0
    board_dropped = (prev.dropped - first_hdr.dropped) & 0xffffffff

    print()
    print(f"packets      {n_pkt:,}  ({bad} unparseable)")
    print(f"samples      {n_samples:,} in {elapsed:.2f}s "
          f"= {n_samples/elapsed:,.0f} sps received")
    print(f"nominal rate {prev.sample_rate:,.1f} sps (from the packet header)")
    print(f"lost in net  {lost_pkts} packets / {lost_samples:,} samples"
          + ("  <-- padded with zeros" if (args.pad_gaps and lost_samples) else ""))
    print(f"board drops  {board_dropped:,} samples")
    if args.out:
        print(f"wrote        {args.out}")

    if lost_samples or board_dropped:
        print("\nNOTE: this capture has gaps. For a clean frequency measurement, "
              "recapture or account for them --")
        if board_dropped:
            print("      board drops mean the FPGA could not feed the link; lower the "
                  "sample rate or check the link is up.")
        if lost_samples and not args.pad_gaps:
            print("      network loss without --pad-gaps shifts every later sample in "
                  "time, which smears an FFT.")

    if args.fft:
        try:
            import numpy as np
        except ImportError:
            print("\n--fft needs numpy", file=sys.stderr)
            return 1
        x = np.frombuffer(b"".join(chunks), dtype="<u2").astype(np.float64)
        if x.size < 64:
            print("\n--fft: too few samples", file=sys.stderr)
            return 1
        fs = prev.sample_rate
        x -= x.mean()
        win = np.hanning(x.size)
        spec = np.abs(np.fft.rfft(x * win))
        freqs = np.fft.rfftfreq(x.size, 1.0 / fs)
        k = int(np.argmax(spec[1:]) + 1)
        print()
        print(f"fft          {x.size:,} samples, {freqs[1]:.2f} Hz bins")
        print(f"peak         {freqs[k]:,.2f} Hz")
        counts = np.frombuffer(b"".join(chunks), dtype="<u2")
        # int() on the way out: these are uint16, and count*3300 overflows that
        # type silently -- 515 counts printed as 56 mV instead of 1661.
        lo, hi = int(counts.min()), int(counts.max())
        print(f"levels       min {lo} max {hi} mean {counts.mean():.1f} counts "
              f"({lo*3300//1023} .. {hi*3300//1023} mV at VREF=3V3)")
        print(f"AC           {counts.std():.2f} counts rms, {hi-lo} counts p-p "
              f"({(hi-lo)*3300/1023:.1f} mV p-p)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
