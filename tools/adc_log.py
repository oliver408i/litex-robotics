#!/usr/bin/env python3
"""Log the ADC stream to a file. Nothing else.

No FFT, no display, no numpy -- just bytes to disk, plus a sidecar that says
what they mean.

    .venv/bin/python tools/adc_log.py                     # until Ctrl-C
    .venv/bin/python tools/adc_log.py --seconds 60 -o walk_test
    .venv/bin/python tools/adc_log.py --wav -o hand_wave  # openable in Audacity

Writes two files per run:

  <name>.raw    the samples, little-endian uint16, one channel, no header.
                Each is a 10-bit count, 0..1023. numpy reads it with
                `np.fromfile(path, dtype="<u2")`.
  <name>.json   what the .raw is: sample rate, channel, start time, how many
                samples, and every gap.

**The sidecar is the point.** A bare .raw is uninterpretable a week later -- you
cannot recover the sample rate, and without the sample rate the frequency axis
(which IS the measurement for Doppler) is unknowable. The board sends its own
`period` and `sys_clk_freq` in every packet, so the rate recorded here is the
true one, not the one someone meant to configure.

**Gaps are always recorded, never silently closed up.** A lost packet is 512
samples missing; if the file just closes the hole, every later sample sits at
the wrong time and an FFT smears. The `gaps` list in the sidecar gives the
sample offset and length of each one. `--pad-gaps` additionally writes zeros in
their place so the file's time axis stays literally correct -- zeros against a
~515-count DC read as an obvious click in a spectrogram, which is the point:
better a visible artifact than an invisible time shift.

`--wav` writes a 16-bit mono WAV alongside, DC-removed and scaled
((count - 512) * 32). The Doppler band is 50 Hz - 5 kHz, i.e. audio, so a WAV
opens in Audacity, sox, or any spectrogram tool you already trust. It is a
convenience copy -- the .raw stays the reference.

Only one process can hold the UDP port, so stop the other receivers first.
"""
from __future__ import annotations

import argparse
import array
import json
import socket
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from tools.adc_stream_rx import parse_all                  # noqa: E402

HZ_PER_MS = 2 * 24.125e9 / 299_792_458.0       # CDM324: 160.94 Hz per m/s


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", help="output name without extension "
                                        "(default adc_YYYYmmdd_HHMMSS)")
    ap.add_argument("--seconds", type=float, help="stop after this long")
    ap.add_argument("--max-mb", type=float, help="stop after this much raw data")
    ap.add_argument("--wav", action="store_true", help="also write a 16-bit mono WAV")
    ap.add_argument("--pad-gaps", action="store_true",
                    help="write zeros for samples lost in transit, keeping the "
                         "file's time axis correct")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    name = args.out or datetime.now().strftime("adc_%Y%m%d_%H%M%S")
    raw_path = Path(f"{name}.raw")
    meta_path = Path(f"{name}.json")

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
        print("  Another receiver already holds the port.", file=sys.stderr)
        return 1
    sock.settimeout(args.timeout)

    n_samples = n_pkt = bad = 0
    lost_pkts = lost_samples = 0
    gaps = []
    first = prev = None
    t0 = None
    tty = sys.stderr.isatty() and not args.quiet
    max_bytes = int(args.max_mb * 1024 * 1024) if args.max_mb else None

    if not args.quiet:
        print(f"logging {args.bind}:{args.port} -> {raw_path}  (Ctrl-C to stop)")

    f = open(raw_path, "wb")
    try:
        while True:
            try:
                data, _peer = sock.recvfrom(65535)
            except socket.timeout:
                if n_pkt == 0:
                    print(f"nothing received for {args.timeout:.0f}s -- is the board "
                          f"streaming to this host?", file=sys.stderr)
                    f.close()
                    raw_path.unlink(missing_ok=True)
                    return 2
                break
            except KeyboardInterrupt:
                break

            try:
                records = list(parse_all(data))
            except ValueError:
                bad += 1
                continue

            # One datagram can carry several records (LiteEth flushes its TX
            # FIFO on full as well as on our packet boundary), so the stop
            # condition needs a flag -- a bare break would only leave the
            # inner loop and the capture would never end.
            stop = False
            for hdr, payload in records:
                if first is None:
                    first = hdr
                    t0 = time.monotonic()
                    started = datetime.now(timezone.utc).isoformat()
                    if not args.quiet:
                        print(f"ch{hdr.channel}  {hdr.sample_rate:,.1f} sps  "
                              f"{hdr.n_samples} samples/packet")

                if prev is not None:
                    gap = (hdr.seq - prev.seq) & 0xffffffff
                    if gap != 1:
                        missing = (hdr.emitted - prev.emitted - prev.n_samples) & 0xffffffff
                        lost_pkts += gap - 1
                        lost_samples += missing
                        gaps.append({"at_sample": n_samples,
                                     "missing_samples": int(missing),
                                     "missing_packets": int(gap - 1)})
                        if args.pad_gaps and missing:
                            f.write(b"\x00\x00" * missing)
                            n_samples += missing

                f.write(payload)
                n_samples += hdr.n_samples
                n_pkt += 1
                prev = hdr

                if tty and n_pkt % 20 == 0:
                    el = time.monotonic() - t0
                    print(f"\r  {el:7.1f}s  {n_samples:>11,} samples  "
                          f"{f.tell()/1e6:6.2f} MB  lost {lost_pkts}",
                          end="", file=sys.stderr)

                if args.seconds and (time.monotonic() - t0) >= args.seconds:
                    stop = True
                if max_bytes and f.tell() >= max_bytes:
                    stop = True
                if stop:
                    break
            if stop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        sock.close()
        if tty:
            print(file=sys.stderr)

    if first is None:
        return 2

    elapsed = time.monotonic() - t0
    fs = first.sample_rate
    meta = {
        "file": raw_path.name,
        "format": "raw-uint16-le",
        "description": "one channel, 10-bit MCP3008 counts 0..1023, no header",
        "started_utc": started,
        "duration_s": round(elapsed, 3),
        "channel": first.channel,
        "sample_rate_hz": fs,
        "period_sys_clocks": first.period,
        "sys_clk_freq_hz": first.sys_clk_freq,
        "samples_per_packet": first.n_samples,
        "samples": n_samples,
        "packets": n_pkt,
        "unparseable_packets": bad,
        "packets_lost_in_transit": lost_pkts,
        "samples_lost_in_transit": lost_samples,
        "board_dropped_samples": int((prev.dropped - first.dropped) & 0xffffffff),
        "gaps_padded_with_zeros": bool(args.pad_gaps),
        "gaps": gaps,
        "adc": {"bits": 10, "counts_full_scale": 1023, "vref_mv": 3300,
                "mv_per_count": 3300 / 1023},
        "radar": {"module": "CDM324", "f0_hz": 24.125e9, "hz_per_m_per_s": HZ_PER_MS},
    }

    wav_path = None
    if args.wav:
        import wave
        wav_path = Path(f"{name}.wav")
        counts = array.array("H")
        counts.frombytes(raw_path.read_bytes())
        if sys.byteorder == "big":
            counts.byteswap()
        # Centre on mid-scale and scale 10 bits up into 16. A convenience copy
        # for audio tools; the .raw remains the reference.
        pcm = array.array("h", (max(-32768, min(32767, (c - 512) * 32)) for c in counts))
        if sys.byteorder == "big":
            pcm.byteswap()
        with wave.open(str(wav_path), "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(int(round(fs)))
            w.writeframes(pcm.tobytes())
        meta["wav"] = {"file": wav_path.name, "transform": "(count - 512) * 32",
                       "note": "16-bit mono; the .raw is the reference copy"}

    meta_path.write_text(json.dumps(meta, indent=2) + "\n")

    if not args.quiet:
        print(f"\n{raw_path}  {raw_path.stat().st_size/1e6:.2f} MB, "
              f"{n_samples:,} samples, {elapsed:.1f}s at {fs:,.1f} sps")
        if wav_path:
            print(f"{wav_path}  {wav_path.stat().st_size/1e6:.2f} MB")
        print(f"{meta_path}")
        if lost_pkts or meta["board_dropped_samples"]:
            print(f"\nWARNING: this capture has gaps -- "
                  f"{lost_pkts} packets / {lost_samples:,} samples lost in transit, "
                  f"{meta['board_dropped_samples']:,} dropped by the board."
                  + ("" if args.pad_gaps else
                     "\n  Without --pad-gaps the hole is closed up, so every later "
                     "sample sits\n  earlier than it really was. See `gaps` in the JSON."))
        else:
            print("no loss")
    return 0


if __name__ == "__main__":
    sys.exit(main())
