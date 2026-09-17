#!/usr/bin/env python3
"""Run one LiteX BIOS command and return its output. Non-interactive, drains first.

    .venv/bin/python tools/soc_console.py 'crc 0x40100000 1159816' --expect CRC32

Exists because `socboot --send` is the wrong tool for scripted verification, in
a way that is actively dangerous rather than merely inconvenient:

* **It matches `--expect` against everything it has read, including backlog.**
  A stale `CRC32:` still sitting in the buffer from an earlier command matches
  immediately, so a verify step can pass without the command having run. When
  the thing being verified is "did the right bytes land in flash", a false pass
  is the worst possible outcome.

* **It does not drain before sending.** The BIOS can be mid-flood -- a
  `flash_write` over a region that was not erased prints an error PER BYTE, and
  at 115200 baud that buries the next command's reply for the better part of an
  hour. The command looks hung; it is actually fine and its answer is somewhere
  behind a megabyte of complaints.

So: drain to silence, send, then read only what arrives afterwards.

Exit status is 0 if `--expect` matched (or no `--expect` was given and the
command completed), 1 on timeout, 2 if `--fail` matched first.
"""
from __future__ import annotations

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit("pyserial not installed (.venv/bin/pip install pyserial)")

DEFAULT_PORT = "/dev/icelink-uart"
PROMPT = b"litex>"


def drain(port, quiet_for=0.5, limit=20.0) -> int:
    """Read until the console has been silent for `quiet_for` seconds."""
    n, last, start = 0, time.time(), time.time()
    while time.time() - last < quiet_for and time.time() - start < limit:
        d = port.read(4096)
        if d:
            n += len(d)
            last = time.time()
    port.reset_input_buffer()
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", help="BIOS command to run, e.g. 'crc 0x40100000 4096'")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--speed", type=int, default=115200)
    ap.add_argument("--expect", help="regex that must appear in the reply")
    ap.add_argument("--fail", help="regex that aborts immediately if seen")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--quiet", action="store_true", help="print only the reply")
    args = ap.parse_args()

    try:
        port = serial.Serial(args.port, args.speed, timeout=0.2)
    except Exception as e:
        print(f"cannot open {args.port}: {e}\n"
              f"  (try: sg dialout -c '...')", file=sys.stderr)
        return 1

    expect_re = re.compile(args.expect) if args.expect else None
    fail_re = re.compile(args.fail) if args.fail else None

    with port:
        backlog = drain(port)
        if backlog and not args.quiet:
            print(f"[console] drained {backlog} bytes of backlog", file=sys.stderr)

        port.write(args.command.encode() + b"\n")
        port.flush()

        buf = b""
        deadline = time.time() + args.timeout
        rc = 0 if expect_re is None else 1
        while time.time() < deadline:
            d = port.read(4096)
            if not d:
                # Idle at a prompt with nothing outstanding: the command is done.
                if expect_re is None and buf.rstrip().endswith(PROMPT):
                    break
                continue
            buf += d
            text = buf.decode("utf-8", "replace")
            if fail_re and fail_re.search(text):
                rc = 2
                break
            if expect_re and expect_re.search(text):
                rc = 0
                break

    text = buf.decode("utf-8", "replace").replace("\r", "")
    # Drop the echoed command line; keep the reply.
    lines = [l for l in text.split("\n") if l.strip() and args.command not in l]
    print("\n".join(lines).strip())

    if rc == 1 and not args.quiet:
        print(f"[console] no match for {args.expect!r} within {args.timeout:.0f}s",
              file=sys.stderr)
    if rc == 2 and not args.quiet:
        print(f"[console] matched --fail {args.fail!r}", file=sys.stderr)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
