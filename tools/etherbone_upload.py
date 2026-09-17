#!/usr/bin/env python3
"""Upload a file into the SoC's memory over Etherbone, and report its CRC32.

Written for staging a bitstream in SDRAM before programming the SPI flash, but
there is nothing bitstream-specific here -- it moves a file to an address.

    .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &
    .venv/bin/python tools/etherbone_upload.py build/.../foo.bit 0x40100000

Prints the length and the host-side CRC32 so the caller can compare against the
BIOS's `crc <addr> <len>`, which is the cheap way to verify: an 8-hex-digit
answer instead of reading 1.15 MB back (0.4 s against ~18 s). The BIOS uses
zlib's CRC32, so `zlib.crc32` here matches it exactly.

Two things this gets right that are easy to get wrong by hand:

* **Bursts are capped at 255 words.** That is the Etherbone protocol maximum,
  and it must also not exceed the gateware's `buffer_depth` -- a longer burst
  overflows the record FIFO and deadlocks the core, taking the whole LiteEth
  stack down with it (the board stops answering even ICMP). Builds here use
  `buffer_depth=256`; see targets/colorlight_i9/README.md.

* **Do not stage at 0x10000000.** That is the CPU's 8 KB SRAM, where the
  running BIOS keeps its stack -- writing there corrupts the live BIOS and
  every console command starts answering "Command not found". 0x40100000 is a
  safe spot in SDRAM, clear of anything the BIOS uses at its prompt.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time
import zlib

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LITEX_SETUP_ROOT = os.path.join(REPO_ROOT, "litex-setup")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
for _rel in ("litex", "litex-boards", "migen", "litedram", "liteeth", "litespi",
             "litesdcard"):
    _p = os.path.join(LITEX_SETUP_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from litex.tools.litex_client import RemoteClient

BURST = 255                      # Etherbone protocol maximum, in 32-bit words
SRAM_TRAP = (0x10000000, 0x10002000)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="file to upload")
    ap.add_argument("addr", type=lambda x: int(x, 0),
                    help="destination address, e.g. 0x40100000")
    ap.add_argument("--csr-csv", default="build/colorlight_i5/csr.csv")
    ap.add_argument("--burst", type=int, default=BURST,
                    help="words per Etherbone burst (max 255, and must not "
                         "exceed the gateware's buffer_depth)")
    ap.add_argument("--pace", type=float, default=0.0,
                    help="seconds to sleep between bursts. Bulk writes into "
                         "SDRAM at full rate have been observed to wedge the "
                         "Wishbone bus -- the CPU goes silent while gateware "
                         "ICMP keeps answering -- so pacing is the knob to "
                         "reach for when a large upload kills the console.")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1234)
    args = ap.parse_args()

    blob = open(args.file, "rb").read()
    if not blob:
        print(f"{args.file} is empty", file=sys.stderr)
        return 1
    pad = (-len(blob)) % 4
    data = blob + b"\x00" * pad
    words = list(struct.unpack("<%dI" % (len(data) // 4), data))

    lo, hi = SRAM_TRAP
    if lo <= args.addr < hi:
        print(f"refusing: 0x{args.addr:08x} is the CPU's SRAM, where the running "
              f"BIOS keeps its stack.\n  Stage in SDRAM instead (0x40100000).",
              file=sys.stderr)
        return 1

    bus = RemoteClient(csr_csv=args.csr_csv, host=args.host, port=args.port)
    bus.open()
    try:
        burst = max(1, min(args.burst, BURST))
        t0 = time.time()
        for i in range(0, len(words), burst):
            bus.write(args.addr + 4 * i, words[i:i + burst])
            if args.pace:
                time.sleep(args.pace)
        dt = time.time() - t0
    finally:
        bus.close()

    crc = zlib.crc32(data) & 0xffffffff
    print(f"file   : {args.file}")
    print(f"bytes  : {len(blob)} (+{pad} pad = {len(data)})")
    print(f"addr   : 0x{args.addr:08x}")
    print(f"time   : {dt:.2f} s ({len(data)/dt/1e6:.1f} MB/s)")
    print(f"crc32  : {crc:08x}")
    print(f"verify : crc 0x{args.addr:08x} {len(data)}")
    # Machine-readable tail, for scripts that drive this.
    print(f"RESULT {len(data)} {crc:08x}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
