#!/usr/bin/env python3
"""Read a region of the SoC's memory over Etherbone into a file.

Written to back up the i9's SPI flash before overwriting it -- the board ships
with a factory image and erasing it is not reversible -- but it just reads an
address range.

    .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &
    .venv/bin/python tools/etherbone_backup.py ~/i9_flash_backup_2MB.bin

Defaults read 2 MB from 0x00800000, the memory-mapped SPI flash window, which
covers the ~1.15 MB a bitstream occupies with room to spare.

**Reads are far slower than writes** -- about 64 KB/s against 11 MB/s -- because
each burst is a request/response round trip through litex_server's Python,
while writes are fire-and-forget. 2 MB therefore takes ~35 s. That asymmetry is
also why verification should use the BIOS's on-device `crc` rather than reading
data back; this tool prints the CRC32 of what it saved so you can compare:

    crc 0x00800000 2097152      # on the board's console
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

BURST = 255


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", help="file to write")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x00800000,
                    help="start address (default 0x00800000, the flash window)")
    ap.add_argument("--size", type=lambda x: int(x, 0), default=2 * 1024 * 1024,
                    help="bytes to read (default 2 MiB)")
    ap.add_argument("--csr-csv", default="build/colorlight_i5/csr.csv")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1234)
    args = ap.parse_args()

    if os.path.exists(args.out):
        print(f"refusing to overwrite an existing backup: {args.out}",
              file=sys.stderr)
        return 1

    total = args.size // 4
    bus = RemoteClient(csr_csv=args.csr_csv, host=args.host, port=args.port)
    bus.open()
    words: list[int] = []
    t0 = time.time()
    try:
        while len(words) < total:
            n = min(BURST, total - len(words))
            words += bus.read(args.addr + 4 * len(words), n)
            done = len(words) / total
            print(f"\r  {done*100:5.1f}%  {len(words)*4/1024:7.0f} KB", end="",
                  flush=True)
    finally:
        bus.close()
    print()

    blob = struct.pack("<%dI" % total, *words)
    with open(args.out, "wb") as f:
        f.write(blob)

    crc = zlib.crc32(blob) & 0xffffffff
    print(f"saved  : {len(blob)} bytes to {args.out} in {time.time()-t0:.1f} s")
    print(f"crc32  : {crc:08x}")
    print(f"verify : crc 0x{args.addr:08x} {args.size}    # on the board's console")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
