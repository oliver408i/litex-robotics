#!/usr/bin/env python3
"""Read or change the SPI flash's block-protection bits, over Etherbone.

The Colorlight i9 ships with its W25Q64 **fully write-protected**: status
register 1 reads 0x1C, i.e. BP2:BP0 = 111 with TB=0 and CMP=0, which locks the
entire array. Measured 2026-09-13.

That failure mode is quiet and very confusing, which is the reason this tool
exists. The chip still accepts WREN and the sector-erase opcode; it simply
declines to modify a protected block. WIP therefore never sets, so the BIOS's
"wait while busy" loop falls straight through, and `spiflash_erase_range` then
verifies the sector and prints `Error: location 0x... not erased` -- once per
byte, 4096 times per sector, which floods the console for the better part of an
hour and looks exactly like a hang in whatever command you run next.

    .venv/bin/python tools/spiflash_protect.py                # read
    .venv/bin/python tools/spiflash_protect.py --unprotect    # BP -> 000
    .venv/bin/python tools/spiflash_protect.py --protect      # BP -> 111

Writes both status bytes in one Write Status Register (0x01) transaction so
that SR2 -- and with it QE, which is set on this board -- is preserved. Writing
only one byte clears SR2 on many Winbond parts.

**This is a non-volatile change**: it survives power cycles, which is what you
want (otherwise every flash attempt would need it again), but it does mean the
board no longer ships-state-protected until you `--protect` it again. It
changes no data.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

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

CS, PHY, RXTX, STAT = 0xf0002808, 0xf000280c, 0xf0002810, 0xf0002814
RX_READY = 1 << 1


class Spi:
    """Bit-bangs the LiteSPI master's CSRs, one byte per transfer."""

    def __init__(self, bus):
        self.bus = bus

    def _drain(self):
        while self.bus.read(STAT) & RX_READY:
            self.bus.read(RXTX)

    def _byte(self, v):
        self.bus.write(RXTX, v & 0xff)
        for _ in range(1000):
            if self.bus.read(STAT) & RX_READY:
                break
        return self.bus.read(RXTX) & 0xff

    def xfer(self, tx, nread=0):
        """Send `tx` (list of bytes), then clock `nread` more; CS held across."""
        self._drain()
        self.bus.write(PHY, (1 << 16) | (1 << 8) | 8)   # mask=1, width=1, 8 bits
        self.bus.write(CS, 1)
        try:
            for b in tx:
                self._byte(b)
            return [self._byte(0x00) for _ in range(nread)]
        finally:
            self.bus.write(CS, 0)


def decode(sr1, sr2):
    bp = (sr1 >> 2) & 7
    return (f"SR1=0x{sr1:02x} SR2=0x{sr2:02x}  "
            f"BP2:0={bp:03b} TB={(sr1>>5)&1} CMP={(sr2>>6)&1} "
            f"SRP0={(sr1>>7)&1} WEL={(sr1>>1)&1} BUSY={sr1&1} QE={(sr2>>1)&1}"
            + ("   -> WHOLE ARRAY PROTECTED" if bp == 7 else
               "   -> unprotected" if bp == 0 else
               "   -> partially protected"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--unprotect", action="store_true", help="clear BP2:BP0")
    g.add_argument("--protect", action="store_true", help="set BP2:BP0 (ship state)")
    ap.add_argument("--csr-csv", default="build/colorlight_i5/csr.csv")
    args = ap.parse_args()

    bus = RemoteClient(csr_csv=args.csr_csv)
    bus.open()
    try:
        spi = Spi(bus)
        jedec = spi.xfer([0x9f], 3)
        print(f"JEDEC   : {jedec[0]:02x} {jedec[1]:02x} {jedec[2]:02x}"
              + ("  (Winbond W25Q64)" if jedec[:2] == [0xef, 0x40] else ""))
        sr1 = spi.xfer([0x05], 1)[0]
        sr2 = spi.xfer([0x35], 1)[0]
        print(f"before  : {decode(sr1, sr2)}")

        if not (args.unprotect or args.protect):
            return 0

        new1 = (sr1 & ~0x1c) | (0x1c if args.protect else 0x00)
        spi.xfer([0x06])                       # WREN
        spi.xfer([0x01, new1, sr2])            # write SR1+SR2 together
        for _ in range(50):                    # wait for WIP to clear
            if not (spi.xfer([0x05], 1)[0] & 1):
                break
            time.sleep(0.05)

        sr1 = spi.xfer([0x05], 1)[0]
        sr2 = spi.xfer([0x35], 1)[0]
        print(f"after   : {decode(sr1, sr2)}")
        want = 0x1c if args.protect else 0x00
        if (sr1 & 0x1c) != want:
            print("the protection bits did not change. If SRP0=1 the /WP pin "
                  "gates status-register writes.", file=sys.stderr)
            return 1
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
