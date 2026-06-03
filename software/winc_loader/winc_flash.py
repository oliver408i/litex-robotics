#!/usr/bin/env python3
"""Host-side uploader for the WiFi flash-loader firmware (winc_loader).

Sends an image over UDP into the board's SDRAM and has the loader program it
into SPI flash at --offset: bitstream @0x0, BIOS @0x100000, firmware
@0x200000 (firmware needs --fbi so the BIOS flashboot finds its length+CRC
header). Chunk-indexed datagrams + a stat/resend loop make UDP loss harmless;
every control packet is retried, every board reply is idempotent.

Examples:
  ./winc_flash.py firmware.bin --offset 0x200000 --fbi --reboot
  ./winc_flash.py icepi_zero.bin --offset 0x0          # then power-cycle
"""
import argparse
import socket
import struct
import sys
import time
import zlib

CHUNK = 1408          # max payload per WFLD (board cap; 1472 B datagram)

ST_NAMES = {
    0: "OK",
    1: "bad arguments (alignment/size/chunk)",
    2: "image incomplete",
    3: "SDRAM image CRC mismatch",
    4: "flash verify FAILED",
    5: "no session (send BEGIN first)",
}


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="image to flash (.bin/.bit/...)")
    ap.add_argument("--host", default="icepi.local")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--offset", type=lambda x: int(x, 0), required=True,
                    help="flash offset (4 KB-aligned), e.g. 0x200000")
    ap.add_argument("--fbi", action="store_true",
                    help="prepend the LiteX flashboot header (u32le length + "
                         "u32le crc32) -- required for the firmware slot")
    ap.add_argument("--reboot", action="store_true",
                    help="ctrl_reset the SoC after a successful flash")
    ap.add_argument("--pps", type=int, default=250,
                    help="data pacing in packets/s (board knee ~290; default 250)")
    return ap.parse_args()


def request(s, dst, pkt, magic, tries=5, timeout=1.0):
    """Send pkt until a reply starting with magic arrives (idempotent cmds)."""
    s.settimeout(timeout)
    for _ in range(tries):
        s.sendto(pkt, dst)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                reply, _ = s.recvfrom(2048)
            except socket.timeout:
                break
            if reply[:4] == magic:
                return reply
            # stray/stale reply (e.g. late WFLT) -- keep listening
    return None


def send_chunks(s, dst, data, indices, pps):
    gap = 1.0 / pps
    next_t = time.monotonic()
    for i in indices:
        s.sendto(b"WFLD" + struct.pack("<I", i) + data[i * CHUNK:(i + 1) * CHUNK], dst)
        next_t += gap
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)


def main():
    args = parse_args()

    with open(args.file, "rb") as f:
        data = f.read()
    if args.fbi:
        data = struct.pack("<II", len(data), zlib.crc32(data)) + data
    crc = zlib.crc32(data)
    total = (len(data) + CHUNK - 1) // CHUNK

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Resolve ONCE -- a hostname tuple in sendto() costs an mDNS lookup per
    # packet (~23 ms each, measured; see winc_throughput.py).
    dst = (socket.gethostbyname(args.host), args.port)
    print(f"{args.file}: {len(data)} B ({total} chunks) "
          f"-> {dst[0]} flash @0x{args.offset:06x}")

    # BEGIN (re-ack of an identical session resumes instead of restarting)
    begin = b"WFLB" + struct.pack("<IIIH", args.offset, len(data), crc, CHUNK)
    reply = request(s, dst, begin, b"WFLA")
    if reply is None:
        sys.exit("no reply to BEGIN -- is winc_loader running on the board?")
    status, _ver, flash_size, img_max = struct.unpack("<BBxxII", reply[4:16])
    if status != 0:
        sys.exit(f"BEGIN refused: {ST_NAMES.get(status, status)}")
    if args.offset + len(data) > flash_size:
        sys.exit(f"image does not fit: flash is {flash_size} B")

    # DATA blast + stat/resend until the board holds the complete image
    t0 = time.monotonic()
    pending = list(range(total))
    while True:
        send_chunks(s, dst, data, pending, args.pps)
        reply = request(s, dst, b"WFLS", b"WFLT")
        if reply is None:
            sys.exit("no reply to STAT -- board gone?")
        got, board_total, nmiss = struct.unpack("<IIH", reply[4:14])
        print(f"\r  transfer: {got}/{board_total} chunks", end="", flush=True)
        if got == board_total:
            break
        pending = [struct.unpack_from("<I", reply, 16 + 4 * i)[0]
                   for i in range(nmiss)]
        if not pending:   # >300 holes: stat again for the next batch
            pending = []
    dt = time.monotonic() - t0
    print(f"  ({len(data)/dt/1e6:.2f} MB/s)")

    # PROGRAM: erase+program+verify takes seconds-tens of seconds with the
    # board not servicing the radio -- keep retrying, the result is cached.
    print("  programming (erase + write + verify)...", flush=True)
    reply = request(s, dst, b"WFLP", b"WFLZ", tries=40, timeout=2.0)
    if reply is None:
        sys.exit("no reply to PROGRAM within 80 s")
    status, ms, flash_crc = struct.unpack("<BxxxII", reply[4:16])
    if status != 0:
        sys.exit(f"PROGRAM failed: {ST_NAMES.get(status, status)}")
    print(f"  flashed + verified in {ms/1000:.1f} s (flash crc 0x{flash_crc:08x})")
    print(f"total: {time.monotonic()-t0:.1f} s")

    if args.reboot:
        # best-effort: the ack may die with the resetting board
        request(s, dst, b"WFLR", b"WFLR", tries=3, timeout=0.5)
        print("reboot requested (bitstream changes need a power-cycle instead)")


if __name__ == "__main__":
    main()
