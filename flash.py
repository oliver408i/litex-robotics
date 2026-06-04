#!/usr/bin/env python3
"""WiFi flasher for the IcePi Zero (host side of software/winc_loader).

Flashes images over WiFi into the board's SPI flash. Slot presets (combinable
in one session):

  ./flash.py --app firmware.bin                # app .fbi  @0x280000 (fbi auto)
  ./flash.py --loader winc_loader.bin          # loader    @0x200000 (fbi auto)
  ./flash.py --bios bios.bin                   # BIOS      @0x100000 (raw)
  ./flash.py --bitstream icepi_zero.bit        # bitstream @0x000000 (raw)
  ./flash.py file.bin --offset 0x300000 [--fbi]  # explicit (legacy form)

Getting the board into the loader is automatic where possible (the loader is
resident: BIOS flash-boots it, and it chain-boots the app unless asked to
stay):
  1. loader already running         -> proceed
  2. app running with loader_hook   -> "WFLE" datagram reboots it into the
     (software/common/loader_hook.c)   loader, cable-free
  3. --port /dev/ttyUSBx given      -> spam the loader's 200 ms boot grace
                                       window with "wflSTAY!" and ask you to
                                       press the reset button

After flashing: app/loader/BIOS apply via soft reset (WFLR -> loader chains
the new app); a new bitstream needs a power-cycle.

Protocol notes: chunk-indexed UDP (loss/reorder/dup-proof), idempotent control
packets, paced under the WINC's ~290 pps knee. Resolve the hostname once --
per-packet mDNS lookups cost ~23 ms each.
"""
import argparse
import socket
import struct
import sys
import time
import zlib

CHUNK = 1408          # max payload per WFLD (board cap; 1472 B datagram)

LOADER_PORT = 5557
HOOK_PORT   = 5558    # software/common/loader_hook.c
GRACE_MAGIC = b"wflSTAY!"

SLOTS = {   # name: (offset, wrap_fbi)
    "bitstream": (0x000000, False),
    "bios":      (0x100000, False),
    "loader":    (0x200000, True),
    "app":       (0x280000, True),
}
BOUNDARIES = [("BIOS @0x100000", 0x100000), ("loader @0x200000", 0x200000),
              ("app @0x280000", 0x280000)]

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
    ap.add_argument("file", nargs="?", help="image for the legacy --offset form")
    ap.add_argument("--offset", type=lambda x: int(x, 0),
                    help="flash offset for the legacy form (4 KB-aligned)")
    ap.add_argument("--fbi", action="store_true",
                    help="legacy form: prepend the LiteX flashboot header")
    for slot in SLOTS:
        ap.add_argument(f"--{slot}", metavar="FILE",
                        help=f"flash FILE to the {slot} slot @0x{SLOTS[slot][0]:06x}")
    ap.add_argument("--host", default="icepi.local")
    ap.add_argument("--udp-port", type=int, default=LOADER_PORT)
    ap.add_argument("--port", metavar="TTY", default=None,
                    help="serial port (e.g. /dev/ttyUSB0) for the grace-window "
                         "entry path when no loader/hook answers over WiFi")
    ap.add_argument("--reboot", action="store_true",
                    help="send WFLR when done even if only bitstream/BIOS were flashed")
    ap.add_argument("--no-reboot", action="store_true",
                    help="never send WFLR (stay in the loader when done)")
    ap.add_argument("--pps", type=int, default=250,
                    help="data pacing in packets/s (board knee ~290; default 250)")
    return ap.parse_args()


def build_jobs(args):
    """[(label, offset, data)] from slot presets and/or the legacy form."""
    jobs = []
    for slot, (offset, wrap) in SLOTS.items():
        path = getattr(args, slot)
        if path is None:
            continue
        with open(path, "rb") as f:
            data = f.read()
        if wrap:
            data = struct.pack("<II", len(data), zlib.crc32(data)) + data
        jobs.append((f"{slot}:{path}", offset, data))
    if args.file is not None:
        if args.offset is None:
            sys.exit("positional file needs --offset (or use a slot preset)")
        with open(args.file, "rb") as f:
            data = f.read()
        if args.fbi:
            data = struct.pack("<II", len(data), zlib.crc32(data)) + data
        jobs.append((args.file, args.offset, data))
    if not jobs:
        sys.exit("nothing to flash -- give a slot preset or FILE --offset X")
    return sorted(jobs, key=lambda j: j[1])


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


# ---- getting the board into the loader ------------------------------------

def probe_loader(s, dst, tries=2, timeout=0.7):
    return request(s, dst, b"WFLS", b"WFLT", tries=tries, timeout=timeout) is not None


def enter_via_hook(s, host_ip):
    """Ask a running app (loader_hook) to reboot into the loader."""
    hook_dst = (host_ip, HOOK_PORT)
    if request(s, hook_dst, b"WFLE", b"WFLE", tries=3, timeout=0.7) is None:
        return False
    print("  app acknowledged -- waiting for the loader to join WiFi...")
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        if probe_loader(s, (host_ip, LOADER_PORT), tries=1, timeout=1.0):
            return True
    return False


def enter_via_serial(s, host_ip, tty):
    """Spam the loader's boot grace window; the user provides the reset."""
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed -- pip install pyserial (or enter the "
                 "loader another way)")
    print(f"  spamming grace-window magic on {tty}")
    print("  >>> press the board's RESET button now <<<")
    deadline = time.monotonic() + 30
    with serial.Serial(tty, 1_000_000, timeout=0) as ser:
        while time.monotonic() < deadline:
            for _ in range(20):                 # ~grace-window worth of spam
                ser.write(GRACE_MAGIC)
                time.sleep(0.01)
            if probe_loader(s, (host_ip, LOADER_PORT), tries=1, timeout=0.5):
                return True
    return False


def ensure_loader(s, host_ip, args):
    dst = (host_ip, LOADER_PORT)
    print("looking for the loader...")
    if probe_loader(s, dst):
        print("  loader is up")
        return
    print(f"  no loader -- trying the app's loader_hook (UDP :{HOOK_PORT})")
    if enter_via_hook(s, host_ip):
        print("  loader is up")
        return
    if args.port:
        print("  no hook -- trying the serial grace window")
        if enter_via_serial(s, host_ip, args.port):
            print("  loader is up")
            return
        sys.exit("loader did not come up after reset -- check wifi_secrets/AP")
    sys.exit(f"board unreachable: no loader on :{LOADER_PORT}, no app hook on "
             f":{HOOK_PORT}.\n"
             "Options: pass --port /dev/ttyUSBx and press reset when asked, or\n"
             "serialboot software/winc_loader/winc_loader.bin manually once.")


# ---- transfer ---------------------------------------------------------------

def send_chunks(s, dst, data, indices, pps):
    gap = 1.0 / pps
    next_t = time.monotonic()
    for i in indices:
        s.sendto(b"WFLD" + struct.pack("<I", i) + data[i * CHUNK:(i + 1) * CHUNK], dst)
        next_t += gap
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)


def flash_image(s, dst, label, offset, data, pps):
    crc = zlib.crc32(data)
    total = (len(data) + CHUNK - 1) // CHUNK
    print(f"{label}: {len(data)} B ({total} chunks) -> flash @0x{offset:06x}")
    for name, b in BOUNDARIES:
        if offset < b < offset + len(data):
            print(f"  WARNING: image crosses {name} -- it will be overwritten")

    # BEGIN (re-ack of an identical session resumes instead of restarting)
    begin = b"WFLB" + struct.pack("<IIIH", offset, len(data), crc, CHUNK)
    reply = request(s, dst, begin, b"WFLA")
    if reply is None:
        sys.exit("no reply to BEGIN -- loader gone?")
    status, _ver, flash_size, _img_max = struct.unpack("<BBxxII", reply[4:16])
    if status != 0:
        sys.exit(f"BEGIN refused: {ST_NAMES.get(status, status)}")
    if offset + len(data) > flash_size:
        sys.exit(f"image does not fit: flash is {flash_size} B")

    # DATA blast + stat/resend until the board holds the complete image
    t0 = time.monotonic()
    pending = list(range(total))
    while True:
        send_chunks(s, dst, data, pending, pps)
        reply = request(s, dst, b"WFLS", b"WFLT")
        if reply is None:
            sys.exit("no reply to STAT -- board gone?")
        got, board_total, nmiss = struct.unpack("<IIH", reply[4:14])
        print(f"\r  transfer: {got}/{board_total} chunks", end="", flush=True)
        if got == board_total:
            break
        pending = [struct.unpack_from("<I", reply, 16 + 4 * i)[0]
                   for i in range(nmiss)]
    dt = time.monotonic() - t0
    print(f"  ({len(data)/dt/1e6:.2f} MB/s)")

    # PROGRAM: erase+program+verify runs with the radio unserviced -- keep
    # retrying, the result is cached and re-sent.
    print("  programming (erase + write + verify)...", flush=True)
    reply = request(s, dst, b"WFLP", b"WFLZ", tries=40, timeout=2.0)
    if reply is None:
        sys.exit("no reply to PROGRAM within 80 s")
    status, ms, flash_crc = struct.unpack("<BxxxII", reply[4:16])
    if status != 0:
        sys.exit(f"PROGRAM failed: {ST_NAMES.get(status, status)}")
    print(f"  flashed + verified in {ms/1000:.1f} s (flash crc 0x{flash_crc:08x})")


def main():
    args = parse_args()
    jobs = build_jobs(args)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    host_ip = socket.gethostbyname(args.host)   # resolve ONCE
    dst = (host_ip, args.udp_port)

    ensure_loader(s, host_ip, args)

    t0 = time.monotonic()
    for label, offset, data in jobs:
        flash_image(s, dst, label, offset, data, args.pps)
    print(f"total: {time.monotonic()-t0:.1f} s")

    flashed = {label.split(":")[0] for label, _, _ in jobs}
    want_reboot = (args.reboot or flashed & {"app", "loader", "bios"}) \
                  and not args.no_reboot
    if want_reboot:
        # best-effort: the ack may die with the resetting board
        request(s, dst, b"WFLR", b"WFLR", tries=3, timeout=0.5)
        print("rebooted -- loader will chain-boot the app")
    if "bitstream" in flashed:
        print("NOTE: a new bitstream needs a POWER-CYCLE to load")


if __name__ == "__main__":
    main()
