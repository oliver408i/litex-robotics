#!/usr/bin/env python3
"""ESP32-C3 flasher for the IcePi Zero (host side of software/c3_flash_esp).

  ./flash.py --app firmware.bin --port /dev/ttyACM0       # app .fbi  @0x280000 (fbi auto)
  ./flash.py --loader [c3_flash.bin] --port /dev/ttyACM0  # loader    @0x200000 (fbi auto)
  ./flash.py --bios [bios.bin] --port /dev/ttyACM0        # BIOS      @0x100000 (raw)
  ./flash.py --bitstream [icepi_zero.bit] --port /dev/ttyACM0  # bitstream @0x000000 (raw)
  ./flash.py --bitstream --bios --loader --port /dev/ttyACM0   # full system from build outputs
  ./flash.py file.bin --offset 0x300000 [--fbi] --port /dev/ttyACM0  # explicit offset
  ./flash.py --reset --port /dev/ttyACM0                  # just reboot the SoC
  ./flash.py --app myapp.bin --boot-app --port /dev/ttyACM0  # flash + chain-boot the app
  ./flash.py --boot-app --port /dev/ttyACM0                  # boot whatever's in the app slot now

Slots are combinable in one session; bare --bitstream/--bios/--loader pick up
the standard build artifacts. --port is the ESP32-C3's USB-CDC port (the
FPGA's own FTDI port is only used for watching boot output, e.g. litex_term
or a plain serial monitor -- flashing goes entirely through the C3). The
loader is flash-resident and comes up on its own after any reset/power-cycle
(no litex_term --kernel step needed); this tool just needs it already running
to talk to it. The boot chain, wire protocol and C3 link design are
documented in docs/boot_chain.md and docs/c3_loader.md.

--boot-app chain-boots whatever's in the app slot: a one-shot request (sticky
flag, self-clears), so the *next* reset lands back in the resident loader
automatically -- no separate "return to loader" step needed. Not the old
WiFi/WINC-era --run (SDRAM-stage-and-execute, no flash write): the app must
already be flashed first.
"""
import argparse
import os
import struct
import sys
import time
import zlib

REPO = os.path.dirname(os.path.abspath(__file__))

WINDOW_PAGES = 16   # must match software/c3_flash_esp/src/main.cpp's WINDOW_PAGES

SLOTS = {   # name: (offset, wrap_fbi, default_file or None)
    "bitstream": (0x000000, False, "build/icepi_zero/gateware/icepi_zero.bit"),
    "bios":      (0x100000, False, "build/icepi_zero/software/bios/bios.bin"),
    "loader":    (0x200000, True,  "software/c3_flash/c3_flash.bin"),
    "app":       (0x280000, True,  None),   # apps vary -- always explicit;
                                             # chain-booted via --boot-app.
}
BUILD_HINTS = {
    "bitstream": ".venv/bin/python icepi_zero_c3flash.py --build",
    "bios":      ".venv/bin/python icepi_zero_c3flash.py --build",
    "loader":    "make -C software/c3_flash",
}
BOUNDARIES = [("BIOS @0x100000", 0x100000), ("loader @0x200000", 0x200000),
              ("app @0x280000", 0x280000)]

# Status bytes the C3 (software/c3_flash_esp) reports over the wire protocol.
C3_STATUS = {
    0x00: "OK",
    0x05: "CRC MISMATCH",
    0xE0: "header timeout",
    0xE1: "erase failed",
    0xE2: "rx timeout (data underrun)",
    0xE3: "program failed",
    0xE4: "crc-read failed",
}


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="image for the legacy --offset form")
    ap.add_argument("--offset", type=lambda x: int(x, 0),
                    help="flash offset for the legacy form (4 KB-aligned)")
    ap.add_argument("--fbi", action="store_true",
                    help="legacy form: prepend the LiteX flashboot header")
    for slot, (offset, _wrap, default) in SLOTS.items():
        if default is None:
            ap.add_argument(f"--{slot}", metavar="FILE",
                            help=f"flash FILE to the {slot} slot @0x{offset:06x}")
        else:
            # optional value: bare flag picks up the standard build artifact
            ap.add_argument(f"--{slot}", metavar="FILE", nargs="?",
                            const=os.path.join(REPO, default),
                            help=f"flash FILE to the {slot} slot @0x{offset:06x} "
                                 f"(default: {default})")
    ap.add_argument("--syspkg", metavar="FILE",
                    help="flash a matched SoC variant from a .syspkg archive "
                         "(see syspkg.py) -- bitstream+bios+app(+loader) in one "
                         "session; combinable with explicit slot flags")
    ap.add_argument("--port", metavar="TTY", default=None,
                    help="the ESP32-C3's USB-CDC serial port (e.g. /dev/ttyACM0) -- required")
    ap.add_argument("--reset", action="store_true",
                    help="no flashing: just pulse the C3->FPGA reset line (requires --port)")
    ap.add_argument("--boot-app", action="store_true",
                    help="chain-boot the app slot (one-shot; runs after any "
                         "flashing in this session, or standalone with no "
                         "other flags)")
    ap.add_argument("--reboot", action="store_true",
                    help="pulse the reset line when done even if only bitstream/BIOS were flashed")
    ap.add_argument("--no-reboot", action="store_true",
                    help="never pulse the reset line (leave the board as-is when done)")
    return ap.parse_args()


def build_jobs(args):
    """[(label, offset, data)] from a syspkg, slot presets and/or the legacy form.

    An explicit slot flag overrides the same offset coming from a --syspkg, so
    you can flash a packaged variant but swap in a locally rebuilt app."""
    by_offset = {}      # offset -> (label, offset, data); later writes win

    if args.syspkg:
        import syspkg
        pkg_jobs, manifest = syspkg.jobs_from_syspkg(args.syspkg)
        print(f"syspkg '{manifest['name']}' ({manifest['git']}, "
              f"csr {str(manifest.get('csr_sha256'))[:12]})")
        for label, offset, data in pkg_jobs:
            by_offset[offset] = (label, offset, data)

    for slot, (offset, wrap, _default) in SLOTS.items():
        path = getattr(args, slot)
        if path is None:
            continue
        if not os.path.exists(path):
            hint = BUILD_HINTS.get(slot)
            sys.exit(f"{slot} image not found: {path}"
                     + (f"\n  build it first: {hint}" if hint else ""))
        with open(path, "rb") as f:
            data = f.read()
        if wrap:
            data = struct.pack("<II", len(data), zlib.crc32(data)) + data
        by_offset[offset] = (f"{slot}:{os.path.relpath(path)}", offset, data)
    if args.file is not None:
        if args.offset is None:
            sys.exit("positional file needs --offset (or use a slot preset)")
        with open(args.file, "rb") as f:
            data = f.read()
        if args.fbi:
            data = struct.pack("<II", len(data), zlib.crc32(data)) + data
        by_offset[args.offset] = (args.file, args.offset, data)
    jobs = list(by_offset.values())
    if not jobs and not args.boot_app:
        sys.exit("nothing to do -- give a slot preset, --syspkg, FILE --offset X, or --boot-app")
    return sorted(jobs, key=lambda j: j[1])


# ---- C3 link (software/c3_flash_esp wire protocol) -------------------------

def open_c3(port):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed -- pip install pyserial")
    print(f"opening {port}...", flush=True)
    ser = serial.Serial(port, 115200, timeout=15)
    print("  waiting for C3 to reboot (opening the port resets native USB-CDC)...", flush=True)
    time.sleep(2.5)              # opening resets the C3 (native USB-CDC); let it reboot
    ser.reset_input_buffer()     # discard the C3 boot banner
    print("  C3 ready", flush=True)
    return ser


def _read_line(ser, timeout=2.0):
    """Read one \\n-terminated reply, capped at `timeout` s. NOT ser.read(N):
    that blocks trying to fill all N bytes and only gives up at the port's
    full 15 s timeout, even when a short reply already arrived (this made
    c3_ping/c3_reset/c3_boot_app appear to hang for ~15s on every call)."""
    old_timeout = ser.timeout
    ser.timeout = timeout
    try:
        return ser.read_until(b"\n").decode(errors="replace")
    finally:
        ser.timeout = old_timeout


def c3_ping_wait(ser, attempts=3, attempt_timeout=6.0):
    """Pre-flight: PING the FPGA loader through the C3 mailbox, retrying --
    catches a wrong --port or a loader that never comes up, while tolerating
    the post-reset settle time (BIOS init + flashboot into the loader).
    attempt_timeout is long on purpose: if the mailbox isn't live yet, the
    C3's own mbx_cmd() poll blocks for its full 5s internal timeout before it
    even replies with an error -- a short host-side read would just time out
    first and make a live-but-slow loader look dead."""
    print("waiting for FPGA loader mailbox (post-reset settle)...", flush=True)
    last = ""
    for _ in range(attempts):
        ser.reset_input_buffer()
        ser.write(b"p")
        reply = _read_line(ser, timeout=attempt_timeout)
        if "st=0" in reply:
            print(f"  {reply.strip()}")
            return
        last = reply
    sys.exit("C3 PING failed -- is the FPGA loader running (flash-resident, "
              f"or serial-booted)?\n  last reply: {last.strip()!r}")


def c3_reset(ser):
    """Pulse the C3->FPGA reset line ('R', software/c3_flash_esp). Resets the
    CPU/clock domains only -- does NOT reconfigure the fabric; a bitstream
    change still needs a real power-cycle."""
    print("pulsing C3 -> FPGA reset line ('R')...", flush=True)
    ser.reset_input_buffer()
    ser.write(b"R")
    reply = _read_line(ser)
    print(f"  {reply.strip() or '(no reply)'}")


def c3_boot_app(ser):
    """Chain-boot the app slot ('b', software/c3_flash_esp). One-shot: the
    loader's sticky boot flag self-clears once consumed, so the next reset
    (from the app, GPIO7, or a power-cycle) lands back in the resident
    loader with no separate 'return to loader' step. No FPGA-side reply to
    wait for -- it's already resetting by the time it could answer."""
    print("requesting boot-app ('b')...", flush=True)
    ser.reset_input_buffer()
    ser.write(b"b")
    reply = _read_line(ser)
    print(f"  {reply.strip() or '(no reply)'}")


def flash_slot(ser, label, offset, data):
    """Erase+program+CRC-verify one slot through the C3 mailbox protocol
    (mirrors software/c3_flash_esp/flash_c3.py -- kept in sync manually)."""
    for name, b in BOUNDARIES:
        if offset < b < offset + len(data):
            print(f"  WARNING: image crosses {name} -- it will be overwritten")
    if offset % 0x1000:
        sys.exit(f"{label}: offset 0x{offset:X} not 4 KB-aligned")

    crc = zlib.crc32(data) & 0xFFFFFFFF   # standard CRC-32 == FPGA libbase crc32
    print(f"{label}: {len(data)} B -> flash @0x{offset:06x}", flush=True)

    t0 = time.monotonic()
    ser.write(b"W" + struct.pack("<III", offset, len(data), crc))
    ser.flush()

    # Ack after ERASE (may take seconds for large images).
    print(f"  erasing (up to {(len(data) + 0xFFF) // 0x1000} sector(s), "
          "can take several seconds for a large image)...", flush=True)
    ea = ser.read(1)
    if ea != b"\x01":
        code = ea[0] if ea else -1
        sys.exit(f"{label}: erase ack failed: 0x{code:02X} ({C3_STATUS.get(code, 'no reply')})")

    # Stream pages in WINDOW_PAGES-sized windows, paced by one ack per window.
    npages = (len(data) + 255) // 256
    nwindows = (npages + WINDOW_PAGES - 1) // WINDOW_PAGES
    i = 0
    w = 0
    while i < npages:
        window = min(WINDOW_PAGES, npages - i)
        for j in range(window):
            page = data[(i + j) * 256:(i + j + 1) * 256]
            ser.write(page)
        ser.flush()
        a = ser.read(1)
        if a != b"\x01":
            code = a[0] if a else -1
            print()
            sys.exit(f"{label}: window at page {i}/{npages} ack failed: "
                      f"0x{code:02X} ({C3_STATUS.get(code, 'no reply')})")
        i += window
        w += 1
        elapsed = time.monotonic() - t0
        print(f"  programming: window {w}/{nwindows}  ({i * 256} / {len(data)} B, "
              f"{elapsed:.1f}s)      ", end="\r", flush=True)
    print()

    print("  verifying (read-back CRC)...", flush=True)
    reply = ser.read(5)             # final status + read-back crc
    dt = time.monotonic() - t0
    if len(reply) != 5:
        sys.exit(f"{label}: no/short final reply ({len(reply)} bytes) -- is the FPGA loader running?")
    status = reply[0]
    readback = struct.unpack("<I", reply[1:5])[0]
    kbps = len(data) / 1024 / dt if dt else 0
    print(f"  host_crc=0x{crc:08X}  readback=0x{readback:08X}  {dt:.2f}s ({kbps:.0f} KB/s)")
    if status != 0x00:
        sys.exit(f"{label}: -> status 0x{status:02X} ({C3_STATUS.get(status, 'unknown')})")


def main():
    args = parse_args()

    if not args.port:
        sys.exit("--port /dev/ttyACMx required (the ESP32-C3's USB-CDC port)")

    if args.reset:
        ser = open_c3(args.port)
        c3_reset(ser)
        return

    jobs = build_jobs(args)

    ser = open_c3(args.port)
    # Force the FPGA back into the resident loader before touching the
    # mailbox, regardless of what it's currently running (e.g. a chain-booted
    # app has no mailbox protocol at all -- PING would just get no reply).
    # GPIO7 always lands back in c3_flash (resident by default, see
    # docs/c3_loader.md), so this makes every session start from a known state.
    c3_reset(ser)
    c3_ping_wait(ser)

    t0 = time.monotonic()
    for label, offset, data in jobs:
        flash_slot(ser, label, offset, data)
    if jobs:
        print(f"total: {time.monotonic()-t0:.1f} s")

    flashed = {label.split(":")[0] for label, _, _ in jobs}
    if "bitstream" in flashed and not args.reboot:
        # A soft reset here would boot the OLD (still-configured) bitstream
        # against the NEW flash contents -- a CSR-map mismatch if the
        # gateware changed. Skip it on purpose (including --boot-app -- it
        # would chain-boot off the stale fabric); the loader itself only
        # needs one power-cycle to pick up everything.
        print("NOTE: bitstream flashed -- POWER-CYCLE to apply everything "
              "(reset/boot-app skipped on purpose)")
    elif args.boot_app:
        c3_boot_app(ser)
    elif (args.reboot or flashed & {"app", "loader", "bios"}) and not args.no_reboot:
        c3_reset(ser)
        print("reset pulsed -- BIOS/loader picked up the new flash contents")


if __name__ == "__main__":
    main()
