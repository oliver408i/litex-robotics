#!/usr/bin/env python3
"""ESP32-C3 flasher for the IcePi Zero (host side of software/c3_flash_esp).

  ./flash.py --app firmware.bin --port /dev/ttyACM0       # app .fbi  @0x280000 (fbi auto)
  ./flash.py --loader [c3_flash.bin] --port /dev/ttyACM0  # loader    @0x200000 (fbi auto)
  ./flash.py --bios [bios.bin] --port /dev/ttyACM0        # BIOS      @0x100000 (raw)
  ./flash.py --bitstream [icepi_zero.bit] --port /dev/ttyACM0  # bitstream @0x000000 (raw)
  ./flash.py --bitstream --bios --loader --port /dev/ttyACM0   # full system from build outputs
  ./flash.py file.bin --offset 0x300000 [--fbi] --port /dev/ttyACM0  # explicit offset
  ./flash.py --reset --port /dev/ttyACM0                  # just reboot the SoC

Slots are combinable in one session; bare --bitstream/--bios/--loader pick up
the standard build artifacts. --port is the ESP32-C3's USB-CDC port (the
FPGA's own FTDI port is only used for watching boot output, e.g. litex_term
or a plain serial monitor -- flashing goes entirely through the C3). The
loader is flash-resident and comes up on its own after any reset/power-cycle
(no litex_term --kernel step needed); this tool just needs it already running
to talk to it. The boot chain, wire protocol and C3 link design are
documented in docs/boot_chain.md and docs/c3_loader.md.

Note: the old WiFi/WINC-era --run (SDRAM-stage-and-execute) has no equivalent
here -- the C3 loader has no SDRAM image buffer. Gone until SDRAM comes back
(see the halfrate-sdram / SDRAM-debug follow-up).
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
    "app":       (0x280000, True,  None),   # apps vary -- always explicit; no
                                             # boot-manager consumes this slot
                                             # yet (needs SDRAM, deferred).
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
    if not jobs:
        sys.exit("nothing to do -- give a slot preset, --syspkg, or FILE --offset X")
    return sorted(jobs, key=lambda j: j[1])


# ---- C3 link (software/c3_flash_esp wire protocol) -------------------------

def open_c3(port):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed -- pip install pyserial")
    ser = serial.Serial(port, 115200, timeout=15)
    time.sleep(2.5)              # opening resets the C3 (native USB-CDC); let it reboot
    ser.reset_input_buffer()     # discard the C3 boot banner
    return ser


def c3_ping(ser):
    """Pre-flight sanity check: PING the FPGA loader through the C3 mailbox --
    catches a wrong --port or a loader that isn't up before we try to flash."""
    ser.reset_input_buffer()
    ser.write(b"p")
    time.sleep(0.3)
    reply = ser.read(500).decode(errors="replace")
    if "st=0" not in reply:
        sys.exit("C3 PING failed -- is the FPGA loader running (flash-resident, "
                  f"or serial-booted)?\n  reply: {reply.strip()!r}")
    print(f"  {reply.strip()}")


def c3_reset(ser):
    """Pulse the C3->FPGA reset line ('R', software/c3_flash_esp). Resets the
    CPU/clock domains only -- does NOT reconfigure the fabric; a bitstream
    change still needs a real power-cycle."""
    ser.reset_input_buffer()
    ser.write(b"R")
    time.sleep(0.3)
    reply = ser.read(500).decode(errors="replace")
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
    print(f"{label}: {len(data)} B -> flash @0x{offset:06x}")

    t0 = time.monotonic()
    ser.write(b"W" + struct.pack("<III", offset, len(data), crc))
    ser.flush()

    # Ack after ERASE (may take seconds for large images).
    ea = ser.read(1)
    if ea != b"\x01":
        code = ea[0] if ea else -1
        sys.exit(f"{label}: erase ack failed: 0x{code:02X} ({C3_STATUS.get(code, 'no reply')})")

    # Stream pages in WINDOW_PAGES-sized windows, paced by one ack per window.
    npages = (len(data) + 255) // 256
    i = 0
    while i < npages:
        window = min(WINDOW_PAGES, npages - i)
        for j in range(window):
            page = data[(i + j) * 256:(i + j + 1) * 256]
            ser.write(page)
        ser.flush()
        a = ser.read(1)
        if a != b"\x01":
            code = a[0] if a else -1
            sys.exit(f"{label}: window at page {i}/{npages} ack failed: "
                      f"0x{code:02X} ({C3_STATUS.get(code, 'no reply')})")
        i += window

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
    c3_ping(ser)

    t0 = time.monotonic()
    for label, offset, data in jobs:
        flash_slot(ser, label, offset, data)
    print(f"total: {time.monotonic()-t0:.1f} s")

    flashed = {label.split(":")[0] for label, _, _ in jobs}
    if "bitstream" in flashed and not args.reboot:
        # A soft reset here would boot the OLD (still-configured) bitstream
        # against the NEW flash contents -- a CSR-map mismatch if the
        # gateware changed. Skip it on purpose; the loader itself only needs
        # one power-cycle to pick up everything.
        print("NOTE: bitstream flashed -- POWER-CYCLE to apply everything "
              "(reset skipped on purpose)")
    elif (args.reboot or flashed & {"app", "loader", "bios"}) and not args.no_reboot:
        c3_reset(ser)
        print("reset pulsed -- BIOS/loader picked up the new flash contents")


if __name__ == "__main__":
    main()
