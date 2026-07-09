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
  ./flash.py --loader --stay --port /dev/ttyACM0             # flash loader + END in the loader

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

--stay ends the session in the resident loader instead of chain-booting the
app (the default). Use it when iterating on the loader/flash itself, or to
verify a fresh flash before running it. It is ONE-SHOT: the loader stays this
boot via the boot_ctl flag (which stay_requested() consumes), and the next
reset chain-boots the app -- --stay releases the boot-mode strap so the stay
can't get stuck asserted across resets.

Both the boot-app and the stay/reset paths reboot the FPGA via the loader's
*mailbox* soft reset (SoCController ctrl_reset + the sticky boot_ctl flag), NOT
the external C3->FPGA reset line (G3) -- that line has proven unreliable, so a
plain --reboot that "does not reset the FPGA" is exactly the symptom this
avoids. The mailbox path needs the loader already running (it is, during any
flashing session); from a *booted app* only the hardware reset line or a
power-cycle can re-enter the loader.
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
    ap.add_argument("--stay", action="store_true",
                    help="end in the resident loader instead of chain-booting the app. "
                         "One-shot: the loader stays this boot (boot_ctl flag), then a "
                         "later reset boots the app")
    args = ap.parse_args()
    if args.stay and args.boot_app:
        sys.exit("--stay and --boot-app are mutually exclusive")
    return args


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
             "or serial-booted)?\n"
             "  If a booted app is stuck (mailbox gone) and the C3->FPGA reset "
             "line is dead, 'l' can't re-enter the loader:\n"
             "  power-cycle the board (or fix the G3 reset wire) to get the "
             "resident loader back.\n"
             f"  last reply: {last.strip()!r}")


def c3_reset(ser):
    """Reboot the FPGA ('R', software/c3_flash_esp). The C3 fires the loader's
    mailbox CMD_REBOOT (SoCController ctrl_reset -- a reliable soft reset) and
    ALSO pulses the external reset line as a fallback. Resets the CPU/clock
    domains only -- does NOT reconfigure the fabric; a bitstream change still
    needs a real power-cycle. Honors the current boot-mode strap/flag: with the
    stay flag cleared (the default) this boots the app."""
    print("rebooting FPGA ('R': mailbox ctrl_reset + line)...", flush=True)
    ser.reset_input_buffer()
    ser.write(b"R")
    reply = _read_line(ser)
    print(f"  {reply.strip() or '(no reply)'}")


def c3_enter_loader(ser):
    """Force the FPGA into the resident loader ('l', software/c3_flash_esp): the
    C3 asserts the boot-mode strap (IO4/R1) low AND fires the loader's mailbox
    CMD_STAY (durable boot_ctl flag + ctrl_reset soft reset), so the board comes
    up in c3_flash. The mailbox path is the reliable one -- it does not depend on
    the external reset line. Needed because the loader is NOT resident by default
    (a plain reset auto-boots the app, docs/c3_loader.md). Requires the loader to
    already be running to take the mailbox command; from a booted app only the
    external line/power-cycle can re-enter it (see c3_ping_wait's failure hint)."""
    print("entering loader ('l': assert stay strap + mailbox CMD_STAY)...", flush=True)
    ser.reset_input_buffer()
    ser.write(b"l")
    reply = _read_line(ser)
    print(f"  {reply.strip() or '(no reply)'}")


def c3_stay(ser):
    """End the session in the resident loader as a ONE-SHOT ('s',
    software/c3_flash_esp): the C3 RELEASES the boot-mode strap and fires the
    loader's mailbox CMD_STAY, which sets the sticky-but-one-shot boot_ctl flag.
    The loader stays this boot and consumes the flag (stay_requested()); the
    NEXT reset chain-boots the app. This is deliberately NOT c3_enter_loader
    ('l'): that holds the strap low for the whole session, which the C3 keeps
    asserting after we exit, so every reset would re-stay -- the stay would
    never clear. Releasing the strap here makes the flag the sole, one-shot
    signal."""
    print("staying in resident loader ('s': release strap + one-shot CMD_STAY)...", flush=True)
    ser.reset_input_buffer()
    ser.write(b"s")
    reply = _read_line(ser)
    print(f"  {reply.strip() or '(no reply)'}")


def c3_boot_app(ser):
    """Boot the app ('b', software/c3_flash_esp): the C3 releases the boot-mode
    strap (-> FPGA reads high) and fires the loader's mailbox CMD_BOOT_APP (clear
    boot_ctl flag + ctrl_reset soft reset), so the loader chain-boots the app
    slot. The mailbox soft reset is the reliable reboot -- the external reset
    line is only a fallback pulse. Normal end-of-session action."""
    print("booting app ('b': release stay strap + mailbox CMD_BOOT_APP)...", flush=True)
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

    # Ack after ERASE (may take many seconds/minutes for a large image). Wait
    # proportionally to the erase size -- the C3 now budgets its mailbox poll
    # the same way (~0.5 s/sector) rather than a flat timeout, so neither side
    # mistakes a legitimate long erase for a hung loader.
    print(f"  erasing (up to {(len(data) + 0xFFF) // 0x1000} sector(s), "
          "can take several seconds for a large image)...", flush=True)
    old_to = ser.timeout
    ser.timeout = 10.0 + (len(data) / 4096) * 0.5
    ea = ser.read(1)
    ser.timeout = old_to
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
    # Force the FPGA into the resident loader before touching the mailbox,
    # regardless of what it's currently running (a chain-booted app has no
    # mailbox protocol -- PING would just get no reply). The loader is no longer
    # resident by default (a plain reset auto-boots the app), so we assert the
    # stay strap; the C3 holds it for the whole session. See docs/c3_loader.md.
    c3_enter_loader(ser)
    c3_ping_wait(ser)

    t0 = time.monotonic()
    for label, offset, data in jobs:
        flash_slot(ser, label, offset, data)
    if jobs:
        print(f"total: {time.monotonic()-t0:.1f} s")

    flashed = {label.split(":")[0] for label, _, _ in jobs}
    if args.stay:
        # Explicit "end in the loader" -- CMD_STAY (durable flag + soft reset).
        # Reliable regardless of the external reset line; --stay conflicts with
        # --boot-app (rejected in parse_args). Takes precedence over the
        # bitstream note below: a bitstream flash still needs a power-cycle to
        # apply the new fabric, but the board is left in the loader meanwhile.
        c3_stay(ser)
        if "bitstream" in flashed:
            print("NOTE: bitstream flashed -- POWER-CYCLE to apply the new fabric")
    elif "bitstream" in flashed and not args.reboot:
        # A soft reset here would boot the OLD (still-configured) bitstream
        # against the NEW flash contents -- a CSR-map mismatch if the gateware
        # changed. Skip it on purpose; POWER-CYCLE picks up the new fabric, and
        # the C3 releases the stay strap on its own reboot so the board then
        # auto-boots the app.
        print("NOTE: bitstream flashed -- POWER-CYCLE to apply everything "
              "(reset skipped on purpose; the board auto-boots the app after)")
    elif args.no_reboot and not args.boot_app:
        # Leave the board exactly as-is (still in the loader from c3_enter_loader
        # at the start of the session). Unlike --stay this issues no reset at
        # all. Power-cycle to auto-boot the app.
        print("left in resident loader (--no-reboot); power-cycle to auto-boot the app")
    else:
        # Normal end state: release the strap + mailbox CMD_BOOT_APP soft reset
        # -> chain-boot the app. (--boot-app, --reboot, or any app/bios/loader
        # flash all land here; "reboot to pick up new flash" means "boot the app".)
        c3_boot_app(ser)


if __name__ == "__main__":
    main()
