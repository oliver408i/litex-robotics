#!/usr/bin/env python3
"""WiFi flasher for the IcePi Zero (host side of software/winc_loader).

  ./flash.py --app firmware.bin                # app .fbi  @0x280000 (fbi auto)
  ./flash.py --loader [winc_loader.bin]        # loader    @0x200000 (fbi auto)
  ./flash.py --bios [bios.bin]                 # BIOS      @0x100000 (raw)
  ./flash.py --bitstream [icepi_zero.bit]      # bitstream @0x000000 (raw)
  ./flash.py --bitstream --bios --loader       # full system from build outputs
  ./flash.py --run firmware.bin                # SDRAM-load + run, NO flashing
  ./flash.py file.bin --offset 0x300000 [--fbi]  # explicit offset
  ./flash.py --reset --port /dev/ttyUSB0       # just reboot the SoC

Slots are combinable in one session; bare --bitstream/--bios/--loader pick up
the standard build artifacts. Entering the loader is automatic: probe :5557,
ask a running app's loader_hook (:5558), or FTDI-reset via --port (with 'l'
key spam + a reset prompt as the fallback). The boot chain, protocol and FTDI
semantics are documented in docs/boot_chain.md.
"""
import argparse
import os
import socket
import struct
import sys
import time
import zlib

REPO = os.path.dirname(os.path.abspath(__file__))

CHUNK = 1408          # max payload per WFLD (board cap; 1472 B datagram)

LOADER_PORT = 5557
HOOK_PORT   = 5558    # software/common/loader_hook.c
GRACE_KEY   = b"l"    # loader boot grace window: 'l' = stay in loader

SLOTS = {   # name: (offset, wrap_fbi, default_file or None)
    "bitstream": (0x000000, False, "build/icepi_zero/gateware/icepi_zero.bit"),
    "bios":      (0x100000, False, "build/icepi_zero/software/bios/bios.bin"),
    "loader":    (0x200000, True,  "software/winc_loader/winc_loader.bin"),
    "app":       (0x280000, True,  None),   # apps vary -- always explicit
}
BUILD_HINTS = {
    "bitstream": ".venv/bin/python icepi_zero_all.py --build",
    "bios":      ".venv/bin/python icepi_zero_all.py --build",
    "loader":    "make -C software/winc_loader",
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
    ap.add_argument("--run", metavar="FILE",
                    help="load FILE into SDRAM and execute it immediately -- "
                         "nothing is flashed, the flashed app slot is untouched "
                         "and the RAM app vanishes on the next reset")
    ap.add_argument("--host", default="icepi.local")
    ap.add_argument("--udp-port", type=int, default=LOADER_PORT)
    ap.add_argument("--port", metavar="TTY", default=None,
                    help="serial port (e.g. /dev/ttyUSB0) for the grace-window "
                         "entry path when no loader/hook answers over WiFi")
    ap.add_argument("--reset", action="store_true",
                    help="no flashing: just reset the SoC via the FTDI sidebands "
                         "(requires --port); board reboots through loader to app")
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
        jobs.append((f"{slot}:{os.path.relpath(path)}", offset, data))
    if args.file is not None:
        if args.offset is None:
            sys.exit("positional file needs --offset (or use a slot preset)")
        with open(args.file, "rb") as f:
            data = f.read()
        if args.fbi:
            data = struct.pack("<II", len(data), zlib.crc32(data)) + data
        jobs.append((args.file, args.offset, data))
    if not jobs and not args.run:
        sys.exit("nothing to do -- give a slot preset, FILE --offset X, or --run")
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


def ftdi_reset(tty):
    """Plain SoC reset via the FTDI sidebands: pulse the reset combo, return
    to idle WITHOUT the stay level -> board reboots through loader to app."""
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed -- pip install pyserial")
    with serial.Serial(tty, 1_000_000, timeout=0) as ser:
        ser.dtr = True
        ser.rts = True
        time.sleep(0.05)
        ser.dtr = False                 # reset combo: RTS asserted, DTR not
        time.sleep(0.15)                # > BootCtl's 50 ms filter
        ser.dtr = True                  # idle (both asserted)
    print("reset pulsed -- BIOS -> loader -> app")


# ---- getting the board into the loader ------------------------------------

def resolve_quiet(host):
    """Resolve once; None instead of an exception (board may be offline --
    icepi.local only exists in mDNS once the loader/app has joined WiFi)."""
    try:
        return socket.gethostbyname(host)
    except (socket.gaierror, OSError):
        return None


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


def enter_via_serial(s, host, tty):
    """FTDI reset + "stay in loader" level (semantics: docs/boot_chain.md).
    Spams the 'l' grace key throughout so bitstreams without BootCtl still
    work with a manual reset button. Re-resolves the hostname while polling
    (an offline board only appears in mDNS once the loader joins WiFi).
    Returns the resolved IP, or None."""
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed -- pip install pyserial (or enter the "
                 "loader another way)")
    with serial.Serial(tty, 1_000_000, timeout=0) as ser:
        ser.dtr = True                  # open() asserted both; make it explicit
        ser.rts = True
        time.sleep(0.05)
        print(f"  FTDI reset pulse on {tty}")
        ser.dtr = False                 # reset combo: RTS asserted, DTR not
        time.sleep(0.15)                # > BootCtl's 50 ms filter
        ser.dtr = True                  # stay level: DTR asserted, RTS not
        ser.rts = False

        def spam_and_probe(deadline):
            while time.monotonic() < deadline:
                for _ in range(20):
                    ser.write(GRACE_KEY)    # fallback for pre-BootCtl gateware
                    time.sleep(0.01)
                ip = resolve_quiet(host)
                if ip and probe_loader(s, (ip, LOADER_PORT), tries=1, timeout=0.5):
                    return ip
            return None

        # WiFi join takes a few seconds after the reset.
        ip = spam_and_probe(time.monotonic() + 12)
        if ip is None:
            print("  no loader yet (no FTDI reset module?)")
            print("  >>> press the board's RESET button now <<<")
            ip = spam_and_probe(time.monotonic() + 30)
        ser.rts = True                  # back to idle (both asserted)
        return ip


def ensure_loader(s, host, args):
    """Walk the entry ladder; returns the resolved board IP."""
    print("looking for the loader...")
    host_ip = resolve_quiet(host)
    if host_ip is None:
        print(f"  cannot resolve {host} -- board offline?")
    else:
        if probe_loader(s, (host_ip, LOADER_PORT)):
            print("  loader is up")
            return host_ip
        print(f"  no loader -- trying the app's loader_hook (UDP :{HOOK_PORT})")
        if enter_via_hook(s, host_ip):
            print("  loader is up")
            return host_ip
    if args.port:
        print("  trying the FTDI/serial path")
        host_ip = enter_via_serial(s, host, args.port)
        if host_ip:
            print("  loader is up")
            return host_ip
        sys.exit("loader did not come up after reset -- check wifi_secrets/AP")
    sys.exit(f"board unreachable: no loader on :{LOADER_PORT}, no app hook on "
             f":{HOOK_PORT}.\n"
             "Options: pass --port /dev/ttyUSBx (FTDI reset / reset prompt), or\n"
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


def upload_image(s, dst, label, offset, data, pps, what="flash"):
    """BEGIN + DATA: stage the image in board SDRAM (complete + acknowledged)."""
    crc = zlib.crc32(data)
    total = (len(data) + CHUNK - 1) // CHUNK
    print(f"{label}: {len(data)} B ({total} chunks) -> {what} @0x{offset:06x}")

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


def flash_image(s, dst, label, offset, data, pps):
    for name, b in BOUNDARIES:
        if offset < b < offset + len(data):
            print(f"  WARNING: image crosses {name} -- it will be overwritten")
    upload_image(s, dst, label, offset, data, pps)

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


def run_image(s, dst, path, pps):
    """Stage a raw binary in SDRAM and execute it -- no flash, no reboot.
    The staging offset is the app slot's only because WFLB validates against
    the flash layout; WFLX never touches flash. No .fbi header (length/crc
    already travel in WFLB)."""
    with open(path, "rb") as f:
        data = f.read()
    upload_image(s, dst, f"run:{os.path.relpath(path)}", SLOTS["app"][0], data,
                 pps, what="SDRAM exec")
    print("  executing from SDRAM (nothing flashed)...", flush=True)
    reply = request(s, dst, b"WFLX", b"WFLX", tries=10, timeout=1.0)
    if reply is None:
        sys.exit("no reply to EXEC -- loader gone?")
    status = reply[4]
    if status != 0:
        sys.exit(f"EXEC refused: {ST_NAMES.get(status, status)}")
    print("  app running from SDRAM -- gone on the next reset")


def main():
    args = parse_args()

    if args.reset:
        if not args.port:
            sys.exit("--reset needs --port /dev/ttyUSBx")
        ftdi_reset(args.port)
        return

    jobs = build_jobs(args)
    if args.run and not os.path.exists(args.run):
        sys.exit(f"run image not found: {args.run}")

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Resolution happens inside the ladder (the board may be offline until
    # the serial path forces it up); the returned IP is then used for every
    # packet -- never per-packet hostname lookups (~23 ms mDNS each).
    host_ip = ensure_loader(s, args.host, args)
    dst = (host_ip, args.udp_port)

    if jobs:
        t0 = time.monotonic()
        for label, offset, data in jobs:
            flash_image(s, dst, label, offset, data, args.pps)
        print(f"total: {time.monotonic()-t0:.1f} s")

    flashed = {label.split(":")[0] for label, _, _ in jobs}
    if args.run:
        # exec is terminal for the loader -- it replaces the WFLR step
        if "bitstream" in flashed:
            print("NOTE: bitstream flashed -- the RAM app runs on the OLD "
                  "fabric; power-cycle later to apply the new bitstream")
        run_image(s, dst, args.run, args.pps)
        return
    if "bitstream" in flashed and not args.reboot:
        # Apply everything with ONE power-cycle. A soft reset here would boot
        # the OLD (still-configured) bitstream with the NEW flash contents --
        # a CSR-map mismatch if the gateware changed.
        print("NOTE: bitstream flashed -- POWER-CYCLE to apply everything "
              "(soft reset skipped on purpose)")
    elif (args.reboot or flashed & {"app", "loader", "bios"}) and not args.no_reboot:
        # best-effort: the ack may die with the resetting board
        request(s, dst, b"WFLR", b"WFLR", tries=3, timeout=0.5)
        print("rebooted -- loader will chain-boot the app")


if __name__ == "__main__":
    main()
