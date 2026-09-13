#!/usr/bin/env python3
"""Non-interactive serial boot and console capture for the LiteX SoCs here.

`litex_term` is an interactive terminal: it calls `termios.tcgetattr` on stdin
at startup, so it dies with "Inappropriate ioctl for device" the moment it is
run from a script, a CI job, or an agent -- anywhere stdin is a pipe. This does
the same two useful jobs without a tty, and exits on its own:

    # load a firmware, watch for its verdict, exit 0/1 on the answer
    .venv/bin/python tools/socboot.py \\
        --kernel software/psc18sr_host/psc18sr_host.bin \\
        --expect "all passed" --fail "FAILED" --timeout 90

    # just watch the console for 10 s
    .venv/bin/python tools/socboot.py --monitor 10

    # type at the BIOS and capture what it says
    .venv/bin/python tools/socboot.py --send "mem_read 0xf0000000 16" --monitor 5

Exit status is the point of the thing: 0 = --expect matched (or the run
finished with nothing to match), 1 = --fail matched, 2 = timed out, 3 = the
upload itself failed. So a test run is a shell conditional, not something a
human has to read.

Resetting the CPU, which is the other half of "non-interactive":

  --reset soft   send `reboot` on the console. This is the LiteX BIOS command,
                 so it needs the BIOS to be the thing listening -- it does
                 nothing while a loaded firmware owns the UART. Fast (~1 s).
  --reset hard   reconfigure the FPGA through the probe (tools/icelink.py),
                 which resets everything on the die including the CPU. Always
                 works, needs --bitstream, takes ~5 s.
  --reset auto   soft, then hard if the BIOS never answers. The default when
                 --bitstream is given; otherwise the default is soft.

There is deliberately no third option: this board has no reset line the probe
can pull (the probe drives JTAG only, and `cpu_reset_n` is a button), and the
`ctrl` CSR's reset bit is reachable only by the CPU that would be resetting
itself. Reconfiguration is the hardware reset, and it is cheap enough.

The SFL protocol constants come from litex_term itself rather than being
copied, so a LiteX bump cannot leave a stale duplicate behind here.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LITEX_SETUP_ROOT = os.path.join(REPO_ROOT, "litex-setup")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
for _rel in ("litex", "litex-boards", "migen", "litedram", "litespi", "litesdcard"):
    _p = os.path.join(LITEX_SETUP_ROOT, _rel)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import serial
except ImportError:
    raise SystemExit("pyserial not installed (.venv/bin/pip install pyserial)")

from litex.tools.litex_term import (
    SFLFrame,
    sfl_cmd_load, sfl_cmd_jump,
    sfl_magic_req, sfl_magic_ack,
    sfl_prompt_req, sfl_prompt_ack,
    sfl_ack_success, sfl_ack_crcerror, sfl_ack_unknown, sfl_ack_error,
)

DEFAULT_PORT  = "/dev/icelink-uart"
DEFAULT_SPEED = 115200
DEFAULT_KERNEL_ADDR = 0x40000000        # main_ram on both boards here

# Conservative framing: 60 payload bytes, one ack per frame, no pipelining.
# litex_term calibrates for speed and falls back to exactly this when
# calibration fails; a few KB of firmware does not need the complexity.
FRAME_PAYLOAD = 60

EXIT_OK, EXIT_FAIL, EXIT_TIMEOUT, EXIT_UPLOAD = 0, 1, 2, 3


class BootError(RuntimeError):
    pass


class Tee:
    """Console output: to stdout as it arrives, and kept for pattern matching.

    Matching is done on a rolling buffer rather than line by line because the
    interesting strings (the SFL magic especially) are not newline-terminated.
    """

    def __init__(self, echo=True, keep=1 << 20):
        self.buf = bytearray()
        self.echo = echo
        self.keep = keep

    def feed(self, data: bytes):
        if not data:
            return
        if self.echo:
            sys.stdout.write(data.decode("utf-8", "replace"))
            sys.stdout.flush()
        self.buf += data
        if len(self.buf) > self.keep:
            del self.buf[:-self.keep]

    def text(self) -> str:
        return self.buf.decode("utf-8", "replace")


def _ack_name(a: bytes) -> str:
    return {sfl_ack_success: "success", sfl_ack_crcerror: "crc error",
            sfl_ack_unknown: "unknown command",
            sfl_ack_error: "error"}.get(a, repr(a))


ACK_BYTES = (sfl_ack_success, sfl_ack_crcerror, sfl_ack_unknown, sfl_ack_error)


def _read_ack(port, timeout: float = 2.0):
    """Next real ack byte, skipping console text.

    The BIOS repeats its magic request every second or so while it waits, so by
    the time a slow reset has been noticed there can be several copies queued
    ahead of the first ack. Reading one byte blind picks a letter out of
    "sL5DdSMmkekro" and calls it a rejection -- which is exactly what it looked
    like. Skip anything that is not one of the four ack codes."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = port.read(1)
        if not b:
            continue
        if b in ACK_BYTES:
            return b
    return b""


def _send_frame(port, frame: SFLFrame, retries: int = 3):
    for _ in range(retries):
        port.write(frame.encode())
        port.flush()
        ack = _read_ack(port)
        if ack == sfl_ack_success:
            return
        if ack == sfl_ack_crcerror:
            continue                         # the one worth retrying
        raise BootError(f"device rejected a frame: {_ack_name(ack)}")
    raise BootError("too many CRC errors on the wire")


def upload(port, path: str, address: int, quiet=False) -> int:
    data = open(path, "rb").read()
    if not data:
        raise BootError(f"{path} is empty")
    t0 = time.time()
    for off in range(0, len(data), FRAME_PAYLOAD):
        chunk = data[off:off + FRAME_PAYLOAD]
        frame = SFLFrame()
        frame.cmd = sfl_cmd_load
        frame.payload = (address + off).to_bytes(4, "big") + chunk
        _send_frame(port, frame)
    if not quiet:
        dt = time.time() - t0
        print(f"[socboot] uploaded {len(data)} bytes to 0x{address:08x} "
              f"in {dt:.1f}s ({len(data)/dt/1024:.1f} KB/s)", flush=True)
    return len(data)


def jump(port, address: int):
    frame = SFLFrame()
    frame.cmd = sfl_cmd_jump
    frame.payload = address.to_bytes(4, "big")
    _send_frame(port, frame)


def wait_for(port, tee: Tee, needle: bytes, timeout: float) -> bool:
    """Read until `needle` appears in the stream. True if it did."""
    deadline = time.time() + timeout
    window = bytearray()
    while time.time() < deadline:
        data = port.read(256) or port.read(1)
        tee.feed(data)
        if data:
            window += data
            if needle in window:
                return True
            del window[:-len(needle)]
    return False


def drain(port, tee: Tee) -> None:
    """Consume everything queued, into the log rather than into the bin.

    Used where a plain `reset_input_buffer()` would do for correctness -- the
    point is that the queue after a reconfigure holds the entire boot (banner,
    SDRAM init, memtest), which is exactly what you want to read when a build
    is timing-marginal. Draining it clears the stale magic just as well, and
    pattern matching downstream is unaffected: wait_for() only inspects bytes
    that arrive after it is called."""
    while port.in_waiting:
        tee.feed(port.read(port.in_waiting))


def request_serialboot(port, tee: Tee, deadline_left, attempts: int = 6) -> bool:
    """Ask the BIOS prompt to re-offer serial boot, and catch the magic live.

    `serialboot` is a BIOS command, so this works whenever the BIOS is the
    thing reading the UART -- including long after the boot-time offer has
    timed out, which is the normal state of a board that was just
    reconfigured. Retried because the prompt may not exist yet: right after
    DONE the BIOS is still in SDRAM init, where the text is simply ignored."""
    for _ in range(attempts):
        if deadline_left() <= 0:
            return False
        drain(port, tee)
        port.write(b"\nserialboot\n")
        port.flush()
        if wait_for(port, tee, sfl_magic_req, min(4.0, deadline_left())):
            return True
    return False


def do_soft_reset(port, tee: Tee, timeout: float = 6.0) -> bool:
    """`reboot` at the BIOS prompt. True if the BIOS came back."""
    drain(port, tee)
    port.write(b"\r\nreboot\r\n")
    port.flush()
    return wait_for(port, tee, sfl_magic_req, timeout)


def do_hard_reset(port, bitstream: str, probe_port: str | None) -> None:
    """Reconfigure the FPGA -- the only real reset this board has.

    Deliberately does NOT try to catch the boot-time serial-boot offer, because
    on this board it cannot be caught. Two facts close that window:

      * The LiteX BIOS sends its magic EXACTLY ONCE per boot (bios/boot.c: one
        write, then check_ack with a short timeout, then "Timeout" and on to
        the next boot method).
      * The probe does not bridge the target UART while it is shifting JTAG.
        Everything the BIOS printed during configuration arrives in one gulp
        when programming finishes -- measured at 6.4 s here, against a magic
        emitted at 5.2 s. Answering it is not late by a little; the bytes did
        not exist yet.

    Listening in a background thread was tried and does not help, for the
    second reason. So the caller lets the BIOS fall through to its prompt and
    asks for serial boot by name -- see request_serialboot().

    `port` (the console) is left OPEN throughout. Closing it drops the CDC line
    coding the probe bridges the target UART at, and the reopened port comes
    back mis-bridged.

    The input buffer is flushed before, so a magic from a PREVIOUS boot cannot
    be mistaken for a live one. It is deliberately NOT flushed after: that gulp
    is the whole boot -- banner, SDRAM init, memtest -- and it is worth keeping,
    because on a timing-marginal build it is the first place trouble shows.
    Nothing downstream can trip over the stale magic inside it, since
    request_serialboot() flushes immediately before it listens."""
    from tools.icelink import program_sram, IcelinkError, DEFAULT_PORT as PROBE
    print(f"[socboot] hard reset: reconfiguring with {bitstream}", flush=True)
    port.reset_input_buffer()
    try:
        program_sram(bitstream, port=probe_port or PROBE, verbose=False)
    except IcelinkError as e:
        raise BootError(f"hard reset failed: {e}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT, help=f"console (default {DEFAULT_PORT})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    ap.add_argument("--kernel", help="binary to serial-boot into main_ram")
    ap.add_argument("--kernel-addr", type=lambda x: int(x, 0), default=DEFAULT_KERNEL_ADDR)
    ap.add_argument("--reset", choices=("none", "soft", "hard", "auto"), default=None,
                    help="how to get back to the BIOS before booting "
                         "(default: auto with --bitstream, else soft)")
    ap.add_argument("--bitstream", help="bitstream for --reset hard/auto")
    ap.add_argument("--probe-port", default=None, help="probe console for a hard reset")
    ap.add_argument("--monitor", type=float, default=0.0,
                    help="seconds to keep printing the console after booting "
                         "(or, with no --kernel, just watch for this long)")
    ap.add_argument("--send", action="append", default=[],
                    help="text to type once the firmware is up; repeatable. "
                         "A literal \\n is sent as a newline.")
    ap.add_argument("--expect", help="regex; exit 0 as soon as it appears")
    ap.add_argument("--fail", help="regex; exit 1 as soon as it appears")
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="overall deadline in seconds (default 120)")
    ap.add_argument("--quiet", action="store_true", help="do not echo the console")
    args = ap.parse_args()

    if args.reset is None:
        args.reset = "auto" if args.bitstream else "soft"
    if args.reset in ("hard", "auto") and not args.bitstream:
        if args.reset == "hard":
            return _die("--reset hard needs --bitstream")
        args.reset = "soft"

    if not os.path.exists(args.port):
        return _die(f"{args.port} not found -- is the probe plugged in? "
                    f"(see probe/icelink-fast/tools/99-icelink-fast.rules)")
    try:
        port = serial.Serial(args.port, args.speed, timeout=0.05)
    except Exception as e:
        return _die(f"cannot open {args.port}: {e}\n"
                    f"  (try: sg dialout -c '...', or add yourself to dialout)")

    tee = Tee(echo=not args.quiet)
    deadline = time.time() + args.timeout
    expect_re = re.compile(args.expect) if args.expect else None
    fail_re   = re.compile(args.fail)   if args.fail   else None

    t0 = time.time()

    def left() -> float:
        """Seconds to the overall deadline. Every wait is bounded by this, so
        --timeout means what it says however the run got here."""
        return max(0.0, deadline - time.time())

    def stamp(msg: str):
        print(f"[socboot +{time.time() - t0:5.1f}s] {msg}", flush=True)

    try:
        if args.kernel:
            got_magic = False
            if args.reset in ("soft", "auto"):
                stamp("soft reset: reboot")
                got_magic = do_soft_reset(port, tee, min(6.0, left()))
            if not got_magic and args.reset in ("hard", "auto"):
                stamp("soft reset did not answer; reconfiguring")
                do_hard_reset(port, args.bitstream, args.probe_port)
                stamp("configured")
                # The boot-time offer is already gone and its magic has been
                # flushed (see do_hard_reset). Ask for it by name instead.
                stamp("asking the BIOS for serial boot")
                got_magic = request_serialboot(port, tee, left)
            if not got_magic:
                stamp("waiting for the serial-boot request")
                got_magic = wait_for(port, tee, sfl_magic_req, left())
            if not got_magic:
                print("[socboot] no serial-boot request from the device.",
                      file=sys.stderr)
                print("  The BIOS asks once, at boot. If a firmware is already "
                      "running it owns the UART and will not answer --\n"
                      "  use --reset hard --bitstream <file> to reconfigure.",
                      file=sys.stderr)
                return EXIT_TIMEOUT

            stamp("serial-boot request received")
            # Drop whatever else is queued -- repeated magics, the boot banner
            # -- so the first thing read after this is an ack and not console
            # text that happens to look like one.
            port.reset_input_buffer()
            port.write(sfl_magic_ack)
            port.flush()
            try:
                upload(port, args.kernel, args.kernel_addr, quiet=args.quiet)
                jump(port, args.kernel_addr)
            except BootError as e:
                print(f"[socboot] {e}", file=sys.stderr)
                return EXIT_UPLOAD
            stamp("booted")

        for text in args.send:
            time.sleep(0.3)
            port.write(text.replace("\\n", "\n").encode())
            port.flush()

        # Watch. --monitor extends the deadline only downward: whichever of the
        # two runs out first ends the run, so --timeout stays the hard stop.
        watch_until = deadline
        if args.monitor:
            watch_until = min(deadline, time.time() + args.monitor)
        rc = EXIT_OK if not expect_re else EXIT_TIMEOUT
        while time.time() < watch_until:
            tee.feed(port.read(256) or port.read(1))
            text = tee.text()
            if fail_re and fail_re.search(text):
                print("\n[socboot] matched --fail", flush=True)
                return EXIT_FAIL
            if expect_re and expect_re.search(text):
                print("\n[socboot] matched --expect", flush=True)
                return EXIT_OK
        if expect_re:
            print(f"\n[socboot] gave up after {time.time() - t0:.0f}s without "
                  f"matching --expect", file=sys.stderr)
        return rc
    except BootError as e:
        print(f"[socboot] {e}", file=sys.stderr)
        return EXIT_UPLOAD
    except KeyboardInterrupt:
        return EXIT_TIMEOUT
    finally:
        try:
            port.close()
        except Exception:
            pass


def _die(msg: str) -> int:
    print(f"socboot: {msg}", file=sys.stderr)
    return EXIT_UPLOAD


if __name__ == "__main__":
    raise SystemExit(main())
