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
import socket
import struct
import subprocess
import zlib
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



# Etherbone kernel upload ---------------------------------------------------------------------------
# SFL over the console is 115200 baud with 60-byte frames and an ack each, so a
# 578 KB firmware (software/mnist_lcd_demo) takes minutes. Etherbone writes the
# same bytes straight into SDRAM over UDP in seconds. The gateware has to have
# been built with Etherbone for this to exist at all -- see
# gateware/soc_features.py: add_eth, on by default for the feature tops.
DEFAULT_CSR_CSV = "build/colorlight_i5/csr.csv"
DEFAULT_EB_IP   = "192.168.0.50"
EB_BURST        = 255            # Etherbone protocol maximum, in 32-bit words
SRAM_TRAP       = (0x10000000, 0x10002000)


def _litex_server_up(host: str, eb_port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.4)
        return s.connect_ex((host, eb_port)) == 0


class _LiteXServer:
    """Spawn litex_server --udp only if one is not already listening.

    A server started here is stopped here; one that was already running is left
    alone, because it is probably the caller's and may be serving other tools.
    """
    def __init__(self, ip: str, host: str, eb_port: int):
        self.ip, self.host, self.eb_port, self.proc = ip, host, eb_port, None

    def __enter__(self):
        if _litex_server_up(self.host, self.eb_port):
            return self
        exe = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           ".venv", "bin", "litex_server")
        cmd = [exe if os.path.exists(exe) else "litex_server", "--udp", "--udp-ip", self.ip]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(40):                       # up to ~8 s
            if _litex_server_up(self.host, self.eb_port):
                return self
            time.sleep(0.2)
        self.__exit__(None, None, None)
        raise BootError(f"litex_server did not come up for {self.ip}; "
                        f"is the board reachable? (ping {self.ip})")

    def __exit__(self, *exc):
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except Exception:
                self.proc.kill()
            self.proc = None


def etherbone_upload(path: str, addr: int, ip: str, host: str, eb_port: int,
                     csr_csv: str, pace: float = 0.0) -> tuple[int, int]:
    """Write `path` into memory at `addr`. Returns (padded length, CRC32).

    The CRC is zlib's, which is what the BIOS `crc` command computes, so the
    caller can verify the landing without reading anything back.
    """
    lo, hi = SRAM_TRAP
    if lo <= addr < hi:
        raise BootError(f"refusing: 0x{addr:08x} is the CPU's SRAM, where the running "
                        f"BIOS keeps its stack")
    blob = open(path, "rb").read()
    if not blob:
        raise BootError(f"{path} is empty")
    data  = blob + b"\x00" * ((-len(blob)) % 4)
    words = list(struct.unpack("<%dI" % (len(data) // 4), data))

    from litex.tools.litex_client import RemoteClient
    with _LiteXServer(ip, host, eb_port):
        bus = RemoteClient(csr_csv=csr_csv, host=host, port=eb_port)
        bus.open()
        try:
            t0 = time.time()
            for i in range(0, len(words), EB_BURST):
                bus.write(addr + 4 * i, words[i:i + EB_BURST])
                if pace:
                    time.sleep(pace)
            dt = time.time() - t0
        finally:
            bus.close()
    print(f"[socboot] etherbone: {len(data)} bytes to 0x{addr:08x} in {dt:.2f}s "
          f"({len(data)/dt/1e6:.1f} MB/s)", flush=True)
    return len(data), zlib.crc32(data) & 0xffffffff


def etherbone_boot(port, tee: Tee, path: str, addr: int, args, deadline_left) -> None:
    """Upload over Etherbone, verify by CRC at the BIOS prompt, then `boot`.

    Verification is not optional here the way it is over SFL: SFL acks every
    frame, Etherbone acks nothing, and the README records a real failure mode
    where an unpaced write wedges the Wishbone bus while ICMP keeps answering.
    A CRC mismatch means the upload did not land -- do not jump into it.
    """
    length, crc = etherbone_upload(path, addr, args.eb_ip, args.eb_host,
                                   args.eb_port, args.csr_csv, args.eb_pace)
    drain(port, tee)
    port.write(f"\ncrc 0x{addr:08x} {length}\n".encode())
    port.flush()
    want = f"{crc:08x}".encode()
    if not wait_for(port, tee, want, min(30.0, deadline_left())):
        raise BootError(f"CRC mismatch after etherbone upload (expected {crc:08x}); "
                        f"the image did not land -- not booting it")
    print(f"[socboot] etherbone: CRC {crc:08x} verified", flush=True)
    drain(port, tee)
    port.write(f"\nboot 0x{addr:08x}\n".encode())
    port.flush()


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
    eb = ap.add_argument_group("etherbone (fast --kernel upload; needs an Etherbone build)")
    eb.add_argument("--etherbone", action="store_true",
                    help="upload --kernel over Etherbone into SDRAM instead of SFL "
                         "over the 115200 console. Seconds instead of minutes for the "
                         "big firmware images; verified by CRC before booting.")
    eb.add_argument("--eb-ip", default=DEFAULT_EB_IP,
                    help=f"the board's IP (default {DEFAULT_EB_IP}). litex_server is "
                         f"started against it if one is not already listening.")
    eb.add_argument("--eb-host", default="localhost", help="litex_server host")
    eb.add_argument("--eb-port", type=int, default=1234, help="litex_server port")
    eb.add_argument("--csr-csv", default=DEFAULT_CSR_CSV,
                    help=f"csr.csv matching the running gateware (default {DEFAULT_CSR_CSV})")
    eb.add_argument("--eb-pace", type=float, default=0.002,
                    help="seconds between bursts (default 0.002). At full rate a large "
                         "write intermittently wedges the Wishbone bus while ICMP keeps "
                         "answering -- see targets/colorlight_i9/README.md.")
    args = ap.parse_args()

    # Whether the caller actually asked for a reset, as opposed to inheriting
    # the default. It matters only without --kernel: a bare `--send`/`--monitor`
    # run must not reboot a board the user is watching.
    reset_explicit = args.reset is not None or args.bitstream is not None
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
        # A reset was asked for but there is no kernel to boot. Without this the
        # reset silently does nothing -- the whole reset path used to live inside
        # the --kernel branch, so `--reset hard --bitstream X --send ...` looked
        # like it reconfigured while actually reading the PREVIOUS build's
        # console. That is a trap worth closing loudly rather than documenting.
        if not args.kernel and reset_explicit and args.reset != "none":
            if args.reset in ("hard", "auto") and args.bitstream:
                do_hard_reset(port, args.bitstream, args.probe_port)
                stamp("configured")
                drain(port, tee)
            elif args.reset == "soft":
                stamp("soft reset: reboot")
                do_soft_reset(port, tee, min(6.0, left()))

        if args.kernel and args.etherbone:
            # Etherbone needs the BIOS at its prompt, not the SFL offer: the
            # bytes go in over UDP and the console is only used to verify and
            # jump. A hard reset still reconfigures first when asked for.
            if args.reset in ("hard", "auto") and args.bitstream:
                do_hard_reset(port, args.bitstream, args.probe_port)
                stamp("configured")
                time.sleep(2.0)          # let the BIOS finish SDRAM init
            elif args.reset in ("soft", "auto"):
                stamp("soft reset: reboot")
                do_soft_reset(port, tee, min(6.0, left()))
            stamp("etherbone upload")
            etherbone_boot(port, tee, args.kernel, args.kernel_addr, args, left)
            stamp("booted")

        elif args.kernel:
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
