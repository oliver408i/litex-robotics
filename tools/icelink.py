#!/usr/bin/env python3
"""Host side of the icelink-fast probe: load an ECP5 bitstream over JTAG.

The Colorlight i9 has no on-board FTDI and openFPGALoader's CH347 backend does
not work against this chip (see probe/icelink-fast/README.md), so the MCU probe
is the only way to configure the FPGA. This is the host half of its `p`
command.

Use it directly:

    .venv/bin/python tools/icelink.py build/.../gateware/foo.bit

or let a target do it for you:

    .venv/bin/python targets/colorlight_i9/bringup.py --build --load

Wire protocol, such as it is: send `p`, then a u32 little-endian length, then
that many bytes. The probe shifts them straight from its USB FIFO to TDI --
a 45F bitstream is ~1.15 MB against the probe's 20 KB of RAM, so nothing is
buffered at either end. It then reports the status register; bit 8 is DONE.
"""
import argparse
import os
import struct
import sys
import time

DEFAULT_PORT = "/dev/icelink-console"   # if00; the udev rule in probe/icelink-fast/tools/
CHUNK = 4096


class IcelinkError(RuntimeError):
    pass


def _open(port):
    try:
        import serial
    except ImportError:
        raise IcelinkError("pyserial not installed (.venv/bin/pip install pyserial)")
    if not os.path.exists(port):
        raise IcelinkError(
            f"{port} not found. Is the probe plugged in?\n"
            f"  Without the udev rule it is an unstable /dev/ttyACM*; see\n"
            f"  probe/icelink-fast/tools/99-icelink-fast.rules"
        )
    try:
        s = serial.Serial(port, 115200, timeout=1.0)
    except Exception as e:                       # permission is the usual one
        raise IcelinkError(f"cannot open {port}: {e}\n"
                           f"  (try: sg dialout -c '...', or install the udev rule)")
    s.dtr = True
    time.sleep(0.4)
    s.reset_input_buffer()
    return s


def program_sram(bitstream, port=DEFAULT_PORT, timeout=60.0, verbose=True):
    """Configure the ECP5's SRAM with `bitstream`. Returns the status register.

    Raises IcelinkError if DONE does not come up.
    """
    with open(bitstream, "rb") as f:
        data = f.read()
    if not data:
        raise IcelinkError(f"{bitstream} is empty")

    s = _open(port)
    try:
        s.write(b"p")
        s.flush()
        time.sleep(0.3)
        s.write(struct.pack("<I", len(data)))
        s.flush()

        t0 = time.time()
        for i in range(0, len(data), CHUNK):
            s.write(data[i:i + CHUNK])
        s.flush()

        # The probe prints a running commentary; wait for a terminal word.
        out = b""
        while time.time() - t0 < timeout:
            out += s.read(4096)
            if b"configured" in out or b"failed" in out or b"aborting" in out \
               or b"implausible" in out or b"timeout" in out:
                time.sleep(0.4)
                out += s.read(4096)
                break
        text = out.decode("utf-8", "replace")
    finally:
        s.close()

    if verbose:
        for line in text.splitlines():
            line = line.strip()
            if line and not line.startswith("send u32le"):
                print(f"  {line}")

    if "DONE -- configured" not in text:
        raise IcelinkError(
            "configuration failed -- DONE never came up.\n"
            "  Check the FPGA is seated (IDCODE should be a real part, not\n"
            "  0x00000000 / 0xffffffff) and that the bitstream is for this device."
        )

    status = None
    for line in text.splitlines():
        if "final status" in line:
            try:
                status = int(line.split("0x")[1].split()[0], 16)
            except (IndexError, ValueError):
                pass
    return status


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bitstream", help="path to the .bit file")
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"probe console port (default {DEFAULT_PORT})")
    args = ap.parse_args()
    try:
        program_sram(args.bitstream, port=args.port)
    except IcelinkError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print("configured.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
