#!/usr/bin/env python3
"""Drive the CH347's TRST pin (GPIO5), which on this board is wired to NRST.

Why this exists: OpenOCD's ch347 driver cannot touch TRST in SWD mode -- the
binary contains "Asserting TRST not supported in SWD mode!" -- so the reset line
has to be set up out-of-band, before OpenOCD attaches, and released after it
exits.

GPIO wire format is lifted verbatim from the verified-on-hardware driver in
~/src/ch347-lcd/ch347.py (mode 1). The 0xCC GPIO command is mode-independent;
what changes between modes is the PID and which interface carries the bulk
endpoints, so both are discovered rather than assumed.

    python3 ch347_nrst.py --state     # read GPIO5 without changing anything
    python3 ch347_nrst.py --high      # drive NRST high
    python3 ch347_nrst.py --low       # drive NRST low
    python3 ch347_nrst.py --release   # back to input (high-Z)
"""

import argparse
import struct
import sys

import usb.core
import usb.util

VID = 0x1A86
PIDS = (0x55DD, 0x55DE, 0x55DB)   # JTAG/SWD modes first, mode 1 last

CMD_GPIO = 0xCC
G_ENABLE = 0xC0     # "this pin is being changed"
G_DIR_OUT = 0x30    # direction = output
G_VAL_HIGH = 0x08   # output value = 1

TRST_GPIO = 5       # CH347 pin 9, DTR0 / SCS1 / TRST -> board NRST


class CH347Error(RuntimeError):
    pass


class CH347Gpio:
    def __init__(self, timeout=1000):
        self.timeout = timeout
        self.dev = None
        for pid in PIDS:
            self.dev = usb.core.find(idVendor=VID, idProduct=pid)
            if self.dev is not None:
                self.pid = pid
                break
        if self.dev is None:
            raise CH347Error(
                "no CH347 found (tried " +
                ", ".join(f"{VID:04x}:{p:04x}" for p in PIDS) + ")")

        self.iface, self.ep_out, self.ep_in = self._find_vendor_iface()

        if self.dev.is_kernel_driver_active(self.iface):
            self.dev.detach_kernel_driver(self.iface)
        usb.util.claim_interface(self.dev, self.iface)

    def _find_vendor_iface(self):
        """Locate the vendor-class interface with a bulk IN/OUT pair.

        Mode 1 is interface 2 with EP 0x06/0x86, but the JTAG/SWD modes differ,
        and OpenOCD's own udev rule only matches 55dd -- so search instead of
        hardcoding.
        """
        for cfg in self.dev:
            for intf in cfg:
                if intf.bInterfaceClass != 0xFF:      # vendor-specific only
                    continue
                outs = [e for e in intf if usb.util.endpoint_direction(
                    e.bEndpointAddress) == usb.util.ENDPOINT_OUT and
                    usb.util.endpoint_type(e.bmAttributes) ==
                    usb.util.ENDPOINT_TYPE_BULK]
                ins = [e for e in intf if usb.util.endpoint_direction(
                    e.bEndpointAddress) == usb.util.ENDPOINT_IN and
                    usb.util.endpoint_type(e.bmAttributes) ==
                    usb.util.ENDPOINT_TYPE_BULK]
                if outs and ins:
                    return (intf.bInterfaceNumber,
                            outs[0].bEndpointAddress,
                            ins[0].bEndpointAddress)
        raise CH347Error("no vendor interface with bulk endpoints found")

    def close(self):
        if self.dev is not None:
            usb.util.release_interface(self.dev, self.iface)
            usb.util.dispose_resources(self.dev)

    def gpio_raw(self, pins=None):
        pins = bytes(pins if pins is not None else b"\x00" * 8)
        if len(pins) != 8:
            raise ValueError("need exactly 8 GPIO bytes")
        for _ in range(2):
            self.dev.write(self.ep_out,
                           bytes([CMD_GPIO, 8, 0]) + pins, self.timeout)
            try:
                reply = bytes(self.dev.read(self.ep_in, 11, self.timeout))
            except usb.core.USBError:
                reply = b""
            if len(reply) >= 11 and reply[0] == CMD_GPIO:
                return reply[3:11]
        raise CH347Error(f"short GPIO reply: {reply.hex()}")

    def state(self):
        return [(i, bool(b & 0x80), bool(b & 0x40))
                for i, b in enumerate(self.gpio_raw())]

    def write(self, index, value):
        pins = bytearray(8)
        pins[index] = G_ENABLE | G_DIR_OUT | (G_VAL_HIGH if value else 0)
        self.gpio_raw(pins)

    def release(self, index):
        """Back to input. NOTE: whatever the net is pulled to now decides
        the target's reset state -- that is the point of high-Z, but it means
        'release' is not the same as 'deassert'."""
        pins = bytearray(8)
        pins[index] = G_ENABLE          # enable the change, leave dir = input
        self.gpio_raw(pins)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--state", action="store_true", help="read all 8 GPIOs")
    g.add_argument("--high", action="store_true", help="drive NRST high")
    g.add_argument("--low", action="store_true", help="drive NRST low")
    g.add_argument("--release", action="store_true", help="NRST back to input")
    ap.add_argument("--pin", type=int, default=TRST_GPIO,
                    help=f"GPIO index (default {TRST_GPIO} = TRST)")
    args = ap.parse_args()

    # A missing adapter is the normal case when nothing is clipped on; report it
    # as a message rather than a traceback, which reads like a crash.
    try:
        ch = CH347Gpio()
    except CH347Error as e:
        print(f"{e}", file=sys.stderr)
        print("  is the CH347 plugged in? (lsusb | grep 1a86)", file=sys.stderr)
        return 2
    print(f"CH347 {VID:04x}:{ch.pid:04x}  iface {ch.iface} "
          f"EP out 0x{ch.ep_out:02x} in 0x{ch.ep_in:02x}")
    try:
        if args.state:
            for i, is_out, level in ch.state():
                tag = " <- TRST/NRST" if i == args.pin else ""
                print(f"  GPIO{i}  {'output' if is_out else 'input '}  "
                      f"{'high' if level else 'low '}{tag}")
        elif args.high:
            ch.write(args.pin, True)
            print(f"  GPIO{args.pin} driven HIGH")
        elif args.low:
            ch.write(args.pin, False)
            print(f"  GPIO{args.pin} driven LOW")
        elif args.release:
            ch.release(args.pin)
            print(f"  GPIO{args.pin} released to input (high-Z)")
    finally:
        ch.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
