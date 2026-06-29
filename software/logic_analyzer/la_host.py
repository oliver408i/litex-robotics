#!/usr/bin/env python3
"""Host-side consumer app for the IcePi Zero logic analyzer.

Talks the LA TCP control protocol (docs/logic_analyzer.md §4) to the board's
WINC firmware: configure -> arm -> poll -> download -> reorder -> export.

  ./la_host.py icepi.local --rate 1e6 --depth 1048576 \
      --trig-mask 0x00001 --trig-value 0x00001 --edge \
      --post 524288 --vcd capture.vcd

Pairs with software/logic_analyzer/main.c (the LA firmware). The protocol and
the readback/reorder math are the contract between the two.
"""
import argparse
import socket
import struct
import sys
import time

PORT = 5559

# Channel index -> board IO label (must match _la_io order in soc_features.py).
CH_IO = [1, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 26, 27]
# NOTE: keep this list in lockstep with gateware/soc_features.py `_la_io`.
#       ch index = position here; the value is the silkscreen IOn number.
N_CH = 18

# status flag bit positions (status CSR field order)
F_RUNNING, F_TRIGGERED, F_DONE, F_OVERRUN, F_WRAPPED, F_LCD_PRESENT = range(6)


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(f"closed with {len(buf)}/{n} bytes")
        buf += chunk
    return bytes(buf)


class LAClient:
    def __init__(self, host, timeout=30):
        self.sock = socket.create_connection((host, PORT), timeout=timeout)
        self.sock.settimeout(timeout)

    def _cmd(self, tag, payload=b"", reply_len=4):
        self.sock.sendall(tag + payload)
        return _recv_exact(self.sock, reply_len) if reply_len else b""

    def info(self):
        """Query board capabilities. -> (magic, n_channels, ring_capacity, sys_hz, lcd_present)."""
        r = self._cmd(b"LNFO", reply_len=17)
        n_ch, ring_cap, sys_hz = struct.unpack("<III", r[4:16])
        return r[:4], n_ch, ring_cap, sys_hz, r[16]

    def configure(self, sample_div, trig_mask, trig_value, trig_edge,
                  ring_size, post_trig):
        # The board owns the ring buffer; the host no longer sends a base.
        p = struct.pack("<IIIBII", sample_div, trig_mask, trig_value,
                        1 if trig_edge else 0, ring_size, post_trig)
        if self._cmd(b"LCFG", p) != b"LOK\0":
            raise RuntimeError("LCFG rejected")

    def arm(self):
        r = self._cmd(b"LARM")
        if r == b"LBSY":
            raise RuntimeError("board refused to arm: LCD/touch module still attached")
        if r != b"LOK\0":
            raise RuntimeError(f"LARM rejected: {r!r}")

    def status(self):
        r = self._cmd(b"LSTA", reply_len=1 + 4 * 3)
        flags, wr_count, wptr, trig_addr = struct.unpack("<BIII", r)
        return flags, wr_count, wptr, trig_addr

    def dump(self, start_sample, n_samples):
        self.sock.sendall(b"LDMP" + struct.pack("<II", start_sample, n_samples))
        return _recv_exact(self.sock, n_samples * 4)

    def abort(self):
        self._cmd(b"LABT")


def reorder(raw, ring_size, wptr, wrapped):
    """raw = ring_size*4 bytes of ring slots -> chronological list of u32 samples."""
    words = list(struct.unpack(f"<{ring_size}I", raw))
    if not wrapped:
        return words[:wptr]
    return words[wptr:] + words[:wptr]   # oldest at wptr


def write_vcd(path, samples, period_ns):
    """Minimal VCD: one 1-bit wire per channel, only emit edges."""
    ids = [chr(ord("!") + i) for i in range(N_CH)]
    with open(path, "w") as f:
        f.write("$timescale 1ns $end\n")
        for i, io in enumerate(CH_IO):
            f.write(f"$var wire 1 {ids[i]} IO{io} $end\n")
        f.write("$enddefinitions $end\n")
        prev = None
        for t, s in enumerate(samples):
            f.write(f"#{int(t * period_ns)}\n")
            for i in range(N_CH):
                bit = (s >> i) & 1
                if prev is None or ((prev >> i) & 1) != bit:
                    f.write(f"{bit}{ids[i]}\n")
            prev = s
    print(f"wrote {len(samples)} samples -> {path}")


def write_csv(path, samples):
    with open(path, "w") as f:
        f.write("sample," + ",".join(f"IO{io}" for io in CH_IO) + "\n")
        for t, s in enumerate(samples):
            f.write(f"{t}," + ",".join(str((s >> i) & 1) for i in range(N_CH)) + "\n")
    print(f"wrote {len(samples)} samples -> {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", nargs="?", default="icepi.local")
    ap.add_argument("--rate", type=float, default=1e6, help="sample rate in Hz (<= 50e6)")
    ap.add_argument("--depth", type=int, default=1 << 20, help="ring size in samples")
    ap.add_argument("--trig-mask",  type=lambda x: int(x, 0), default=0)
    ap.add_argument("--trig-value", type=lambda x: int(x, 0), default=0)
    ap.add_argument("--edge", action="store_true", help="trigger on transition into match")
    ap.add_argument("--post", type=int, default=None, help="post-trigger samples (default depth/2)")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for done")
    ap.add_argument("--vcd"); ap.add_argument("--csv")
    args = ap.parse_args()

    la = LAClient(args.host, timeout=max(args.timeout, 30))
    magic, n_ch, ring_cap, sys_hz, lcd = la.info()
    if magic != b"LA01":
        sys.exit(f"unexpected LNFO magic {magic!r} -- wrong firmware?")
    if lcd:
        sys.exit("board reports an LCD/touch module is still attached -- unplug it before capturing")
    if n_ch != N_CH:
        print(f"WARNING: board reports {n_ch} channels, host expects {N_CH}", file=sys.stderr)
    depth = min(args.depth, ring_cap)
    if depth < args.depth:
        print(f"depth clamped to board capacity {ring_cap} samples", file=sys.stderr)

    sample_div = max(0, round(sys_hz / args.rate) - 1)
    actual = sys_hz / (sample_div + 1)
    period_ns = 1e9 / actual
    post = args.post if args.post is not None else depth // 2

    la.configure(sample_div, args.trig_mask, args.trig_value, args.edge, depth, post)
    print(f"rate {actual/1e6:.3f} MS/s (div={sample_div}), depth {depth}, post {post}")
    la.arm()
    print("armed; waiting for done...")

    deadline = time.monotonic() + args.timeout
    while True:
        flags, wr_count, wptr, trig_addr = la.status()
        if flags & (1 << F_OVERRUN):
            print("WARNING: overrun -- sample rate outran SDRAM, samples dropped", file=sys.stderr)
        if flags & (1 << F_DONE):
            break
        if time.monotonic() > deadline:
            print("timeout waiting for trigger/done; aborting", file=sys.stderr)
            la.abort()
            flags, wr_count, wptr, trig_addr = la.status()
            break
        time.sleep(0.05)

    wrapped = bool(flags & (1 << F_WRAPPED))
    print(f"done: wr_count={wr_count} wptr={wptr} trig_addr={trig_addr} wrapped={wrapped}")

    raw = la.dump(0, depth)
    samples = reorder(raw, depth, wptr, wrapped)

    if args.vcd:
        write_vcd(args.vcd, samples, period_ns)
    if args.csv:
        write_csv(args.csv, samples)
    if not args.vcd and not args.csv:
        print(f"captured {len(samples)} samples (no --vcd/--csv given; nothing written)")


if __name__ == "__main__":
    main()
