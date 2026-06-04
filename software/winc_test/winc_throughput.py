#!/usr/bin/env python3
"""Host-side TCP throughput test for the WINC bring-up firmware.

Connects to the board's echo server, switches it into sink mode
("BLST" + u32le byte-count header), blasts the payload, and times until the
board's "DONE" ack -- i.e. it measures true delivered application throughput
(host -> WINC -> SPI -> firmware), the same direction the code loader uses.

Usage:  ./winc_throughput.py [icepi.local] [--port 5555] [--mb 2]
"""
import argparse
import socket
import struct
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("host", nargs="?", default="icepi.local")
ap.add_argument("--port", type=int, default=5555)
ap.add_argument("--mb", type=float, default=2.0, help="payload size in MiB")
ap.add_argument("--udp", action="store_true",
                help="UDP blast to port 5556 instead (bypasses WINC TCP; "
                     "measures raw radio->SPI delivery + loss)")
ap.add_argument("--pps", type=int, default=0,
                help="UDP only: rate-limit to N packets/s (0 = full blast)")
args = ap.parse_args()

n = int(args.mb * 1024 * 1024)
payload = (bytes(range(256)) * (n // 256 + 1))[:n]

if args.udp:
    DGRAM = 1400
    count = n // DGRAM
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(5)
    # Resolve ONCE -- passing a hostname tuple to sendto() makes Python do an
    # mDNS lookup per packet (~23 ms each), which rate-limits the blast.
    dst = (socket.gethostbyname(args.host), 5556)
    chunk = payload[:DGRAM]
    t0 = time.monotonic()
    for i in range(count):
        s.sendto(chunk, dst)
        if args.pps:
            time.sleep(1 / args.pps)
    dt_send = time.monotonic() - t0
    time.sleep(0.5)                      # let the pipe drain
    got = None
    for _ in range(5):                   # STOP may be lost; retry
        s.sendto(b"USTP", dst)
        try:
            reply, _ = s.recvfrom(64)
            got, dgrams = struct.unpack("<II", reply[:8])
            break
        except socket.timeout:
            pass
    if got is None:
        sys.exit("no reply to USTP -- board unreachable?")
    sent = count * DGRAM
    print(f"sent {sent} B ({count} datagrams) in {dt_send:.2f} s "
          f"= {sent/dt_send/1e6:.2f} MB/s offered")
    print(f"board received {got} B ({dgrams} datagrams) "
          f"= {got/dt_send/1e6:.2f} MB/s delivered, "
          f"loss {100*(1-got/sent):.1f}%")
    sys.exit(0)

s = socket.create_connection((args.host, args.port), timeout=10)
s.settimeout(120)
print(f"banner: {s.recv(256).decode(errors='replace').strip()}")

t0 = time.monotonic()
s.sendall(b"BLST" + struct.pack("<I", n) + payload)
resp = b""
while b"DONE" not in resp:
    chunk = s.recv(64)
    if not chunk:
        sys.exit("connection closed before DONE ack")
    resp += chunk
dt = time.monotonic() - t0
s.close()

print(f"{n} bytes in {dt:.2f} s = {n/dt/1e6:.2f} MB/s ({n*8/dt/1e6:.2f} Mbit/s)")
print(f"(a 578 KB mnist_lcd_demo.bin would take ~{578*1024/(n/dt):.1f} s)")
