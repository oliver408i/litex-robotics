#!/usr/bin/env python3
"""Read and change the ADC stream's settings at runtime, over Etherbone.

Channel, sample rate and destination are CSRs, so none of them needs a rebuild.
This reaches them through `litex_server`, which must already be running against
the board:

    .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &
    .venv/bin/python tools/adc_stream_ctl.py                 # show status
    .venv/bin/python tools/adc_stream_ctl.py --channel 3 --rate 40000
    .venv/bin/python tools/adc_stream_ctl.py --reset-counters
    .venv/bin/python tools/adc_stream_ctl.py --auto           # stream to ME specifically

`--auto` is the answer to "what's my IP on this machine, again" -- it never
needs typing out by hand, so it is correct on whatever computer runs it. It
opens a UDP socket toward `--board-ip` (default the board's usual static
address) and reads back the local address the OS picked to reach it -- the
same trick tools/adc_web.py uses to print its own URL -- then writes that as
`--dest-ip`. No packets are sent by this: UDP `connect()` only picks a route.
The build-time default is already the broadcast address (every host on the
link receives it, no config needed), so `--auto` is for the opposite case:
stop broadcasting and send to exactly one machine.

Etherbone and the sample stream are two UDP ports on one core, so this works
while a capture is running -- changing the channel mid-capture takes effect on
the next conversion, and the packet header reports the new channel, so the
receiver's file stays self-describing.

Two `litex_server` traps that have cost time on this bench before: its worker
thread can die silently while the process stays up and listening, so a
successful connect proves nothing until a read returns something sane; and it
must be restarted after every board reload, or every read comes back zero. If
`status` prints all zeros including `sys_clk_freq`, restart the server before
believing anything here.
"""
from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LITEX_SETUP = REPO / "litex-setup"
for _p in ["litex", "litex-boards", "migen"]:
    _cand = LITEX_SETUP / _p
    if _cand.is_dir() and str(_cand) not in sys.path:
        sys.path.insert(0, str(_cand))

from litex import RemoteClient   # noqa: E402


def _auto_dest_ip(board_ip: str) -> str:
    """This machine's IP as the OS would route it toward `board_ip`.

    UDP `connect()` does not send anything -- it just asks the routing table
    which local address and interface a packet to that destination would use,
    which `getsockname()` then reveals. Same trick tools/adc_web.py uses to
    print its own URL. Raises OSError if `board_ip` is unroutable (no network
    path at all, not even ARP-unresolved -- that would still succeed here).
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((board_ip, 1))
        return s.getsockname()[0]
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csr-csv", default=str(REPO / "build/colorlight_i5/csr.csv"),
                    help="CSR map of the loaded bitstream")
    ap.add_argument("--channel", type=int, choices=range(8),
                    help="MCP3008 channel to stream")
    ap.add_argument("--rate", type=float,
                    help="sample rate in Hz (realised as an integer divisor of sys)")
    ap.add_argument("--period", type=int,
                    help="sys clocks between conversions, instead of --rate")
    ap.add_argument("--enable", type=int, choices=(0, 1), help="stop/start the stream")
    ap.add_argument("--reset-counters", action="store_true",
                    help="zero seq/emitted/dropped, so the next capture starts clean")
    ap.add_argument("--dest-ip", help="re-point the stream at another host, e.g. 192.168.0.11")
    ap.add_argument("--dest-port", type=int, help="destination UDP port")
    ap.add_argument("--auto", action="store_true",
                    help="detect this machine's own IP toward --board-ip and use that "
                         "as --dest-ip, instead of typing an address by hand. Correct "
                         "on any computer/network; mutually exclusive with --dest-ip.")
    ap.add_argument("--board-ip", default="192.168.0.50",
                    help="the board's address, for --auto to route toward (default: "
                         "the usual static --local-ip from targets/colorlight_i9/base.py)")
    args = ap.parse_args()

    if args.auto:
        if args.dest_ip is not None:
            raise SystemExit("--auto and --dest-ip are mutually exclusive -- "
                             "--auto picks the address for you")
        try:
            args.dest_ip = _auto_dest_ip(args.board_ip)
        except OSError as e:
            raise SystemExit(f"--auto: can't find a route to {args.board_ip}: {e}")
        print(f"--auto: this machine is {args.dest_ip} as seen from {args.board_ip}")

    bus = RemoteClient(csr_csv=args.csr_csv)
    bus.open()
    try:
        regs = bus.regs
        if not hasattr(regs, "adc_stream_period"):
            raise SystemExit("this csr.csv has no adc_stream block -- wrong bitstream "
                             "or a stale build/colorlight_i5/csr.csv")

        # sys_clk_freq is not a CSR; take it from the generated constants.
        sys_clk = getattr(bus.constants, "config_clock_frequency", None)
        if sys_clk is None:
            sys_clk = 90_000_000

        if args.reset_counters:
            regs.adc_stream_reset_counters.write(1)
        if args.channel is not None:
            regs.adc_stream_channel.write(args.channel)
        if args.period is not None:
            regs.adc_stream_period.write(max(2, args.period))
        elif args.rate is not None:
            regs.adc_stream_period.write(max(2, int(sys_clk // args.rate)))
        if args.dest_ip is not None:
            ip = 0
            for octet in args.dest_ip.split("."):
                ip = (ip << 8) | int(octet)
            regs.adc_stream_dest_ip.write(ip)
        if args.dest_port is not None:
            regs.adc_stream_dest_port.write(args.dest_port)
        if args.enable is not None:
            regs.adc_stream_enable.write(args.enable)

        period = regs.adc_stream_period.read()
        print(f"enable       {regs.adc_stream_enable.read()}")
        print(f"channel      {regs.adc_stream_channel.read()}")
        print(f"period       {period} sys clocks  ->  {sys_clk/period:,.1f} sps")
        print(f"seq          {regs.adc_stream_seq.read():,} packets sent")
        print(f"emitted      {regs.adc_stream_emitted.read():,} samples sent")
        print(f"dropped      {regs.adc_stream_dropped.read():,} samples discarded")
        print(f"overruns     {regs.adc_stream_overruns.read():,} "
              f"(rate ticks that arrived mid-conversion -- non-zero means the "
              f"requested rate is faster than SCLK allows)")
        ip = regs.adc_stream_dest_ip.read()
        print(f"destination  {(ip>>24)&0xff}.{(ip>>16)&0xff}.{(ip>>8)&0xff}.{ip&0xff}"
              f":{regs.adc_stream_dest_port.read()}")
        print(f"last sample  {regs.adc_stream_last.read()}")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
