#!/usr/bin/env python3
"""Read and change the ADC stream's settings at runtime, over Etherbone.

Channel, sample rate and destination are CSRs, so none of them needs a rebuild.
This reaches them through `litex_server`, which must already be running against
the board:

    .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &
    .venv/bin/python tools/adc_stream_ctl.py                 # show status
    .venv/bin/python tools/adc_stream_ctl.py --channel 3 --rate 40000
    .venv/bin/python tools/adc_stream_ctl.py --reset-counters

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
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LITEX_SETUP = REPO / "litex-setup"
for _p in ["litex", "litex-boards", "migen"]:
    _cand = LITEX_SETUP / _p
    if _cand.is_dir() and str(_cand) not in sys.path:
        sys.path.insert(0, str(_cand))

from litex import RemoteClient   # noqa: E402


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
    args = ap.parse_args()

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
