#!/usr/bin/env python3
import argparse
import time

from uart_client import UartProtocol


def parse_color(args):
    return args.g, args.r, args.b


def main():
    parser = argparse.ArgumentParser(description="Host-side test for uart_icepi neopixel firmware.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--count", type=int, default=300, help="Number of strip LEDs.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")
    sub.add_parser("get")

    p_set = sub.add_parser("set")
    p_set.add_argument("g", type=int)
    p_set.add_argument("r", type=int)
    p_set.add_argument("b", type=int)
    p_set.add_argument("--brightness", type=int, default=255)

    p_led = sub.add_parser("led")
    p_led.add_argument("index", type=int)
    p_led.add_argument("g", type=int)
    p_led.add_argument("r", type=int)
    p_led.add_argument("b", type=int)
    p_led.add_argument("--brightness", type=int, default=255)

    p_fill = sub.add_parser("fill")
    p_fill.add_argument("g", type=int)
    p_fill.add_argument("r", type=int)
    p_fill.add_argument("b", type=int)
    p_fill.add_argument("--chunk", type=int, default=80)

    p_chase = sub.add_parser("chase")
    p_chase.add_argument("g", type=int)
    p_chase.add_argument("r", type=int)
    p_chase.add_argument("b", type=int)
    p_chase.add_argument("--delay", type=float, default=0.05)
    p_chase.add_argument("--loops", type=int, default=1)

    p_interp = sub.add_parser("interp")
    p_interp.add_argument("color_step", type=int)
    p_interp.add_argument("brightness_step", type=int)

    args = parser.parse_args()

    client = UartProtocol(args.port, args.baud)
    try:
        if args.cmd == "ping":
            rsp = client.ping()
            print("PING:", rsp.decode("ascii", errors="replace"))
        elif args.cmd == "get":
            en, bri, g, r, b = client.get_neopixel()
            print(f"NEO: en={en} bri={bri} g={g} r={r} b={b}")
        elif args.cmd == "set":
            g, r, b = parse_color(args)
            client.set_neopixel(1, args.brightness, g, r, b)
            print("NEO set")
        elif args.cmd == "led":
            if args.index < 0 or args.index >= args.count:
                raise ValueError("index out of range")
            client.set_strip_bri(args.index, args.g, args.r, args.b, args.brightness)
            print(f"LED{args.index} set")
        elif args.cmd == "fill":
            colors = []
            for _ in range(args.chunk):
                colors.append((args.g, args.r, args.b))
            for start in range(0, args.count, args.chunk):
                count = min(args.chunk, args.count - start)
                client.set_strip_bulk(start, colors[:count])
            print("Strip filled")
        elif args.cmd == "chase":
            for _ in range(args.loops):
                for i in range(args.count):
                    prev = (i - 1) % args.count
                    client.set_strip(prev, 0, 0, 0)
                    client.set_strip(i, args.g, args.r, args.b)
                    time.sleep(args.delay)
            print("Chase done")
        elif args.cmd == "interp":
            client.set_strip_interp(args.color_step, args.brightness_step)
            print("Interpolation set")
    finally:
        client.close()


if __name__ == "__main__":
    main()
