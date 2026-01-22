#!/usr/bin/env python3
import argparse
import math
import time

from uart_client import UartProtocol


def bulk_fill(client: UartProtocol, count: int, g: int, r: int, b: int):
    chunk = 80
    for start in range(0, count, chunk):
        end = min(count, start + chunk)
        colors = [(g, r, b)] * (end - start)
        client.set_strip_bulk(start, colors)


def solid(client: UartProtocol, count: int, g: int, r: int, b: int, brightness: int):
    scale = brightness / 255.0
    g_out = int(round(g * scale))
    r_out = int(round(r * scale))
    b_out = int(round(b * scale))
    bulk_fill(client, count, g_out, r_out, b_out)


def chase(client: UartProtocol, count: int, g: int, r: int, b: int, brightness: int, delay_s: float):
    scale = brightness / 255.0
    g_out = int(round(g * scale))
    r_out = int(round(r * scale))
    b_out = int(round(b * scale))
    for i in range(count):
        # Clear previous LED.
        prev = (i - 1) % count
        client.set_strip(prev, 0, 0, 0)
        client.set_strip(i, g_out, r_out, b_out)
        time.sleep(delay_s)
    # Leave the last LED on.


def wheel(pos: int):
    # Simple GRB wheel (0-255).
    if pos < 85:
        return 255 - pos * 3, 0, pos * 3
    if pos < 170:
        pos -= 85
        return 0, pos * 3, 255 - pos * 3
    pos -= 170
    return pos * 3, 255 - pos * 3, 0


def rainbow(client: UartProtocol, count: int, brightness: int, delay_s: float, cycles: int):
    scale = brightness / 255.0
    for c in range(cycles * 256):
        for i in range(count):
            g, r, b = wheel((i + c) & 0xFF)
            client.set_strip(i, int(round(g * scale)), int(round(r * scale)), int(round(b * scale)))
        time.sleep(delay_s)


def breathe(client: UartProtocol, count: int, g: int, r: int, b: int, period_s: float):
    steps = 100
    while True:
        for i in range(steps):
            phase = (2.0 * math.pi * i) / steps
            scale = (math.sin(phase) + 1.0) / 2.0
            g_out = int(round(g * scale))
            r_out = int(round(r * scale))
            b_out = int(round(b * scale))
            bulk_fill(client, count, g_out, r_out, b_out)
            time.sleep(period_s / steps)

def configure_interp(client: UartProtocol, color_step: int, brightness_step: int) -> bool:
    if not hasattr(client, "set_strip_interp"):
        return False
    try:
        client.set_strip_interp(color_step, brightness_step)
        return True
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser(description="WS2812 strip pattern tests (LED1..LED149).")
    parser.add_argument("--port", default="/dev/ttyUSB1")
    parser.add_argument("--baud", default=115200, type=int)
    parser.add_argument("--count", default=299, type=int, help="Number of strip LEDs (excludes LED0).")
    parser.add_argument("--interp-color-step", type=int, default=0,
                        help="Per-tick color interpolation step (0 disables).")
    parser.add_argument("--interp-brightness-step", type=int, default=0,
                        help="Per-tick brightness interpolation step (0 disables).")
    sub = parser.add_subparsers(dest="pattern", required=True)

    solid_p = sub.add_parser("solid")
    solid_p.add_argument("g", type=int)
    solid_p.add_argument("r", type=int)
    solid_p.add_argument("b", type=int)
    solid_p.add_argument("--brightness", type=int, default=255)

    chase_p = sub.add_parser("chase")
    chase_p.add_argument("g", type=int)
    chase_p.add_argument("r", type=int)
    chase_p.add_argument("b", type=int)
    chase_p.add_argument("--brightness", type=int, default=255)
    chase_p.add_argument("--delay", type=float, default=0.03)

    rainbow_p = sub.add_parser("rainbow")
    rainbow_p.add_argument("--brightness", type=int, default=255)
    rainbow_p.add_argument("--delay", type=float, default=0.02)
    rainbow_p.add_argument("--cycles", type=int, default=1)
    breathe_p = sub.add_parser("breathe")
    breathe_p.add_argument("g", type=int)
    breathe_p.add_argument("r", type=int)
    breathe_p.add_argument("b", type=int)
    breathe_p.add_argument("--period", type=float, default=2.0)

    args = parser.parse_args()

    client = UartProtocol(args.port, args.baud)
    interp_enabled = False
    try:
        if args.interp_color_step or args.interp_brightness_step:
            interp_enabled = configure_interp(
                client, args.interp_color_step, args.interp_brightness_step
            )
        if args.pattern == "solid":
            solid(client, args.count, args.g, args.r, args.b, args.brightness)
        elif args.pattern == "chase":
            chase(client, args.count, args.g, args.r, args.b, args.brightness, args.delay)
        elif args.pattern == "rainbow":
            rainbow(client, args.count, args.brightness, args.delay, args.cycles)
        elif args.pattern == "breathe":
            breathe(client, args.count, args.g, args.r, args.b, args.period)
    finally:
        if interp_enabled:
            configure_interp(client, 0, 0)
        client.close()


if __name__ == "__main__":
    main()
