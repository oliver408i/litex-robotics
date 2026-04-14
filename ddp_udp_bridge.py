#!/usr/bin/env python3
import argparse
import socket
import struct
from typing import Iterable

from uart_client import MAX_BULK_LEDS, UartProtocol

DDP_FLAG_PUSH = 0x01
DDP_FLAG_TIMECODE = 0x10
DDP_DATA_TYPE_RGB = 1
DDP_PIXEL_24BIT = 5
DDP_PIXEL_24BIT_PACKED = 3


def parse_ddp_header(packet: bytes):
    if len(packet) < 10:
        return None
    header_len = 14 if packet[0] & DDP_FLAG_TIMECODE else 10
    if len(packet) < header_len:
        return None
    return {
        "flags": packet[0],
        "pixel_config": packet[2],
        "offset": struct.unpack(">I", packet[4:8])[0],
        "length": struct.unpack(">H", packet[8:10])[0],
        "header_len": header_len,
    }


def decode_pixel_config(pixel_config: int):
    return (pixel_config >> 3) & 0x07, pixel_config & 0x07


def iter_colors(data: bytes, order: str) -> Iterable[tuple[int, int, int]]:
    usable = len(data) - (len(data) % 3)
    for i in range(0, usable, 3):
        c0, c1, c2 = data[i : i + 3]
        if order == "rgb":
            yield (c1, c0, c2)
        else:
            yield (c0, c1, c2)


def send_colors(client: UartProtocol, start_led: int, colors: list[tuple[int, int, int]], led_count: int):
    index = start_led
    while colors and index < led_count:
        chunk = colors[:MAX_BULK_LEDS]
        colors = colors[MAX_BULK_LEDS:]
        if index + len(chunk) > led_count:
            chunk = chunk[: led_count - index]
        if not chunk:
            break
        client.set_strip_bulk(index, chunk)
        index += len(chunk)


def main():
    parser = argparse.ArgumentParser(description="Forward DDP UDP traffic to the IcePi Zero NeoPixel UART firmware.")
    parser.add_argument("--udp-port", type=int, default=4048)
    parser.add_argument("--uart-port", default="/dev/ttyUSB0")
    parser.add_argument("--uart-baud", type=int, default=1_000_000)
    parser.add_argument("--led-count", type=int, default=300)
    parser.add_argument("--ddp-order", choices=["rgb", "grb"], default="rgb")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.udp_port))

    client = UartProtocol(args.uart_port, args.uart_baud)
    try:
        if args.verbose:
            print(f"listening on udp/{args.udp_port}, forwarding to {args.uart_port} @ {args.uart_baud}")
        while True:
            packet, addr = sock.recvfrom(65535)
            header = parse_ddp_header(packet)
            if header is None:
                if args.verbose:
                    print(f"ignoring short packet from {addr}")
                continue

            data_type, data_size = decode_pixel_config(header["pixel_config"])
            if data_type != DDP_DATA_TYPE_RGB or data_size not in (DDP_PIXEL_24BIT, DDP_PIXEL_24BIT_PACKED):
                if args.verbose:
                    print(f"ignoring unsupported pixel config {header['pixel_config']}")
                continue
            if header["offset"] % 3 != 0:
                if args.verbose:
                    print(f"ignoring unaligned offset {header['offset']}")
                continue

            start = header["header_len"]
            end = start + header["length"]
            data = packet[start:end]
            if not data:
                continue

            start_led = header["offset"] // 3
            colors = list(iter_colors(data, args.ddp_order))
            send_colors(client, start_led, colors, args.led_count)

            if args.verbose and (header["flags"] & DDP_FLAG_PUSH):
                print(f"push frame from {addr}")
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        sock.close()


if __name__ == "__main__":
    main()
