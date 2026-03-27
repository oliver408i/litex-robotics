#!/usr/bin/env python3
import argparse
import socket
import struct
from typing import List, Tuple

from uart_client import UartProtocol

DDP_FLAG_TIMECODE = 0x10
DDP_FLAG_PUSH = 0x01

DDP_DATA_TYPE_RGB = 1
DDP_PIXEL_24BIT = 5
DDP_PIXEL_24BIT_PACKED = 3

MAX_BULK = 80


def parse_ddp_header(packet: bytes):
    if len(packet) < 10:
        return None
    flags = packet[0]
    seq = packet[1]
    pixel_config = packet[2]
    pkt_id = packet[3]
    offset = struct.unpack(">I", packet[4:8])[0]
    length = struct.unpack(">H", packet[8:10])[0]
    header_len = 14 if (flags & DDP_FLAG_TIMECODE) else 10
    if len(packet) < header_len:
        return None
    return {
        "flags": flags,
        "seq": seq,
        "pixel_config": pixel_config,
        "id": pkt_id,
        "offset": offset,
        "length": length,
        "header_len": header_len,
    }


def decode_pixel_config(pixel_config: int):
    data_type = (pixel_config >> 3) & 0x07
    data_size = pixel_config & 0x07
    return data_type, data_size


def ddp_bytes_to_colors(data: bytes, order: str) -> List[Tuple[int, int, int]]:
    colors = []
    for i in range(0, len(data) - (len(data) % 3), 3):
        c0 = data[i]
        c1 = data[i + 1]
        c2 = data[i + 2]
        if order == "rgb":
            r, g, b = c0, c1, c2
            colors.append((g, r, b))
        else:
            g, r, b = c0, c1, c2
            colors.append((g, r, b))
    return colors


def send_colors(client: UartProtocol, start_led: int, colors: List[Tuple[int, int, int]], led_count: int):
    idx = start_led
    remaining = colors
    while remaining and idx < led_count:
        chunk = remaining[:MAX_BULK]
        remaining = remaining[MAX_BULK:]
        if idx + len(chunk) > led_count:
            chunk = chunk[: max(0, led_count - idx)]
            if not chunk:
                break
        client.set_strip_bulk(idx, chunk)
        idx += len(chunk)


def main():
    parser = argparse.ArgumentParser(description="DDP UDP (port 4048) to IcePi UART NeoPixel bridge.")
    parser.add_argument("--udp-port", type=int, default=4048)
    parser.add_argument("--uart-port", default="/dev/ttyUSB0")
    parser.add_argument("--uart-baud", type=int, default=1_000_000)
    parser.add_argument("--led-count", type=int, default=300)
    parser.add_argument("--ddp-order", choices=["rgb", "grb"], default="rgb",
                        help="Color byte order in incoming DDP payload.")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.udp_port))

    client = UartProtocol(args.uart_port, args.uart_baud)
    try:
        if args.verbose:
            print(f"Listening UDP :{args.udp_port}, forwarding to {args.uart_port} @ {args.uart_baud}")
        while True:
            packet, addr = sock.recvfrom(65535)
            header = parse_ddp_header(packet)
            if header is None:
                if args.verbose:
                    print("DDP: short packet from", addr)
                continue
            data_type, data_size = decode_pixel_config(header["pixel_config"])
            if data_type != DDP_DATA_TYPE_RGB or data_size not in (DDP_PIXEL_24BIT, DDP_PIXEL_24BIT_PACKED):
                if args.verbose:
                    print("DDP: unsupported pixel config", header["pixel_config"])
                continue
            start = header["header_len"]
            end = start + header["length"]
            data = packet[start:end]
            if not data:
                continue
            if header["offset"] % 3 != 0:
                if args.verbose:
                    print("DDP: offset not aligned:", header["offset"])
                continue
            start_led = header["offset"] // 3
            colors = ddp_bytes_to_colors(data, args.ddp_order)
            send_colors(client, start_led, colors, args.led_count)
            if args.verbose and (header["flags"] & DDP_FLAG_PUSH):
                print("DDP: push frame from", addr)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        sock.close()


if __name__ == "__main__":
    main()
