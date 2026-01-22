#!/usr/bin/env python3
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    serial = None

PREAMBLE_0 = 0xAA
PREAMBLE_1 = 0x55

CMD_PING = 0x01
CMD_GET_VERSION = 0x02
CMD_SET_MOTOR = 0x10
CMD_GET_MOTOR = 0x11
CMD_SET_SERVO = 0x12
CMD_GET_SERVO = 0x13
CMD_SET_GPIO = 0x14
CMD_GET_GPIO = 0x15
CMD_ESTOP = 0x16
CMD_GET_STATUS = 0x20
CMD_SET_NEOPIXEL = 0x30
CMD_GET_NEOPIXEL = 0x31
CMD_SET_STRIP = 0x32
CMD_SET_STRIP_BRI = 0x33
CMD_SET_STRIP_BULK = 0x34
CMD_SET_STRIP_INTERP = 0x35
CMD_GET_ADC = 0x40
CMD_SET_ADC_CFG = 0x41
CMD_CLR_ADC_UPD = 0x42
CMD_GET_ESTOP = 0x50
CMD_GET_AS5600 = 0x60

RSP_ERROR = 0x7F


def checksum_xor(data: bytes) -> int:
    chk = 0
    for b in data:
        chk ^= b
    return chk


class UartProtocol:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.5, debug: bool = False, flush: bool = True):
        if serial is None:
            raise RuntimeError("pyserial is required: pip install pyserial")
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.debug = debug
        self.flush = flush

    def close(self):
        self.ser.close()

    def send_frame(self, cmd: int, payload: bytes = b""):
        length = 1 + len(payload)
        header = bytes([PREAMBLE_0, PREAMBLE_1, length, cmd])
        chk = checksum_xor(header[2:] + payload)
        frame = header + payload + bytes([chk])
        if self.debug:
            print(f"tx: {frame.hex()}")
        self.ser.write(frame)

    def read_frame(self, timeout: float = 1.0):
        deadline = time.time() + timeout
        state = 0
        length = 0
        cmd = 0
        payload = bytearray()
        chk = 0

        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            byte = b[0]
            if self.debug:
                print(f"rx: {byte:02x}")

            if state == 0:
                if byte == PREAMBLE_0:
                    state = 1
            elif state == 1:
                if byte == PREAMBLE_1:
                    state = 2
                else:
                    state = 0
            elif state == 2:
                length = byte
                chk = byte
                payload.clear()
                state = 3
            elif state == 3:
                cmd = byte
                chk ^= byte
                if length == 1:
                    state = 5
                else:
                    state = 4
            elif state == 4:
                payload.append(byte)
                chk ^= byte
                if len(payload) >= length - 1:
                    state = 5
            elif state == 5:
                if chk != byte:
                    raise RuntimeError("checksum mismatch")
                return cmd, bytes(payload)

        raise TimeoutError("no response")

    def transact(self, cmd: int, payload: bytes = b"", timeout: float = 1.0):
        if self.flush:
            self.ser.reset_input_buffer()
        self.send_frame(cmd, payload)
        rsp_cmd, rsp_payload = self.read_frame(timeout=timeout)
        if self.debug:
            print(f"rsp: cmd=0x{rsp_cmd:02x} payload={rsp_payload.hex()}")
        if rsp_cmd == RSP_ERROR:
            if len(rsp_payload) >= 2:
                raise RuntimeError(
                    f"device error: cmd=0x{rsp_payload[0]:02X} code={rsp_payload[1]}"
                )
            raise RuntimeError("device error")
        if rsp_cmd != (cmd | 0x80):
            raise RuntimeError(f"unexpected response cmd 0x{rsp_cmd:02X}")
        return rsp_payload

    def ping(self):
        return self.transact(CMD_PING)

    def get_version(self):
        data = self.transact(CMD_GET_VERSION)
        return data[0], data[1]

    def set_motor(self, index: int, speed: int):
        payload = struct.pack("<Bh", index, speed)
        self.transact(CMD_SET_MOTOR, payload)

    def get_motor(self, index: int) -> int:
        payload = struct.pack("<B", index)
        data = self.transact(CMD_GET_MOTOR, payload)
        return struct.unpack("<h", data[1:3])[0]

    def set_servo(self, index: int, pulse_us: int):
        payload = struct.pack("<BH", index, pulse_us)
        self.transact(CMD_SET_SERVO, payload)

    def get_servo(self, index: int) -> int:
        payload = struct.pack("<B", index)
        data = self.transact(CMD_GET_SERVO, payload)
        return struct.unpack("<H", data[1:3])[0]

    def set_gpio(self, mask: int, value: int):
        payload = struct.pack("<II", mask, value)
        self.transact(CMD_SET_GPIO, payload)

    def get_gpio(self):
        data = self.transact(CMD_GET_GPIO)
        return struct.unpack("<II", data[0:8])

    def estop(self):
        self.transact(CMD_ESTOP)

    def get_status(self):
        data = self.transact(CMD_GET_STATUS)
        uptime_ms = struct.unpack("<I", data[0:4])[0]
        last_error = data[4]
        return uptime_ms, last_error

    def set_neopixel(self, enabled: int, brightness: int, g: int, r: int, b: int):
        payload = bytes([enabled & 0x01, brightness & 0xFF, g & 0xFF, r & 0xFF, b & 0xFF])
        self.transact(CMD_SET_NEOPIXEL, payload)

    def get_neopixel(self):
        data = self.transact(CMD_GET_NEOPIXEL)
        return tuple(data[0:5])

    def set_strip(self, index: int, g: int, r: int, b: int):
        payload = struct.pack("<HBBB", index & 0xFFFF, g & 0xFF, r & 0xFF, b & 0xFF)
        self.transact(CMD_SET_STRIP, payload)

    def set_strip_bri(self, index: int, g: int, r: int, b: int, brightness: int):
        payload = struct.pack("<HBBBB", index & 0xFFFF, g & 0xFF, r & 0xFF, b & 0xFF, brightness & 0xFF)
        self.transact(CMD_SET_STRIP_BRI, payload)

    def set_strip_bulk(self, start: int, colors):
        if len(colors) > 80:
            raise ValueError("bulk strip write supports up to 80 LEDs per frame")
        payload = bytearray()
        payload += struct.pack("<HB", start & 0xFFFF, len(colors) & 0xFF)
        for g, r, b in colors:
            payload += bytes([g & 0xFF, r & 0xFF, b & 0xFF])
        self.transact(CMD_SET_STRIP_BULK, bytes(payload))

    def set_strip_interp(self, color_step: int, brightness_step: int):
        payload = bytes([color_step & 0xFF, brightness_step & 0xFF])
        self.transact(CMD_SET_STRIP_INTERP, payload)

    def get_adc(self):
        data = self.transact(CMD_GET_ADC)
        samples = [struct.unpack_from("<H", data, 2 * i)[0] for i in range(8)]
        update_mask = data[16]
        last_channel = data[17]
        return samples, update_mask, last_channel

    def set_adc_cfg(self, enable: int, channel_mask: int, interval_ticks: int):
        payload = struct.pack("<BBI", enable & 0x01, channel_mask & 0xFF, interval_ticks & 0xFFFFFFFF)
        self.transact(CMD_SET_ADC_CFG, payload)

    def clear_adc_update(self, mask: int):
        payload = struct.pack("<B", mask & 0xFF)
        self.transact(CMD_CLR_ADC_UPD, payload)

    def get_estop(self):
        data = self.transact(CMD_GET_ESTOP)
        return data[0], data[1], data[2]

    def get_as5600(self):
        data = self.transact(CMD_GET_AS5600)
        present = data[0]
        ok = data[1]
        status = data[2]
        angle = struct.unpack("<H", data[3:5])[0]
        magnitude = struct.unpack("<H", data[5:7])[0]
        return present, ok, status, angle, magnitude


def main():
    parser = argparse.ArgumentParser(description="LiteX UART robotics protocol client")
    parser.add_argument("--port", default="/dev/ttyUSB1")
    parser.add_argument("--baud", default=115200, type=int)
    parser.add_argument("--debug", action="store_true", help="Dump raw UART bytes.")
    parser.add_argument("--no-flush", action="store_true", help="Do not flush RX before each command.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")
    sub.add_parser("version")
    mset = sub.add_parser("motor-set")
    mset.add_argument("index", type=int)
    mset.add_argument("speed", type=int)
    mget = sub.add_parser("motor-get")
    mget.add_argument("index", type=int)
    sset = sub.add_parser("servo-set")
    sset.add_argument("index", type=int)
    sset.add_argument("pulse_us", type=int)
    sget = sub.add_parser("servo-get")
    sget.add_argument("index", type=int)
    gset = sub.add_parser("gpio-set")
    gset.add_argument("mask", type=lambda x: int(x, 0))
    gset.add_argument("value", type=lambda x: int(x, 0))
    sub.add_parser("gpio-get")
    sub.add_parser("estop")
    sub.add_parser("status")
    nset = sub.add_parser("neo-set")
    nset.add_argument("enabled", type=int)
    nset.add_argument("brightness", type=int)
    nset.add_argument("g", type=int)
    nset.add_argument("r", type=int)
    nset.add_argument("b", type=int)
    sub.add_parser("neo-get")
    sstrip = sub.add_parser("strip-set")
    sstrip.add_argument("index", type=int)
    sstrip.add_argument("g", type=int)
    sstrip.add_argument("r", type=int)
    sstrip.add_argument("b", type=int)
    sstripb = sub.add_parser("strip-set-bri")
    sstripb.add_argument("index", type=int)
    sstripb.add_argument("g", type=int)
    sstripb.add_argument("r", type=int)
    sstripb.add_argument("b", type=int)
    sstripb.add_argument("brightness", type=int)
    sbulk = sub.add_parser("strip-bulk")
    sbulk.add_argument("start", type=int)
    sbulk.add_argument("colors", nargs="+", help="GRB triplets like 0,255,0")
    sinterp = sub.add_parser("strip-interp")
    sinterp.add_argument("color_step", type=int, help="Per-tick color step (0 disables).")
    sinterp.add_argument("brightness_step", type=int, help="Per-tick brightness step (0 disables).")
    sub.add_parser("adc-get")
    acfg = sub.add_parser("adc-cfg")
    acfg.add_argument("enable", type=int)
    acfg.add_argument("channel_mask", type=lambda x: int(x, 0))
    acfg.add_argument("interval_ticks", type=lambda x: int(x, 0))
    aclear = sub.add_parser("adc-clear")
    aclear.add_argument("mask", type=lambda x: int(x, 0))
    sub.add_parser("estop-get")
    sub.add_parser("as5600-get")

    args = parser.parse_args()

    client = UartProtocol(args.port, args.baud, debug=args.debug, flush=not args.no_flush)
    try:
        if args.cmd == "ping":
            print(client.ping().decode("ascii", errors="replace"))
        elif args.cmd == "version":
            major, minor = client.get_version()
            print(f"{major}.{minor}")
        elif args.cmd == "motor-set":
            client.set_motor(args.index, args.speed)
            print("ok")
        elif args.cmd == "motor-get":
            print(client.get_motor(args.index))
        elif args.cmd == "servo-set":
            client.set_servo(args.index, args.pulse_us)
            print("ok")
        elif args.cmd == "servo-get":
            print(client.get_servo(args.index))
        elif args.cmd == "gpio-set":
            client.set_gpio(args.mask, args.value)
            print("ok")
        elif args.cmd == "gpio-get":
            mask, value = client.get_gpio()
            print(f"mask=0x{mask:08x} value=0x{value:08x}")
        elif args.cmd == "estop":
            client.estop()
            print("ok")
        elif args.cmd == "status":
            uptime_ms, last_error = client.get_status()
            print(f"uptime_ms={uptime_ms} last_error={last_error}")
        elif args.cmd == "neo-set":
            client.set_neopixel(args.enabled, args.brightness, args.g, args.r, args.b)
            print("ok")
        elif args.cmd == "neo-get":
            en, bri, g, r, b = client.get_neopixel()
            print(f"en={en} brightness={bri} g={g} r={r} b={b}")
        elif args.cmd == "strip-set":
            client.set_strip(args.index, args.g, args.r, args.b)
            print("ok")
        elif args.cmd == "strip-set-bri":
            client.set_strip_bri(args.index, args.g, args.r, args.b, args.brightness)
            print("ok")
        elif args.cmd == "strip-bulk":
            colors = []
            for item in args.colors:
                parts = item.split(",")
                if len(parts) != 3:
                    raise ValueError("colors must be GRB triplets like 0,255,0")
                g, r, b = (int(p, 0) for p in parts)
                colors.append((g, r, b))
            client.set_strip_bulk(args.start, colors)
            print("ok")
        elif args.cmd == "strip-interp":
            client.set_strip_interp(args.color_step, args.brightness_step)
            print("ok")
        elif args.cmd == "adc-get":
            samples, update_mask, last_channel = client.get_adc()
            print("samples_mv=" + ",".join(str(x) for x in samples))
            print(f"update_mask=0x{update_mask:02x} last_channel={last_channel}")
        elif args.cmd == "adc-cfg":
            client.set_adc_cfg(args.enable, args.channel_mask, args.interval_ticks)
            print("ok")
        elif args.cmd == "adc-clear":
            client.clear_adc_update(args.mask)
            print("ok")
        elif args.cmd == "estop-get":
            active, debounced, raw = client.get_estop()
            print(f"active={active} debounced={debounced} raw={raw}")
        elif args.cmd == "as5600-get":
            present, ok, status, angle, magnitude = client.get_as5600()
            md = 1 if (status & 0x20) else 0
            ml = 1 if (status & 0x10) else 0
            mh = 1 if (status & 0x08) else 0
            print(
                f"present={present} ok={ok} status=0x{status:02x} "
                f"md={md} ml={ml} mh={mh} angle={angle} magnitude={magnitude}"
            )
    finally:
        client.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
