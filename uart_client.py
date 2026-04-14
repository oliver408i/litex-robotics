#!/usr/bin/env python3
import struct
import time

try:
    import serial
except ImportError:
    serial = None

PREAMBLE_0 = 0xAA
PREAMBLE_1 = 0x55

CMD_PING = 0x01
CMD_GET_VERSION = 0x02
CMD_SET_NEOPIXEL = 0x30
CMD_GET_NEOPIXEL = 0x31
CMD_SET_STRIP_BULK = 0x34

RSP_ERROR = 0x7F
MAX_BULK_LEDS = 80


def checksum_xor(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


class UartProtocol:
    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.5, debug: bool = False):
        if serial is None:
            raise RuntimeError("pyserial is required: pip install pyserial")
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.debug = debug

    def close(self):
        self.ser.close()

    def send_frame(self, cmd: int, payload: bytes = b""):
        header = bytes([PREAMBLE_0, PREAMBLE_1, 1 + len(payload), cmd])
        frame = header + payload + bytes([checksum_xor(header[2:] + payload)])
        if self.debug:
            print(f"tx {frame.hex()}")
        self.ser.write(frame)

    def read_frame(self, timeout: float = 1.0):
        deadline = time.time() + timeout
        state = 0
        length = 0
        cmd = 0
        payload = bytearray()
        chk = 0

        while time.time() < deadline:
            byte = self.ser.read(1)
            if not byte:
                continue
            value = byte[0]
            if self.debug:
                print(f"rx {value:02x}")

            if state == 0:
                state = 1 if value == PREAMBLE_0 else 0
            elif state == 1:
                state = 2 if value == PREAMBLE_1 else 0
            elif state == 2:
                length = value
                chk = value
                payload.clear()
                state = 3
            elif state == 3:
                cmd = value
                chk ^= value
                state = 5 if length == 1 else 4
            elif state == 4:
                payload.append(value)
                chk ^= value
                if len(payload) >= length - 1:
                    state = 5
            else:
                if chk != value:
                    raise RuntimeError("checksum mismatch")
                return cmd, bytes(payload)

        raise TimeoutError("no response")

    def transact(self, cmd: int, payload: bytes = b"", timeout: float = 1.0):
        self.ser.reset_input_buffer()
        self.send_frame(cmd, payload)
        rsp_cmd, rsp_payload = self.read_frame(timeout=timeout)
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
        payload = self.transact(CMD_GET_VERSION)
        return tuple(payload[:2])

    def set_neopixel(self, enabled: int, brightness: int, g: int, r: int, b: int):
        payload = bytes([enabled & 0x01, brightness & 0xFF, g & 0xFF, r & 0xFF, b & 0xFF])
        self.transact(CMD_SET_NEOPIXEL, payload)

    def get_neopixel(self):
        payload = self.transact(CMD_GET_NEOPIXEL)
        return tuple(payload[:5])

    def set_strip_bulk(self, start: int, colors):
        if not 0 < len(colors) <= MAX_BULK_LEDS:
            raise ValueError(f"bulk write supports 1..{MAX_BULK_LEDS} LEDs")
        payload = bytearray(struct.pack("<HB", start & 0xFFFF, len(colors) & 0xFF))
        for g, r, b in colors:
            payload.extend((g & 0xFF, r & 0xFF, b & 0xFF))
        self.transact(CMD_SET_STRIP_BULK, bytes(payload))
