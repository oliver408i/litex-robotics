#!/usr/bin/env python3

import argparse
import atexit
import shlex
import sys
import time
from pathlib import Path

import serial

try:
    import readline
except ImportError:  # pragma: no cover
    readline = None


SOF0 = 0xAA
SOF1 = 0x55

CMD_PING = 0x01
CMD_SET_EN = 0x10
CMD_GET_EN = 0x11
CMD_UART_WRITE = 0x20
CMD_UART_LINE = 0x21
CMD_UART_READ = 0x22
CMD_UART_FLUSH = 0x23
CMD_UART_STATUS = 0x24
CMD_LR_RESET = 0x30
CMD_LR_STATUS = 0x31
CMD_LR_SPI = 0x32
CMD_LR_CS = 0x33
CMD_MCP_READ = 0x40

RSP_ERROR = 0x7F
HISTORY_FILE = Path.home() / ".hc05_tool_history"


def init_history():
    if readline is None:
        return
    try:
        readline.read_history_file(HISTORY_FILE)
    except FileNotFoundError:
        pass
    readline.set_history_length(200)
    atexit.register(save_history)


def save_history():
    if readline is None:
        return
    try:
        readline.write_history_file(HISTORY_FILE)
    except OSError:
        pass


def checksum_xor(length, cmd, payload):
    chk = length ^ cmd
    for b in payload:
        chk ^= b
    return chk & 0xFF


class ProtocolError(RuntimeError):
    pass


class HC05Tool:
    def __init__(self, port, baud, timeout):
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def close(self):
        self.ser.close()

    def _read_exact(self, n):
        data = bytearray()
        while len(data) < n:
            chunk = self.ser.read(n - len(data))
            if not chunk:
                raise TimeoutError("timed out waiting for FPGA response")
            data.extend(chunk)
        return bytes(data)

    def _read_frame(self):
        while True:
            b = self._read_exact(1)[0]
            if b != SOF0:
                continue
            if self._read_exact(1)[0] != SOF1:
                continue
            break

        length = self._read_exact(1)[0]
        cmd = self._read_exact(1)[0]
        payload = self._read_exact(length - 1) if length > 1 else b""
        chk = self._read_exact(1)[0]
        if chk != checksum_xor(length, cmd, payload):
            raise ProtocolError("bad checksum in FPGA response")
        return cmd, payload

    def transact(self, cmd, payload=b""):
        if len(payload) > 254:
            raise ValueError("payload too large")
        length = len(payload) + 1
        frame = bytes([SOF0, SOF1, length, cmd]) + payload + bytes([checksum_xor(length, cmd, payload)])
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        rsp_cmd, rsp_payload = self._read_frame()
        if rsp_cmd == RSP_ERROR:
            if len(rsp_payload) >= 2:
                raise ProtocolError(f"FPGA error for cmd 0x{rsp_payload[0]:02x}: code {rsp_payload[1]}")
            raise ProtocolError("FPGA returned malformed error")
        expected = cmd | 0x80
        if rsp_cmd != expected:
            raise ProtocolError(f"unexpected response 0x{rsp_cmd:02x}, expected 0x{expected:02x}")
        return rsp_payload

    def ping(self):
        return self.transact(CMD_PING).decode("ascii", errors="replace")

    def set_en(self, value):
        payload = self.transact(CMD_SET_EN, bytes([1 if value else 0]))
        return bool(payload[0]) if payload else bool(value)

    def get_en(self):
        payload = self.transact(CMD_GET_EN)
        return bool(payload[0]) if payload else False

    def write(self, data):
        self.transact(CMD_UART_WRITE, data)

    def writeline(self, text):
        self.transact(CMD_UART_LINE, text.encode("utf-8"))

    def read(self, count=64):
        payload = self.transact(CMD_UART_READ, bytes([count & 0xFF]))
        return payload

    def flush(self):
        self.transact(CMD_UART_FLUSH)

    def status(self):
        payload = self.transact(CMD_UART_STATUS)
        if len(payload) != 3:
            raise ProtocolError("malformed status payload")
        return {
            "en": bool(payload[0]),
            "rx_count": payload[1],
            "txfull": bool(payload[2]),
        }

    def lr_reset(self, value):
        payload = self.transact(CMD_LR_RESET, bytes([1 if value else 0]))
        return bool(payload[0]) if payload else bool(value)

    def lr_status(self):
        payload = self.transact(CMD_LR_STATUS)
        if len(payload) != 2:
            raise ProtocolError("malformed lr_status payload")
        return {
            "reset_n": bool(payload[0]),
            "busy": bool(payload[1] & 0x1),
            "dio9": bool(payload[1] & 0x2),
        }

    def lr_spi(self, data):
        return self.transact(CMD_LR_SPI, data)

    def lr_cs(self, asserted):
        payload = self.transact(CMD_LR_CS, bytes([1 if asserted else 0]))
        return bool(payload[0]) if payload else bool(asserted)

    def lr_wait_busy(self, busy, timeout=1.0, poll=0.01):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.lr_status()
            if last["busy"] == busy:
                return last
            time.sleep(poll)
        raise TimeoutError(f"LR1121 busy did not become {int(busy)} within {timeout:.2f}s; last={last}")

    def lr_wait_not_busy(self, timeout=1.0, poll=0.01):
        return self.lr_wait_busy(False, timeout=timeout, poll=poll)

    def lr_pulse_reset(self, low_time=0.05, settle_time=0.05, timeout=1.0):
        self.lr_reset(False)
        time.sleep(low_time)
        self.lr_reset(True)
        time.sleep(settle_time)
        return self.lr_wait_not_busy(timeout=timeout)

    def lr_exchange(self, data, pre_wait=True, post_wait=True, timeout=1.0):
        if pre_wait:
            self.lr_wait_not_busy(timeout=timeout)
        rsp = self.lr_spi(data)
        if post_wait:
            self.lr_wait_not_busy(timeout=timeout)
        return rsp

    def lr_get_status(self):
        return self.lr_exchange(bytes.fromhex("01 00"))

    def lr_get_version(self):
        return self.lr_exchange(bytes.fromhex("01 01 00 00 00 00"))

    def lr_reboot(self, stay_in_bootloader=False):
        return self.lr_exchange(bytes([0x01, 0x18, 0x01 if stay_in_bootloader else 0x00]), post_wait=False)

    def mcp_read(self, channel):
        if not 0 <= channel <= 7:
            raise ValueError("MCP3008 channel must be 0..7")
        payload = self.transact(CMD_MCP_READ, bytes([channel]))
        if len(payload) != 2:
            raise ProtocolError("malformed MCP3008 response")
        return (payload[0] << 8) | payload[1]


def parse_escaped_bytes(text):
    return text.encode("utf-8").decode("unicode_escape").encode("latin1")


def format_bytes(data):
    try:
        text = data.decode("utf-8")
        return text
    except UnicodeDecodeError:
        return " ".join(f"{b:02x}" for b in data)


def format_hex_bytes(data):
    return " ".join(f"{b:02x}" for b in data) if data else "(empty)"


def run_repl(tool):
    print("hc05_tool repl")
    print("Commands: ping, en [0|1], status, flush, write <text>, line <text>, read [n], at <cmd>, rawhex <hex bytes>, lrreset [0|1], lrpulse, lrwait [0|1], lrcs [0|1], lrstatus, lrspi <hex bytes>, lrgetstatus, lrversion, lrreboot [0|1], mcpread <0-7>, quit")
    while True:
        try:
            line = input("hc05> ")
        except EOFError:
            print()
            return 0
        if not line.strip():
            continue
        try:
            argv = shlex.split(line)
        except ValueError as exc:
            print(f"parse error: {exc}")
            continue

        cmd = argv[0].lower()
        try:
            if cmd in {"quit", "exit"}:
                return 0
            if cmd == "ping":
                print(tool.ping())
            elif cmd == "en":
                if len(argv) == 1:
                    print(int(tool.get_en()))
                else:
                    print(int(tool.set_en(int(argv[1]) != 0)))
            elif cmd == "status":
                print(tool.status())
            elif cmd == "flush":
                tool.flush()
                print("ok")
            elif cmd == "write":
                if len(argv) < 2:
                    print("usage: write <text>")
                    continue
                tool.write(" ".join(argv[1:]).encode("utf-8"))
                print("ok")
            elif cmd == "line":
                if len(argv) < 2:
                    print("usage: line <text>")
                    continue
                tool.writeline(" ".join(argv[1:]))
                print("ok")
            elif cmd == "at":
                if len(argv) < 2:
                    print("usage: at <command>")
                    continue
                tool.flush()
                tool.writeline(" ".join(argv[1:]))
                time.sleep(0.2)
                data = tool.read(128)
                print(format_bytes(data) if data else "(no response)")
            elif cmd == "read":
                count = int(argv[1], 0) if len(argv) > 1 else 64
                data = tool.read(count)
                print(format_bytes(data) if data else "(empty)")
            elif cmd == "rawhex":
                if len(argv) < 2:
                    print("usage: rawhex <hex bytes>")
                    continue
                data = bytes.fromhex(" ".join(argv[1:]))
                tool.write(data)
                print("ok")
            elif cmd == "lrreset":
                if len(argv) == 1:
                    print(int(tool.lr_status()["reset_n"]))
                else:
                    print(int(tool.lr_reset(int(argv[1]) != 0)))
            elif cmd == "lrpulse":
                print(tool.lr_pulse_reset())
            elif cmd == "lrwait":
                busy = int(argv[1], 0) != 0 if len(argv) > 1 else False
                print(tool.lr_wait_busy(busy))
            elif cmd == "lrstatus":
                print(tool.lr_status())
            elif cmd == "lrcs":
                if len(argv) == 1:
                    print("usage: lrcs <0|1>")
                    continue
                print(int(tool.lr_cs(int(argv[1]) != 0)))
            elif cmd == "lrspi":
                if len(argv) < 2:
                    print("usage: lrspi <hex bytes>")
                    continue
                data = bytes.fromhex(" ".join(argv[1:]))
                print(format_hex_bytes(tool.lr_spi(data)))
            elif cmd == "lrgetstatus":
                print(format_hex_bytes(tool.lr_get_status()))
            elif cmd == "lrversion":
                print(format_hex_bytes(tool.lr_get_version()))
            elif cmd == "lrreboot":
                stay = int(argv[1], 0) != 0 if len(argv) > 1 else False
                print(format_hex_bytes(tool.lr_reboot(stay)))
            elif cmd == "mcpread":
                if len(argv) != 2:
                    print("usage: mcpread <0-7>")
                    continue
                raw = tool.mcp_read(int(argv[1], 0))
                print(f"{raw} (0x{raw:03x})")
            else:
                print("unknown command")
        except (ProtocolError, TimeoutError, ValueError, serial.SerialException) as exc:
            print(f"error: {exc}")


def main():
    parser = argparse.ArgumentParser(description="CLI/REPL for the HC-05 FPGA debug protocol.")
    parser.add_argument("--port", required=True, help="Host serial port connected to the LiteX console UART.")
    parser.add_argument("--baud", type=int, default=115200, help="LiteX console UART baudrate.")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout in seconds.")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="Optional one-shot REPL command.")
    args = parser.parse_args()
    init_history()

    tool = HC05Tool(args.port, args.baud, args.timeout)
    try:
        if args.command:
            line = " ".join(args.command)
            try:
                argv = shlex.split(line)
            except ValueError as exc:
                print(f"parse error: {exc}", file=sys.stderr)
                return 2
            if not argv:
                return 0
            cmd = argv[0].lower()
            if cmd == "ping":
                print(tool.ping())
            elif cmd == "status":
                print(tool.status())
            elif cmd == "en":
                if len(argv) == 1:
                    print(int(tool.get_en()))
                else:
                    print(int(tool.set_en(int(argv[1]) != 0)))
            elif cmd == "flush":
                tool.flush()
                print("ok")
            elif cmd == "write":
                tool.write(" ".join(argv[1:]).encode("utf-8"))
                print("ok")
            elif cmd == "line":
                tool.writeline(" ".join(argv[1:]))
                print("ok")
            elif cmd == "read":
                count = int(argv[1], 0) if len(argv) > 1 else 64
                data = tool.read(count)
                print(format_bytes(data) if data else "(empty)")
            elif cmd == "at":
                tool.flush()
                tool.writeline(" ".join(argv[1:]))
                time.sleep(0.2)
                data = tool.read(128)
                print(format_bytes(data) if data else "(no response)")
            elif cmd == "rawhex":
                data = bytes.fromhex(" ".join(argv[1:]))
                tool.write(data)
                print("ok")
            elif cmd == "lrreset":
                if len(argv) == 1:
                    print(int(tool.lr_status()["reset_n"]))
                else:
                    print(int(tool.lr_reset(int(argv[1]) != 0)))
            elif cmd == "lrpulse":
                print(tool.lr_pulse_reset())
            elif cmd == "lrwait":
                busy = int(argv[1], 0) != 0 if len(argv) > 1 else False
                print(tool.lr_wait_busy(busy))
            elif cmd == "lrstatus":
                print(tool.lr_status())
            elif cmd == "lrcs":
                print(int(tool.lr_cs(int(argv[1]) != 0)))
            elif cmd == "lrspi":
                data = bytes.fromhex(" ".join(argv[1:]))
                print(format_hex_bytes(tool.lr_spi(data)))
            elif cmd == "lrgetstatus":
                print(format_hex_bytes(tool.lr_get_status()))
            elif cmd == "lrversion":
                print(format_hex_bytes(tool.lr_get_version()))
            elif cmd == "lrreboot":
                stay = int(argv[1], 0) != 0 if len(argv) > 1 else False
                print(format_hex_bytes(tool.lr_reboot(stay)))
            elif cmd == "mcpread":
                raw = tool.mcp_read(int(argv[1], 0))
                print(f"{raw} (0x{raw:03x})")
            else:
                print("unknown command", file=sys.stderr)
                return 2
            return 0
        return run_repl(tool)
    finally:
        tool.close()


if __name__ == "__main__":
    raise SystemExit(main())
