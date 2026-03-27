#!/usr/bin/env python3
import argparse
import sys
import time

from uart_client import UartProtocol


def drain_bt(client: UartProtocol, settle_s: float = 0.05, timeout_s: float = 0.5) -> bytes:
    data = bytearray()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        valid, value, overrun = client.bt_read()
        if overrun:
            raise RuntimeError("HC-05 RX overrun reported by FPGA")
        if valid:
            data.append(value)
            deadline = time.time() + settle_s
        else:
            time.sleep(0.01)
    return bytes(data)


def write_bt(client: UartProtocol, payload: bytes, byte_delay_s: float = 0.01):
    for value in payload:
        deadline = time.time() + 1.0
        while time.time() < deadline:
            if client.bt_write(value):
                break
            time.sleep(0.005)
        else:
            raise TimeoutError(f"timed out waiting to send byte 0x{value:02x}")
        if byte_delay_s > 0:
            time.sleep(byte_delay_s)


def at_command(client: UartProtocol, command: str, line_ending: str, response_timeout_s: float) -> str:
    drain_bt(client, timeout_s=0.1)
    write_bt(client, command.encode("ascii") + line_ending.encode("ascii"))
    response = drain_bt(client, timeout_s=response_timeout_s)
    return response.decode("ascii", errors="replace")


def encode_uart_params(baud: int, stop_bits: int, parity: str) -> str:
    parity_map = {"N": 0, "O": 1, "E": 2}
    if stop_bits not in (1, 2):
        raise ValueError("stop_bits must be 1 or 2")
    if parity not in parity_map:
        raise ValueError("parity must be N, O, or E")
    return f"AT+UART={baud},{stop_bits},{parity_map[parity]}"


def run_info(client: UartProtocol, line_ending: str, response_timeout_s: float):
    commands = [
        "AT",
        "AT+VERSION?",
        "AT+ADDR?",
        "AT+NAME?",
        "AT+ROLE?",
        "AT+UART?",
        "AT+PSWD?",
    ]
    for command in commands:
        print(f"> {command}")
        print(at_command(client, command, line_ending, response_timeout_s).strip() or "<no response>")


def main():
    parser = argparse.ArgumentParser(description="HC-05 AT-mode helper over the IcePi host UART link")
    parser.add_argument("--port", default="/dev/ttyUSB1", help="Host UART port used for the LiteX protocol.")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Host UART baud rate.")
    parser.add_argument("--line-ending", default="\\r\\n",
                        choices=["\\r", "\\n", "\\r\\n"],
                        help="Line ending to append to AT commands.")
    parser.add_argument("--response-timeout", type=float, default=0.75,
                        help="How long to wait for HC-05 response bytes.")
    parser.add_argument("--enter-at", action="store_true",
                        help="Drive bt_en high before running the command sequence.")
    parser.add_argument("--exit-at", action="store_true",
                        help="Drive bt_en low before exiting.")
    parser.add_argument("--power-cycle-note", action="store_true",
                        help="Print a reminder that many HC-05 boards need power-cycling after bt_en changes.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("info", help="Query common HC-05 settings.")

    cmd = sub.add_parser("cmd", help="Send a raw AT command.")
    cmd.add_argument("text", help="Command text without line ending, for example AT+NAME?")

    set_name = sub.add_parser("set-name", help="Set the HC-05 advertised name.")
    set_name.add_argument("name")

    set_pin = sub.add_parser("set-pin", help="Set the HC-05 pairing PIN/password.")
    set_pin.add_argument("pin")

    set_role = sub.add_parser("set-role", help="Set HC-05 role.")
    set_role.add_argument("role", type=int, choices=[0, 1, 2], help="0=slave, 1=master, 2=slave-loop")

    set_uart = sub.add_parser("set-uart", help="Set HC-05 UART format.")
    set_uart.add_argument("baud", type=int)
    set_uart.add_argument("--stop-bits", type=int, default=1, choices=[1, 2])
    set_uart.add_argument("--parity", default="N", choices=["N", "O", "E"])

    args = parser.parse_args()

    client = UartProtocol(args.port, args.baud)
    try:
        if args.enter_at:
            client.set_bt_en(1)
            print("bt_en=1")
            if args.power_cycle_note:
                print("power-cycle the HC-05 now if your board requires KEY/EN high at boot for AT mode")
                time.sleep(1.0)

        if args.cmd == "info":
            run_info(client, args.line_ending, args.response_timeout)
        elif args.cmd == "cmd":
            print(at_command(client, args.text, args.line_ending, args.response_timeout).strip() or "<no response>")
        elif args.cmd == "set-name":
            print(at_command(client, f"AT+NAME={args.name}", args.line_ending, args.response_timeout).strip() or "<no response>")
        elif args.cmd == "set-pin":
            print(at_command(client, f"AT+PSWD={args.pin}", args.line_ending, args.response_timeout).strip() or "<no response>")
        elif args.cmd == "set-role":
            print(at_command(client, f"AT+ROLE={args.role}", args.line_ending, args.response_timeout).strip() or "<no response>")
        elif args.cmd == "set-uart":
            command = encode_uart_params(args.baud, args.stop_bits, args.parity)
            print(at_command(client, command, args.line_ending, args.response_timeout).strip() or "<no response>")

        if args.exit_at:
            client.set_bt_en(0)
            print("bt_en=0")
    finally:
        client.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
