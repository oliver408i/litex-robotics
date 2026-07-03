#!/usr/bin/env python3
"""Stream an image to the ESP32-C3 SPIBone flash loader over USB-CDC.

The C3 (software/c3_flash_esp) erases the target region, programs it page by
page over SPIBone into the FPGA (software/c3_flash), and CRC-verifies the
read-back against the CRC computed here. Uses the same standard CRC-32 as the
FPGA's libbase crc32 (== zlib.crc32).

Usage:
    flash_c3.py <port> <offset> <file>
    e.g.  flash_c3.py /dev/ttyACM0 0xF00000 test.bin

Close any serial monitor on the port first (it is exclusive). Offset must be
4 KB-aligned (erase granularity). Prints the status + read-back CRC.
"""
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:
    sys.exit("pyserial required: pip install pyserial")

STATUS = {
    0x00: "OK",
    0x05: "CRC MISMATCH",
    0xE0: "header timeout",
    0xE1: "erase failed",
    0xE2: "rx timeout (data underrun)",
    0xE3: "program failed",
    0xE4: "crc-read failed",
}


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    port, off_s, path = sys.argv[1], sys.argv[2], sys.argv[3]
    off = int(off_s, 0)
    data = open(path, "rb").read()
    if not data:
        sys.exit("empty file")
    if off % 0x1000:
        sys.exit(f"offset 0x{off:X} not 4 KB-aligned")

    crc = zlib.crc32(data) & 0xFFFFFFFF   # standard CRC-32 == FPGA libbase crc32

    ser = serial.Serial(port, 115200, timeout=15)
    time.sleep(2.5)                       # opening resets the C3 (native USB-CDC); let it reboot
    ser.reset_input_buffer()              # discard the C3 boot banner / prompts

    t0 = time.time()
    ser.write(b"W" + struct.pack("<III", off, len(data), crc))
    ser.flush()

    # Ack after ERASE (may take seconds for large images).
    ea = ser.read(1)
    if ea != b"\x01":
        code = ea[0] if ea else -1
        sys.exit(f"erase ack failed: 0x{code:02X} ({STATUS.get(code, 'no reply')})")

    # Stream pages, paced by a per-page ack (prevents C3 RX overflow).
    npages = (len(data) + 255) // 256
    for i in range(npages):
        page = data[i * 256:(i + 1) * 256]
        ser.write(page)
        ser.flush()
        a = ser.read(1)
        if a != b"\x01":
            code = a[0] if a else -1
            sys.exit(f"page {i}/{npages} ack failed: 0x{code:02X} ({STATUS.get(code, 'no reply')})")

    reply = ser.read(5)                   # final status + read-back crc
    dt = time.time() - t0
    if len(reply) != 5:
        sys.exit(f"no/short final reply ({len(reply)} bytes) -- is the FPGA loader running?")
    status = reply[0]
    readback = struct.unpack("<I", reply[1:5])[0]
    kbps = len(data) / 1024 / dt if dt else 0
    print(f"offset=0x{off:X}  len={len(data)}  host_crc=0x{crc:08X}  "
          f"readback=0x{readback:08X}  {dt:.2f}s ({kbps:.0f} KB/s)")
    print(f"-> status 0x{status:02X} ({STATUS.get(status, 'unknown')})")
    sys.exit(0 if status == 0x00 else 2)


if __name__ == "__main__":
    main()
