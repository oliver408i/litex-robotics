#!/usr/bin/env python3
"""Stream an image to the ESP32-C3 SPIBone flash loader over USB-CDC.

The C3 (software/c3_flash_esp) erases the target region, programs it page by
page over SPIBone into the FPGA (software/c3_flash), and CRC-verifies the
read-back against the CRC computed here. Uses the same standard CRC-32 as the
FPGA's libbase crc32 (== zlib.crc32). Pages stream in WINDOW_PAGES-sized
windows, acked once per window rather than once per page.

Usage:
    flash_c3.py <port> <offset> <file> [--fbi]
    e.g.  flash_c3.py /dev/ttyACM0 0xF00000 test.bin
    e.g.  flash_c3.py /dev/ttyACM0 0x200000 software/c3_flash/c3_flash.bin --fbi

--fbi prepends the LiteX flashboot header (u32le length + u32le crc32) before
sending -- use it for the loader/app slots, which the BIOS's flashboot() reads
via that header. Not used for the bitstream/BIOS slots (those are raw).

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

WINDOW_PAGES = 16   # must match software/c3_flash_esp/src/main.cpp's WINDOW_PAGES

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
    argv = sys.argv[1:]
    fbi = "--fbi" in argv
    argv = [a for a in argv if a != "--fbi"]
    if len(argv) != 3:
        sys.exit(__doc__)
    port, off_s, path = argv
    off = int(off_s, 0)
    data = open(path, "rb").read()
    if not data:
        sys.exit("empty file")
    if fbi:
        # .fbi = LiteX flashboot image: u32le length + u32le crc32 + payload
        # (icepi_zero_base.py's run_build --flash-firmware path wraps the same way).
        payload_crc = zlib.crc32(data) & 0xFFFFFFFF
        data = len(data).to_bytes(4, "little") + payload_crc.to_bytes(4, "little") + data
    if off % 0x1000:
        sys.exit(f"offset 0x{off:X} not 4 KB-aligned")

    crc = zlib.crc32(data) & 0xFFFFFFFF   # standard CRC-32 == FPGA libbase crc32 (whole image, incl. .fbi header)

    ser = serial.Serial(port, 115200, timeout=15)
    time.sleep(2.5)                       # opening resets the C3 (native USB-CDC); let it reboot
    ser.reset_input_buffer()              # discard the C3 boot banner / prompts

    t0 = time.time()
    ser.write(b"W" + struct.pack("<III", off, len(data), crc))
    ser.flush()

    # Ack after ERASE (may take many seconds/minutes for a large image). Wait
    # proportionally to the erase size, matching the C3's size-scaled mailbox
    # poll (~0.5 s/sector), so a legitimate long erase isn't taken for a hang.
    old_to = ser.timeout
    ser.timeout = 10.0 + (len(data) / 4096) * 0.5
    ea = ser.read(1)
    ser.timeout = old_to
    if ea != b"\x01":
        code = ea[0] if ea else -1
        sys.exit(f"erase ack failed: 0x{code:02X} ({STATUS.get(code, 'no reply')})")

    # Stream pages in WINDOW_PAGES-sized windows, paced by one ack per window
    # (not per page -- the C3's RX buffer comfortably holds a window, see
    # RX_BUFFER_SIZE in main.cpp) instead of a full USB round-trip per 256 B.
    npages = (len(data) + 255) // 256
    i = 0
    while i < npages:
        window = min(WINDOW_PAGES, npages - i)
        for j in range(window):
            page = data[(i + j) * 256:(i + j + 1) * 256]
            ser.write(page)
        ser.flush()
        a = ser.read(1)
        if a != b"\x01":
            code = a[0] if a else -1
            sys.exit(f"window at page {i}/{npages} ack failed: 0x{code:02X} ({STATUS.get(code, 'no reply')})")
        i += window

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
