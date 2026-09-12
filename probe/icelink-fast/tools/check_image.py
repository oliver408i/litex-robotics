#!/usr/bin/env python3
"""Sanity-check an interface image before it goes anywhere near the board.

The DAPLink bootloader validates the vector table of whatever you drop on it
(validate_bin_nvic): stack pointer inside RAM, reset vector inside the
interface region. Catching a bad table here costs nothing; catching it after
flashing costs IC grabbers.
"""
import struct
import sys

IF_BASE, IF_SIZE = 0x0800C000, 80 * 1024
RAM_BASE, RAM_SIZE = 0x20000000, 20 * 1024

BUILD_KEY_IF, HIC_ID = 0x9B939E8F, 0x97969908

def main(path):
    img = open(path, "rb").read()
    sp, pc = struct.unpack("<II", img[:8])
    errs = []

    # daplink_info is retained but no longer required: the stm32duino DFU
    # bootloader validates only the vector table.
    key, hic = struct.unpack("<II", img[0x20:0x28])

    if len(img) > IF_SIZE:
        errs.append(f"image {len(img)} B exceeds the {IF_SIZE} B interface slot")
    if not (RAM_BASE < sp <= RAM_BASE + RAM_SIZE):
        errs.append(f"SP 0x{sp:08x} outside RAM")
    if not (IF_BASE <= (pc & ~1) < IF_BASE + IF_SIZE):
        errs.append(f"reset vector 0x{pc:08x} outside the interface region")
    if not (pc & 1):
        errs.append(f"reset vector 0x{pc:08x} has no Thumb bit")

    print(f"  size {len(img):6d} B   free {IF_SIZE - len(img):6d} B"
          f"   SP 0x{sp:08x}   PC 0x{pc:08x}")
    for e in errs:
        print(f"  FAIL: {e}")
    return 1 if errs else 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
