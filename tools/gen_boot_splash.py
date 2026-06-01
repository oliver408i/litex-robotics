#!/usr/bin/env python3
"""Generate the SPI-flash boot-splash blob for the LCD hardware boot sequencer.

The blob is what gateware/lcd_engine.py's boot sequencer reads straight from
flash (no CPU involved) to bring the panel up and paint a splash before the
firmware has even started. Layout (see gateware/st7796_boot.py):

    [ ST7796S init payload bytes | pad ]  <- INIT_REGION_BYTES
    [ 320x480 RGB565 image, row-major  ]  <- IMAGE_BYTES

Pixel byte order matches what the firmware already streams to the panel
(LVGL LV_COLOR_FORMAT_RGB565_SWAPPED -> big-endian RGB565 in memory, MSB-first
on the wire), so the same DMA path renders it correctly.

This is a *placeholder* generator: it renders "Booting <name>..." on a dark
background. Swap in real art later by replacing render_image() or pointing
--image at a PNG.

Flash it with:
    python icepi_zero_mnist_lcd.py --boot-splash --flash-splash build/boot_splash.bin
"""
import argparse
import os
import struct
import sys

from PIL import Image, ImageDraw, ImageFont

# Import the shared layout/recipe from the gateware package.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gateware.st7796_boot import (  # noqa: E402
    LCD_WIDTH, LCD_HEIGHT, INIT_REGION_BYTES, IMAGE_BYTES, init_data_blob,
)


def rgb565_swapped(r, g, b):
    """Pack 8-bit RGB into RGB565, big-endian (high byte first in memory).

    Matches the firmware's LV_COLOR_FORMAT_RGB565_SWAPPED framebuffer layout,
    so the existing DMA -> panel path renders these bytes with correct color.
    """
    v = ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3)
    return struct.pack(">H", v)


def render_image(text, bg=(0x10, 0x14, 0x20), fg=(0xE0, 0xE8, 0xF0)):
    """Render the placeholder splash to a (LCD_WIDTH x LCD_HEIGHT) PIL image."""
    img = Image.new("RGB", (LCD_WIDTH, LCD_HEIGHT), bg)
    draw = ImageDraw.Draw(img)

    # Try a real TrueType font for a crisp look; fall back to PIL's bitmap
    # font (always present) so the tool never hard-fails on a bare system.
    font = None
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
    ):
        if os.path.exists(path):
            font = ImageFont.truetype(path, 22)
            break
    if font is None:
        font = ImageFont.load_default()

    # Word-wrap the text to the panel width and center the block.
    words = text.split()
    lines, cur = [], ""
    for w in words:
        trial = (cur + " " + w).strip()
        if draw.textlength(trial, font=font) <= LCD_WIDTH - 24:
            cur = trial
        else:
            if cur:
                lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)

    bboxes = [draw.textbbox((0, 0), ln, font=font) for ln in lines]
    line_h = max((b[3] - b[1]) for b in bboxes) + 8
    total_h = line_h * len(lines)
    y = (LCD_HEIGHT - total_h) // 2
    for ln, b in zip(lines, bboxes):
        w = b[2] - b[0]
        draw.text(((LCD_WIDTH - w) // 2, y), ln, fill=fg, font=font)
        y += line_h

    # A thin accent rule under the text, just so it reads as a "boot screen".
    draw.rectangle(
        [40, y + 12, LCD_WIDTH - 40, y + 16],
        fill=(0x3a, 0x7a, 0xd0),
    )
    return img


def image_to_rgb565(img):
    """Flatten a PIL RGB image to row-major big-endian RGB565 bytes."""
    out = bytearray()
    px = img.load()
    for y in range(LCD_HEIGHT):
        for x in range(LCD_WIDTH):
            r, g, b = px[x, y][:3]
            out += rgb565_swapped(r, g, b)
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="IcePi Zero MNIST",
                    help="SoC name shown as 'Booting <name>...'.")
    ap.add_argument("--image", default=None,
                    help="Optional PNG to use instead of the generated placeholder "
                         "(scaled/cropped to 320x480).")
    ap.add_argument("--out", default="build/boot_splash.bin",
                    help="Output blob path (default: build/boot_splash.bin).")
    ap.add_argument("--preview", default=None,
                    help="Optional PNG dump of the rendered splash for eyeballing.")
    args = ap.parse_args()

    if args.image:
        img = Image.open(args.image).convert("RGB").resize((LCD_WIDTH, LCD_HEIGHT))
    else:
        img = render_image("Booting %s..." % args.name)

    if args.preview:
        img.save(args.preview)
        print("wrote preview %s" % args.preview)

    init = init_data_blob()
    if len(init) > INIT_REGION_BYTES:
        raise SystemExit("init payload %d B exceeds INIT_REGION_BYTES %d B"
                         % (len(init), INIT_REGION_BYTES))
    init_region = init + b"\xff" * (INIT_REGION_BYTES - len(init))

    image = image_to_rgb565(img)
    assert len(image) == IMAGE_BYTES, (len(image), IMAGE_BYTES)

    blob = init_region + image
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "wb") as f:
        f.write(blob)

    print("wrote %s" % args.out)
    print("  init payload : %d B (region %d B)" % (len(init), INIT_REGION_BYTES))
    print("  image        : %d B (%dx%d RGB565)" % (len(image), LCD_WIDTH, LCD_HEIGHT))
    print("  total        : %d B" % len(blob))


if __name__ == "__main__":
    main()
