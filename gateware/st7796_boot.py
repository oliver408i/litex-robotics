"""Shared ST7796S bring-up recipe for the hardware boot splash.

This is the single source of truth for the panel init sequence, mirrored
between two consumers:

  - gateware/lcd_engine.py   builds the hardware boot sequencer's command
                             table (cmd opcode, payload length, payload
                             offset, post-delay) from INIT_SEQUENCE.
  - tools/gen_boot_splash.py concatenates the per-command payload bytes into
                             the flash blob the sequencer DMA-fetches from.

Keeping it in one pure-Python module (no migen import) means the gateware and
the asset tool can never drift out of sync on the recipe.

The sequence is the exact recipe firmware runs in software/common/lcd.c
(`lcd_init`), minus the hardware-reset pulse (the sequencer drives reset_n
directly) -- so a panel brought up by hardware is bit-identical to one brought
up by firmware.

Each entry is (cmd_byte, (payload bytes...), delay_ms_after):
  - delay_ms_after mirrors the busy_wait()s in lcd_init (120 ms after the
    software reset and after sleep-out; 0 elsewhere).
"""

# Panel geometry (matches the LCD_WIDTH / LCD_HEIGHT SoC constants).
LCD_WIDTH  = 320
LCD_HEIGHT = 480

# Internal layout of the flash splash blob: the init payload bytes live at
# the start, padded up to INIT_REGION_BYTES, and the RGB565 image follows.
# Both the gateware (image base = blob base + INIT_REGION_BYTES) and the asset
# tool (pads the init region to this size) reference it, so they stay aligned.
# 4 KiB is wildly more than the ~42 init bytes need -- it's just a clean,
# erase-sector-friendly boundary with headroom if the recipe ever grows.
INIT_REGION_BYTES = 0x1000

# Bytes per pixel on the wire (COLMOD = 0x55 -> RGB565).
BYTES_PER_PIXEL = 2

# Full-frame image size in bytes.
IMAGE_BYTES = LCD_WIDTH * LCD_HEIGHT * BYTES_PER_PIXEL

INIT_SEQUENCE = [
    (0x01, (),                                                         120),  # software reset
    (0xf0, (0xc3,),                                                      0),  # command-set unlock 1
    (0xf0, (0x96,),                                                      0),  # command-set unlock 2
    (0x36, (0x48,),                                                      0),  # MADCTL
    (0x3a, (0x55,),                                                      0),  # COLMOD = 16bpp
    (0xb4, (0x01,),                                                      0),  # inversion control
    (0xb7, (0xc6,),                                                      0),  # entry mode
    (0xc0, (0x80, 0x45),                                                 0),  # power control 1
    (0xc1, (0x13,),                                                      0),  # power control 2
    (0xc2, (0xa7,),                                                      0),  # power control 3
    (0xc5, (0x0a,),                                                      0),  # VCOM control
    (0xb6, (0x80,),                                                      0),  # display function control
    (0xe0, (0xf0, 0x09, 0x0b, 0x06, 0x04, 0x15, 0x2f,
            0x54, 0x42, 0x3c, 0x17, 0x14, 0x18, 0x1b),                   0),  # positive gamma
    (0xe1, (0xe0, 0x09, 0x0b, 0x06, 0x04, 0x03, 0x2b,
            0x43, 0x42, 0x3b, 0x16, 0x14, 0x17, 0x1b),                   0),  # negative gamma
    (0xf0, (0x3c,),                                                      0),  # command-set lock 1
    (0xf0, (0x69,),                                                      0),  # command-set lock 2
    (0x11, (),                                                         120),  # sleep out
    (0x29, (),                                                           0),  # display on
]


def init_data_blob():
    """All command payload bytes concatenated, in INIT_SEQUENCE order.

    This is what gets written to the init region of the flash splash blob;
    the sequencer's table stores each command's byte offset into it.
    """
    return bytes(b for (_cmd, data, _delay) in INIT_SEQUENCE for b in data)


def init_table():
    """Per-command table the gateware sequencer iterates.

    Returns a list of (cmd_byte, payload_len, payload_offset, delay_ms),
    where payload_offset indexes into init_data_blob().
    """
    table = []
    offset = 0
    for cmd, data, delay in INIT_SEQUENCE:
        table.append((cmd, len(data), offset, delay))
        offset += len(data)
    return table
