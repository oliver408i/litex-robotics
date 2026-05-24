# Common firmware modules

Shared C modules under `software/common/` that any firmware target on
this board can `#include` and link against. Factored out of
`software/lcd_test` and `software/lvgl_demo` so the next firmware
target doesn't fork the same ~300 lines of LCD bring-up and HW I2C
driver again.

## What's there

| Header   | Purpose                                                     |
|----------|-------------------------------------------------------------|
| `lcd.h`  | LCD engine helpers: bring-up, low-level commands, rect ops. |
| `touch.h`| HW I2C master + FT6336U touch IC.                           |
| `log.h`  | UART logging conveniences (`log_puts`, `log_hex*`, etc.).   |

All three have small `.c` counterparts. None depend on each other
except `touch.c` uses `log.c`'s helpers in `ft6336u_probe()`.

## What each module covers

### `lcd.h`

Everything needed to bring the ST7796S up and push pixels at it.
Exposes both the raw op kind / status / pad bitmasks (so hot paths can
write the engine's CSRs directly) and the higher-level wrappers for the
common cases.

Typical use:

```c
#include "lcd.h"

lcd_pads_apply();          /* push initial pad state */
lcd_init();                /* hw reset, init sequence, backlight on */

lcd_fill_rect(x, y, w, h, 0x07e0);          /* solid green rect */
lcd_dma_rect(x, y, w, h, pixel_buf, len);   /* RGB565 buffer */
lcd_write_cmd(0x29);                         /* arbitrary cmd byte */
lcd_cmd_data(0x36, &madctl, 1);              /* cmd + data payload */
```

For hot paths that need to bypass the helpers (LVGL's `flush_cb`, a
custom blitter, anything that wants the engine's stride-aware DMA),
use `lcd_wait_can_accept()`, write the engine CSRs directly, and fire
`lcd_op_write(LCD_OP_*)`.

### `touch.h`

Hardware I2C driver targeted at the FT6336U, plus a low-level layer
you can use to talk to any other I2C slave on the same bus.

```c
#include "touch.h"

touch_init();              /* configure SCL divider; call once at boot */
ft6336u_probe();           /* log chip_id / vendor_id / fw_ver */

uint8_t buf[7];
if(ft6336u_read(0x02, buf, sizeof(buf))) {
    unsigned int n = buf[0] & 0x0f;
    /* ... */
}
```

The low-level `touch_i2c_*` primitives (addr, write_byte, read_byte,
stop, wait_idle) are exposed for any I2C peripheral that might share
the bus.

### `log.h`

Light wrappers over `libbase/uart.h`. Just call `uart_init()` once at
boot, then:

```c
#include "log.h"

log_puts("hello "); log_hex8(0xaa); log_nl();
log_puts("val="); log_uint(value); log_nl();
```

`log_nl()` writes `\r\n` and flushes the UART. CR insertion on
embedded `\n` is automatic.

## How to use them in a new firmware target

Easiest path: copy `software/lvgl_demo/Makefile` (or `lcd_test`'s) as a
template and keep these two lines:

```make
COMMON_DIR := ../common
CFLAGS    += -I$(COMMON_DIR)
COMMON_OBJS = lcd.o touch.o log.o
```

then add them into the object list:

```make
OBJECTS = main.o crt0.o $(COMMON_OBJS)
```

and into `VPATH` so make can find the `.c` files via the standard
`%.o: %.c` rule:

```make
VPATH = $(BIOS_DIRECTORY):$(CPU_DIRECTORY):$(COMMON_DIR)
```

If you only need a subset (e.g., a UART-only diagnostic firmware that
doesn't touch the LCD), drop the unneeded objects from `COMMON_OBJS`
and the headers from your sources. `--gc-sections` is enabled, so
unused module entries inside compiled common objects also disappear
from the final binary.

## What's intentionally not here

A few things live in the firmware targets themselves rather than in
common because they're application-specific:

- **LVGL display/indev/tick callbacks.** `lvgl_demo`'s `flush_cb`
  writes engine CSRs directly for performance; that wiring depends on
  LVGL data types we don't want to drag into a non-LVGL target.
- **Test pattern, animation loop, FPS measurement.** `lcd_test`-only.
- **`__ffssi2` and `bm_get_idle_percent`.** LVGL-only linker shims;
  belong with the LVGL build.

If a third target ends up duplicating something not currently in
common, that's the signal to factor it out.

## Build-time dependencies

Both `lcd.h` and `touch.h` include LiteX's `generated/csr.h` /
`generated/mem.h` and `#error` if the SoC wasn't built with
`--with-lcd`. That keeps the failure mode at compile time rather than
runtime.

`log.h` has no LCD/touch dependency and is safe to use from any
firmware target whose SoC has the default UART.
