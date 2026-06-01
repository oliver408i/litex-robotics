#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <generated/csr.h>

#ifndef CSR_LCD_BASE
#error "lcd.h: build the SoC with --with-lcd"
#endif

/* ---- Op kinds + status / pad bitmasks ----------------------------------
 * Convenience wrappers below cover the common cases; expose the raw
 * bits so callers (e.g. an LVGL flush_cb that writes CSRs directly for
 * performance) can issue ops without rebuilding the constants. */

#define LCD_OP_START          (1u << CSR_LCD_OP_START_OFFSET)
#define LCD_OP_CMD            ((1u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_CMD_DATA_DMA   ((2u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_CMD_DATA_FILL  ((3u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_FILL_RECT      ((4u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_DMA_RECT       ((5u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)

#define LCD_BUSY              (1u << CSR_LCD_STATUS_BUSY_OFFSET)
#define LCD_CAN_ACCEPT        (1u << CSR_LCD_STATUS_CAN_ACCEPT_OFFSET)
#define LCD_BOOT_DONE         (1u << CSR_LCD_STATUS_BOOT_DONE_OFFSET)

#define LCD_PADS_RESET_N      (1u << CSR_LCD_PADS_CTRL_RESET_N_OFFSET)
#define LCD_PADS_BACKLIGHT    (1u << CSR_LCD_PADS_CTRL_BACKLIGHT_OFFSET)

/* ---- Pad control ------------------------------------------------------- */

void lcd_pads_apply(void);
void lcd_pads_set(uint32_t mask, int value);
void lcd_reset_n(int v);
void lcd_backlight(int v);

/* ---- Engine status ----------------------------------------------------- */

/* Block until the engine has no work (current + queue both empty). */
void lcd_wait_idle(void);
/* Block until the queue slot is free (fast path; engine may still be
 * shifting the previous op). Prefer this on hot paths. */
void lcd_wait_can_accept(void);

/* ---- Low-level op helpers --------------------------------------------- */

/* Send a single command byte (DC=0), CS framed by the engine. */
void lcd_write_cmd(uint8_t cmd);

/* Send a command byte followed by `len` data bytes (DC=1), all in one
 * CS frame. `data` may live in any wishbone-addressable memory (the
 * engine DMA-fetches it). len == 0 collapses to lcd_write_cmd(cmd). */
void lcd_cmd_data(uint8_t cmd, const uint8_t *data, unsigned int len);

/* Hardware reset pulse for the LCD (shared with FT6336U reset). */
void lcd_hw_reset(void);

/* Full ST7796S bring-up: hw reset, soft reset, init sequence, sleep
 * out, display on, backlight on. Run once at boot before any drawing. */
void lcd_init(void);

#ifdef LCD_BOOT_SPLASH
/* Hand the engine back from the HW boot-splash sequencer. Built only when the
 * SoC includes the boot splash (icepi_zero_mnist_lcd.py --boot-splash): the
 * gateware has already done lcd_init() + painted the splash from flash, so
 * firmware calls this *instead* of lcd_init(). Blocks until the sequencer
 * reports done, pre-loads the pad shadow (backlight on, out of reset) so there
 * is no flicker, then releases the engine to the CPU. */
void lcd_boot_takeover(void);
#endif

/* ---- High-level rect ops ---------------------------------------------- */

/* Queue a contiguous DMA RECT op: window = (x,y,w,h), pixels read from
 * `buf` (len bytes, RGB565 MSB-first on the wire). Returns immediately;
 * use lcd_wait_idle() / op-done IRQ if you need to know it's done. */
void lcd_dma_rect(int16_t x, int16_t y, int16_t w, int16_t h,
                  const void *buf, uint32_t len);

/* Queue a solid-color FILL RECT op: window = (x,y,w,h), each pixel set
 * to `color` (RGB565). */
void lcd_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
