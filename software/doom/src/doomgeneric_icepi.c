/* doomgeneric platform layer for the IcePi Zero SoC.
 *
 * Video : DG_ScreenBuffer (320x200 XRGB8888) -> RGB565 -> LCD engine DMA blit.
 * Timing: RISC-V-independent free-running timer0 as a millisecond clock.
 * Input : FT6336U capacitive touch -> DOOM keys (stubbed for first bring-up).
 *
 * Built at 320x200 (native DOOM resolution, no scaling) via
 * -DDOOMGENERIC_RESX=320 -DDOOMGENERIC_RESY=200. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <system.h>
#include <irq.h>
#include <libbase/uart.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "doomkeys.h"
#include "doomgeneric.h"

#include "lcd.h"
#include "touch.h"
#include "log.h"

#define DW DOOMGENERIC_RESX
#define DH DOOMGENERIC_RESY

/* No RGB565 scratch buffer: the LCD engine converts XRGB8888 -> RGB565 as it
 * DMAs, so it reads DG_ScreenBuffer directly. That removes 128 KB of .bss and,
 * more to the point, a whole CPU pass over the frame -- see the `convert` note
 * on the frame profile below. Falls back to a firmware repack if the gateware
 * predates the engine's cvt field. */
#ifndef CSR_LCD_OP_CVT_OFFSET
#define DOOM_CPU_CONVERT 1
static uint16_t fb565[DW * DH];
#endif

/* ---- millisecond clock ------------------------------------------------- *
 * timer0 as a free-running down-counter (reload = 0xffffffff). We accumulate
 * elapsed sys-clock ticks into a 64-bit counter across reloads; unsigned
 * subtraction (last - now) yields correct per-poll deltas through one wrap, so
 * as long as we are polled more than once per 2^32 ticks (~57 s @ 75 MHz) the
 * clock never loses time. DOOM polls I_GetTime many times per frame.
 * Deliberately avoids busy_wait() (which reloads timer0) after init. */
static uint64_t s_acc_ticks;
static uint32_t s_last_val;
static int      s_timer_up;

static void timer_start(void)
{
    timer0_en_write(0);
    timer0_load_write(0xffffffffu);
    timer0_reload_write(0xffffffffu);
    timer0_en_write(1);
    timer0_update_value_write(1);
    s_last_val  = timer0_value_read();
    s_acc_ticks = 0;
    s_timer_up  = 1;
}

uint32_t DG_GetTicksMs(void)
{
    if (!s_timer_up)
        timer_start();
    timer0_update_value_write(1);
    uint32_t v = timer0_value_read();          /* counts down */
    s_acc_ticks += (uint32_t)(s_last_val - v);  /* correct across one reload */
    s_last_val = v;
    return (uint32_t)(s_acc_ticks / (CONFIG_CLOCK_FREQUENCY / 1000u));
}

/* Raw 64-bit sys-clock tick count. Same accumulator as DG_GetTicksMs, but
 * undivided: a frame is ~10-100 ms and we are splitting it three ways, so ms
 * granularity would quantize the phases into noise. */
static uint64_t ticks_now(void)
{
    if (!s_timer_up)
        timer_start();
    timer0_update_value_write(1);
    uint32_t v = timer0_value_read();
    s_acc_ticks += (uint32_t)(s_last_val - v);
    s_last_val = v;
    return s_acc_ticks;
}

/* ---- frame profile ----------------------------------------------------- *
 * Three phases, accumulated over a window and reported as per-frame averages:
 *
 *   render  - everything doomgeneric_Tick() does before DG_DrawFrame: game
 *             logic plus the software rasterizer. Pure CPU+memory, and the
 *             only phase a faster core can move much.
 *   convert - the XRGB8888 -> byte-swapped RGB565 repack. Also CPU, but a
 *             tight predictable loop, so it scales differently from render.
 *   flush   - lcd_dma_rect + lcd_wait_idle: the panel SPI link. Bound by SCK,
 *             not by the CPU, and on this hardware it dominates -- which is
 *             exactly why a single end-to-end frame time understates a CPU
 *             change (raising sys 50->90 MHz moved lcd_test by 0.5%).
 *
 * Reporting these separately is the point: `render` is the CPU-bound number to
 * judge a core swap on, `render+convert+flush` is what the player sees. */
#ifndef DOOM_PROFILE_FRAMES
#define DOOM_PROFILE_FRAMES 50u
#endif

static uint64_t prof_render, prof_convert, prof_flush;
static uint32_t prof_frames;

static void prof_report(void)
{
    /* Integer microseconds per frame; CONFIG_CLOCK_FREQUENCY/1000000 is the
     * ticks-per-us divisor. Printed as us to keep one decimal of ms. */
    uint32_t per_us = CONFIG_CLOCK_FREQUENCY / 1000000u;
    uint32_t n      = prof_frames;
    uint32_t r      = (uint32_t)(prof_render  / n / per_us);
    uint32_t c      = (uint32_t)(prof_convert / n / per_us);
    uint32_t f      = (uint32_t)(prof_flush   / n / per_us);
    uint32_t total  = r + c + f;
    /* mfps = frames per second * 1000, computed without floating point. */
    uint32_t mfps   = total ? (1000000u * 1000u) / total : 0;

    printf("doom: %lu frames  render %lu us  convert %lu us  flush %lu us  "
           "total %lu us  %lu.%03lu fps\n",
           (unsigned long)n, (unsigned long)r, (unsigned long)c,
           (unsigned long)f, (unsigned long)total,
           (unsigned long)(mfps / 1000u), (unsigned long)(mfps % 1000u));

    prof_render = prof_convert = prof_flush = 0;
    prof_frames = 0;
}

void DG_SleepMs(uint32_t ms)
{
    uint32_t t0 = DG_GetTicksMs();
    while ((DG_GetTicksMs() - t0) < ms) { /* spin */ }
}

/* ---- touch input + on-screen control pad ------------------------------ *
 * DOOM fills only rows 0..199; the bottom of the 320x480 panel (rows 200..479)
 * holds an 8-button pad. Touch (x,y) maps 1:1 to panel coords. Up to 2 touch
 * points are read so you can move AND fire at once. Button press/release edges
 * become DOOM key down/up events in a small ring the engine drains via
 * DG_GetKey. Layout (4 cols x 2 rows, each 80x140):
 *   col0: FWD / BACK    col1: TURN-L / TURN-R
 *   col2: FIRE / USE     col3: ENTER / ESC                                    */
#define PAD_Y0 200
#define PAD_H  (480 - PAD_Y0)      /* 280 */
#define COL_W  80
#define ROW_H  (PAD_H / 2)         /* 140 */

/* Zone index = col*2 + row (see hit_button). Laid out as a keyboard WASD
 * cluster in cols 0-2, with Q/E as fire/use above A/D, and ENTER/ESC in col 3:
 *   [Q=FIRE ][W=FWD ][E=USE ][ENTER]
 *   [A=TURNL][S=BACK][D=TURNR][ESC ]                                        */
#define N_BTN 8
static const unsigned char btn_key[N_BTN] = {
    /* col0 */ KEY_FIRE,    KEY_LEFTARROW,   /* Q=fire  / A=turn-L */
    /* col1 */ KEY_UPARROW, KEY_DOWNARROW,   /* W=fwd   / S=back   */
    /* col2 */ KEY_USE,     KEY_RIGHTARROW,  /* E=use   / D=turn-R */
    /* col3 */ KEY_ENTER,   KEY_ESCAPE,
};

/* key-event ring: (pressed<<8)|doomkey */
static uint16_t kq[32];
static unsigned kq_w, kq_r;
static void kq_push(int pressed, unsigned char key) {
    unsigned nw = (kq_w + 1) & 31u;
    if (nw != kq_r) { kq[kq_w] = (uint16_t)(((pressed ? 1 : 0) << 8) | key); kq_w = nw; }
}
static int kq_pop(int *pressed, unsigned char *key) {
    if (kq_r == kq_w) return 0;
    uint16_t e = kq[kq_r]; kq_r = (kq_r + 1) & 31u;
    *pressed = (e >> 8) & 1; *key = (unsigned char)(e & 0xff);
    return 1;
}

static int hit_button(int x, int y) {
    if (y < PAD_Y0) return -1;                 /* touch on the game area = ignore */
    int col = x / COL_W;      if (col > 3) col = 3;
    int row = (y - PAD_Y0) / ROW_H; if (row > 1) row = 1;
    return col * 2 + row;                      /* matches enum order */
}

static uint8_t  pad_mask;                      /* buttons currently held */
static uint32_t last_poll_ms;

static void poll_touch(void) {
    uint8_t buf[11];                           /* 0x02..0x0C = status + 2 points */
    uint8_t mask = 0;
    if (ft6336u_read(0x02, buf, sizeof buf)) {
        int n = buf[0] & 0x0f;
        if (n >= 1) {
            int x = ((buf[1] & 0x0f) << 8) | buf[2];
            int y = ((buf[3] & 0x0f) << 8) | buf[4];
            int b = hit_button(x, y); if (b >= 0) mask |= (uint8_t)(1u << b);
        }
        if (n >= 2) {
            int x = ((buf[7] & 0x0f) << 8) | buf[8];
            int y = ((buf[9] & 0x0f) << 8) | buf[10];
            int b = hit_button(x, y); if (b >= 0) mask |= (uint8_t)(1u << b);
        }
    }
    uint8_t changed = mask ^ pad_mask;
    for (int b = 0; b < N_BTN; b++)
        if (changed & (1u << b)) kq_push((mask >> b) & 1, btn_key[b]);
    pad_mask = mask;
}

static void draw_btn(int col, int row, uint16_t color) {
    lcd_fill_rect(col * COL_W + 2, PAD_Y0 + row * ROW_H + 2,
                  COL_W - 4, ROW_H - 4, color);  /* fill_rect: MSB-first, no swap */
}
/* small white nub hinting a direction inside a movement button */
static void draw_nub(int col, int row, int dx, int dy) {
    int cx = col * COL_W + COL_W / 2, cy = PAD_Y0 + row * ROW_H + ROW_H / 2;
    lcd_fill_rect((int16_t)(cx + dx * 20 - 8), (int16_t)(cy + dy * 44 - 8),
                  16, 16, 0xFFFF);
}
static void draw_controls(void) {
    lcd_fill_rect(0, PAD_Y0, 320, PAD_H, 0x0000);          /* clear pad area */
    draw_btn(0, 0, 0xF800);                                /* Q = FIRE  (red)   */
    draw_btn(0, 1, 0x2D7F); draw_nub(0, 1, -1, 0);         /* A = turn-L (blue) */
    draw_btn(1, 0, 0x2D7F); draw_nub(1, 0,  0, -1);        /* W = forward       */
    draw_btn(1, 1, 0x2D7F); draw_nub(1, 1,  0,  1);        /* S = back          */
    draw_btn(2, 0, 0x07E0);                                /* E = USE   (green) */
    draw_btn(2, 1, 0x2D7F); draw_nub(2, 1,  1,  0);        /* D = turn-R        */
    draw_btn(3, 0, 0xFFFF);                                /* ENTER (white)     */
    draw_btn(3, 1, 0xFC00);                                /* ESC   (orange)    */
    lcd_wait_idle();
}

/* ---- init -------------------------------------------------------------- */
void DG_Init(void)
{
    /* DOOM's printf() goes through LiteX stdout -> litex_putc -> the IRQ-driven
     * uart_write, which blocks forever on an undrained TX ring unless the UART
     * ISR is running. Bring up interrupts + the UART before anything prints. */
    uart_init();
    irq_setie(1);

    lcd_pads_apply();
    lcd_init();
    busy_wait(200);            /* FT6336U needs ~200 ms after reset */
    touch_init();
    timer_start();             /* last thing: nothing may busy_wait() after */
    draw_controls();           /* paint the touch pad in the bottom 280 px */
    log_puts("doom: DG_Init done\n");
}

/* ---- video ------------------------------------------------------------- */
void DG_DrawFrame(void)
{
    uint64_t t_convert0 = ticks_now();
#ifdef DOOM_CPU_CONVERT
    const uint32_t *s = (const uint32_t *)DG_ScreenBuffer;
    for (int i = 0; i < DW * DH; i++) {
        uint32_t p = s[i];     /* doomgeneric rgba8888: 0xAARRGGBB */
        uint16_t v = ((p >> 8) & 0xf800) | ((p >> 5) & 0x07e0) | ((p >> 3) & 0x001f);
        /* The LCD engine shifts DMA bytes out MSB-first, so store byte-swapped
         * (big-endian) RGB565 -- matches the working LVGL path's
         * LV_COLOR_FORMAT_RGB565_SWAPPED. Native LE uint16_t here = colors
         * scrambled/"inverted" on the panel. */
        fb565[i] = (uint16_t)((v >> 8) | (v << 8));
    }
#endif
    uint64_t t_flush0 = ticks_now();
#ifdef DOOM_CPU_CONVERT
    lcd_dma_rect(0, 0, DW, DH, fb565, DW * DH * 2);
#else
    /* Engine-side conversion: source is XRGB8888, so the length is 4 B/pixel.
     * The engine emits the same 2 B/pixel on the wire, so `flush` is directly
     * comparable to the firmware-convert path above. */
    lcd_dma_rect_xrgb8888(0, 0, DW, DH, DG_ScreenBuffer, DW * DH * 4);
#endif
    lcd_wait_idle();
    uint64_t t_end = ticks_now();

    prof_convert += t_flush0 - t_convert0;
    prof_flush   += t_end    - t_flush0;
}

/* ---- input ------------------------------------------------------------- */
int DG_GetKey(int *pressed, unsigned char *key)
{
    /* Refill from the touch pad when the queue drains, rate-limited to ~100 Hz
     * so a single I_GetEvent drain loop doesn't re-poll the I2C panel per call. */
    if (kq_r == kq_w) {
        uint32_t now = DG_GetTicksMs();
        if (now - last_poll_ms >= 10) { last_poll_ms = now; poll_touch(); }
    }
    return kq_pop(pressed, key);
}

void DG_SetWindowTitle(const char *title)
{
    if (title) {
        log_puts((char *)title);
        log_puts("\n");
    }
}

/* ---- entry ------------------------------------------------------------- */
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    static char *dg_argv[] = { "doom", 0 };
    doomgeneric_Create(1, dg_argv);
    for (;;) {
        /* doomgeneric_Tick() runs the game + rasterizer and calls DG_DrawFrame
         * at the end, so DrawFrame's own two phases are already accounted for;
         * render is whatever is left of the tick. Subtracting rather than
         * instrumenting doom's internals keeps the engine source untouched. */
        uint64_t t0 = ticks_now();
        uint64_t c0 = prof_convert, f0 = prof_flush;
        doomgeneric_Tick();
        uint64_t spent = ticks_now() - t0;
        uint64_t drawn = (prof_convert - c0) + (prof_flush - f0);
        prof_render += (spent > drawn) ? (spent - drawn) : 0;

        if (++prof_frames >= DOOM_PROFILE_FRAMES)
            prof_report();
    }
    return 0;
}
