/* Live Doppler spectrum + waterfall on the LCD -- no host, no network required.
 *
 * The on-board half of next-radar.md's DPU thread. gateware/adc_cpu.py times
 * MCP3008 samples in hardware (jitter-free) and exposes them as CSRs; this
 * firmware polls them, runs the fixed-point FFT from software/common/fft_q15.h,
 * and draws straight to the ST7796S panel with software/common/lcd.c's engine
 * ops. No LVGL: direct fill/DMA rects are simpler than a widget tree here.
 *
 * **Physical layout: the panel is mounted rotated 90 degrees clockwise from
 * the controller's native portrait addressing.** Rather than fight that with
 * a software rotation, the layout below just uses the controller's native Y
 * axis (0..479, the long axis) as frequency -- which is what ends up running
 * horizontally once the panel is physically rotated -- and native X (0..319)
 * as the short vertical axis, split into a live bar region on top and a
 * scrolling waterfall below. If this reads backwards or upside down once
 * flashed, the fix is one of the two "flip to reverse" constants below, not a
 * redesign.
 *
 * Window size is 1024 samples -- at 20 ksps that's 51.2ms, same bin spacing
 * tools/adc_scope.py's own default (20000/1024 ~= 19.53 Hz/bin, matching the
 * "1024-pt FFT at 20 ksps -> 19.5 Hz bins" note in next-radar.md) and it
 * dominates the frame budget (the FFT itself measured 5.84ms on VexRiscv in
 * software/fft_bench -- comfortably inside the 51ms window).
 *
 * **Every column is redrawn in full every frame**, not diffed against the
 * previous frame -- see git history/next-radar.md for why an earlier
 * per-column diff (tracked in a too-small uint8_t) could get stuck with stale
 * pixels. A full redraw of the live-bar region plus one bulk DMA blit of the
 * whole waterfall both cost low milliseconds, a rounding error against the
 * 51ms collection-bound budget.
 */
#include <stdint.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#include "lcd.h"
#include "log.h"
#include "fft_q15.h"

#define WIN        1024               /* samples per FFT window, power of two */

/* ---- Frequency axis (native Y, 0..479) --------------------------------- */
/* 60 bins * 8 native-Y px/bin = 480, an exact fit. bin index `bi` (0..59)
 * is FFT bin (bi+1) -- bin 0 (DC) is the sample mean, not a Doppler
 * component. 60 bins * 19.53 Hz/bin ~= 1172 Hz ~= 7.3 m/s top speed --
 * zoomed in on indoor/human-scale motion (hand waves, walking, light
 * jogging) rather than the full 4.69 kHz / 29 m/s range up to "car at
 * 100 km/h", which left the actually-interesting slow-target region
 * squeezed into the first few percent of the screen. First tried 32 bins
 * (~3.9 m/s, FREQ_Y_PX=15) and that was too tight a zoom (2026-10-10); this
 * is roughly double that range -- FREQ_BINS must keep dividing 480 evenly
 * for FREQ_Y_PX to stay a whole pixel count, so it's 60/8 rather than a
 * literal 64/7.5. */
#define FREQ_BINS     60
#define FREQ_Y_PX     8
/* Both in milli-Hz (x1000), to keep this integer -- no FPU on this core, see
 * next-radar.md, and fixed-point is already the house style (fft_q15.h). */
#define HZ_PER_BIN_MHZ  (20000000u / WIN)           /* 19531 (19.53125 Hz/bin) */
#define HZ_PER_MS_MHZ   160940u                      /* 160.94 Hz per m/s, next-radar.md */
/* Set to 1 if bin 0 ends up on the physical right instead of left. */
#define FREQ_Y_REVERSED 0

/* ---- Vertical axis (native X, 0..319): bars on top, waterfall below ---- */
#define BAR_X0      0
#define BAR_X1      159                              /* 160 px tall */
#define BAR_LEN     (BAR_X1 - BAR_X0 + 1)
/* Set to 1 if bars grow downward instead of up once flashed. */
#define BAR_GROWS_DOWN 0

#define GAP_X0      160
#define GAP_X1      169                              /* 10 px: tick marks live here */

#define WF_X0       170
#define WF_ROWS     150                               /* 170..319 = 150 px, 1 px/time-slot */
#define WF_X1       (WF_X0 + WF_ROWS - 1)             /* 319 -- exact fit to LCD_WIDTH */

#define COLOR_BG    0x0000   /* RGB565 black */
#define COLOR_TICK  0x4208   /* dim grey, same as clocking-stance-style markers elsewhere */
#define COLOR_TICK5 0xFFFF   /* white: every 5th tick, for a visual anchor */

static fft_cplx win_buf[WIN];

/* Per-bin peak-hold envelope for the live bar region, in BAR_LEN pixels.
 * Instant attack / slow decay -- same idea as tools/adc_scope.py's peak-hold
 * key -- so the bars read as a moving shape instead of raw per-frame
 * flicker. uint16_t: BAR_LEN (160) does fit a byte today, but the window-size
 * change already bit once on a too-small type (see file header); not
 * repeating that mistake for a two-byte cost. */
static uint16_t envelope[FREQ_BINS];
#define ENVELOPE_DECAY 8   /* px/frame the envelope falls once the signal quiets */

/* Waterfall history, stored in the EXACT byte order lcd_dma_rect() needs for
 * one bulk blit: native Y (bin pair) outer, native X (time slot) inner.
 * WF_X1 is "now"; WF_X0 is the oldest sample still on screen. Each bin's two
 * native-Y rows (FREQ_Y_PX=2) hold identical data -- simplest way to thicken
 * a logical row without the DMA engine's row_count/stride path. */
static uint16_t wf_hist[FREQ_BINS * FREQ_Y_PX][WF_ROWS];

/* ---- ADC window collection -------------------------------------------- */

/* Busy-poll `count` for each new sample: at 20 ksps / 90 MHz there are
 * ~4500 sys clocks between samples, ample margin for a CSR-read loop to
 * never miss an increment. If this call is a bit late starting (the
 * previous frame's FFT+draw ran over), it just picks up whatever `last`
 * already holds and moves on -- costs one window a few samples' timing
 * precision at the start, not a hang, since the hardware timer keeps
 * incrementing `count` regardless of whether firmware is watching. */
static void collect_window(void)
{
    uint32_t count = adc_cpu_count_read();
    for (unsigned i = 0; i < WIN; ) {
        uint32_t c = adc_cpu_count_read();
        if (c != count) {
            count = c;
            int32_t raw      = (int32_t)(adc_cpu_last_read() & 0x3ffu); /* 10-bit */
            int32_t centered = raw - 512;    /* roughly DC-center the 0..1023 range */
            win_buf[i].re = centered << 5;   /* up into Q15 headroom for the FFT */
            win_buf[i].im = 0;
            i++;
        }
    }
}

/* ---- magnitude -------------------------------------------------------- */

/* floor(log2(v)), v > 0. */
static unsigned ilog2_u32(uint32_t v)
{
    unsigned n = 0;
    while (v >>= 1) n++;
    return n;
}

/* Bin magnitude is log2-compressed power, not linear amplitude: the 70 dB
 * swing next-radar.md measured between idle and a close target (Real
 * returns, 2026-09-20) would otherwise be an invisible flat line punctuated
 * by one spike. MAG_MAX/MAG_FLOOR measured on hardware, not derived
 * constants -- the old MAG_MAX=24 clipped real targets, which were
 * indistinguishable from idle until raised. Same squelch idea as
 * tools/adc_scope.py's "AC > 0.35 / peak > 0.5 LSB" velocity gate, applied
 * per-bin to the display instead of to one headline number.
 *
 * MAG_FLOOR history: a pre-averaging 45s idle capture clustered at 14-19
 * with a sustained close wave reaching 20-28, so the floor first went to 19
 * (just above that ceiling). Once avg_power (below) was added, idle tightened
 * further to a near-constant 18, but a weak/far wave's post-average peak
 * only reached 19-22 -- averaging smooths noise, but it also smears a fast-
 * changing target's peak across bins/time (a hand wave's Doppler frequency
 * isn't stationary within the averaging window the way a slow, steady
 * target's would be). Dropped the floor to 18 (idle's own ceiling, not one above
 * it) to recover that lost sensitivity; if idle ever flickers because of
 * this, that's the tradeoff to revisit -- lower AVG_SHIFT (less smoothing,
 * preserves more transient peak) before raising the floor back up. */
#define MAG_MAX   30
#define MAG_FLOOR 18
#define EFF_MAX   (MAG_MAX - MAG_FLOOR)

/* Non-coherent integration: an exponential moving average of each bin's raw
 * power across consecutive FFT windows, BEFORE the log2 compression below.
 * Return amplitude falls as 1/r^4 (next-radar.md), so a distant target's
 * power can sit only barely above the single-frame noise floor -- but noise
 * is uncorrelated frame to frame while a real target's power is not, so
 * averaging pulls the noise estimate down while a persistent weak target
 * barely moves.
 *
 * avg_power[] stores the accumulator SCALED by 2^AVG_SHIFT, not the average
 * itself -- `avg_power[bi] += power - (avg_power[bi] >> AVG_SHIFT)` instead
 * of the naive `avg += (power - avg) >> AVG_SHIFT`. The naive form discards
 * the sub-shift remainder on every update, and once two consecutive frames'
 * power differs by less than 2^AVG_SHIFT, that shift rounds to exactly zero
 * and the average FREEZES permanently -- measured on hardware 2026-10-10:
 * AVG_SHIFT=1 pinned mag at a constant 18 for 490 straight frames, including
 * through a confirmed wave, because idle's own power never happened to move
 * by 2 in one frame. The scaled form keeps the remainder inside the
 * accumulator across updates instead of truncating it away, so it can't get
 * stuck; only the read side (ilog2 below) shifts down to recover the actual
 * average. int64_t headroom: at steady state avg_power settles near
 * power*2^AVG_SHIFT, and power itself can approach 2^30 for the loudest
 * bins this project has measured, so uint32_t would be too tight once
 * AVG_SHIFT grows past a couple of bits. */
static int64_t avg_power[FREQ_BINS];
#define AVG_SHIFT 2

/* Magnitude (0..MAG_MAX) per bin for the CURRENT frame, computed once by
 * update_mags() below. mag_of_bin() mutates avg_power[], so it must run
 * exactly once per bin per frame -- draw_bars() and draw_waterfall() both
 * need the result, so they read this cache instead of calling it twice
 * (which would advance the average twice as fast as AVG_SHIFT intends). */
static unsigned cur_mag[FREQ_BINS];

static unsigned mag_of_bin(unsigned bi)
{
    unsigned bin = bi + 1; /* skip DC (bin 0) */
    int64_t re = win_buf[bin].re;
    int64_t im = win_buf[bin].im;
    int64_t power = (re * re + im * im) >> 16;

    avg_power[bi] += power - (avg_power[bi] >> AVG_SHIFT);
    uint32_t avg = (uint32_t)(avg_power[bi] >> AVG_SHIFT);

    unsigned mag = avg ? (ilog2_u32(avg) + 1) : 0;
    return mag > MAG_MAX ? MAG_MAX : mag;
}

static void update_mags(void)
{
    for (unsigned bi = 0; bi < FREQ_BINS; bi++)
        cur_mag[bi] = mag_of_bin(bi);
}

/* Raw log2-power mag (0..MAG_MAX) rescaled so MAG_FLOOR and below is 0 and
 * the floor..MAG_MAX range is stretched across the full 0..EFF_MAX display
 * range -- squelch plus contrast stretch in one step. */
static unsigned mag_above_floor(unsigned mag)
{
    return mag > MAG_FLOOR ? (mag - MAG_FLOOR) : 0;
}

/* Single-hue dark->light ramp, same convention as tools/adc_scope.py's
 * waterfall ("Colour: single-hue dark->light ramp, not rainbow", next-radar.md)
 * -- brightness alone encodes magnitude, nothing to misread as hue. `eff` is
 * post-floor (0..EFF_MAX): 0 is background now that the floor is squelched,
 * so there's no more "dim floor colour" to confuse with real signal. */
static uint16_t color_for_mag(unsigned eff)
{
    if (eff == 0) return COLOR_BG;
    unsigned g = 10 + (eff * (63 - 10)) / EFF_MAX;
    if (g > 63) g = 63;
    return (uint16_t)(g << 5);
}

static inline unsigned freq_y(unsigned bi)
{
    unsigned i = FREQ_Y_REVERSED ? (FREQ_BINS - 1 - bi) : bi;
    return i * FREQ_Y_PX;
}

/* ---- drawing ------------------------------------------------------------ */

static void draw_bars(void)
{
    for (unsigned bi = 0; bi < FREQ_BINS; bi++) {
        unsigned eff = mag_above_floor(cur_mag[bi]);
        unsigned h   = (eff * BAR_LEN) / EFF_MAX;

        unsigned env = envelope[bi];
        if (h > env)                    env = h;
        else if (env > ENVELOPE_DECAY)  env -= ENVELOPE_DECAY;
        else                            env = 0;
        envelope[bi] = (uint16_t)env;

        uint16_t color = color_for_mag((env * EFF_MAX) / BAR_LEN);
        int16_t  y     = (int16_t)freq_y(bi);

        if (!BAR_GROWS_DOWN) {
            /* baseline at BAR_X1, grows toward BAR_X0 */
            if (env < BAR_LEN)
                lcd_fill_rect((int16_t)BAR_X0, y, (int16_t)(BAR_LEN - env), FREQ_Y_PX, COLOR_BG);
            if (env > 0)
                lcd_fill_rect((int16_t)(BAR_X1 - env + 1), y, (int16_t)env, FREQ_Y_PX, color);
        } else {
            /* baseline at BAR_X0, grows toward BAR_X1 */
            if (env > 0)
                lcd_fill_rect((int16_t)BAR_X0, y, (int16_t)env, FREQ_Y_PX, color);
            if (env < BAR_LEN)
                lcd_fill_rect((int16_t)(BAR_X0 + env), y, (int16_t)(BAR_LEN - env), FREQ_Y_PX, COLOR_BG);
        }
    }
}

/* Shift every row's time history left by one slot and push the newest
 * column in at WF_X1, then blit the whole region in one DMA burst. wf_hist's
 * layout already matches the engine's native scan order (Y outer, X inner),
 * so no reordering copy is needed -- just shift and go. */
static void draw_waterfall(void)
{
    for (unsigned bi = 0; bi < FREQ_BINS; bi++) {
        unsigned eff   = mag_above_floor(cur_mag[bi]);
        uint16_t color = color_for_mag(eff);
        unsigned y0    = freq_y(bi);
        for (unsigned r = 0; r < FREQ_Y_PX; r++) {
            uint16_t *row = wf_hist[y0 + r];
            memmove(&row[0], &row[1], (WF_ROWS - 1) * sizeof(uint16_t));
            row[WF_ROWS - 1] = color;
        }
    }
    lcd_dma_rect((int16_t)WF_X0, 0, (int16_t)WF_ROWS, (int16_t)(FREQ_BINS * FREQ_Y_PX),
                wf_hist, sizeof(wf_hist));
}

/* Drawn once at boot, not per frame -- the gap band is otherwise untouched.
 * The zoomed-in range (0..~3.9 m/s) is too narrow for 1 m/s/5 m/s ticks to
 * mean much, so this ticks every 0.5 m/s, with a brighter/wider mark every
 * whole m/s as a visual anchor. No text (no font renderer in this firmware),
 * but a ruler beats an unmarked axis. */
static void draw_ticks(void)
{
    lcd_fill_rect((int16_t)GAP_X0, 0, (int16_t)(GAP_X1 - GAP_X0 + 1), LCD_HEIGHT, COLOR_BG);
    for (unsigned half_ms = 1; ; half_ms++) {   /* half_ms counts 0.5 m/s steps */
        unsigned bi = (half_ms * HZ_PER_MS_MHZ) / (2 * HZ_PER_BIN_MHZ);
        if (bi >= FREQ_BINS) break;
        unsigned y = freq_y(bi);
        int tall = (half_ms % 2) == 0;   /* whole m/s */
        int16_t tx = tall ? (int16_t)GAP_X0 : (int16_t)(GAP_X0 + 3);
        int16_t tw = tall ? (int16_t)(GAP_X1 - GAP_X0 + 1) : (int16_t)(GAP_X1 - GAP_X0 - 2);
        lcd_fill_rect(tx, (int16_t)y, tw, (int16_t)FREQ_Y_PX,
                     tall ? COLOR_TICK5 : COLOR_TICK);
    }
}

int main(void)
{
    uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
    /* uart_write() is IRQ-driven and blocks forever once its ring buffer
     * fills unless the UART ISR is running -- see software/fft_bench. */
    irq_setie(1);
#endif
    log_puts("\nradar_lcd: live Doppler spectrum + waterfall, no host required\n");

    lcd_init();
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, COLOR_BG);
    draw_ticks();

    adc_cpu_channel_write(0);
    adc_cpu_enable_write(1);

    for (;;) {
        collect_window();
        fft_q15(win_buf, WIN);
        update_mags();
        draw_bars();
        draw_waterfall();

        /* DIAGNOSTIC: re-measuring idle vs. target now that power is
         * averaged across frames, to find a tighter (lower) MAG_FLOOR. */
        unsigned best_mag = 0, best_bi = 0;
        for (unsigned bi = 0; bi < FREQ_BINS; bi++) {
            if (cur_mag[bi] > best_mag) { best_mag = cur_mag[bi]; best_bi = bi; }
        }
        log_puts("raw peak="); log_uint(best_mag);
        log_puts(" bin="); log_uint(best_bi);
        log_nl();
    }
}
