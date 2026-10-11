/* Option-A measurement: how long does a plain fixed-point radix-2 FFT take on
 * the main CPU, no gateware help at all?
 *
 * Board-agnostic like rv32_test -- depends on nothing but the CPU, UART,
 * timer0 and main_ram, so it runs on whatever target BUILD_DIR points at.
 * This exists to settle a specific question (docs/next-radar.md's DPU
 * thread): is a CSR-mapped hardware butterfly accelerator actually needed to
 * get a live spectrum on the LCD, or does software already clear the frame
 * budget by a wide margin? Runs 64..1024-point FFTs back to back and prints
 * cycles/us for each, plus an exact correctness check (impulse in -> flat
 * spectrum out, bit-exact, no tolerance needed -- see the comment by
 * run_one()). The FFT itself lives in software/common/fft_q15.h, shared with
 * software/radar_lcd.
 *
 *     make -C software/fft_bench BUILD_DIR=/path/to/build
 *     litex_term /dev/icelink-uart --speed 115200 --kernel software/fft_bench/fft_bench.bin
 */
#include <stdint.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#include "fft_q15.h"

/* ---- minimal output (same pattern as rv32_test: no printf) ---- */
static void put(char c)        { if (c == '\n') uart_write('\r'); uart_write(c); }
static void puts_(const char *s) { while (*s) put(*s++); }
static void put_u32(uint32_t v)
{
    char t[11]; int n = 0;
    if (!v) t[n++] = '0';
    while (v) { t[n++] = (char)('0' + v % 10u); v /= 10u; }
    while (n) put(t[--n]);
}

/* ---- timer0 as a cycle counter (pattern from software/lvgl_demo/main.c and
 * software/doom/src/doomgeneric_icepi.c): free-running down-counter, so
 * elapsed = before - after for any interval short enough not to wrap (max
 * ~47 s worth of cycles at 90 MHz -- this benchmark never gets close). */
static void timer_start(void)
{
    timer0_en_write(0);
    timer0_load_write(0xffffffffu);
    timer0_reload_write(0xffffffffu);
    timer0_en_write(1);
    timer0_update_value_write(1);
}
static uint32_t timer_now(void)
{
    timer0_update_value_write(1);
    return timer0_value_read();
}

static fft_cplx buf[FFT_NMAX];

/* Impulse in -> flat spectrum out is exact, not approximate: bin 0 of the
 * bit-reversed array is never the "odd" partner of any butterfly (reverse(0)
 * is always 0), so it only ever receives additions of zero from the other
 * (zero) samples and is itself never multiplied by a twiddle. X[k] == x[0]
 * for every k, bit for bit -- a real correctness check, not a tolerance band. */
static int run_one(unsigned n)
{
    for (unsigned i = 0; i < n; i++) { buf[i].re = 0; buf[i].im = 0; }
    buf[0].re = 16384; /* 0.5 in Q15 */

    timer_start();
    uint32_t t0 = timer_now();
    fft_q15(buf, n);
    uint32_t t1 = timer_now();
    uint32_t cycles = t0 - t1; /* counts down */

    int ok = 1;
    for (unsigned i = 0; i < n; i++) {
        if (buf[i].re != 16384 || buf[i].im != 0) { ok = 0; break; }
    }

    uint32_t us = cycles / (CONFIG_CLOCK_FREQUENCY / 1000000u);
    puts_("N="); put_u32(n);
    puts_("\tcycles="); put_u32(cycles);
    puts_("\tus="); put_u32(us);
    puts_("\t"); puts_(ok ? "PASS" : "FAIL");
    put('\n');
    return ok;
}

int main(void)
{
    uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
    /* uart_write() is IRQ-driven and blocks forever once its ring buffer
     * fills unless the UART ISR is running -- same trap as rv32_test. */
    irq_setie(1);
#endif

    puts_("\nfft_bench: fixed-point radix-2 DIT FFT, software only (Option A)\n");
    puts_("CONFIG_CLOCK_FREQUENCY="); put_u32(CONFIG_CLOCK_FREQUENCY); put('\n');

    static const unsigned sizes[] = {64, 128, 256, 512, 1024};
    int all_ok = 1;
    for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
        if (!run_one(sizes[i])) all_ok = 0;
    }

    puts_(all_ok ? "\nALL PASS\n" : "\nSOME FAILED\n");
    while (1) {}
    return 0;
}
