#include <stdint.h>
#include <stdbool.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "lcd.h"
#include "touch.h"
#include "log.h"

/* ---- Static color band test pattern -------------------------------------- */

static void lcd_test_pattern(void)
{
	const uint16_t colors[] = {
		0xf800, 0x07e0, 0x001f, 0xffff, 0x0000, 0xffe0
	};
	const unsigned int nbands = sizeof(colors) / sizeof(colors[0]);
	const unsigned int band_h = LCD_HEIGHT / nbands;

	for(unsigned int i = 0; i < nbands; i++) {
		log_puts("lcd: fill band "); log_uint(i);
		log_puts(" color=0x"); log_hex16(colors[i]); log_nl();
		lcd_fill_rect(0, i * band_h, LCD_WIDTH, band_h, colors[i]);
	}
	if((band_h * nbands) < LCD_HEIGHT)
		lcd_fill_rect(0, band_h * nbands, LCD_WIDTH,
			LCD_HEIGHT - band_h * nbands, 0x07ff);
}

/* ---- DMA self-test: small RGB565 gradient buffer ------------------------- */

#define LCD_DMA_TEST_W 64
#define LCD_DMA_TEST_H 16

static uint16_t lcd_dma_buf[LCD_DMA_TEST_W * LCD_DMA_TEST_H]
	__attribute__((aligned(4)));

static void lcd_dma_buf_init_gradient(void)
{
	for(unsigned int row = 0; row < LCD_DMA_TEST_H; row++) {
		for(unsigned int col = 0; col < LCD_DMA_TEST_W; col++) {
			uint16_t r = (col * 31) / (LCD_DMA_TEST_W - 1);
			uint16_t g = (row * 63) / (LCD_DMA_TEST_H - 1);
			uint16_t b = ((LCD_DMA_TEST_W - 1 - col) * 31) / (LCD_DMA_TEST_W - 1);
			uint16_t px = (r << 11) | (g << 5) | b;
			/* ST7796S expects high byte first on the wire; pre-swap here. */
			lcd_dma_buf[row * LCD_DMA_TEST_W + col] = (px >> 8) | (px << 8);
		}
	}
}

static void lcd_dma_self_test(uint16_t x, uint16_t y)
{
	log_puts("lcd: dma self-test x="); log_uint(x);
	log_puts(" y="); log_uint(y); log_nl();
	lcd_dma_rect(x, y, LCD_DMA_TEST_W, LCD_DMA_TEST_H,
	             lcd_dma_buf, sizeof(lcd_dma_buf));
}

/* ---- Bouncing-rect benchmark animation ----------------------------------- */
/* Uses timer0 as a free-running cycle counter for FPS measurement. After
 * timer_take_over() we can't call busy_wait() anymore - libbase's
 * busy_wait reloads timer0. */

static void timer_take_over(void)
{
	timer0_en_write(0);
	timer0_load_write(0xffffffffu);
	timer0_reload_write(0xffffffffu);
	timer0_en_write(1);
}

static uint32_t timer_now(void)
{
	timer0_update_value_write(1);
	return timer0_value_read();
}

#ifndef LCD_FRAME_DELAY_US
#define LCD_FRAME_DELAY_US 1
#endif

static void lcd_animate(void)
{
	const uint16_t bg = 0x0000;
	const int16_t rw = LCD_DMA_TEST_W;
	const int16_t rh = LCD_DMA_TEST_H;
	int16_t x = 60, y = 60;
	int16_t vx = 4, vy = 3;
	int16_t prev_x = x, prev_y = y;
	uint32_t frame = 0;
	uint32_t window_frames = 0;
	uint32_t cycles_per_sec = CONFIG_CLOCK_FREQUENCY;
	uint32_t frame_delay_cycles = (uint32_t)LCD_FRAME_DELAY_US *
	                              (CONFIG_CLOCK_FREQUENCY / 1000000u);

	log_puts("lcd: animation running (reset to stop)"); log_nl();
	lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, bg);

	timer_take_over();
	uint32_t window_start = timer_now();

	while(1) {
		uint32_t frame_start = timer_now();

		/* Draw the new rect first, then erase only the L-shape of the
		 * old rect not covered by the new one. Keeps the rect visible
		 * at every instant - no flicker between erase and draw. */
		lcd_dma_rect(x, y, rw, rh, lcd_dma_buf, sizeof(lcd_dma_buf));

		int16_t dx = x - prev_x;
		int16_t dy = y - prev_y;
		if (dx > 0)      lcd_fill_rect(prev_x, prev_y, dx, rh, bg);
		else if (dx < 0) lcd_fill_rect(x + rw, prev_y, -dx, rh, bg);
		if (dy > 0)      lcd_fill_rect(prev_x, prev_y, rw, dy, bg);
		else if (dy < 0) lcd_fill_rect(prev_x, y + rh, rw, -dy, bg);

		prev_x = x; prev_y = y;
		x += vx; y += vy;
		if(x < 0)              { x = 0;               vx = -vx; }
		if(x + rw > LCD_WIDTH) { x = LCD_WIDTH - rw;  vx = -vx; }
		if(y < 0)              { y = 0;               vy = -vy; }
		if(y + rh > LCD_HEIGHT){ y = LCD_HEIGHT - rh; vy = -vy; }

		if(frame_delay_cycles)
			while((frame_start - timer_now()) < frame_delay_cycles)
				;

		frame++;
		window_frames++;

		uint32_t cur = timer_now();
		uint32_t elapsed = window_start - cur;
		if(elapsed >= cycles_per_sec) {
			log_puts("fps="); log_uint(window_frames);
			log_puts(" frame="); log_uint(frame);
			log_puts(" us/frame=");
			log_uint(elapsed / (window_frames ? window_frames : 1) /
			         (CONFIG_CLOCK_FREQUENCY / 1000000u));
			log_nl();
			window_start = cur;
			window_frames = 0;
		}
	}
}

/* ---- Touch poll loop ----------------------------------------------------- */

static void ft6336u_touch_test(unsigned int duration_ms)
{
	const unsigned int poll_ms = 30;
	const unsigned int polls   = duration_ms / poll_ms;
	int prev_n = -1;

	log_puts("ft6336u: touch test for "); log_uint(duration_ms);
	log_puts(" ms - touch the screen now"); log_nl();

	for(unsigned int i = 0; i < polls; i++) {
		uint8_t buf[7];
		if(ft6336u_read(0x02, buf, sizeof(buf))) {
			unsigned int n = buf[0] & 0x0f;
			unsigned int x = ((buf[1] & 0x0f) << 8) | buf[2];
			unsigned int y = ((buf[3] & 0x0f) << 8) | buf[4];
			if(n > 0) {
				log_puts("touch n="); log_uint(n);
				log_puts(" x=");      log_uint(x);
				log_puts(" y=");      log_uint(y);
				log_puts(" int=");    log_uint(ctp_int_in_read() & 1);
				log_nl();
			}
			else if(prev_n > 0) {
				log_puts("touch release"); log_nl();
			}
			prev_n = (int)n;
		}
		busy_wait(poll_ms);
	}

	log_puts("ft6336u: touch test done"); log_nl();
}

/* ---- main ---------------------------------------------------------------- */

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("ST7796S LCD engine test"); log_nl();
	log_puts("sys clk : "); log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();
	log_puts("SPI SCK : "); log_uint(LCD_SPI_FREQUENCY); log_puts(" Hz"); log_nl();
	log_puts("LCD size: "); log_uint(LCD_WIDTH); log_puts(" x "); log_uint(LCD_HEIGHT); log_nl();
	log_puts("CSR lcd base=0x"); log_hex32(CSR_LCD_BASE); log_nl();

	lcd_pads_apply();
	lcd_init();
	/* FT6336U shares the LCD reset line; needs ~200 ms post-reset
	 * before responding to I2C. */
	busy_wait(200);
	lcd_dma_buf_init_gradient();
	lcd_test_pattern();
	lcd_dma_self_test(LCD_WIDTH / 2 - LCD_DMA_TEST_W / 2,
	                  LCD_HEIGHT / 2 - LCD_DMA_TEST_H / 2);

	touch_init();
	ft6336u_probe();
	ft6336u_touch_test(/* duration_ms */ 8000);

	log_puts("LCD static checks done; starting animation"); log_nl();
	busy_wait(1000);
	lcd_animate();

	return 0;
}
