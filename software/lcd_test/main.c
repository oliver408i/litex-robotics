#include <stdint.h>
#include <stdbool.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/i2c.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#ifndef CSR_LCD_BASE
#error "Build the SoC with --with-lcd before compiling software/lcd_test."
#endif

#define FT6336U_I2C_ADDR  0x38

#define LCD_PADS_CS_N      (1u << CSR_LCD_PADS_CTRL_CS_N_OFFSET)
#define LCD_PADS_DC        (1u << CSR_LCD_PADS_CTRL_DC_OFFSET)
#define LCD_PADS_RESET_N   (1u << CSR_LCD_PADS_CTRL_RESET_N_OFFSET)
#define LCD_PADS_BACKLIGHT (1u << CSR_LCD_PADS_CTRL_BACKLIGHT_OFFSET)

#define LCD_OP_START       (1u << CSR_LCD_OP_START_OFFSET)
#define LCD_OP_SINGLE      ((1u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_DMA         ((2u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_FILL        ((3u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_BUSY           (1u << CSR_LCD_STATUS_BUSY_OFFSET)

static uint32_t lcd_pads_state = LCD_PADS_CS_N | LCD_PADS_RESET_N;

static void log_char(char c)
{
	if(c == '\n')
		uart_write('\r');
	uart_write(c);
}

static void log_puts(const char *s)
{
	while(*s)
		log_char(*s++);
}

static void log_hex4(uint8_t v)
{
	v &= 0x0f;
	log_char(v < 10 ? '0' + v : 'a' + v - 10);
}

static void log_hex8(uint8_t v)  { log_hex4(v >> 4); log_hex4(v); }
static void log_hex16(uint16_t v){ log_hex8(v >> 8); log_hex8(v); }
static void log_hex32(uint32_t v){ log_hex16(v >> 16); log_hex16(v); }

static void log_uint(uint32_t v)
{
	char buf[10];
	unsigned int i = 0;

	if(v == 0) {
		log_char('0');
		return;
	}
	while(v && i < sizeof(buf)) {
		buf[i++] = '0' + v % 10;
		v /= 10;
	}
	while(i)
		log_char(buf[--i]);
}

static void log_nl(void)
{
	log_char('\n');
	uart_sync();
}

static void lcd_pads_apply(void)
{
	lcd_pads_ctrl_write(lcd_pads_state);
}

static void lcd_pads_set(uint32_t mask, int value)
{
	if(value)
		lcd_pads_state |= mask;
	else
		lcd_pads_state &= ~mask;
	lcd_pads_apply();
}

static void lcd_wait_idle(void)
{
	while(lcd_status_read() & LCD_BUSY)
		;
}

static void lcd_send_byte(uint8_t b)
{
	lcd_wait_idle();
	lcd_byte_write(b);
	lcd_op_write(LCD_OP_SINGLE);
}

static void lcd_send_dma(const void *src, uint32_t len)
{
	lcd_wait_idle();
	lcd_dma_src_write((uint32_t)src);
	lcd_dma_len_write(len);
	lcd_op_write(LCD_OP_DMA);
}

static void lcd_send_fill(uint16_t color, uint32_t pixels)
{
	lcd_wait_idle();
	lcd_fill_color_write(color);
	lcd_fill_count_write(pixels);
	lcd_op_write(LCD_OP_FILL);
}

static void lcd_select(void)   { lcd_pads_set(LCD_PADS_CS_N, 0); }
static void lcd_deselect(void) { lcd_wait_idle(); lcd_pads_set(LCD_PADS_CS_N, 1); }
static void lcd_dc(int v)      { lcd_wait_idle(); lcd_pads_set(LCD_PADS_DC, v); }
static void lcd_reset_n(int v) { lcd_pads_set(LCD_PADS_RESET_N, v); }
static void lcd_backlight(int v){ lcd_pads_set(LCD_PADS_BACKLIGHT, v); }

static void lcd_write_cmd(uint8_t cmd)
{
	lcd_select();
	lcd_dc(0);
	lcd_send_byte(cmd);
	lcd_deselect();
}

static void lcd_cmd_data(uint8_t cmd, const uint8_t *data, unsigned int len)
{
	lcd_select();
	lcd_dc(0);
	lcd_send_byte(cmd);
	if(len) {
		lcd_dc(1);
		for(unsigned int i = 0; i < len; i++)
			lcd_send_byte(data[i]);
	}
	lcd_deselect();
}

static void lcd_hw_reset(void)
{
	log_puts("lcd: hardware reset"); log_nl();
	lcd_pads_state = LCD_PADS_CS_N | LCD_PADS_RESET_N;
	lcd_pads_apply();
	busy_wait(20);
	lcd_reset_n(0);
	busy_wait(20);
	lcd_reset_n(1);
	busy_wait(120);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	uint8_t caset[] = {x0 >> 8, x0 & 0xff, x1 >> 8, x1 & 0xff};
	uint8_t raset[] = {y0 >> 8, y0 & 0xff, y1 >> 8, y1 & 0xff};

	lcd_cmd_data(0x2a, caset, sizeof(caset));
	lcd_cmd_data(0x2b, raset, sizeof(raset));
	lcd_write_cmd(0x2c);
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	log_puts("lcd: fill x="); log_uint(x);
	log_puts(" y="); log_uint(y);
	log_puts(" w="); log_uint(w);
	log_puts(" h="); log_uint(h);
	log_puts(" color=0x"); log_hex16(color); log_nl();

	lcd_set_window(x, y, x + w - 1, y + h - 1);
	lcd_select();
	lcd_dc(1);
	lcd_send_fill(color, (uint32_t)w * h);
	lcd_deselect();
}

static void lcd_init(void)
{
	static const uint8_t unlock1[]   = {0xc3};
	static const uint8_t unlock2[]   = {0x96};
	static const uint8_t madctl[]    = {0x48};
	static const uint8_t pixfmt[]    = {0x55};
	static const uint8_t invctr[]    = {0x01};
	static const uint8_t entry[]     = {0xc6};
	static const uint8_t pwr1[]      = {0x80, 0x45};
	static const uint8_t pwr2[]      = {0x13};
	static const uint8_t pwr3[]      = {0xa7};
	static const uint8_t vcom[]      = {0x0a};
	static const uint8_t display[]   = {0x80};
	static const uint8_t pos_gamma[] = {
		0xf0, 0x09, 0x0b, 0x06, 0x04, 0x15, 0x2f,
		0x54, 0x42, 0x3c, 0x17, 0x14, 0x18, 0x1b
	};
	static const uint8_t neg_gamma[] = {
		0xe0, 0x09, 0x0b, 0x06, 0x04, 0x03, 0x2b,
		0x43, 0x42, 0x3b, 0x16, 0x14, 0x17, 0x1b
	};
	static const uint8_t lock1[] = {0x3c};
	static const uint8_t lock2[] = {0x69};

	lcd_hw_reset();
	log_puts("lcd: software reset"); log_nl();
	lcd_write_cmd(0x01);
	busy_wait(120);

	log_puts("lcd: init sequence start"); log_nl();
	lcd_cmd_data(0xf0, unlock1, sizeof(unlock1));
	lcd_cmd_data(0xf0, unlock2, sizeof(unlock2));
	lcd_cmd_data(0x36, madctl,  sizeof(madctl));
	lcd_cmd_data(0x3a, pixfmt,  sizeof(pixfmt));
	lcd_cmd_data(0xb4, invctr,  sizeof(invctr));
	lcd_cmd_data(0xb7, entry,   sizeof(entry));
	lcd_cmd_data(0xc0, pwr1,    sizeof(pwr1));
	lcd_cmd_data(0xc1, pwr2,    sizeof(pwr2));
	lcd_cmd_data(0xc2, pwr3,    sizeof(pwr3));
	lcd_cmd_data(0xc5, vcom,    sizeof(vcom));
	lcd_cmd_data(0xb6, display, sizeof(display));
	lcd_cmd_data(0xe0, pos_gamma, sizeof(pos_gamma));
	lcd_cmd_data(0xe1, neg_gamma, sizeof(neg_gamma));
	lcd_cmd_data(0xf0, lock1, sizeof(lock1));
	lcd_cmd_data(0xf0, lock2, sizeof(lock2));

	log_puts("lcd: sleep out"); log_nl();
	lcd_write_cmd(0x11);
	busy_wait(120);
	log_puts("lcd: display on"); log_nl();
	lcd_write_cmd(0x29);
	busy_wait(20);
	lcd_backlight(1);
}

static void lcd_test_pattern(void)
{
	const uint16_t colors[] = {
		0xf800, 0x07e0, 0x001f, 0xffff, 0x0000, 0xffe0
	};
	const unsigned int nbands = sizeof(colors) / sizeof(colors[0]);
	const unsigned int band_h = LCD_HEIGHT / nbands;

	for(unsigned int i = 0; i < nbands; i++)
		lcd_fill_rect(0, i * band_h, LCD_WIDTH, band_h, colors[i]);

	if((band_h * nbands) < LCD_HEIGHT)
		lcd_fill_rect(0, band_h * nbands, LCD_WIDTH,
			LCD_HEIGHT - band_h * nbands, 0x07ff);
}

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

static void lcd_fill_rect_silent(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	lcd_set_window(x, y, x + w - 1, y + h - 1);
	lcd_select();
	lcd_dc(1);
	lcd_send_fill(color, (uint32_t)w * h);
	lcd_deselect();
}

static void lcd_dma_rect(int16_t x, int16_t y, int16_t w, int16_t h, const void *buf, uint32_t len)
{
	lcd_set_window(x, y, x + w - 1, y + h - 1);
	lcd_select();
	lcd_dc(1);
	lcd_send_dma(buf, len);
	lcd_deselect();
}

static void lcd_dma_self_test(uint16_t x, uint16_t y)
{
	log_puts("lcd: dma self-test x="); log_uint(x);
	log_puts(" y="); log_uint(y);
	log_puts(" w="); log_uint(LCD_DMA_TEST_W);
	log_puts(" h="); log_uint(LCD_DMA_TEST_H); log_nl();

	lcd_dma_rect(x, y, LCD_DMA_TEST_W, LCD_DMA_TEST_H, lcd_dma_buf, sizeof(lcd_dma_buf));
}

/* Free-running cycle counter via timer0. Taking it over here means we can't
 * use busy_wait() after this point - the animation loop doesn't need it. */
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

/* Cap the animation refresh rate below the LCD's internal refresh so we
 * don't tear. 16000us ~= 60 FPS; bump higher (e.g. 33000) if you still
 * see tearing on a slower panel. */
#ifndef LCD_FRAME_DELAY_US
#define LCD_FRAME_DELAY_US 1
#endif

//16000

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
	lcd_fill_rect_silent(0, 0, LCD_WIDTH, LCD_HEIGHT, bg);

	timer_take_over();
	uint32_t window_start = timer_now();

	while(1) {
		uint32_t frame_start = timer_now();

		/* Draw the new rect first, then erase only the strips of the old
		 * rect that aren't covered by the new one (an L-shape). This keeps
		 * the rect visible on the panel at every instant - no flicker
		 * during the brief window between erase and draw. */
		lcd_dma_rect(x, y, rw, rh, lcd_dma_buf, sizeof(lcd_dma_buf));

		int16_t dx = x - prev_x;
		int16_t dy = y - prev_y;
		if (dx > 0)
			lcd_fill_rect_silent(prev_x, prev_y, dx, rh, bg);
		else if (dx < 0)
			lcd_fill_rect_silent(x + rw, prev_y, -dx, rh, bg);
		if (dy > 0)
			lcd_fill_rect_silent(prev_x, prev_y, rw, dy, bg);
		else if (dy < 0)
			lcd_fill_rect_silent(prev_x, y + rh, rw, -dy, bg);

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
			/* Window is at least 1s of cycles. fps ~= window_frames. */
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

static bool ft6336u_read(uint8_t reg, uint8_t *buf, unsigned int len)
{
	return i2c_read(FT6336U_I2C_ADDR, reg, buf, len, /* send_stop */ true,
	                /* addr_size */ 1);
}

static void ft6336u_probe(void)
{
	uint8_t chip_id = 0, vendor_id = 0, fw_ver = 0;

	log_puts("ft6336u: probing 0x"); log_hex8(FT6336U_I2C_ADDR); log_nl();
	if(!i2c_poll(FT6336U_I2C_ADDR)) {
		log_puts("ft6336u: no ACK at slave addr (check wiring / power / reset)");
		log_nl();
		return;
	}
	(void)ft6336u_read(0xa3, &chip_id,   1);
	(void)ft6336u_read(0xa8, &vendor_id, 1);
	(void)ft6336u_read(0xa6, &fw_ver,    1);
	log_puts("ft6336u: chip_id=0x");   log_hex8(chip_id);
	log_puts(" vendor_id=0x");         log_hex8(vendor_id);
	log_puts(" fw_ver=0x");            log_hex8(fw_ver);
	log_nl();
}

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
	/* FT6336U shares the LCD reset line. lcd_init's post-reset wait is
	 * enough for the LCD but the touch IC wants ~200ms total. */
	busy_wait(200);
	lcd_dma_buf_init_gradient();
	lcd_test_pattern();
	lcd_dma_self_test(LCD_WIDTH / 2 - LCD_DMA_TEST_W / 2,
	                  LCD_HEIGHT / 2 - LCD_DMA_TEST_H / 2);

	ft6336u_probe();
	ft6336u_touch_test(/* duration_ms */ 8000);

	log_puts("LCD static checks done; starting animation"); log_nl();
	busy_wait(1000);
	lcd_animate();

	return 0;
}
