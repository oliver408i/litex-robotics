#include <stdint.h>
#include <stdbool.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#ifndef CSR_LCD_BASE
#error "Build the SoC with --with-lcd before compiling software/lcd_test."
#endif
#ifndef CTP_I2C_BASE
#error "Build the SoC with --with-lcd before compiling software/lcd_test (ctp_i2c missing)."
#endif

#define FT6336U_I2C_ADDR  0x38

/* HW I2C master (wishbone-mapped at CTP_I2C_BASE). Two word registers:
 *   offset 0: XFER  - write to issue start/stop/read/write commands;
 *                     read for received data byte and slave-ACK bit.
 *   offset 1: CONFIG - clock divider for SCL.
 */
#define CTP_I2C_REG(off) (*(volatile uint32_t *)(CTP_I2C_BASE + ((off) << 2)))
#define I2C_XFER_OFFSET     0
#define I2C_CONFIG_OFFSET   1

#define I2C_ACK    (1u << 8)
#define I2C_READ   (1u << 9)
#define I2C_WRITE  (1u << 10)
#define I2C_START  (1u << 11)
#define I2C_STOP   (1u << 12)
#define I2C_IDLE   (1u << 13)

#define LCD_PADS_RESET_N   (1u << CSR_LCD_PADS_CTRL_RESET_N_OFFSET)
#define LCD_PADS_BACKLIGHT (1u << CSR_LCD_PADS_CTRL_BACKLIGHT_OFFSET)

#define LCD_OP_START          (1u << CSR_LCD_OP_START_OFFSET)
#define LCD_OP_CMD            ((1u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_CMD_DATA_DMA   ((2u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_CMD_DATA_FILL  ((3u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_FILL_RECT      ((4u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_DMA_RECT       ((5u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_BUSY              (1u << CSR_LCD_STATUS_BUSY_OFFSET)
#define LCD_CAN_ACCEPT        (1u << CSR_LCD_STATUS_CAN_ACCEPT_OFFSET)

static uint32_t lcd_pads_state = LCD_PADS_RESET_N;

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

/* Wait until the engine's queue slot is free. Returns as soon as the
 * next op can be staged, even if a previous op is still shifting on
 * SPI. Use this on the hot path instead of lcd_wait_idle so CPU CSR
 * setup for op N+1 overlaps with the engine running op N. */
static void lcd_wait_can_accept(void)
{
	while(!(lcd_status_read() & LCD_CAN_ACCEPT))
		;
}

static void lcd_reset_n(int v) { lcd_pads_set(LCD_PADS_RESET_N, v); }
static void lcd_backlight(int v){ lcd_pads_set(LCD_PADS_BACKLIGHT, v); }

static void lcd_write_cmd(uint8_t cmd)
{
	lcd_wait_can_accept();
	lcd_cmd_byte_write(cmd);
	lcd_op_write(LCD_OP_CMD);
}

static void lcd_cmd_data(uint8_t cmd, const uint8_t *data, unsigned int len)
{
	lcd_wait_can_accept();
	lcd_cmd_byte_write(cmd);
	if(len) {
		lcd_dma_src_write((uint32_t)data);
		lcd_dma_len_write(len);
		lcd_op_write(LCD_OP_CMD_DATA_DMA);
	} else {
		lcd_op_write(LCD_OP_CMD);
	}
}

static void lcd_hw_reset(void)
{
	log_puts("lcd: hardware reset"); log_nl();
	lcd_pads_state = LCD_PADS_RESET_N;
	lcd_pads_apply();
	busy_wait(20);
	lcd_reset_n(0);
	busy_wait(20);
	lcd_reset_n(1);
	busy_wait(120);
}

static void lcd_fill_rect_silent(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	uint16_t x1 = (uint16_t)(x + w - 1);
	uint16_t y1 = (uint16_t)(y + h - 1);

	lcd_wait_can_accept();
	lcd_rect_x_write(((uint32_t)x1 << 16) | (uint32_t)(uint16_t)x);
	lcd_rect_y_write(((uint32_t)y1 << 16) | (uint32_t)(uint16_t)y);
	lcd_fill_color_write(color);
	lcd_fill_count_write((uint32_t)w * (uint32_t)h);
	lcd_op_write(LCD_OP_FILL_RECT);
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	log_puts("lcd: fill x="); log_uint(x);
	log_puts(" y="); log_uint(y);
	log_puts(" w="); log_uint(w);
	log_puts(" h="); log_uint(h);
	log_puts(" color=0x"); log_hex16(color); log_nl();

	lcd_fill_rect_silent((int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h, color);
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
	lcd_wait_idle();   /* ensure 0x29 has reached the panel before backlight */
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

static void lcd_dma_rect(int16_t x, int16_t y, int16_t w, int16_t h, const void *buf, uint32_t len)
{
	uint16_t x1 = (uint16_t)(x + w - 1);
	uint16_t y1 = (uint16_t)(y + h - 1);

	lcd_wait_can_accept();
	lcd_rect_x_write(((uint32_t)x1 << 16) | (uint32_t)(uint16_t)x);
	lcd_rect_y_write(((uint32_t)y1 << 16) | (uint32_t)(uint16_t)y);
	lcd_dma_src_write((uint32_t)buf);
	lcd_dma_len_write(len);
	lcd_op_write(LCD_OP_DMA_RECT);
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

/* ---- Hardware I2C driver (FT6336U) ----------------------------------- */

static inline void ctp_i2c_wait_idle(void)
{
	while(!(CTP_I2C_REG(I2C_XFER_OFFSET) & I2C_IDLE))
		;
}

static void ctp_i2c_init(void)
{
	/* SCL period = 2*(load+1) sys cycles per half-bit; the state machine
	 * also takes multiple ticks per logical bit, so the actual SCL rate
	 * is ~sys_clk / (2*(load+1)). For 100 kHz at 50 MHz sys, load = 249. */
	uint32_t load = (CONFIG_CLOCK_FREQUENCY / (2u * 100000u)) - 1u;
	CTP_I2C_REG(I2C_CONFIG_OFFSET) = load;
}

/* Issue START followed by the address byte. Returns 1 if the slave ACKed. */
static int ctp_i2c_addr(uint8_t addr7, int read)
{
	ctp_i2c_wait_idle();
	CTP_I2C_REG(I2C_XFER_OFFSET) = I2C_START;
	ctp_i2c_wait_idle();
	uint32_t byte = (uint32_t)((addr7 << 1) | (read ? 1u : 0u));
	CTP_I2C_REG(I2C_XFER_OFFSET) = I2C_WRITE | byte;
	ctp_i2c_wait_idle();
	return (CTP_I2C_REG(I2C_XFER_OFFSET) & I2C_ACK) != 0;
}

static int ctp_i2c_write_byte(uint8_t b)
{
	ctp_i2c_wait_idle();
	CTP_I2C_REG(I2C_XFER_OFFSET) = I2C_WRITE | (uint32_t)b;
	ctp_i2c_wait_idle();
	return (CTP_I2C_REG(I2C_XFER_OFFSET) & I2C_ACK) != 0;
}

/* `ack` = whether we ACK the byte we're about to receive (ACK = continue
 * reading, NACK = last byte before STOP). */
static uint8_t ctp_i2c_read_byte(int ack)
{
	ctp_i2c_wait_idle();
	CTP_I2C_REG(I2C_XFER_OFFSET) = I2C_READ | (ack ? I2C_ACK : 0u);
	ctp_i2c_wait_idle();
	return (uint8_t)(CTP_I2C_REG(I2C_XFER_OFFSET) & 0xff);
}

static void ctp_i2c_stop(void)
{
	ctp_i2c_wait_idle();
	CTP_I2C_REG(I2C_XFER_OFFSET) = I2C_STOP;
	ctp_i2c_wait_idle();
}

static bool ft6336u_read(uint8_t reg, uint8_t *buf, unsigned int len)
{
	if(!ctp_i2c_addr(FT6336U_I2C_ADDR, /* read */ 0)) {
		ctp_i2c_stop();
		return false;
	}
	if(!ctp_i2c_write_byte(reg)) {
		ctp_i2c_stop();
		return false;
	}
	if(!ctp_i2c_addr(FT6336U_I2C_ADDR, /* read */ 1)) {
		ctp_i2c_stop();
		return false;
	}
	for(unsigned int i = 0; i < len; i++)
		buf[i] = ctp_i2c_read_byte(i < (len - 1));
	ctp_i2c_stop();
	return true;
}

/* Address-poll: send START + address-write, NACK -> not present. */
static bool ft6336u_present(void)
{
	int ack = ctp_i2c_addr(FT6336U_I2C_ADDR, /* read */ 0);
	ctp_i2c_stop();
	return ack != 0;
}

static void ft6336u_probe(void)
{
	uint8_t chip_id = 0, vendor_id = 0, fw_ver = 0;

	log_puts("ft6336u: probing 0x"); log_hex8(FT6336U_I2C_ADDR); log_nl();
	if(!ft6336u_present()) {
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

	ctp_i2c_init();
	ft6336u_probe();
	ft6336u_touch_test(/* duration_ms */ 8000);

	log_puts("LCD static checks done; starting animation"); log_nl();
	busy_wait(1000);
	lcd_animate();

	return 0;
}
