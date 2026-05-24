#include <stdint.h>
#include <stdbool.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#include "lvgl.h"

#ifndef CSR_LCD_BASE
#error "Build the SoC with --with-lcd before compiling software/lvgl_demo."
#endif
#ifndef CTP_I2C_BASE
#error "Build the SoC with --with-lcd before compiling software/lvgl_demo (ctp_i2c missing)."
#endif

/* ---- FT6336U over hardware I2C ------------------------------------------ */

#define FT6336U_I2C_ADDR  0x38

#define CTP_I2C_REG(off) (*(volatile uint32_t *)(CTP_I2C_BASE + ((off) << 2)))
#define I2C_XFER_OFFSET     0
#define I2C_CONFIG_OFFSET   1
#define I2C_ACK    (1u << 8)
#define I2C_READ   (1u << 9)
#define I2C_WRITE  (1u << 10)
#define I2C_START  (1u << 11)
#define I2C_STOP   (1u << 12)
#define I2C_IDLE   (1u << 13)

/* LVGL's TLSF allocator uses __builtin_ffs(), which on RV32 without the B
 * extension expands to a libgcc helper we don't ship. Provide a portable
 * implementation so the linker is happy under -nostdlib -nodefaultlibs. */
int __ffssi2(int x);
int __ffssi2(int x)
{
	if(x == 0) return 0;
	int n = 1;
	if((x & 0x0000ffff) == 0) { n += 16; x >>= 16; }
	if((x & 0x000000ff) == 0) { n += 8;  x >>= 8;  }
	if((x & 0x0000000f) == 0) { n += 4;  x >>= 4;  }
	if((x & 0x00000003) == 0) { n += 2;  x >>= 2;  }
	if((x & 0x00000001) == 0) { n += 1; }
	return n;
}

/* ---- LCD engine CSR helpers ----------------------------------------------- */

#define LCD_PADS_RESET_N   (1u << CSR_LCD_PADS_CTRL_RESET_N_OFFSET)
#define LCD_PADS_BACKLIGHT (1u << CSR_LCD_PADS_CTRL_BACKLIGHT_OFFSET)

#define LCD_OP_START          (1u << CSR_LCD_OP_START_OFFSET)
#define LCD_OP_CMD            ((1u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_CMD_DATA_DMA   ((2u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_OP_DMA_RECT       ((5u << CSR_LCD_OP_KIND_OFFSET) | LCD_OP_START)
#define LCD_BUSY              (1u << CSR_LCD_STATUS_BUSY_OFFSET)
#define LCD_CAN_ACCEPT        (1u << CSR_LCD_STATUS_CAN_ACCEPT_OFFSET)

static uint32_t lcd_pads_state = LCD_PADS_RESET_N;

static void log_char(char c) { if(c == '\n') uart_write('\r'); uart_write(c); }
static void log_puts(const char *s) { while(*s) log_char(*s++); }
static void log_nl(void) { log_char('\n'); uart_sync(); }
static void log_hex4(uint8_t v) { v &= 0x0f; log_char(v < 10 ? '0' + v : 'a' + v - 10); }
static void log_hex8(uint8_t v) { log_hex4(v >> 4); log_hex4(v); }
static void log_hex16(uint16_t v) { log_hex8(v >> 8); log_hex8(v); }
static void log_hex32(uint32_t v) { log_hex16(v >> 16); log_hex16(v); }
static void log_uint(uint32_t v) {
	char buf[10]; unsigned int i = 0;
	if(v == 0) { log_char('0'); return; }
	while(v && i < sizeof(buf)) { buf[i++] = '0' + v % 10; v /= 10; }
	while(i) log_char(buf[--i]);
}

static void lcd_pads_apply(void) { lcd_pads_ctrl_write(lcd_pads_state); }
static void lcd_pads_set(uint32_t mask, int value) {
	if(value) lcd_pads_state |= mask; else lcd_pads_state &= ~mask;
	lcd_pads_apply();
}

static void lcd_wait_idle(void)       { while(lcd_status_read() & LCD_BUSY); }
static void lcd_wait_can_accept(void) { while(!(lcd_status_read() & LCD_CAN_ACCEPT)); }

static void lcd_reset_n(int v)   { lcd_pads_set(LCD_PADS_RESET_N,   v); }
static void lcd_backlight(int v) { lcd_pads_set(LCD_PADS_BACKLIGHT, v); }

static void lcd_write_cmd(uint8_t cmd) {
	lcd_wait_can_accept();
	lcd_cmd_byte_write(cmd);
	lcd_op_write(LCD_OP_CMD);
}

static void lcd_cmd_data(uint8_t cmd, const uint8_t *data, unsigned int len) {
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

static void lcd_hw_reset(void) {
	lcd_pads_state = LCD_PADS_RESET_N;
	lcd_pads_apply();
	busy_wait(20);
	lcd_reset_n(0); busy_wait(20);
	lcd_reset_n(1); busy_wait(120);
}

static void lcd_init(void) {
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
	lcd_write_cmd(0x01); busy_wait(120);

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

	lcd_write_cmd(0x11); busy_wait(120);
	lcd_write_cmd(0x29);
	lcd_wait_idle();
	busy_wait(20);
	lcd_backlight(1);
}

/* ---- Hardware I2C / FT6336U helpers --------------------------------------- */

static inline void ctp_i2c_wait_idle(void)
{
	while(!(CTP_I2C_REG(I2C_XFER_OFFSET) & I2C_IDLE))
		;
}

static void ctp_i2c_init(void)
{
	/* 100 kHz SCL at sys_clk; see lcd_test for the derivation. */
	uint32_t load = (CONFIG_CLOCK_FREQUENCY / (2u * 100000u)) - 1u;
	CTP_I2C_REG(I2C_CONFIG_OFFSET) = load;
}

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
	if(!ctp_i2c_addr(FT6336U_I2C_ADDR, 0)) { ctp_i2c_stop(); return false; }
	if(!ctp_i2c_write_byte(reg))           { ctp_i2c_stop(); return false; }
	if(!ctp_i2c_addr(FT6336U_I2C_ADDR, 1)) { ctp_i2c_stop(); return false; }
	for(unsigned int i = 0; i < len; i++)
		buf[i] = ctp_i2c_read_byte(i < (len - 1));
	ctp_i2c_stop();
	return true;
}

static bool ft6336u_present(void)
{
	int ack = ctp_i2c_addr(FT6336U_I2C_ADDR, 0);
	ctp_i2c_stop();
	return ack != 0;
}

static void ft6336u_probe(void)
{
	uint8_t chip_id = 0, vendor_id = 0, fw_ver = 0;
	log_puts("ft6336u: probing 0x"); log_hex8(FT6336U_I2C_ADDR); log_nl();
	if(!ft6336u_present()) {
		log_puts("ft6336u: NO ACK at slave addr"); log_nl();
		return;
	}
	(void)ft6336u_read(0xa3, &chip_id,   1);
	(void)ft6336u_read(0xa8, &vendor_id, 1);
	(void)ft6336u_read(0xa6, &fw_ver,    1);
	log_puts("ft6336u: chip_id=0x");  log_hex8(chip_id);
	log_puts(" vendor_id=0x");        log_hex8(vendor_id);
	log_puts(" fw_ver=0x");           log_hex8(fw_ver);
	log_nl();
}

/* ---- LVGL tick source ----------------------------------------------------- */
/* timer0 runs free counting down from 0xffffffff. We sample it on every
 * tick query and accumulate elapsed cycles into a ms counter that wraps
 * every ~50 days. Wrap of the underlying 32-bit cycle counter is handled
 * by the unsigned subtraction below (gives the elapsed cycles regardless
 * of wrap, as long as queries are less than 2^32 cycles apart). */

static uint32_t tick_prev_cycles;
static uint32_t tick_accum_ms;
static uint32_t tick_cycles_carry;

static uint32_t lvgl_tick_get_ms_cb(void) {
	timer0_update_value_write(1);
	uint32_t now   = timer0_value_read();
	uint32_t delta = (tick_prev_cycles - now) + tick_cycles_carry;
	tick_prev_cycles = now;

	uint32_t cycles_per_ms = CONFIG_CLOCK_FREQUENCY / 1000u;
	tick_accum_ms     += delta / cycles_per_ms;
	tick_cycles_carry  = delta % cycles_per_ms;
	return tick_accum_ms;
}

static void lvgl_tick_init(void) {
	timer0_en_write(0);
	timer0_load_write(0xffffffffu);
	timer0_reload_write(0xffffffffu);
	timer0_en_write(1);
	timer0_update_value_write(1);
	tick_prev_cycles  = timer0_value_read();
	tick_accum_ms     = 0;
	tick_cycles_carry = 0;
}

/* ---- LVGL display driver -------------------------------------------------- */

/* Two partial buffers in SDRAM. 320 cols x 40 rows x 2 bytes = 25.6 KB each.
 * Renderer draws into one while the other is being flushed. */
#define LV_BUF_LINES 40
static uint16_t lvgl_buf1[LCD_WIDTH * LV_BUF_LINES] __attribute__((aligned(4)));
static uint16_t lvgl_buf2[LCD_WIDTH * LV_BUF_LINES] __attribute__((aligned(4)));

/* Async flush: flush_cb queues the DMA and stashes the display pointer
 * here. The LCD op-done ISR uses it to signal flush_ready once the
 * engine has finished shifting the buffer out. With two partial buffers
 * + double-buffered render mode, LVGL renders into the second buffer
 * while the first is being transmitted. */
static volatile lv_display_t *flush_in_flight_disp;

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
	uint16_t x1 = (uint16_t)area->x1;
	uint16_t x2 = (uint16_t)area->x2;
	uint16_t y1 = (uint16_t)area->y1;
	uint16_t y2 = (uint16_t)area->y2;
	uint32_t w  = (uint32_t)(x2 - x1 + 1);
	uint32_t h  = (uint32_t)(y2 - y1 + 1);

	/* LVGL calls wait_for_flushing() before re-entering flush_cb, so the
	 * previous transfer is guaranteed complete and the engine queue is
	 * free by the time we get here. The wait_can_accept is paranoia
	 * cheap insurance. */
	lcd_wait_can_accept();
	lcd_rect_x_write(((uint32_t)x2 << 16) | (uint32_t)x1);
	lcd_rect_y_write(((uint32_t)y2 << 16) | (uint32_t)y1);
	lcd_dma_src_write((uint32_t)px_map);
	lcd_dma_len_write(w * h * 2u);

	flush_in_flight_disp = disp;
	lcd_op_write(LCD_OP_DMA_RECT);
	/* Return immediately. lv_display_flush_ready() is called from isr()
	 * when the engine raises its op-done event. */
}

/* ---- LVGL touch indev ---------------------------------------------------- */

/* FT6336U returns 12-bit X/Y already in panel pixel units (the IC is
 * factory-calibrated for the matched panel). For other orientations or
 * mismatched mountings, swap/flip here. We default to identity for
 * portrait mode. */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
	(void)indev;

	uint8_t buf[7];
	if(!ft6336u_read(0x02, buf, sizeof(buf))) {
		/* Rate-limited: real I2C trouble is rare, so flag bursts not
		 * individual misses. */
		static uint32_t fail_count;
		if((fail_count++ & 0xff) == 0)
			{ log_puts("touch: i2c read fail #"); log_uint(fail_count); log_nl(); }
		data->state = LV_INDEV_STATE_RELEASED;
		return;
	}

	unsigned int n = buf[0] & 0x0f;
	if(n == 0) {
		data->state = LV_INDEV_STATE_RELEASED;
		return;
	}
	unsigned int x = ((buf[1] & 0x0f) << 8) | buf[2];
	unsigned int y = ((buf[3] & 0x0f) << 8) | buf[4];

	if(x >= LCD_WIDTH)  x = LCD_WIDTH  - 1;
	if(y >= LCD_HEIGHT) y = LCD_HEIGHT - 1;
	data->point.x = (lv_coord_t)x;
	data->point.y = (lv_coord_t)y;
	data->state   = LV_INDEV_STATE_PRESSED;
}

/* ---- Demo UI ------------------------------------------------------------- */

static uint32_t click_count;
static lv_obj_t *click_label;

static lv_obj_t *anim_bar;
static lv_obj_t *anim_btn_label;
static bool      anim_running;

static void demo_button_event_cb(lv_event_t *e)
{
	(void)e;
	click_count++;
	lv_label_set_text_fmt(click_label, "Clicks: %u", (unsigned)click_count);
}

static void anim_set_x_cb(void *obj, int32_t v)
{
	lv_obj_set_x((lv_obj_t *)obj, (int32_t)v);
}

static void anim_start(void)
{
	int32_t travel = LCD_WIDTH - lv_obj_get_width(anim_bar);
	if(travel < 0) travel = 0;

	lv_anim_t a;
	lv_anim_init(&a);
	lv_anim_set_var(&a, anim_bar);
	lv_anim_set_exec_cb(&a, anim_set_x_cb);
	lv_anim_set_values(&a, 0, travel);
	lv_anim_set_duration(&a, 1500);
	lv_anim_set_playback_duration(&a, 1500);
	lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
	lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
	lv_anim_start(&a);
}

static void anim_stop(void)
{
	lv_anim_delete(anim_bar, anim_set_x_cb);
	lv_obj_set_x(anim_bar, 0);
}

static void anim_toggle_event_cb(lv_event_t *e)
{
	(void)e;
	anim_running = !anim_running;
	if(anim_running) {
		lv_label_set_text(anim_btn_label, "Animation: On");
		anim_start();
	} else {
		lv_label_set_text(anim_btn_label, "Animation: Off");
		anim_stop();
	}
}

static void demo_build_ui(void)
{
	lv_obj_t *root = lv_screen_active();

	lv_obj_t *title = lv_label_create(root);
	lv_label_set_text(title, "IcePi Zero + LVGL");
	lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

	lv_obj_t *btn = lv_button_create(root);
	lv_obj_set_size(btn, 200, 60);
	lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 70);
	lv_obj_add_event_cb(btn, demo_button_event_cb, LV_EVENT_CLICKED, NULL);

	lv_obj_t *btn_label = lv_label_create(btn);
	lv_label_set_text(btn_label, "Tap me");
	lv_obj_center(btn_label);

	click_label = lv_label_create(root);
	lv_label_set_text(click_label, "Clicks: 0");
	lv_obj_align(click_label, LV_ALIGN_TOP_MID, 0, 140);

	lv_obj_t *anim_btn = lv_button_create(root);
	lv_obj_set_size(anim_btn, 200, 60);
	lv_obj_align(anim_btn, LV_ALIGN_TOP_MID, 0, 180);
	lv_obj_add_event_cb(anim_btn, anim_toggle_event_cb, LV_EVENT_CLICKED, NULL);

	anim_btn_label = lv_label_create(anim_btn);
	lv_label_set_text(anim_btn_label, "Animation: Off");
	lv_obj_center(anim_btn_label);

	/* The bouncing bar that the toggle controls. */
	anim_bar = lv_obj_create(root);
	lv_obj_set_size(anim_bar, 60, 40);
	lv_obj_align(anim_bar, LV_ALIGN_BOTTOM_LEFT, 0, -40);
	lv_obj_set_style_bg_color(anim_bar, lv_palette_main(LV_PALETTE_BLUE), 0);
	lv_obj_set_style_border_width(anim_bar, 0, 0);
	lv_obj_set_style_radius(anim_bar, 6, 0);
}

/* ---- LCD op-done ISR ----------------------------------------------------- */

/* Installed via irq_attach() so it slots into the standard libbase isr()
 * dispatcher. Fires once per completed engine op; signals flush_ready
 * for whichever buffer was in flight. */
static void lcd_op_done_isr(void)
{
	lcd_ev_pending_write(lcd_ev_pending_read());  /* W1C clear */
	lv_display_t *d = (lv_display_t *)flush_in_flight_disp;
	if(d) {
		flush_in_flight_disp = NULL;
		lv_display_flush_ready(d);
	}
}

/* ---- main ----------------------------------------------------------------- */

int main(void) {
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("LVGL demo starting"); log_nl();
	log_puts("sys clk : "); log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();
	log_puts("LCD size: "); log_uint(LCD_WIDTH); log_puts(" x "); log_uint(LCD_HEIGHT); log_nl();
	log_puts("CSR lcd base=0x"); log_hex32(CSR_LCD_BASE); log_nl();

	lcd_pads_apply();
	lcd_init();
	/* FT6336U shares the LCD reset line; the touch IC wants ~200 ms
	 * post-reset before responding to I2C. */
	busy_wait(200);
	ctp_i2c_init();
	ft6336u_probe();
	log_puts("LCD + touch ready"); log_nl();

	lvgl_tick_init();
	lv_init();
	lv_tick_set_cb(lvgl_tick_get_ms_cb);

	lv_display_t *disp = lv_display_create(LCD_WIDTH, LCD_HEIGHT);
	lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
	lv_display_set_buffers(disp, lvgl_buf1, lvgl_buf2,
	                       sizeof(lvgl_buf1),
	                       LV_DISPLAY_RENDER_MODE_PARTIAL);
	lv_display_set_flush_cb(disp, lvgl_flush_cb);

	/* Enable the LCD engine's op-done event, attach our ISR to libbase's
	 * dispatcher, and unmask the IRQ at the CPU. The pending bit fires
	 * once per completed op; the ISR signals lv_display_flush_ready for
	 * whichever buffer was in flight. */
	lcd_ev_pending_write(lcd_ev_pending_read());  /* drop any stale pending */
	lcd_ev_enable_write(1);
	irq_attach(LCD_INTERRUPT, lcd_op_done_isr);
	irq_setmask(irq_getmask() | (1u << LCD_INTERRUPT));

	lv_indev_t *indev = lv_indev_create();
	lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
	lv_indev_set_read_cb(indev, lvgl_touch_read_cb);

	demo_build_ui();

	log_puts("LVGL ready, entering main loop"); log_nl();

	/* NOTE: busy_wait() reloads timer0, which we're using as the LVGL
	 * tick source. Don't call it after lvgl_tick_init(). Just spin on
	 * lv_timer_handler() - the returned delay is advisory and the CPU
	 * has plenty of slack at 50 MHz for the polling loop. */
	for(;;) {
		(void)lv_timer_handler();
	}

	return 0;
}
