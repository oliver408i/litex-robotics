#include <stdint.h>
#include <stdbool.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "lvgl.h"

#include "lcd.h"
#include "touch.h"
#include "log.h"

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

/* LVGL sysmon idle-percent hook (lv_conf.h points LV_SYSMON_GET_IDLE here).
 * Bare-metal with a tight spin loop has no meaningful idle measure, so just
 * report 0. The FPS and memory stats in the perf overlay are still real. */
uint32_t bm_get_idle_percent(void);
uint32_t bm_get_idle_percent(void) { return 0; }

/* ---- LVGL tick source ----------------------------------------------------- */
/* timer0 runs free counting down from 0xffffffff. We sample it on every
 * tick query and accumulate elapsed cycles into a ms counter that wraps
 * every ~50 days. Wrap of the underlying 32-bit cycle counter is handled
 * by the unsigned subtraction below. */

static uint32_t tick_prev_cycles;
static uint32_t tick_accum_ms;
static uint32_t tick_cycles_carry;

static uint32_t lvgl_tick_get_ms_cb(void)
{
	timer0_update_value_write(1);
	uint32_t now   = timer0_value_read();
	uint32_t delta = (tick_prev_cycles - now) + tick_cycles_carry;
	tick_prev_cycles = now;

	uint32_t cycles_per_ms = CONFIG_CLOCK_FREQUENCY / 1000u;
	tick_accum_ms     += delta / cycles_per_ms;
	tick_cycles_carry  = delta % cycles_per_ms;
	return tick_accum_ms;
}

static void lvgl_tick_init(void)
{
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

/* Two full-screen framebuffers in SDRAM. 320 x 480 x 2 bytes = 307.2 KB
 * each, 614 KB total - a rounding error in the 32 MB SDRAM. With direct
 * render mode, LVGL only repaints dirty pixels into the persistent FB
 * rather than re-rasterizing everything beneath each dirty rect. */
static uint16_t lvgl_fb1[LCD_WIDTH * LCD_HEIGHT] __attribute__((aligned(4)));
static uint16_t lvgl_fb2[LCD_WIDTH * LCD_HEIGHT] __attribute__((aligned(4)));

/* Async flush: flush_cb queues the DMA and stashes the display pointer
 * here. The LCD op-done ISR uses it to signal flush_ready once the
 * engine has finished shifting the buffer out. */
static volatile lv_display_t *flush_in_flight_disp;

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
	uint16_t x1 = (uint16_t)area->x1;
	uint16_t x2 = (uint16_t)area->x2;
	uint16_t y1 = (uint16_t)area->y1;
	uint16_t y2 = (uint16_t)area->y2;
	uint32_t w  = (uint32_t)(x2 - x1 + 1);
	uint32_t h  = (uint32_t)(y2 - y1 + 1);

	/* Direct mode: px_map is the base of the full framebuffer; the
	 * dirty area's pixels live as h horizontal slices at offset
	 * (y1*LCD_WIDTH + x1)*2 bytes, each 2*w bytes long, separated by a
	 * 2*LCD_WIDTH byte stride. The engine's stride-aware DMA walks
	 * that layout directly. */
	uint32_t src_off  = ((uint32_t)y1 * LCD_WIDTH + x1) * 2u;
	uint32_t row_b    = w * 2u;
	uint32_t stride_b = (uint32_t)LCD_WIDTH * 2u;

	lcd_wait_can_accept();
	lcd_rect_x_write(((uint32_t)x2 << 16) | (uint32_t)x1);
	lcd_rect_y_write(((uint32_t)y2 << 16) | (uint32_t)y1);
	lcd_dma_src_write((uint32_t)px_map + src_off);
	lcd_dma_row_bytes_write(row_b);
	lcd_dma_row_count_write(h);
	lcd_dma_stride_write(stride_b);

	flush_in_flight_disp = disp;
	lcd_op_write(LCD_OP_DMA_RECT);
	/* Return immediately. lv_display_flush_ready() is called from the
	 * op-done ISR when the engine signals completion. */
}

/* ---- LVGL touch indev ---------------------------------------------------- */

/* FT6336U returns 12-bit X/Y already in panel pixel units (the IC is
 * factory-calibrated for the matched panel). For other orientations or
 * mismatched mountings, swap/flip here. */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
	(void)indev;

	uint8_t buf[7];
	if(!ft6336u_read(0x02, buf, sizeof(buf))) {
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

/* Tuned for visible-but-usable stress on a 50 MHz VexRiscv with SDRAM:
 *  - N_BALLS=6 keeps touch responsive during heavy redraws. Crank up to
 *    10-16 to see the renderer choke (FPS drops well into single digits,
 *    button presses start needing multiple attempts).
 *  - TICK_MS=25 is ~40 Hz movement updates. 16 ms (60 Hz) is smoother
 *    but doubles the per-second render load.
 *  - Square balls (radius=0) render via simple solid fills - no AA edge
 *    math per pixel. Round balls look nicer but cost ~3x on this CPU. */
#define STRESS_N_BALLS  6
#define STRESS_BALL_SZ  28
#define STRESS_TICK_MS  25
static lv_obj_t   *stress_balls[STRESS_N_BALLS];
static int16_t     stress_x[STRESS_N_BALLS], stress_y[STRESS_N_BALLS];
static int16_t     stress_vx[STRESS_N_BALLS], stress_vy[STRESS_N_BALLS];
static lv_timer_t *stress_timer;
static lv_obj_t   *stress_btn_label;
static bool        stress_running;

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

/* Stress test: N balls bouncing all over the screen. */

static void stress_tick_cb(lv_timer_t *t)
{
	(void)t;
	const int16_t sz = STRESS_BALL_SZ;
	for(int i = 0; i < STRESS_N_BALLS; i++) {
		stress_x[i] += stress_vx[i];
		stress_y[i] += stress_vy[i];
		if(stress_x[i] < 0)               { stress_x[i] = 0;               stress_vx[i] = -stress_vx[i]; }
		if(stress_x[i] + sz > LCD_WIDTH)  { stress_x[i] = LCD_WIDTH  - sz; stress_vx[i] = -stress_vx[i]; }
		if(stress_y[i] < 0)               { stress_y[i] = 0;               stress_vy[i] = -stress_vy[i]; }
		if(stress_y[i] + sz > LCD_HEIGHT) { stress_y[i] = LCD_HEIGHT - sz; stress_vy[i] = -stress_vy[i]; }
		lv_obj_set_pos(stress_balls[i], stress_x[i], stress_y[i]);
	}
}

static void stress_start(void)
{
	static const lv_palette_t palette[] = {
		LV_PALETTE_RED,    LV_PALETTE_BLUE,   LV_PALETTE_GREEN,
		LV_PALETTE_ORANGE, LV_PALETTE_PURPLE, LV_PALETTE_CYAN,
		LV_PALETTE_PINK,   LV_PALETTE_TEAL,   LV_PALETTE_AMBER,
		LV_PALETTE_INDIGO,
	};
	const int n_palette = (int)(sizeof(palette) / sizeof(palette[0]));

	lv_obj_t *root = lv_screen_active();
	const int16_t sz = STRESS_BALL_SZ;

	for(int i = 0; i < STRESS_N_BALLS; i++) {
		lv_obj_t *o = lv_obj_create(root);
		lv_obj_set_size(o, sz, sz);
		lv_obj_set_style_bg_color(o, lv_palette_main(palette[i % n_palette]), 0);
		lv_obj_set_style_border_width(o, 0, 0);
		lv_obj_set_style_radius(o, 0, 0);
		lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
		stress_balls[i] = o;

		/* Cheap deterministic spread; no PRNG needed. */
		stress_x[i]  = (int16_t)((i * 31) % (LCD_WIDTH  - sz));
		stress_y[i]  = (int16_t)((i * 47) % (LCD_HEIGHT - sz));
		stress_vx[i] = (int16_t)(((i & 3) + 1) * ((i & 1) ? 1 : -1));
		stress_vy[i] = (int16_t)((((i >> 1) & 3) + 1) * ((i & 2) ? 1 : -1));
		lv_obj_set_pos(o, stress_x[i], stress_y[i]);
	}

	stress_timer = lv_timer_create(stress_tick_cb, STRESS_TICK_MS, NULL);
}

static void stress_stop(void)
{
	if(stress_timer) {
		lv_timer_delete(stress_timer);
		stress_timer = NULL;
	}
	for(int i = 0; i < STRESS_N_BALLS; i++) {
		if(stress_balls[i]) {
			lv_obj_delete(stress_balls[i]);
			stress_balls[i] = NULL;
		}
	}
}

static void stress_toggle_event_cb(lv_event_t *e)
{
	(void)e;
	stress_running = !stress_running;
	if(stress_running) {
		lv_label_set_text(stress_btn_label, "Stress: On");
		stress_start();
	} else {
		lv_label_set_text(stress_btn_label, "Stress: Off");
		stress_stop();
	}
}

static void demo_build_ui(void)
{
	lv_obj_t *root = lv_screen_active();
	/* The screen shouldn't scroll under stress-test balls. */
	lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);

	lv_obj_t *title = lv_label_create(root);
	lv_label_set_text(title, "IcePi Zero + LVGL");
	lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

	lv_obj_t *btn = lv_button_create(root);
	lv_obj_set_size(btn, 200, 55);
	lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 50);
	lv_obj_add_event_cb(btn, demo_button_event_cb, LV_EVENT_CLICKED, NULL);

	lv_obj_t *btn_label = lv_label_create(btn);
	lv_label_set_text(btn_label, "Tap me");
	lv_obj_center(btn_label);

	click_label = lv_label_create(root);
	lv_label_set_text(click_label, "Clicks: 0");
	lv_obj_align(click_label, LV_ALIGN_TOP_MID, 0, 115);

	lv_obj_t *anim_btn = lv_button_create(root);
	lv_obj_set_size(anim_btn, 200, 55);
	lv_obj_align(anim_btn, LV_ALIGN_TOP_MID, 0, 145);
	lv_obj_add_event_cb(anim_btn, anim_toggle_event_cb, LV_EVENT_CLICKED, NULL);

	anim_btn_label = lv_label_create(anim_btn);
	lv_label_set_text(anim_btn_label, "Animation: Off");
	lv_obj_center(anim_btn_label);

	/* Bouncing bar that the animation toggle controls. */
	anim_bar = lv_obj_create(root);
	lv_obj_set_size(anim_bar, 60, 36);
	lv_obj_align(anim_bar, LV_ALIGN_TOP_LEFT, 0, 215);
	lv_obj_set_style_bg_color(anim_bar, lv_palette_main(LV_PALETTE_BLUE), 0);
	lv_obj_set_style_border_width(anim_bar, 0, 0);
	lv_obj_set_style_radius(anim_bar, 6, 0);
	lv_obj_remove_flag(anim_bar, LV_OBJ_FLAG_SCROLLABLE);

	lv_obj_t *stress_btn = lv_button_create(root);
	lv_obj_set_size(stress_btn, 200, 55);
	lv_obj_align(stress_btn, LV_ALIGN_TOP_MID, 0, 265);
	lv_obj_add_event_cb(stress_btn, stress_toggle_event_cb, LV_EVENT_CLICKED, NULL);

	stress_btn_label = lv_label_create(stress_btn);
	lv_label_set_text(stress_btn_label, "Stress: Off");
	lv_obj_center(stress_btn_label);
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

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("LVGL demo starting"); log_nl();
	log_puts("sys clk : "); log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();
	log_puts("LCD size: "); log_uint(LCD_WIDTH); log_puts(" x "); log_uint(LCD_HEIGHT); log_nl();

	lcd_pads_apply();
	lcd_init();
	/* FT6336U shares the LCD reset line; touch IC wants ~200 ms
	 * post-reset before responding to I2C. */
	busy_wait(200);
	touch_init();
	ft6336u_probe();
	log_puts("LCD + touch ready"); log_nl();

	lvgl_tick_init();
	lv_init();
	lv_tick_set_cb(lvgl_tick_get_ms_cb);

	lv_display_t *disp = lv_display_create(LCD_WIDTH, LCD_HEIGHT);
	lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
	lv_display_set_buffers(disp, lvgl_fb1, lvgl_fb2,
	                       sizeof(lvgl_fb1),
	                       LV_DISPLAY_RENDER_MODE_DIRECT);
	lv_display_set_flush_cb(disp, lvgl_flush_cb);

	/* Enable the LCD engine's op-done event, attach the ISR to libbase's
	 * dispatcher, and unmask the IRQ at the CPU. */
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
	 * has plenty of slack at the demo workload. */
	for(;;) {
		(void)lv_timer_handler();
	}

	return 0;
}
