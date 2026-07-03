/* IcePi Zero field data logger -- on-FPGA LVGL app (lv_tabview).
 *
 * Pairs with icepi_zero_logger.py (WINC + LCD/touch + SD + IMU + GPS, no SNN).
 *
 * Tabs:
 *   - Logger : IMU -> SD logging status (state, file, samples/bytes/overruns)
 *              with a Start/Stop button. Engine = common/imu_log.c (LSM6DS3
 *              FIFO -> FatFs), polled from the main loop.
 *   - GPS    : NMEA GPS bring-up readout (link, fix, sats, lat/lon, UTC, speed
 *              + sentence counters). Reader = common/gps_nmea.c on the second
 *              UART. Once a fix is confirmed here, GPS records (IMU_REC_GPS) get
 *              folded into the SD log.
 *   - Power  : 2S LiPo voltage via the MCP3008 aux ADC (common/power.c).
 *
 * The LVGL display/touch/tick glue is copied verbatim from mnist_lcd_demo
 * (which copied it from software/lvgl_demo); the MNIST/SNN tab and drawing
 * canvas are dropped (this SoC has no SNN).
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#include "lvgl.h"

#include "lcd.h"
#include "touch.h"
#include "log.h"
#include "adc.h"
#include "power.h"
#include "imu_log.h"
#include "gps_nmea.h"

/* LVGL's TLSF allocator uses __builtin_ffs(); RV32 without the B extension
 * needs this libgcc helper we don't ship. (Same shim as mnist_lcd_demo.) */
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

uint32_t bm_get_idle_percent(void);
uint32_t bm_get_idle_percent(void) { return 0; }

/* ======================================================================== */
/*  LVGL tick source -- timer0 free-running countdown                        */
/* ======================================================================== */

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

/* The SD card driver calls busy_wait(), which repurposes timer0 and leaves it
 * stopped at 0 -- killing the LVGL tick (it reads timer0). Call this after any
 * SD operation to re-arm timer0 free-running and re-anchor the tick WITHOUT
 * resetting tick_accum_ms. The wall time spent inside the SD op isn't counted
 * (invisible to the UI; log timestamps under-count by the flush time). */
static void lvgl_tick_resync(void)
{
	timer0_en_write(0);
	timer0_load_write(0xffffffffu);
	timer0_reload_write(0xffffffffu);
	timer0_en_write(1);
	timer0_update_value_write(1);
	tick_prev_cycles  = timer0_value_read();
	tick_cycles_carry = 0;
}

/* ======================================================================== */
/*  LVGL display driver -- synchronous DMA flush                             */
/* ======================================================================== */

static uint16_t lvgl_fb1[LCD_WIDTH * LCD_HEIGHT] __attribute__((aligned(4)));
static uint16_t lvgl_fb2[LCD_WIDTH * LCD_HEIGHT] __attribute__((aligned(4)));

/* Fire the DMA op, drain the engine, THEN signal ready: LVGL can never touch a
 * region the engine is still reading (the multi-area async race; see the long
 * note in mnist_lcd_demo/main.c). Cost is negligible for widget redraws. */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
	uint16_t x1 = (uint16_t)area->x1;
	uint16_t x2 = (uint16_t)area->x2;
	uint16_t y1 = (uint16_t)area->y1;
	uint16_t y2 = (uint16_t)area->y2;
	uint32_t w  = (uint32_t)(x2 - x1 + 1);
	uint32_t h  = (uint32_t)(y2 - y1 + 1);

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

	lcd_op_write(LCD_OP_DMA_RECT);

	lcd_wait_idle();                 /* drain before releasing the buffer to LVGL */
	lv_display_flush_ready(disp);
}

/* ======================================================================== */
/*  LVGL touch indev                                                         */
/* ======================================================================== */

static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
	(void)indev;

	uint8_t buf[7];
	bool ok = ft6336u_read(0x02, buf, sizeof(buf));

#ifdef TOUCH_DEBUG
	/* Diagnostic: log what the FT6336U reports. Throttled -- print whenever the
	 * touch-point count changes, plus a ~2 s idle heartbeat so we can see the
	 * callback is running and the controller is being read. */
	{
		static uint32_t last_log;
		static int      last_n = -1;
		int      n_dbg = ok ? (int)(buf[0] & 0x0f) : -2;   /* -2 = I2C read failed */
		uint32_t now   = lvgl_tick_get_ms_cb();
		if(n_dbg != last_n || (now - last_log) > 2000u) {
			log_puts("touch: ok="); log_uint(ok);
			log_puts(" td=0x");     log_hex8(ok ? buf[0] : 0);
			log_puts(" n=");        log_uint((unsigned)(ok ? (buf[0] & 0x0f) : 0));
			if(ok && (buf[0] & 0x0f)) {
				unsigned x = ((buf[1] & 0x0f) << 8) | buf[2];
				unsigned y = ((buf[3] & 0x0f) << 8) | buf[4];
				log_puts(" x="); log_uint(x);
				log_puts(" y="); log_uint(y);
			}
			log_nl();
			last_log = now;
			last_n   = n_dbg;
		}
	}
#endif

	if(!ok) {
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

#define TAB_BAR_H 32

/* ======================================================================== */
/*  Logger tab: IMU->SD status + Start/Stop                                  */
/* ======================================================================== */

static lv_obj_t *lgr_state_label;
static lv_obj_t *lgr_file_label;
static lv_obj_t *lgr_stat_label;
static lv_obj_t *lgr_btn_label;

static void logger_timer_cb(lv_timer_t *t)
{
	(void)t;
	const imu_log_status_t *st = imu_log_get_status();

	const char *state_str = (st->state == IMU_LOG_RUNNING) ? "LOGGING"
	                      : (st->state == IMU_LOG_IDLE)     ? "idle"
	                                                        : "ERROR";
	lv_color_t col = (st->state == IMU_LOG_RUNNING) ? lv_palette_main(LV_PALETTE_GREEN)
	               : (st->state == IMU_LOG_ERROR)   ? lv_palette_main(LV_PALETTE_RED)
	                                                : lv_palette_main(LV_PALETTE_GREY);
	lv_label_set_text(lgr_state_label, state_str);
	lv_obj_set_style_text_color(lgr_state_label, col, 0);

	if(st->state == IMU_LOG_ERROR)
		lv_label_set_text_fmt(lgr_file_label, "err %d   imu:%s",
		                      st->last_err, st->imu_ok ? "ok" : "FAIL");
	else
		lv_label_set_text_fmt(lgr_file_label, "%s", st->filename);

	lv_label_set_text_fmt(lgr_stat_label, "samples: %u\nbytes: %u\noverruns: %u",
	                      (unsigned)st->samples, (unsigned)st->bytes,
	                      (unsigned)st->overruns);

	lv_label_set_text(lgr_btn_label,
	                  st->state == IMU_LOG_RUNNING ? "STOP" : "START");
}

static void logger_btn_event_cb(lv_event_t *e)
{
	(void)e;
	const imu_log_status_t *st = imu_log_get_status();
	if(st->state == IMU_LOG_RUNNING)
		imu_log_stop();
	else
		imu_log_start(lvgl_tick_get_ms_cb());
	/* Both paths hit the SD card (busy_wait -> timer0 clobbered). */
	lvgl_tick_resync();
}

static void build_logger_tab(lv_obj_t *parent)
{
	lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_pad_all(parent, 12, 0);
	lv_obj_set_style_bg_color(parent, lv_color_hex(0x202020), 0);
	lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_SPACE_EVENLY,
	                      LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	lv_obj_t *cap = lv_label_create(parent);
	lv_obj_set_style_text_color(cap, lv_palette_main(LV_PALETTE_GREY), 0);
	lv_label_set_text(cap, "IMU LOGGER");

	lgr_state_label = lv_label_create(parent);
	lv_obj_set_style_text_font(lgr_state_label, &lv_font_montserrat_28, 0);
	lv_obj_set_style_text_color(lgr_state_label, lv_palette_main(LV_PALETTE_GREY), 0);
	lv_label_set_text(lgr_state_label, "...");

	lgr_file_label = lv_label_create(parent);
	lv_obj_set_style_text_color(lgr_file_label, lv_color_white(), 0);
	lv_label_set_text(lgr_file_label, "--");

	lgr_stat_label = lv_label_create(parent);
	lv_obj_set_style_text_align(lgr_stat_label, LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_style_text_color(lgr_stat_label, lv_palette_main(LV_PALETTE_GREY), 0);
	lv_label_set_text(lgr_stat_label, "samples: --\nbytes: --\noverruns: --");

	lv_obj_t *btn = lv_button_create(parent);
	lv_obj_add_event_cb(btn, logger_btn_event_cb, LV_EVENT_CLICKED, NULL);
	lgr_btn_label = lv_label_create(btn);
	lv_label_set_text(lgr_btn_label, "START");
	lv_obj_center(lgr_btn_label);

	lv_timer_create(logger_timer_cb, 500, NULL);
}

/* ======================================================================== */
/*  GPS tab: NMEA bring-up readout                                           */
/* ======================================================================== */

#define GPS_LINK_TIMEOUT_MS 3000

static lv_obj_t *gps_link_label;
static lv_obj_t *gps_fix_label;
static lv_obj_t *gps_pos_label;
static lv_obj_t *gps_aux_label;
static lv_obj_t *gps_cnt_label;
static lv_obj_t *gps_raw_label;

/* Signed microdegrees -> "-?D.dddddd" into buf. */
static void fmt_udeg(char *buf, size_t n, int32_t udeg)
{
	int32_t a = udeg < 0 ? -udeg : udeg;
	lv_snprintf(buf, n, "%s%d.%06u", udeg < 0 ? "-" : "",
	            (int)(a / 1000000), (unsigned)(a % 1000000u));
}

static void gps_timer_cb(lv_timer_t *t)
{
	(void)t;
	const gps_status_t *g = gps_nmea_get_status();
	uint32_t now = lvgl_tick_get_ms_cb();

	bool alive = gps_nmea_link_alive(now, GPS_LINK_TIMEOUT_MS);
	if(!alive) {
		lv_label_set_text(gps_link_label,
		                  g->sentences ? "LINK STALE" : "NO DATA");
		lv_obj_set_style_text_color(gps_link_label,
		                            lv_palette_main(LV_PALETTE_RED), 0);
	} else {
		lv_label_set_text(gps_link_label, g->valid ? "FIX" : "ACQUIRING");
		lv_obj_set_style_text_color(gps_link_label,
		                            g->valid ? lv_palette_main(LV_PALETTE_GREEN)
		                                     : lv_palette_main(LV_PALETTE_AMBER), 0);
	}

	lv_label_set_text_fmt(gps_fix_label, "fixQ %u   sats %u",
	                      (unsigned)g->fix_quality, (unsigned)g->sats);

	char lat[20], lon[20];
	fmt_udeg(lat, sizeof(lat), g->lat_udeg);
	fmt_udeg(lon, sizeof(lon), g->lon_udeg);
	lv_label_set_text_fmt(gps_pos_label, "%s\n%s", lat, lon);

	lv_label_set_text_fmt(gps_aux_label,
	                      "UTC %06u   alt %d.%u m\nspd %u.%03u kn",
	                      (unsigned)g->time_hms,
	                      (int)(g->alt_dm / 10), (unsigned)((g->alt_dm < 0 ? -g->alt_dm : g->alt_dm) % 10),
	                      (unsigned)(g->speed_mknots / 1000u),
	                      (unsigned)(g->speed_mknots % 1000u));

	lv_label_set_text_fmt(gps_cnt_label,
	                      "sent %u  gga %u  rmc %u  csErr %u",
	                      (unsigned)g->sentences, (unsigned)g->gga,
	                      (unsigned)g->rmc, (unsigned)g->csum_errors);

	lv_label_set_text_fmt(gps_raw_label, "%s", g->last[0] ? g->last : "(no sentence yet)");
}

static void build_gps_tab(lv_obj_t *parent)
{
	lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_pad_all(parent, 10, 0);
	lv_obj_set_style_bg_color(parent, lv_color_hex(0x202020), 0);
	lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_SPACE_EVENLY,
	                      LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	lv_obj_t *cap = lv_label_create(parent);
	lv_obj_set_style_text_color(cap, lv_palette_main(LV_PALETTE_GREY), 0);
	lv_label_set_text(cap, "GPS (NMEA)");

	gps_link_label = lv_label_create(parent);
	lv_obj_set_style_text_font(gps_link_label, &lv_font_montserrat_28, 0);
	lv_obj_set_style_text_color(gps_link_label, lv_palette_main(LV_PALETTE_GREY), 0);
	lv_label_set_text(gps_link_label, "...");

	gps_fix_label = lv_label_create(parent);
	lv_obj_set_style_text_color(gps_fix_label, lv_color_white(), 0);
	lv_label_set_text(gps_fix_label, "fixQ --   sats --");

	gps_pos_label = lv_label_create(parent);
	lv_obj_set_style_text_align(gps_pos_label, LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_style_text_color(gps_pos_label, lv_color_white(), 0);
	lv_label_set_text(gps_pos_label, "--.------\n--.------");

	gps_aux_label = lv_label_create(parent);
	lv_obj_set_style_text_align(gps_aux_label, LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_style_text_color(gps_aux_label, lv_palette_main(LV_PALETTE_GREY), 0);
	lv_label_set_text(gps_aux_label, "UTC --   alt -- m\nspd -- kn");

	gps_cnt_label = lv_label_create(parent);
	lv_obj_set_style_text_color(gps_cnt_label, lv_palette_main(LV_PALETTE_GREY), 0);
	lv_label_set_text(gps_cnt_label, "sent 0  gga 0  rmc 0  csErr 0");

	gps_raw_label = lv_label_create(parent);
	lv_obj_set_width(gps_raw_label, LCD_WIDTH - 24);
	lv_label_set_long_mode(gps_raw_label, LV_LABEL_LONG_WRAP);
	lv_obj_set_style_text_color(gps_raw_label, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
	lv_label_set_text(gps_raw_label, "(no sentence yet)");

	lv_timer_create(gps_timer_cb, 500, NULL);
}

/* ======================================================================== */
/*  Power tab: 2S LiPo voltage via MCP3008                                   */
/* ======================================================================== */

#define POWER_PERIOD_MS 250

static lv_obj_t *pwr_v_label;
static lv_obj_t *pwr_soc_bar;
static lv_obj_t *pwr_soc_label;
static lv_obj_t *pwr_cell_label;

static void power_timer_cb(lv_timer_t *t)
{
	(void)t;
	power_reading_t r;
	power_read(&r);

	unsigned pct     = battery_soc_pct(r.bus_mV);
	unsigned cell_mV = (unsigned)(r.bus_mV / BATTERY_CELLS);

	lv_label_set_text_fmt(pwr_v_label, "%u.%01u V",
	                      (unsigned)(r.bus_mV / 1000u),
	                      (unsigned)((r.bus_mV % 1000u) / 100u));
	lv_label_set_text_fmt(pwr_cell_label, "%u.%02u V/cell  (2S LiPo)",
	                      cell_mV / 1000u, (cell_mV % 1000u) / 10u);
	lv_label_set_text_fmt(pwr_soc_label, "%u%%", pct);
	lv_bar_set_value(pwr_soc_bar, (int)pct, LV_ANIM_OFF);

	lv_color_t c = (pct > 50)  ? lv_palette_main(LV_PALETTE_GREEN)
	             : (pct >= 20) ? lv_palette_main(LV_PALETTE_AMBER)
	                           : lv_palette_main(LV_PALETTE_RED);
	lv_obj_set_style_bg_color(pwr_soc_bar, c, LV_PART_INDICATOR);
	lv_obj_set_style_text_color(pwr_v_label, (pct < 20) ? c : lv_color_white(), 0);
}

static void build_power_tab(lv_obj_t *parent)
{
	lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_pad_all(parent, 12, 0);
	lv_obj_set_style_bg_color(parent, lv_color_hex(0x202020), 0);
	lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_SPACE_EVENLY,
	                      LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	lv_obj_t *cap = lv_label_create(parent);
	lv_label_set_text(cap, "BATTERY");
	lv_obj_set_style_text_color(cap, lv_palette_main(LV_PALETTE_GREY), 0);

	pwr_v_label = lv_label_create(parent);
	lv_obj_set_style_text_font(pwr_v_label, &lv_font_montserrat_48, 0);
	lv_obj_set_style_text_color(pwr_v_label, lv_color_white(), 0);
	lv_label_set_text(pwr_v_label, "--.- V");

	pwr_soc_bar = lv_bar_create(parent);
	lv_obj_set_size(pwr_soc_bar, 240, 28);
	lv_bar_set_range(pwr_soc_bar, 0, 100);
	lv_bar_set_value(pwr_soc_bar, 0, LV_ANIM_OFF);

	pwr_soc_label = lv_label_create(parent);
	lv_obj_set_style_text_font(pwr_soc_label, &lv_font_montserrat_28, 0);
	lv_obj_set_style_text_color(pwr_soc_label, lv_color_white(), 0);
	lv_label_set_text(pwr_soc_label, "-- %");

	pwr_cell_label = lv_label_create(parent);
	lv_obj_set_style_text_color(pwr_cell_label, lv_palette_main(LV_PALETTE_GREY), 0);
	lv_label_set_text(pwr_cell_label, "-.-- V/cell  (2S LiPo)");

	lv_timer_create(power_timer_cb, POWER_PERIOD_MS, NULL);
}

static void build_ui(void)
{
	lv_obj_t *root = lv_screen_active();
	lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_bg_color(root, lv_color_hex(0x202020), 0);

	lv_obj_t *tv = lv_tabview_create(root);
	lv_tabview_set_tab_bar_size(tv, TAB_BAR_H);
	lv_obj_set_style_bg_color(tv, lv_color_hex(0x202020), 0);

	build_logger_tab(lv_tabview_add_tab(tv, "Logger"));
	build_gps_tab   (lv_tabview_add_tab(tv, "GPS"));
	build_power_tab (lv_tabview_add_tab(tv, "Power"));
}

/* ======================================================================== */
/*  main                                                                     */
/* ======================================================================== */

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("imu_logger starting"); log_nl();
	log_puts("sys clk : "); log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();

	lcd_pads_apply();
	lcd_init();
	busy_wait(200);                  /* FT6336U wants ~200 ms after reset */
	touch_init();
	ft6336u_probe();
	log_puts("LCD + touch ready"); log_nl();

	lvgl_tick_init();
	lv_init();
	lv_tick_set_cb(lvgl_tick_get_ms_cb);

	lv_display_t *disp = lv_display_create(LCD_WIDTH, LCD_HEIGHT);
	lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
	lv_display_set_buffers(disp, lvgl_fb1, lvgl_fb2, sizeof(lvgl_fb1),
	                       LV_DISPLAY_RENDER_MODE_DIRECT);
	lv_display_set_flush_cb(disp, lvgl_flush_cb);

	lv_indev_t *indev = lv_indev_create();
	lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
	lv_indev_set_read_cb(indev, lvgl_touch_read_cb);

	build_ui();

	/* Prime BOTH framebuffers (direct render mode) before the user sees them. */
	for(int i = 0; i < 2; i++) {
		lv_obj_invalidate(lv_screen_active());
		lv_refr_now(disp);
	}

	log_puts("ui primed"); log_nl();

	/* GPS reader up first (no card dependency) so the GPS tab shows link
	 * traffic even if the SD card is absent. */
	gps_nmea_init(lvgl_tick_get_ms_cb());
	log_puts("gps init done"); log_nl();

	/* Logging is NOT auto-started. SD bring-up is unverified, and a blocking SD
	 * init (sdcard_init spins if the card doesn't respond) would wedge the whole
	 * single-threaded loop -- taking touch and GPS down with it. Bring up the UI
	 * first; press START on the Logger tab to begin logging. imu_log_start()
	 * reports its progress when built with IMU_LOG_DEBUG=1. */
	log_puts("logging deferred -- press START on the Logger tab"); log_nl();

	log_puts("entering main loop"); log_nl();

	/* No busy_wait() past lvgl_tick_init() -- it reloads timer0. Both the IMU
	 * FIFO and the GPS UART FIFO absorb the jitter from LVGL's render frames. */
	for(;;) {
		uint32_t now = lvgl_tick_get_ms_cb();
		(void)lv_timer_handler();
		if(imu_log_poll(now))    /* flushed to SD -> busy_wait clobbered timer0 */
			lvgl_tick_resync();
		gps_nmea_poll(now);
	}

	return 0;
}
