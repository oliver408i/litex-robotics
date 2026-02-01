#include <stdint.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/i2c.h>
#include <libbase/uart.h>

#define PREAMBLE_0 0xAA
#define PREAMBLE_1 0x55
#define MAX_PAYLOAD 254

#define CMD_PING        0x01
#define CMD_GET_VERSION 0x02
#define CMD_SET_MOTOR   0x10
#define CMD_GET_MOTOR   0x11
#define CMD_SET_SERVO   0x12
#define CMD_GET_SERVO   0x13
#define CMD_SET_GPIO    0x14
#define CMD_GET_GPIO    0x15
#define CMD_ESTOP       0x16
#define CMD_GET_STATUS  0x20
#define CMD_SET_NEOPIXEL 0x30
#define CMD_GET_NEOPIXEL 0x31
#define CMD_SET_STRIP    0x32
#define CMD_SET_STRIP_BRI 0x33
#define CMD_SET_STRIP_BULK 0x34
#define CMD_SET_STRIP_INTERP 0x35
#define CMD_GET_ADC     0x40
#define CMD_SET_ADC_CFG 0x41
#define CMD_CLR_ADC_UPD 0x42
#define CMD_GET_ESTOP   0x50
#define CMD_GET_AS5600  0x60
#define CMD_SET_LASER   0x70
#define CMD_GET_LASER   0x71

#define RSP_ERROR       0x7F
#define STRIP_LED_COUNT 299
#define STRIP_INTERP_CHUNK 16

enum {
	ERR_BAD_LEN = 1,
	ERR_BAD_CHECKSUM = 2,
	ERR_BAD_CMD = 3,
	ERR_BAD_INDEX = 4,
};

static uint32_t uptime_ms;
static uint32_t tick_load;
static uint8_t last_error;

static int16_t motor_speed[4];
#define SERVO_CH_COUNT 5
static uint16_t servo_pulse_us[8];
static uint32_t gpio_mask;
static uint32_t gpio_value;
static uint8_t neo_en;
static uint8_t neo_brightness;
static uint8_t neo_grb[3];
static uint8_t strip_cur_grb[STRIP_LED_COUNT][3];
static uint8_t strip_tgt_grb[STRIP_LED_COUNT][3];
static uint8_t strip_cur_brightness[STRIP_LED_COUNT];
static uint8_t strip_tgt_brightness[STRIP_LED_COUNT];
static uint8_t strip_interp_color_step;
static uint8_t strip_interp_brightness_step;
static uint16_t strip_interp_index;
static uint8_t manual_mode;
static uint8_t joy_btn_state;
static uint16_t joy_btn_debounce_ms;
static uint16_t joy_btn_hold_ms;
static uint16_t joy_btn_release_ms;
static uint8_t joy_btn_latched;
static uint8_t joy_btn_debug;
static uint8_t estop_active;
static uint8_t estop_ntc_active;
static uint8_t ntc_warning;
static uint8_t estop_btn_db;
static uint8_t estop_btn_db_d;
static uint8_t estop_btn_raw_prev;
static uint32_t estop_db_ms;
static uint32_t estop_hold_ms;
static uint32_t estop_blink_ms;
static uint8_t estop_blink_on;

static uint16_t adc_raw_read(uint8_t ch);
static uint16_t adc_raw_to_mv(uint16_t raw);
static void servo_pwm_apply(uint8_t index);
static void servo_pwm_apply_all(void);
static void motor_pwm_apply(uint8_t index);
static void motor_pwm_apply_all(void);
static void ws2812_apply_state(void);
static void as5600_apply_status(uint8_t *present, uint8_t *ok, uint8_t *status, uint16_t *angle, uint16_t *magnitude);
static void ntc_update_on_tick(void);
static void ws2812_strip_write(uint16_t index, uint8_t g, uint8_t r, uint8_t b, uint8_t brightness);
static void ws2812_strip_set_target(uint16_t index, uint8_t g, uint8_t r, uint8_t b, uint8_t brightness);
static void ws2812_strip_interpolate_tick(void);

static void uart_write_str(const char *s)
{
	while (*s) {
		uart_write(*s++);
	}
}

static uint8_t checksum_xor(const uint8_t *data, uint8_t len)
{
	uint8_t chk = 0;
	uint8_t i;

	for (i = 0; i < len; i++) {
		chk ^= data[i];
	}
	return chk;
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
	uint8_t header[2 + 1 + 1];
	uint8_t len = (uint8_t)(1 + payload_len);
	uint8_t chk;

	header[0] = PREAMBLE_0;
	header[1] = PREAMBLE_1;
	header[2] = len;
	header[3] = cmd;

	chk = checksum_xor(&header[2], 2);
	if (payload_len) {
		chk ^= checksum_xor(payload, payload_len);
	}

	uart_write((char)header[0]);
	uart_write((char)header[1]);
	uart_write((char)header[2]);
	uart_write((char)header[3]);
	for (uint8_t i = 0; i < payload_len; i++) {
		uart_write((char)payload[i]);
	}
	uart_write((char)chk);
}

static void send_error(uint8_t cmd, uint8_t err)
{
	uint8_t payload[2];

	payload[0] = cmd;
	payload[1] = err;
	last_error = err;
	send_frame(RSP_ERROR, payload, sizeof(payload));
}

static void ws2812_activity_pulse(void)
{
#ifdef CSR_RGB_LED_BASE
	rgb_led_neo_activity_write(1);
#endif
}

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

static uint8_t step_u8_toward(uint8_t current, uint8_t target, uint8_t step)
{
	if (step == 0) {
		return target;
	}
	if (current < target) {
		uint8_t delta = (uint8_t)(target - current);
		if (delta > step) {
			return (uint8_t)(current + step);
		}
		return target;
	}
	if (current > target) {
		uint8_t delta = (uint8_t)(current - target);
		if (delta > step) {
			return (uint8_t)(current - step);
		}
		return target;
	}
	return current;
}

static uint16_t lerp_u16(uint16_t in, uint16_t in_min, uint16_t in_max,
			 uint16_t out_min, uint16_t out_max)
{
	uint32_t num;
	uint32_t den;

	if (in_max <= in_min) {
		return out_min;
	}
	if (in <= in_min) {
		return out_min;
	}
	if (in >= in_max) {
		return out_max;
	}
	num = (uint32_t)(in - in_min) * (uint32_t)(out_max - out_min);
	den = (uint32_t)(in_max - in_min);
	return (uint16_t)(out_min + (num + (den / 2)) / den);
}

static uint16_t joystick_mv_to_servo(uint16_t mv)
{
	const uint16_t min_mv = 0;
	const uint16_t max_mv = 3300;
	const uint16_t dead_low = 1300;
	const uint16_t dead_high = 1700;
	const uint16_t servo_min = 1000;
	const uint16_t servo_center = 1500;
	const uint16_t servo_max = 2000;

	mv = clamp_u16(mv, min_mv, max_mv);
	if (mv >= dead_low && mv <= dead_high) {
		return servo_center;
	}
	if (mv < dead_low) {
		return lerp_u16(mv, min_mv, dead_low, servo_min, servo_center);
	}
	return lerp_u16(mv, dead_high, max_mv, servo_center, servo_max);
}

static void joystick_update_on_tick(void)
{
#ifdef CSR_MCP3008_BASE
	const uint16_t btn_pressed_mv = 1000;
	const uint16_t btn_released_mv = 2000;
	const uint16_t debounce_ms = 50;
	const uint16_t hold_ms = 2000;
	uint16_t sw_mv = adc_raw_to_mv(adc_raw_read(0));
	uint8_t pressed;

	if (!joy_btn_state) {
		if (sw_mv <= btn_pressed_mv) {
			if (joy_btn_debounce_ms < debounce_ms) {
				joy_btn_debounce_ms++;
			}
			if (joy_btn_debounce_ms >= debounce_ms) {
				joy_btn_state = 1;
				joy_btn_hold_ms = 0;
				joy_btn_release_ms = 0;
			}
		} else {
			joy_btn_debounce_ms = 0;
		}
	} else {
		if (sw_mv >= btn_released_mv) {
			if (joy_btn_release_ms < debounce_ms) {
				joy_btn_release_ms++;
			}
			if (joy_btn_release_ms >= debounce_ms) {
				joy_btn_state = 0;
				joy_btn_latched = 0;
				joy_btn_hold_ms = 0;
				joy_btn_debounce_ms = 0;
				joy_btn_release_ms = 0;
			}
		} else {
			joy_btn_release_ms = 0;
		}
	}

	pressed = joy_btn_state;
	{
		uint8_t debug_now = (uint8_t)(pressed && !joy_btn_latched);
		if (debug_now != joy_btn_debug) {
			joy_btn_debug = debug_now;
			ws2812_apply_state();
		}
	}

	if (pressed) {
		if (joy_btn_hold_ms < hold_ms) {
			joy_btn_hold_ms++;
		}
		if (!joy_btn_latched && joy_btn_hold_ms >= hold_ms) {
			manual_mode = (uint8_t)(!manual_mode);
			joy_btn_latched = 1;
			ws2812_apply_state();
		}
	}

	if (!manual_mode || estop_active) {
		return;
	}

	uint16_t horz_mv = adc_raw_to_mv(adc_raw_read(1));
	uint16_t vert_mv = adc_raw_to_mv(adc_raw_read(2));
	uint16_t servo_h = joystick_mv_to_servo(horz_mv);
	uint16_t servo_v = joystick_mv_to_servo(vert_mv);

	if (servo_pulse_us[1] != servo_h) {
		servo_pulse_us[1] = servo_h;
		servo_pwm_apply(1);
	}
	if (servo_pulse_us[0] != servo_v) {
		servo_pulse_us[0] = servo_v;
		servo_pwm_apply(0);
	}
#endif
}

static void ws2812_apply_state(void)
{
#ifdef CSR_RGB_LED_BASE
	if (estop_active) {
		rgb_led_neo_en_write(1);
		rgb_led_neo_brightness_write(estop_blink_on ? 0xff : 0x00);
		rgb_led_neo_color_hi_write(0x00ff);
		rgb_led_neo_color_lo_write(0x00);
	} else if (ntc_warning) {
		rgb_led_neo_en_write(1);
		rgb_led_neo_brightness_write(estop_blink_on ? 0xff : 0x00);
		rgb_led_neo_color_hi_write(0xffff);
		rgb_led_neo_color_lo_write(0x00);
	} else if (joy_btn_debug) {
		rgb_led_neo_en_write(1);
		rgb_led_neo_brightness_write(estop_blink_on ? 0xff : 0x10);
		rgb_led_neo_color_hi_write(0xff00);
		rgb_led_neo_color_lo_write(0x00);
	} else if (manual_mode) {
		rgb_led_neo_en_write(1);
		rgb_led_neo_brightness_write(0xff);
		rgb_led_neo_color_hi_write(0x0000);
		rgb_led_neo_color_lo_write(0xff);
	} else {
		uint16_t hi = (uint16_t)((neo_grb[0] << 8) | neo_grb[1]);
		rgb_led_neo_en_write(neo_en ? 1 : 0);
		rgb_led_neo_brightness_write(neo_brightness);
		rgb_led_neo_color_hi_write(hi);
		rgb_led_neo_color_lo_write(neo_grb[2]);
	}
#endif

}

static void ws2812_strip_write(uint16_t index, uint8_t g, uint8_t r, uint8_t b, uint8_t brightness)
{
#ifdef CSR_RGB_LED_BASE
	uint16_t sg = (uint16_t)g * brightness;
	uint16_t sr = (uint16_t)r * brightness;
	uint16_t sb = (uint16_t)b * brightness;
	uint8_t g_out = (uint8_t)((sg + 127) / 255);
	uint8_t r_out = (uint8_t)((sr + 127) / 255);
	uint8_t b_out = (uint8_t)((sb + 127) / 255);
	rgb_led_strip_index_write(index);
	rgb_led_strip_color_hi_write((uint16_t)((g_out << 8) | r_out));
	rgb_led_strip_color_lo_write(b_out);
	rgb_led_strip_write_write(1);
#else
	(void)index;
	(void)g;
	(void)r;
	(void)b;
	(void)brightness;
#endif
}

static void ws2812_strip_set_target(uint16_t index, uint8_t g, uint8_t r, uint8_t b, uint8_t brightness)
{
	strip_tgt_grb[index][0] = g;
	strip_tgt_grb[index][1] = r;
	strip_tgt_grb[index][2] = b;
	strip_tgt_brightness[index] = brightness;

	if (strip_interp_color_step == 0 && strip_interp_brightness_step == 0) {
		strip_cur_grb[index][0] = g;
		strip_cur_grb[index][1] = r;
		strip_cur_grb[index][2] = b;
		strip_cur_brightness[index] = brightness;
		ws2812_strip_write(index, g, r, b, brightness);
	}
}

static void ws2812_strip_sync_all(void)
{
	for (uint16_t i = 0; i < STRIP_LED_COUNT; i++) {
		strip_cur_grb[i][0] = strip_tgt_grb[i][0];
		strip_cur_grb[i][1] = strip_tgt_grb[i][1];
		strip_cur_grb[i][2] = strip_tgt_grb[i][2];
		strip_cur_brightness[i] = strip_tgt_brightness[i];
		ws2812_strip_write(i,
				   strip_cur_grb[i][0],
				   strip_cur_grb[i][1],
				   strip_cur_grb[i][2],
				   strip_cur_brightness[i]);
	}
}

static void ws2812_strip_interpolate_tick(void)
{
	if (strip_interp_color_step == 0 && strip_interp_brightness_step == 0) {
		return;
	}
	for (uint16_t j = 0; j < STRIP_INTERP_CHUNK; j++) {
		uint16_t i = (uint16_t)(strip_interp_index + j);
		if (i >= STRIP_LED_COUNT) {
			i = (uint16_t)(i - STRIP_LED_COUNT);
		}
		uint8_t g_next = step_u8_toward(strip_cur_grb[i][0], strip_tgt_grb[i][0],
						strip_interp_color_step);
		uint8_t r_next = step_u8_toward(strip_cur_grb[i][1], strip_tgt_grb[i][1],
						strip_interp_color_step);
		uint8_t b_next = step_u8_toward(strip_cur_grb[i][2], strip_tgt_grb[i][2],
						strip_interp_color_step);
		uint8_t bri_next = step_u8_toward(strip_cur_brightness[i], strip_tgt_brightness[i],
						  strip_interp_brightness_step);
		if (g_next == strip_cur_grb[i][0] &&
		    r_next == strip_cur_grb[i][1] &&
		    b_next == strip_cur_grb[i][2] &&
		    bri_next == strip_cur_brightness[i]) {
			continue;
		}
		strip_cur_grb[i][0] = g_next;
		strip_cur_grb[i][1] = r_next;
		strip_cur_grb[i][2] = b_next;
		strip_cur_brightness[i] = bri_next;
		ws2812_strip_write(i, g_next, r_next, b_next, bri_next);
	}
	strip_interp_index = (uint16_t)(strip_interp_index + STRIP_INTERP_CHUNK);
	if (strip_interp_index >= STRIP_LED_COUNT) {
		strip_interp_index = (uint16_t)(strip_interp_index - STRIP_LED_COUNT);
	}
}

static void as5600_apply_status(uint8_t *present, uint8_t *ok, uint8_t *status, uint16_t *angle, uint16_t *magnitude)
{
#ifdef CONFIG_HAS_I2C
	const uint8_t addr = 0x36;
	uint8_t status_reg = 0;
	uint8_t angle_buf[2] = {0, 0};
	uint8_t mag_buf[2] = {0, 0};

	*present = i2c_poll(addr) ? 1 : 0;
	if (!*present) {
		*ok = 0;
		*status = 0;
		*angle = 0;
		*magnitude = 0;
		return;
	}

	if (!i2c_read(addr, 0x0B, &status_reg, 1, true, 1)) {
		*ok = 0;
		*status = 0;
		*angle = 0;
		*magnitude = 0;
		return;
	}
	if (!i2c_read(addr, 0x0E, angle_buf, 2, true, 1)) {
		*ok = 0;
		*status = status_reg;
		*angle = 0;
		*magnitude = 0;
		return;
	}
	if (!i2c_read(addr, 0x1B, mag_buf, 2, true, 1)) {
		*ok = 0;
		*status = status_reg;
		*angle = (uint16_t)((angle_buf[0] << 8) | angle_buf[1]);
		*magnitude = 0;
		return;
	}

	*ok = 1;
	*status = status_reg;
	*angle = (uint16_t)((angle_buf[0] << 8) | angle_buf[1]);
	*magnitude = (uint16_t)((mag_buf[0] << 8) | mag_buf[1]);
#else
	*present = 0;
	*ok = 0;
	*status = 0;
	*angle = 0;
	*magnitude = 0;
#endif
}

static uint8_t estop_raw_active(void)
{
#ifdef CSR_ESTOP_BTNS_BASE
	uint32_t v = estop_btns_in_read();
	uint8_t primary = (uint8_t)(v & 0x1);
	uint8_t ext_raw = (uint8_t)((v >> 1) & 0x1);
	uint8_t ext_active = (uint8_t)(ext_raw ? 0 : 1);
	return (uint8_t)((primary | ext_active) ? 1 : 0);
#else
	return 0;
#endif
}

static void servo_pwm_apply_all(void);

static void estop_update_on_tick(void)
{
	const uint32_t debounce_ms = 10;
	const uint32_t release_ms = 2000;
	const uint32_t blink_period_ms = 200;
	uint8_t raw = estop_raw_active();

	if (raw == estop_btn_db) {
		estop_db_ms = 0;
	} else {
		if (raw == estop_btn_raw_prev) {
			if (estop_db_ms >= (debounce_ms - 1)) {
				estop_btn_db = raw;
				estop_db_ms = 0;
			} else {
				estop_db_ms++;
			}
		} else {
			estop_db_ms = 0;
		}
	}
	estop_btn_raw_prev = raw;

	{
		uint8_t btn_rise = (uint8_t)(estop_btn_db && !estop_btn_db_d);
		estop_btn_db_d = estop_btn_db;
		if (!estop_active) {
			if (btn_rise) {
				estop_active = 1;
				estop_hold_ms = 0;
				for (uint8_t i = 0; i < SERVO_CH_COUNT; i++) {
					servo_pulse_us[i] = 1500;
				}
				servo_pwm_apply_all();
				memset(motor_speed, 0, sizeof(motor_speed));
				motor_pwm_apply_all();
			}
		} else {
			if (estop_btn_db && !estop_ntc_active) {
				if (estop_hold_ms >= (release_ms - 1)) {
					estop_active = 0;
					estop_hold_ms = 0;
				} else {
					estop_hold_ms++;
				}
			} else {
				estop_hold_ms = 0;
			}
		}
	}

	if (estop_blink_ms >= (blink_period_ms - 1)) {
		estop_blink_ms = 0;
	} else {
		estop_blink_ms++;
	}
	estop_blink_on = (uint8_t)(estop_blink_ms < (blink_period_ms / 2));
}

static void ntc_update_on_tick(void)
{
#ifdef CSR_MCP3008_BASE
	const uint16_t warn_mv = 1500;
	const uint16_t estop_mv = 1000;
	uint16_t mv = adc_raw_to_mv(adc_raw_read(7));

	ntc_warning = (uint8_t)(mv <= warn_mv);
	estop_ntc_active = (uint8_t)(mv <= estop_mv);
	if (estop_ntc_active && !estop_active) {
		estop_active = 1;
		estop_hold_ms = 0;
		for (uint8_t i = 0; i < SERVO_CH_COUNT; i++) {
			servo_pulse_us[i] = 1500;
		}
		servo_pwm_apply_all();
		memset(motor_speed, 0, sizeof(motor_speed));
		motor_pwm_apply_all();
	}
#else
	ntc_warning = 0;
	estop_ntc_active = 0;
#endif
}

static uint16_t adc_raw_read(uint8_t ch)
{
#ifdef CSR_MCP3008_BASE
	switch (ch) {
	case 0: return (uint16_t)mcp3008_sample_ch0_read();
	case 1: return (uint16_t)mcp3008_sample_ch1_read();
	case 2: return (uint16_t)mcp3008_sample_ch2_read();
	case 3: return (uint16_t)mcp3008_sample_ch3_read();
	case 4: return (uint16_t)mcp3008_sample_ch4_read();
	case 5: return (uint16_t)mcp3008_sample_ch5_read();
	case 6: return (uint16_t)mcp3008_sample_ch6_read();
	case 7: return (uint16_t)mcp3008_sample_ch7_read();
	default:
		break;
	}
#endif
	return 0;
}

static uint16_t adc_raw_to_mv(uint16_t raw)
{
	uint32_t mv = ((uint32_t)raw * 3300u + 511u) / 1023u;
	if (mv > 0xffffu) {
		mv = 0xffffu;
	}
	return (uint16_t)mv;
}

static uint32_t servo_ticks_from_us(uint16_t us)
{
	uint64_t ticks = ((uint64_t)us * (uint64_t)CONFIG_CLOCK_FREQUENCY) / 1000000u;
	if (ticks > 0xffffffffu) {
		ticks = 0xffffffffu;
	}
	return (uint32_t)ticks;
}

static uint16_t motor_pulse_from_speed(int32_t speed)
{
	const uint16_t esc_min_us = 1000;
	const uint16_t esc_max_us = 2000;
	const uint32_t span = (uint32_t)(esc_max_us - esc_min_us);
	uint32_t clamped = (speed < 0) ? 0u : (uint32_t)speed;
	if (clamped > 32767u) {
		clamped = 32767u;
	}
	return (uint16_t)(esc_min_us + (clamped * span + 16383u) / 32767u);
}

static void servo_pwm_apply(uint8_t index)
{
#ifdef CSR_SERVO_PWM_BASE
	uint32_t ticks = servo_ticks_from_us(servo_pulse_us[index]);
	switch (index) {
	case 0: servo_pwm_duty0_write(ticks); break;
	case 1: servo_pwm_duty1_write(ticks); break;
	case 2: servo_pwm_duty2_write(ticks); break;
	case 3: servo_pwm_duty3_write(ticks); break;
	case 4: servo_pwm_duty4_write(ticks); break;
	case 5: servo_pwm_duty5_write(ticks); break;
	case 6: servo_pwm_duty6_write(ticks); break;
	default: break;
	}
#endif
}

static void servo_pwm_apply_all(void)
{
	for (uint8_t i = 0; i < SERVO_CH_COUNT; i++) {
		servo_pwm_apply(i);
	}
}

static void motor_pwm_apply(uint8_t index)
{
#if defined(CSR_SERVO_PWM_BASE)
	if (index >= 2) {
		return;
	}
	int32_t speed = motor_speed[index];
	uint16_t pulse_us = motor_pulse_from_speed(speed);
	uint32_t ticks = servo_ticks_from_us(pulse_us);
	switch (index) {
	case 0: servo_pwm_duty5_write(ticks); break;
	case 1: servo_pwm_duty6_write(ticks); break;
	default: break;
	}
#elif defined(CSR_MOTOR_PWM_BASE)
	if (index >= 2) {
		return;
	}
	int32_t speed = motor_speed[index];
	if (speed < 0) {
		speed = 0;
	}
	if (speed > 32767) {
		speed = 32767;
	}
	uint8_t duty = (uint8_t)(speed >> 7);
	switch (index) {
	case 0: motor_pwm_duty0_write(duty); break;
	case 1: motor_pwm_duty1_write(duty); break;
	default: break;
	}
#endif
}

static void motor_pwm_apply_all(void)
{
	for (uint8_t i = 0; i < 2; i++) {
		motor_pwm_apply(i);
	}
}

static void handle_command(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
	uint8_t rsp[16];

	ws2812_activity_pulse();

	switch (cmd) {
	case CMD_PING:
		send_frame((uint8_t)(cmd | 0x80), (const uint8_t *)"PONG", 4);
		break;
	case CMD_GET_VERSION:
		rsp[0] = 1;
		rsp[1] = 0;
		send_frame((uint8_t)(cmd | 0x80), rsp, 2);
		break;
	case CMD_SET_MOTOR:
		if (payload_len != 3) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		if (payload[0] >= (uint8_t)(sizeof(motor_speed) / sizeof(motor_speed[0]))) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		motor_speed[payload[0]] = (int16_t)((payload[2] << 8) | payload[1]);
		motor_pwm_apply(payload[0]);
		rsp[0] = payload[0];
		send_frame((uint8_t)(cmd | 0x80), rsp, 1);
		break;
	case CMD_GET_MOTOR:
		if (payload_len != 1) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		if (payload[0] >= (uint8_t)(sizeof(motor_speed) / sizeof(motor_speed[0]))) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		rsp[0] = payload[0];
		rsp[1] = (uint8_t)(motor_speed[payload[0]] & 0xff);
		rsp[2] = (uint8_t)((motor_speed[payload[0]] >> 8) & 0xff);
		send_frame((uint8_t)(cmd | 0x80), rsp, 3);
		break;
	case CMD_SET_SERVO:
		if (payload_len != 3) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		if (payload[0] >= SERVO_CH_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		servo_pulse_us[payload[0]] = (uint16_t)((payload[2] << 8) | payload[1]);
		servo_pwm_apply(payload[0]);
		rsp[0] = payload[0];
		send_frame((uint8_t)(cmd | 0x80), rsp, 1);
		break;
	case CMD_GET_SERVO:
		if (payload_len != 1) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		if (payload[0] >= SERVO_CH_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		rsp[0] = payload[0];
		rsp[1] = (uint8_t)(servo_pulse_us[payload[0]] & 0xff);
		rsp[2] = (uint8_t)((servo_pulse_us[payload[0]] >> 8) & 0xff);
		send_frame((uint8_t)(cmd | 0x80), rsp, 3);
		break;
	case CMD_SET_GPIO:
		if (payload_len != 8) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		gpio_mask = (uint32_t)payload[0] |
			((uint32_t)payload[1] << 8) |
			((uint32_t)payload[2] << 16) |
			((uint32_t)payload[3] << 24);
		gpio_value = (uint32_t)payload[4] |
			((uint32_t)payload[5] << 8) |
			((uint32_t)payload[6] << 16) |
			((uint32_t)payload[7] << 24);
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_GET_GPIO:
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		rsp[0] = (uint8_t)(gpio_mask & 0xff);
		rsp[1] = (uint8_t)((gpio_mask >> 8) & 0xff);
		rsp[2] = (uint8_t)((gpio_mask >> 16) & 0xff);
		rsp[3] = (uint8_t)((gpio_mask >> 24) & 0xff);
		rsp[4] = (uint8_t)(gpio_value & 0xff);
		rsp[5] = (uint8_t)((gpio_value >> 8) & 0xff);
		rsp[6] = (uint8_t)((gpio_value >> 16) & 0xff);
		rsp[7] = (uint8_t)((gpio_value >> 24) & 0xff);
		send_frame((uint8_t)(cmd | 0x80), rsp, 8);
		break;
	case CMD_ESTOP:
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		for (uint8_t i = 0; i < SERVO_CH_COUNT; i++) {
			servo_pulse_us[i] = 1500;
		}
		servo_pwm_apply_all();
		memset(motor_speed, 0, sizeof(motor_speed));
		motor_pwm_apply_all();
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_GET_STATUS:
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		rsp[0] = (uint8_t)(uptime_ms & 0xff);
		rsp[1] = (uint8_t)((uptime_ms >> 8) & 0xff);
		rsp[2] = (uint8_t)((uptime_ms >> 16) & 0xff);
		rsp[3] = (uint8_t)((uptime_ms >> 24) & 0xff);
		rsp[4] = last_error;
		send_frame((uint8_t)(cmd | 0x80), rsp, 5);
		break;
	case CMD_SET_NEOPIXEL:
		if (payload_len != 5) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		neo_en = payload[0];
		neo_brightness = payload[1];
		neo_grb[0] = payload[2];
		neo_grb[1] = payload[3];
		neo_grb[2] = payload[4];
		ws2812_apply_state();
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_SET_STRIP:
		if (payload_len != 5) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		uint16_t strip_index = (uint16_t)(payload[0] | (payload[1] << 8));
		if (strip_index >= STRIP_LED_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		ws2812_strip_set_target(strip_index, payload[2], payload[3], payload[4], 255);
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_SET_STRIP_BRI:
		if (payload_len != 6) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		strip_index = (uint16_t)(payload[0] | (payload[1] << 8));
		if (strip_index >= STRIP_LED_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		ws2812_strip_set_target(strip_index, payload[2], payload[3], payload[4], payload[5]);
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_SET_STRIP_BULK: {
		if (payload_len < 3) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		uint16_t start = (uint16_t)(payload[0] | (payload[1] << 8));
		uint8_t count = payload[2];
		uint8_t expected = (uint8_t)(3 + count * 3);
		if (payload_len != expected) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		if ((start + count) > STRIP_LED_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		for (uint8_t i = 0; i < count; i++) {
			uint8_t base = (uint8_t)(3 + (i * 3));
			ws2812_strip_set_target((uint16_t)(start + i),
						payload[base + 0],
						payload[base + 1],
						payload[base + 2],
						255);
		}
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	}
	case CMD_SET_STRIP_INTERP:
		if (payload_len != 2) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		strip_interp_color_step = payload[0];
		strip_interp_brightness_step = payload[1];
		if (strip_interp_color_step == 0 && strip_interp_brightness_step == 0) {
			ws2812_strip_sync_all();
		}
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_GET_NEOPIXEL:
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		rsp[0] = neo_en;
		rsp[1] = neo_brightness;
		rsp[2] = neo_grb[0];
		rsp[3] = neo_grb[1];
		rsp[4] = neo_grb[2];
		send_frame((uint8_t)(cmd | 0x80), rsp, 5);
		break;
	case CMD_GET_ESTOP:
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		rsp[0] = estop_active;
		rsp[1] = estop_btn_db;
		rsp[2] = estop_raw_active();
		send_frame((uint8_t)(cmd | 0x80), rsp, 3);
		break;
	case CMD_GET_AS5600: {
		uint8_t present = 0;
		uint8_t ok = 0;
		uint8_t status = 0;
		uint16_t angle = 0;
		uint16_t magnitude = 0;
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		as5600_apply_status(&present, &ok, &status, &angle, &magnitude);
		rsp[0] = present;
		rsp[1] = ok;
		rsp[2] = status;
		rsp[3] = (uint8_t)(angle & 0xff);
		rsp[4] = (uint8_t)((angle >> 8) & 0xff);
		rsp[5] = (uint8_t)(magnitude & 0xff);
		rsp[6] = (uint8_t)((magnitude >> 8) & 0xff);
		send_frame((uint8_t)(cmd | 0x80), rsp, 7);
		break;
	}
	case CMD_SET_LASER:
		if (payload_len != 1) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_LASER_BASE
		laser_enable_write(payload[0] ? 1 : 0);
#endif
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_GET_LASER:
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_LASER_BASE
		rsp[0] = (uint8_t)laser_status_read();
#else
		rsp[0] = 0;
#endif
		send_frame((uint8_t)(cmd | 0x80), rsp, 1);
		break;
	case CMD_GET_ADC: {
		uint8_t adc_rsp[18];
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_MCP3008_BASE
		for (uint8_t ch = 0; ch < 8; ch++) {
			uint16_t mv = adc_raw_to_mv(adc_raw_read(ch));
			adc_rsp[2 * ch] = (uint8_t)(mv & 0xff);
			adc_rsp[2 * ch + 1] = (uint8_t)((mv >> 8) & 0xff);
		}
		adc_rsp[16] = (uint8_t)mcp3008_update_mask_read();
		adc_rsp[17] = (uint8_t)mcp3008_last_channel_read();
#else
		for (uint8_t ch = 0; ch < 18; ch++) {
			adc_rsp[ch] = 0;
		}
#endif
		send_frame((uint8_t)(cmd | 0x80), adc_rsp, sizeof(adc_rsp));
		break;
	}
	case CMD_SET_ADC_CFG:
		if (payload_len != 6) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_MCP3008_BASE
		mcp3008_enable_write(payload[0] ? 1 : 0);
		mcp3008_channel_mask_write(payload[1]);
		mcp3008_sample_interval_write(
			(uint32_t)payload[2] |
			((uint32_t)payload[3] << 8) |
			((uint32_t)payload[4] << 16) |
			((uint32_t)payload[5] << 24)
		);
#endif
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_CLR_ADC_UPD:
		if (payload_len != 1) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_MCP3008_BASE
		mcp3008_clear_update_write(payload[0]);
#endif
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	default:
		send_error(cmd, ERR_BAD_CMD);
		break;
	}
}

static void timer_init_tick(void)
{
	uint64_t ticks = ((uint64_t)CONFIG_CLOCK_FREQUENCY / 1000u);

	if (ticks == 0) {
		ticks = 1;
	}
	if (ticks > 0xffffffffu) {
		ticks = 0xffffffffu;
	}

	tick_load = (uint32_t)ticks;
	timer0_en_write(0);
	timer0_reload_write(0);
	timer0_load_write(tick_load);
	timer0_en_write(1);
}

static uint8_t timer_poll_tick(void)
{
	uint8_t ticked = 0;

	timer0_update_value_write(1);
	if (timer0_value_read() == 0) {
		uptime_ms++;
		timer0_en_write(0);
		timer0_reload_write(0);
		timer0_load_write(tick_load);
		timer0_en_write(1);
		ticked = 1;
	}
	return ticked;
}

static void process_rx_byte(uint8_t byte)
{
	static uint8_t state;
	static uint8_t len;
	static uint8_t cmd;
	static uint8_t payload[MAX_PAYLOAD];
	static uint8_t payload_pos;
	static uint8_t chk;

	switch (state) {
	case 0:
		if (byte == PREAMBLE_0) {
			state = 1;
		}
		break;
	case 1:
		if (byte == PREAMBLE_1) {
			state = 2;
		} else {
			state = 0;
		}
		break;
	case 2:
		if (byte == 0 || byte > (MAX_PAYLOAD + 1)) {
			send_error(0, ERR_BAD_LEN);
			state = 0;
			break;
		}
		len = byte;
		chk = byte;
		payload_pos = 0;
		state = 3;
		break;
	case 3:
		cmd = byte;
		chk ^= byte;
		if (len == 1) {
			state = 5;
		} else {
			state = 4;
		}
		break;
	case 4:
		payload[payload_pos++] = byte;
		chk ^= byte;
		if (payload_pos >= (uint8_t)(len - 1)) {
			state = 5;
		}
		break;
	case 5:
		if (chk != byte) {
			send_error(cmd, ERR_BAD_CHECKSUM);
		} else {
			handle_command(cmd, payload, (uint8_t)(len - 1));
		}
		state = 0;
		break;
	default:
		state = 0;
		break;
	}
}

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(1);
#endif
	uart_init();
	timer_init_tick();
#ifdef CONFIG_HAS_I2C
	i2c_reset();
	i2c_send_init_cmds();
#endif
	estop_btn_raw_prev = estop_raw_active();
	estop_btn_db = estop_btn_raw_prev;
	estop_btn_db_d = estop_btn_raw_prev;
	estop_active = 0;
	estop_ntc_active = 0;
	ntc_warning = 0;
	estop_blink_ms = 0;
	estop_blink_on = 1;
	manual_mode = 0;
	joy_btn_state = 0;
	joy_btn_debounce_ms = 0;
	joy_btn_hold_ms = 0;
	joy_btn_release_ms = 0;
	joy_btn_latched = 0;
	joy_btn_debug = 0;
	memset(strip_cur_grb, 0, sizeof(strip_cur_grb));
	memset(strip_tgt_grb, 0, sizeof(strip_tgt_grb));
	memset(strip_cur_brightness, 0, sizeof(strip_cur_brightness));
	memset(strip_tgt_brightness, 0, sizeof(strip_tgt_brightness));
	strip_interp_color_step = 0;
	strip_interp_brightness_step = 0;
	strip_interp_index = 0;
	for (uint8_t i = 0; i < SERVO_CH_COUNT; i++) {
		servo_pulse_us[i] = 1500;
	}
	servo_pwm_apply_all();
	motor_pwm_apply_all();
	ws2812_apply_state();

	uart_write_str("LiteX UART protocol ready.\\r\\n");

#ifdef CSR_MCP3008_BASE
	{
		uint8_t mask = mcp3008_channel_mask_read();
		mask |= (uint8_t)((1u << 1) | (1u << 2) | (1u << 3) | (1u << 7));
		mcp3008_enable_write(1);
		mcp3008_channel_mask_write(mask);
	}
#endif

	while (1) {
		if (timer_poll_tick()) {
			uint8_t estop_prev = estop_active;
			uint8_t ntc_prev = ntc_warning;
			uint8_t blink_prev = estop_blink_on;
			estop_update_on_tick();
			ntc_update_on_tick();
			joystick_update_on_tick();
			ws2812_strip_interpolate_tick();
			if (estop_active || estop_prev || ntc_warning || ntc_prev) {
				if ((estop_prev != estop_active) ||
				    (ntc_prev != ntc_warning) ||
				    (blink_prev != estop_blink_on)) {
					ws2812_apply_state();
				}
			}
		}
		if (uart_read_nonblock()) {
			process_rx_byte((uint8_t)uart_read());
		}
	}
}
