#include <stdint.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/uart.h>

#define PREAMBLE_0 0xAA
#define PREAMBLE_1 0x55
#define MAX_PAYLOAD 254

#define CMD_PING            0x01
#define CMD_GET_VERSION     0x02
#define CMD_SET_NEOPIXEL    0x30
#define CMD_GET_NEOPIXEL    0x31
#define CMD_SET_STRIP       0x32
#define CMD_SET_STRIP_BRI   0x33
#define CMD_SET_STRIP_BULK  0x34
#define CMD_SET_STRIP_INTERP 0x35
#define CMD_SET_BT_EN       0x72
#define CMD_GET_BT_STATE    0x73
#define CMD_BT_WRITE        0x74
#define CMD_BT_READ         0x75

#define RSP_ERROR       0x7F
#define STRIP_LED_COUNT 300
#define STRIP_INTERP_CHUNK 16

enum {
	ERR_BAD_LEN = 1,
	ERR_BAD_CHECKSUM = 2,
	ERR_BAD_CMD = 3,
	ERR_BAD_INDEX = 4,
};

static uint32_t tick_load;
static uint8_t last_error;

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

static void uart_write_str(const char *s)
{
	while (*s) {
		uart_write(*s++);
	}
}

static uint8_t checksum_xor(const uint8_t *data, uint8_t len)
{
	uint8_t chk = 0;
	for (uint8_t i = 0; i < len; i++) {
		chk ^= data[i];
	}
	return chk;
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
	uint8_t header[4];
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

static uint8_t step_u8_toward(uint8_t current, uint8_t target, uint8_t step)
{
	if (step == 0) {
		return target;
	}
	if (current < target) {
		uint8_t delta = (uint8_t)(target - current);
		return (delta > step) ? (uint8_t)(current + step) : target;
	}
	if (current > target) {
		uint8_t delta = (uint8_t)(current - target);
		return (delta > step) ? (uint8_t)(current - step) : target;
	}
	return current;
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

static void neopixel_apply(void)
{
	if (!neo_en) {
		ws2812_strip_set_target(0, 0, 0, 0, 0);
		return;
	}
	ws2812_strip_set_target(0, neo_grb[0], neo_grb[1], neo_grb[2], neo_brightness);
}

static void handle_command(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
	switch (cmd) {
	case CMD_PING:
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		send_frame((uint8_t)(cmd | 0x80), (const uint8_t *)"PONG", 4);
		break;
	case CMD_GET_VERSION: {
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		uint8_t rsp[2] = {1, 0};
		send_frame((uint8_t)(cmd | 0x80), rsp, sizeof(rsp));
		break;
	}
	case CMD_SET_NEOPIXEL:
		if (payload_len != 5) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		neo_en = payload[0] ? 1 : 0;
		neo_brightness = payload[1];
		neo_grb[0] = payload[2];
		neo_grb[1] = payload[3];
		neo_grb[2] = payload[4];
		neopixel_apply();
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_GET_NEOPIXEL: {
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		uint8_t rsp[5] = {neo_en, neo_brightness, neo_grb[0], neo_grb[1], neo_grb[2]};
		send_frame((uint8_t)(cmd | 0x80), rsp, sizeof(rsp));
		break;
	}
	case CMD_SET_STRIP: {
		if (payload_len != 5) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		uint16_t index = (uint16_t)(payload[0] | (payload[1] << 8));
		if (index >= STRIP_LED_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		ws2812_strip_set_target(index, payload[2], payload[3], payload[4], 255);
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	}
	case CMD_SET_STRIP_BRI: {
		if (payload_len != 6) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		uint16_t index = (uint16_t)(payload[0] | (payload[1] << 8));
		if (index >= STRIP_LED_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		ws2812_strip_set_target(index, payload[2], payload[3], payload[4], payload[5]);
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	}
	case CMD_SET_STRIP_BULK: {
		if (payload_len < 3) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		uint16_t start = (uint16_t)(payload[0] | (payload[1] << 8));
		uint8_t count = payload[2];
		if ((uint16_t)count * 3u + 3u != payload_len) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		if (count == 0 || (uint16_t)(start + count) > STRIP_LED_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}
		for (uint8_t i = 0; i < count; i++) {
			const uint8_t *c = &payload[3 + i * 3];
			ws2812_strip_set_target((uint16_t)(start + i), c[0], c[1], c[2], 255);
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
	case CMD_SET_BT_EN:
		if (payload_len != 1) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_BT_EN_BASE
		bt_en_out_write(payload[0] ? 1 : 0);
#endif
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_GET_BT_STATE: {
		uint8_t rsp[4];
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_HC05_UART_BASE
		rsp[0] = (uint8_t)bt_en_out_read();
		rsp[1] = (uint8_t)(hc05_uart_txfull_read() ? 0 : 1);
		rsp[2] = (uint8_t)(hc05_uart_rxempty_read() ? 0 : 1);
		rsp[3] = 0;
#else
		rsp[0] = 0;
		rsp[1] = 0;
		rsp[2] = 0;
		rsp[3] = 0;
#endif
		send_frame((uint8_t)(cmd | 0x80), rsp, sizeof(rsp));
		break;
	}
	case CMD_BT_WRITE: {
		uint8_t rsp[1];
		if (payload_len != 1) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_HC05_UART_BASE
		if (hc05_uart_txfull_read()) {
			last_error = ERR_BAD_INDEX;
			rsp[0] = 0;
		} else {
			hc05_uart_rxtx_write(payload[0]);
			rsp[0] = 1;
		}
#else
		rsp[0] = 0;
#endif
		send_frame((uint8_t)(cmd | 0x80), rsp, sizeof(rsp));
		break;
	}
	case CMD_BT_READ: {
		uint8_t rsp[3];
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
#ifdef CSR_HC05_UART_BASE
		rsp[0] = (uint8_t)(hc05_uart_rxempty_read() ? 0 : 1);
		rsp[1] = rsp[0] ? (uint8_t)hc05_uart_rxtx_read() : 0;
		rsp[2] = 0;
#else
		rsp[0] = 0;
		rsp[1] = 0;
		rsp[2] = 0;
#endif
		send_frame((uint8_t)(cmd | 0x80), rsp, sizeof(rsp));
		break;
	}
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
		state = (len == 1) ? 5 : 4;
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

	memset(strip_cur_grb, 0, sizeof(strip_cur_grb));
	memset(strip_tgt_grb, 0, sizeof(strip_tgt_grb));
	memset(strip_cur_brightness, 0, sizeof(strip_cur_brightness));
	memset(strip_tgt_brightness, 0, sizeof(strip_tgt_brightness));
	strip_interp_color_step = 0;
	strip_interp_brightness_step = 0;
	strip_interp_index = 0;
	neo_en = 0;
	neo_brightness = 0;
	neo_grb[0] = 0;
	neo_grb[1] = 0;
	neo_grb[2] = 0;
	last_error = 0;

	uart_write_str("uart_icepi: neopixel control ready.\r\n");

	while (1) {
		if (timer_poll_tick()) {
			ws2812_strip_interpolate_tick();
		}
		if (uart_read_nonblock()) {
			process_rx_byte((uint8_t)uart_read());
		}
	}
}
