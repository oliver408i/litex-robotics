#include <stdint.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/uart.h>

#ifndef CSR_RGB_LED_BASE
#error "This firmware requires rgb_led to be present in the SoC."
#endif

#define PREAMBLE_0 0xAA
#define PREAMBLE_1 0x55
#define MAX_PAYLOAD 254

#define CMD_PING          0x01
#define CMD_GET_VERSION   0x02
#define CMD_SET_NEOPIXEL  0x30
#define CMD_GET_NEOPIXEL  0x31
#define CMD_SET_STRIP_BULK 0x34

#define RSP_ERROR 0x7F

enum {
	ERR_BAD_LEN = 1,
	ERR_BAD_CHECKSUM = 2,
	ERR_BAD_CMD = 3,
	ERR_BAD_INDEX = 4,
};

#define STRIP_LED_COUNT 300

static uint8_t neo_enabled;
static uint8_t neo_brightness;
static uint8_t neo_grb[3];

static void uart_write_str(const char *value)
{
	while (*value) {
		uart_write(*value++);
	}
}

static uint8_t checksum_xor(const uint8_t *data, uint8_t len)
{
	uint8_t value = 0;
	for (uint8_t i = 0; i < len; ++i) {
		value ^= data[i];
	}
	return value;
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
	uint8_t header[4];
	uint8_t chk;

	header[0] = PREAMBLE_0;
	header[1] = PREAMBLE_1;
	header[2] = (uint8_t)(payload_len + 1);
	header[3] = cmd;

	chk = checksum_xor(&header[2], 2);
	if (payload_len != 0) {
		chk ^= checksum_xor(payload, payload_len);
	}

	for (uint8_t i = 0; i < sizeof(header); ++i) {
		uart_write((char)header[i]);
	}
	for (uint8_t i = 0; i < payload_len; ++i) {
		uart_write((char)payload[i]);
	}
	uart_write((char)chk);
}

static void send_error(uint8_t cmd, uint8_t err)
{
	uint8_t payload[2] = {cmd, err};
	send_frame(RSP_ERROR, payload, sizeof(payload));
}

static void strip_write(uint16_t index, uint8_t g, uint8_t r, uint8_t b, uint8_t brightness)
{
	uint16_t scaled_g = (uint16_t)g * brightness;
	uint16_t scaled_r = (uint16_t)r * brightness;
	uint16_t scaled_b = (uint16_t)b * brightness;

	rgb_led_strip_index_write(index);
	rgb_led_strip_color_hi_write((uint16_t)((((scaled_g + 127) / 255) & 0xFF) << 8) |
				     (uint16_t)(((scaled_r + 127) / 255) & 0xFF));
	rgb_led_strip_color_lo_write((uint8_t)(((scaled_b + 127) / 255) & 0xFF));
	rgb_led_strip_write_write(1);
}

static void neopixel_apply(void)
{
	if (!neo_enabled) {
		strip_write(0, 0, 0, 0, 0);
		return;
	}
	strip_write(0, neo_grb[0], neo_grb[1], neo_grb[2], neo_brightness);
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
		uint8_t version[2] = {1, 0};
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		send_frame((uint8_t)(cmd | 0x80), version, sizeof(version));
		break;
	}
	case CMD_SET_NEOPIXEL:
		if (payload_len != 5) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		neo_enabled = payload[0] ? 1 : 0;
		neo_brightness = payload[1];
		memcpy(neo_grb, &payload[2], sizeof(neo_grb));
		neopixel_apply();
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	case CMD_GET_NEOPIXEL: {
		uint8_t response[5] = {
			neo_enabled,
			neo_brightness,
			neo_grb[0],
			neo_grb[1],
			neo_grb[2],
		};
		if (payload_len != 0) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		send_frame((uint8_t)(cmd | 0x80), response, sizeof(response));
		break;
	}
	case CMD_SET_STRIP_BULK: {
		uint16_t start;
		uint8_t count;

		if (payload_len < 3) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}

		start = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
		count = payload[2];
		if ((uint16_t)(3 + count * 3) != payload_len) {
			send_error(cmd, ERR_BAD_LEN);
			break;
		}
		if (count == 0 || (uint16_t)(start + count) > STRIP_LED_COUNT) {
			send_error(cmd, ERR_BAD_INDEX);
			break;
		}

		for (uint8_t i = 0; i < count; ++i) {
			const uint8_t *color = &payload[3 + i * 3];
			strip_write((uint16_t)(start + i), color[0], color[1], color[2], 255);
		}
		send_frame((uint8_t)(cmd | 0x80), NULL, 0);
		break;
	}
	default:
		send_error(cmd, ERR_BAD_CMD);
		break;
	}
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
		state = (byte == PREAMBLE_1) ? 2 : 0;
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

	neo_enabled = 0;
	neo_brightness = 0;
	memset(neo_grb, 0, sizeof(neo_grb));
	neopixel_apply();

	uart_write_str("uart_icepi: neopixel bridge ready.\r\n");

	while (1) {
		if (uart_read_nonblock()) {
			process_rx_byte((uint8_t)uart_read());
		}
	}
}
