#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/uart.h>

static void uart_write_str(const char *s)
{
	while (*s) {
		uart_write(*s++);
	}
}

static void uart_write_hex32(uint32_t v)
{
	for (int i = 7; i >= 0; i--) {
		uint8_t nib = (v >> (i * 4)) & 0xF;
		uart_write(nib < 10 ? ('0' + nib) : ('A' + (nib - 10)));
	}
}

static void uart_write_dec(int32_t v)
{
	char buf[12];
	int i = 11;
	uint32_t u = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;

	buf[i--] = '\0';
	if (v == 0) {
		buf[i] = '0';
		uart_write_str(&buf[i]);
		return;
	}
	while (u && i >= 0) {
		buf[i--] = '0' + (u % 10);
		u /= 10;
	}
	if (v < 0 && i >= 0) {
		buf[i--] = '-';
	}
	uart_write_str(&buf[i + 1]);
}

static void mlp2_write_input(uint8_t idx, int8_t val)
{
	mlp_accel_in_addr_write(idx);
	mlp_accel_in_data_write((uint8_t)val);
	mlp_accel_in_we_write(1);
}

static void mlp2_write_w1(uint8_t h, uint8_t i, int8_t val)
{
	uint16_t addr = ((uint16_t)h << 8) | i;
	mlp_accel_w1_addr_write(addr);
	mlp_accel_w1_data_write((uint8_t)val);
	mlp_accel_w1_we_write(1);
}

static void mlp2_write_b1(uint8_t h, int16_t val)
{
	mlp_accel_b1_addr_write(h);
	mlp_accel_b1_data_write((uint16_t)val);
	mlp_accel_b1_we_write(1);
}

static void mlp2_write_w2(uint8_t o, uint8_t h, int8_t val)
{
	uint16_t addr = ((uint16_t)o << 8) | h;
	mlp_accel_w2_addr_write(addr);
	mlp_accel_w2_data_write((uint8_t)val);
	mlp_accel_w2_we_write(1);
}

static void mlp2_write_b2(uint8_t o, int16_t val)
{
	mlp_accel_b2_addr_write(o);
	mlp_accel_b2_data_write((uint16_t)val);
	mlp_accel_b2_we_write(1);
}

int main(void)
{
	static const int8_t inputs[16] = {
		3, -2, 1, 4, -1, 2, 0, -3, 1, 1, -2, 2, -1, 0, 3, -2
	};
	static const int8_t w1[8][16] = {
		{ 1, -1, 0, 2, -2, 1, 1, 0, 1, -1, 2, 0, 0, 1, -1, 1 },
		{ 0, 2, -1, 1, 1, -2, 0, 1, -1, 0, 1, 2, -2, 1, 0, -1 },
		{ 2, 0, 1, -1, 0, 1, -2, 2, 1, 0, -1, 1, 2, -1, 0, 1 },
		{ -1, 1, 2, 0, 1, 0, -1, 1, 0, 2, 1, -2, 1, 0, -1, 2 },
		{ 1, 0, -2, 1, 2, -1, 0, 1, -1, 1, 0, 2, -2, 1, 0, -1 },
		{ 0, 1, 1, -2, 1, 0, 2, -1, 0, 1, -1, 2, 1, 0, -2, 1 },
		{ 2, -1, 0, 1, -1, 2, 1, 0, 1, -2, 0, 1, 2, -1, 0, 1 },
		{ -1, 0, 2, 1, 0, -1, 1, 2, -2, 1, 0, 1, -1, 2, 1, 0 },
	};
	static const int16_t b1[8] = { 1, -1, 0, 2, -2, 1, 0, -1 };
	static const int8_t w2[4][8] = {
		{ 1, -1, 2, 0, 1, 0, -1, 1 },
		{ 0, 2, -1, 1, -2, 1, 0, -1 },
		{ 1, 0, 1, -1, 0, 1, -2, 2 },
		{ -1, 1, 0, 2, 1, -2, 1, 0 },
	};
	static const int16_t b2[4] = { 0, 1, -1, 2 };

	uart_init();
	irq_setie(1);

	uart_write_str("\r\nMLP2 demo start\r\n");

	for (uint8_t i = 0; i < 16; i++) {
		mlp2_write_input(i, inputs[i]);
	}

	for (uint8_t h = 0; h < 8; h++) {
		for (uint8_t i = 0; i < 16; i++) {
			mlp2_write_w1(h, i, w1[h][i]);
		}
		mlp2_write_b1(h, b1[h]);
	}

	for (uint8_t o = 0; o < 4; o++) {
		for (uint8_t h = 0; h < 8; h++) {
			mlp2_write_w2(o, h, w2[o][h]);
		}
		mlp2_write_b2(o, b2[o]);
	}

	mlp_accel_ctrl_write(0x2); /* clear done */
	mlp_accel_ctrl_write(0x1); /* start */

	while (1) {
		uint32_t status = mlp_accel_status_read();
		if (status & 0x2) {
			break;
		}
	}

	uart_write_str("MLP2 done. Outputs:\r\n");
	for (uint8_t o = 0; o < 4; o++) {
		mlp_accel_out_addr_write(o);
		uint32_t raw = mlp_accel_out_data_read();
		int32_t val = (int32_t)raw;
		uart_write_str("  out[");
		uart_write_dec(o);
		uart_write_str("] = ");
		uart_write_dec(val);
		uart_write_str(" (0x");
		uart_write_hex32((uint32_t)val);
		uart_write_str(")\r\n");
	}

	while (1) {
		/* idle */
	}
}
