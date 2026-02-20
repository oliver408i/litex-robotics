#include <stdint.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/uart.h>

#ifndef MLP_DIM_IN
#define MLP_DIM_IN 4
#endif

#ifndef MLP_DIM_HID
#define MLP_DIM_HID 8
#endif

#ifndef MLP_DIM_OUT
#define MLP_DIM_OUT 2
#endif

#ifndef MLP_SHIFT
#define MLP_SHIFT 8
#endif

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
	char buf[16];
	int i = 14;
	int neg = 0;
	buf[15] = '\0';
	if (v == 0) {
		uart_write('0');
		return;
	}
	if (v < 0) {
		neg = 1;
		v = -v;
	}
	while (v && i >= 0) {
		buf[i--] = '0' + (v % 10);
		v /= 10;
	}
	if (neg) {
		buf[i--] = '-';
	}
	uart_write_str(&buf[i + 1]);
}

static uint32_t align_up(uint32_t v, uint32_t align)
{
	return (v + (align - 1)) & ~(align - 1);
}

static int16_t clamp16(int32_t v)
{
	if (v > 32767) return 32767;
	if (v < -32768) return -32768;
	return (int16_t)v;
}

static void pack16_to_mem(volatile uint32_t *base, uint32_t count, const int16_t *src)
{
	for (uint32_t i = 0; i < count; i++) {
		uint16_t u = (uint16_t)src[i];
		base[i] = (uint32_t)u;
	}
}

static void unpack16_from_mem(const volatile uint32_t *base, uint32_t count, int16_t *dst)
{
	for (uint32_t i = 0; i < count; i++) {
		dst[i] = (int16_t)(base[i] & 0xFFFFu);
	}
}

static void mlp_cpu_ref(const int16_t *in, const int16_t *w1, const int16_t *b1,
                        const int16_t *w2, const int16_t *b2,
                        int16_t *out)
{
	int32_t hid[MLP_DIM_HID];
	for (uint32_t h = 0; h < MLP_DIM_HID; h++) {
		int32_t acc = b1[h];
		const int16_t *row = &w1[h * MLP_DIM_IN];
		for (uint32_t i = 0; i < MLP_DIM_IN; i++) {
			acc += (int32_t)row[i] * (int32_t)in[i];
		}
		acc >>= MLP_SHIFT;
		if (acc < 0) acc = 0;
		hid[h] = acc;
	}

	for (uint32_t o = 0; o < MLP_DIM_OUT; o++) {
		int32_t acc = b2[o];
		const int16_t *row = &w2[o * MLP_DIM_HID];
		for (uint32_t h = 0; h < MLP_DIM_HID; h++) {
			acc += (int32_t)row[h] * (int32_t)hid[h];
		}
		acc >>= MLP_SHIFT;
		out[o] = clamp16(acc);
	}
}

int main(void)
{
	uart_init();
	irq_setie(1);

	uart_write_str("mlp_demo: streaming MLP proof-of-concept\r\n");
#ifndef CSR_MLP_BASE
	uart_write_str("ERROR: MLP core not in gateware. Rebuild with --with-mlp-stream\r\n");
	return 1;
#endif

	extern uint32_t _erodata;
	uint32_t code_end = (uint32_t)&_erodata;
	uint32_t base = align_up(code_end, 0x1000u);

	/* Layout in SDRAM: IN, W1, B1, HID, W2, B2, OUT */
	uint32_t in_words  = MLP_DIM_IN;
	uint32_t w1_words  = MLP_DIM_HID * MLP_DIM_IN;
	uint32_t b1_words  = MLP_DIM_HID;
	uint32_t hid_words = MLP_DIM_HID;
	uint32_t w2_words  = MLP_DIM_OUT * MLP_DIM_HID;
	uint32_t b2_words  = MLP_DIM_OUT;
	uint32_t out_words = MLP_DIM_OUT;

	uint32_t in_off  = 0;
	uint32_t w1_off  = in_off + in_words;
	uint32_t b1_off  = w1_off + w1_words;
	uint32_t hid_off = b1_off + b1_words;
	uint32_t w2_off  = hid_off + hid_words;
	uint32_t b2_off  = w2_off + w2_words;
	uint32_t out_off = b2_off + b2_words;

	volatile uint32_t *mem = (volatile uint32_t *)base;

	int16_t in[MLP_DIM_IN] = { 32, -16, 24, 8 };
	int16_t w1[MLP_DIM_HID * MLP_DIM_IN];
	int16_t b1[MLP_DIM_HID];
	int16_t w2[MLP_DIM_OUT * MLP_DIM_HID];
	int16_t b2[MLP_DIM_OUT];
	int16_t out_ref[MLP_DIM_OUT];
	int16_t out_hw[MLP_DIM_OUT];

	uart_write_str("Input: ");
	for (uint32_t i = 0; i < MLP_DIM_IN; i++) {
		uart_write_dec(in[i]);
		uart_write_str(i + 1 == MLP_DIM_IN ? "\r\n" : ", ");
	}

	/* Deterministic toy weights for demo. */
	for (uint32_t h = 0; h < MLP_DIM_HID; h++) {
		b1[h] = (int16_t)(h - 4);
		for (uint32_t i = 0; i < MLP_DIM_IN; i++) {
			w1[h * MLP_DIM_IN + i] = (int16_t)((h + 1) * (i + 1));
		}
	}
	for (uint32_t o = 0; o < MLP_DIM_OUT; o++) {
		b2[o] = (int16_t)(o - 1);
		for (uint32_t h = 0; h < MLP_DIM_HID; h++) {
			w2[o * MLP_DIM_HID + h] = (int16_t)((o + 1) * (h + 1));
		}
	}

	uart_write_str("W1[0]: ");
	for (uint32_t i = 0; i < MLP_DIM_IN; i++) {
		uart_write_dec(w1[i]);
		uart_write_str(i + 1 == MLP_DIM_IN ? "\r\n" : ", ");
	}
	uart_write_str("B1[0..1]: ");
	uart_write_dec(b1[0]);
	uart_write_str(", ");
	uart_write_dec(b1[1]);
	uart_write_str("\r\n");

	pack16_to_mem(mem + in_off, in_words, in);
	pack16_to_mem(mem + w1_off, w1_words, w1);
	pack16_to_mem(mem + b1_off, b1_words, b1);
	pack16_to_mem(mem + w2_off, w2_words, w2);
	pack16_to_mem(mem + b2_off, b2_words, b2);

	uart_write_str("SDRAM IN[0..3]: ");
	for (uint32_t i = 0; i < MLP_DIM_IN; i++) {
		int16_t v = (int16_t)(mem[in_off + i] & 0xFFFFu);
		uart_write_dec(v);
		uart_write_str(i + 1 == MLP_DIM_IN ? "\r\n" : ", ");
	}
	uart_write_str("SDRAM W1[0..3]: ");
	for (uint32_t i = 0; i < MLP_DIM_IN; i++) {
		int16_t v = (int16_t)(mem[w1_off + i] & 0xFFFFu);
		uart_write_dec(v);
		uart_write_str(i + 1 == MLP_DIM_IN ? "\r\n" : ", ");
	}
	uart_write_str("SDRAM W1 hex[0..1]: 0x");
	uart_write_hex32(mem[w1_off + 0]);
	uart_write_str(" 0x");
	uart_write_hex32(mem[w1_off + 1]);
	uart_write_str("\r\n");
	uart_write_str("SDRAM IN hex[0..1]: 0x");
	uart_write_hex32(mem[in_off + 0]);
	uart_write_str(" 0x");
	uart_write_hex32(mem[in_off + 1]);
	uart_write_str("\r\n");
	uart_write_str("SDRAM B1[0..1]: ");
	for (uint32_t i = 0; i < 2; i++) {
		int16_t v = (int16_t)(mem[b1_off + i] & 0xFFFFu);
		uart_write_dec(v);
		uart_write_str(i + 1 == 2 ? "\r\n" : ", ");
	}

	mlp_dim_in_write(MLP_DIM_IN);
	mlp_dim_hid_write(MLP_DIM_HID);
	mlp_dim_out_write(MLP_DIM_OUT);
	mlp_shift_write(MLP_SHIFT);
	mlp_relu_en_write(1);

	mlp_base_in_write(base + (in_off << 2));
	mlp_base_w1_write(base + (w1_off << 2));
	mlp_base_b1_write(base + (b1_off << 2));
	mlp_base_w2_write(base + (w2_off << 2));
	mlp_base_b2_write(base + (b2_off << 2));
	mlp_base_hid_write(base + (hid_off << 2));
	mlp_base_out_write(base + (out_off << 2));

	uart_write_str("CFG dim_in=");
	uart_write_dec(mlp_dim_in_read());
	uart_write_str(" dim_hid=");
	uart_write_dec(mlp_dim_hid_read());
	uart_write_str(" dim_out=");
	uart_write_dec(mlp_dim_out_read());
	uart_write_str(" shift=");
	uart_write_dec(mlp_shift_read());
	uart_write_str(" relu=");
	uart_write_dec(mlp_relu_en_read());
	uart_write_str("\r\n");
	uart_write_str("BASE in=0x");
	uart_write_hex32(mlp_base_in_read());
	uart_write_str(" w1=0x");
	uart_write_hex32(mlp_base_w1_read());
	uart_write_str(" b1=0x");
	uart_write_hex32(mlp_base_b1_read());
	uart_write_str("\r\n");
	uart_write_str("BASE w2=0x");
	uart_write_hex32(mlp_base_w2_read());
	uart_write_str(" b2=0x");
	uart_write_hex32(mlp_base_b2_read());
	uart_write_str(" hid=0x");
	uart_write_hex32(mlp_base_hid_read());
	uart_write_str("\r\n");
	uart_write_str("BASE out=0x");
	uart_write_hex32(mlp_base_out_read());
	uart_write_str("\r\n");

	mlp_start_write(1);
	while (mlp_busy_read()) {
	}

	if (mlp_error_read()) {
		uart_write_str("MLP error: dimensions exceed hardware limits\r\n");
		return 2;
	}

	unpack16_from_mem(mem + out_off, out_words, out_hw);
	mlp_cpu_ref(in, w1, b1, w2, b2, out_ref);

	uart_write_str("DBG acc_lo=0x");
	uart_write_hex32(mlp_dbg_acc_lo_read());
	uart_write_str(" acc_hi=0x");
	uart_write_hex32(mlp_dbg_acc_hi_read());
	uart_write_str(" w=0x");
	uart_write_hex32(mlp_dbg_w_read() & 0xFFFFu);
	uart_write_str(" x=0x");
	uart_write_hex32(mlp_dbg_x_read() & 0xFFFFu);
	uart_write_str("\r\n");
	mlp_dbg_sel_write(0);
	uart_write_str("DBG in_mem[0]=0x");
	uart_write_hex32(mlp_dbg_in_read() & 0xFFFFu);
	uart_write_str(" hid_mem[0]=0x");
	uart_write_hex32(mlp_dbg_hid_read() & 0xFFFFu);
	uart_write_str(" in_x=0x");
	uart_write_hex32(mlp_dbg_in_x_read() & 0xFFFFu);
	uart_write_str(" hid_x=0x");
	uart_write_hex32(mlp_dbg_hid_x_read() & 0xFFFFu);
	uart_write_str(" req_addr=0x");
	uart_write_hex32(mlp_dbg_req_addr_read());
	uart_write_str("\r\n");

	uart_write_str("HW output: ");
	for (uint32_t o = 0; o < MLP_DIM_OUT; o++) {
		uart_write_dec(out_hw[o]);
		uart_write_str(o + 1 == MLP_DIM_OUT ? "\r\n" : ", ");
	}

	uart_write_str("SW output: ");
	for (uint32_t o = 0; o < MLP_DIM_OUT; o++) {
		uart_write_dec(out_ref[o]);
		uart_write_str(o + 1 == MLP_DIM_OUT ? "\r\n" : ", ");
	}

	uart_write_str("MLP demo done\r\n");
	return 0;
}
