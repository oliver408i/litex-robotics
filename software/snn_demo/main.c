#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/uart.h>

static const int16_t k_demo_measurements[] = {
	1980, 1820, 1710, 1490, 1210, 980, 760, 610,
	520, 610, 790, 1080, 1370, 1640, 1870, 2040,
	1980, 1760, 1510, 1260, 1010, 840, 760, 810,
};

#define MEAS_HIGH  2048
#define MEAS_LOW  -2048
#define DELTA_HIGH 819
#define DELTA_LOW -819
#define SNN_DEMO_VERBOSE 1
#define SNN_OUTPUT_CSV 1
#define SNN_ENABLE_HOST_COMMANDS 1
#define HOST_LINE_MAX 48

static void uart_write_str(const char *s)
{
	while (*s) {
		uart_write(*s++);
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

#if !SNN_OUTPUT_CSV
static void uart_write_hex4(uint32_t v)
{
	for (int i = 3; i >= 0; i--) {
		uint8_t nib = (v >> (i * 4)) & 0xF;
		uart_write((nib < 10) ? ('0' + nib) : ('A' + (nib - 10)));
	}
}

static void uart_write_q4_12(int32_t raw)
{
	int32_t signed_raw = raw;
	int32_t whole;
	int32_t frac;

	if (signed_raw & 0x8000) {
		signed_raw -= 0x10000;
	}
	if (signed_raw < 0) {
		uart_write('-');
		signed_raw = -signed_raw;
	}

	whole = signed_raw >> 12;
	frac = ((signed_raw & 0x0fff) * 1000 + 2048) >> 12;

	uart_write_dec(whole);
	uart_write('.');
	uart_write((char)('0' + ((frac / 100) % 10)));
	uart_write((char)('0' + ((frac / 10) % 10)));
	uart_write((char)('0' + (frac % 10)));
}
#endif

static void delay_ms(uint32_t ms)
{
#ifdef CSR_TIMER0_BASE
	uint64_t ticks = ((uint64_t)CONFIG_CLOCK_FREQUENCY / 1000u) * ms;

	if (ticks == 0) {
		ticks = 1;
	}
	if (ticks > 0xffffffffu) {
		ticks = 0xffffffffu;
	}

	timer0_en_write(0);
	timer0_reload_write(0);
	timer0_load_write((uint32_t)ticks);
	timer0_en_write(1);

	while (1) {
		timer0_update_value_write(1);
		if (timer0_value_read() == 0) {
			break;
		}
	}
#else
	volatile uint32_t loops = ms * 8000u;
	while (loops--) {
		__asm__ volatile("nop");
	}
#endif
}

static void snn_clear_state(void)
{
	snn_control_write(0x2);
	snn_control_write(0x0);
}

static uint32_t snn_wait_done(void)
{
	uint32_t status;

	while (1) {
		status = snn_status_read();
		if ((status >> 1) & 0x1u) {
			return status;
		}
	}
}

static void snn_push_sample(int16_t measurement)
{
	snn_measurement_write((uint16_t)measurement);
	snn_control_write(0x1);
	snn_control_write(0x0);
}

static void snn_push_override_sample(int16_t measurement, uint8_t input_override)
{
	snn_input_override_write(input_override);
	snn_push_sample(measurement);
}

static uint8_t encode_spikes(int16_t measurement, int16_t prev_measurement)
{
	int16_t delta = (int16_t)(measurement - prev_measurement);
	uint8_t bits = 0;

	if (measurement > MEAS_HIGH) {
		bits |= 0x1;
	}
	if (measurement < MEAS_LOW) {
		bits |= 0x2;
	}
	if (delta > DELTA_HIGH) {
		bits |= 0x4;
	}
	if (delta < DELTA_LOW) {
		bits |= 0x8;
	}
	return bits;
}

static int parse_int32(const char **cursor, int32_t *out)
{
	const char *p = *cursor;
	int neg = 0;
	int32_t value = 0;
	int any = 0;

	while (*p == ' ' || *p == '\t' || *p == ',') {
		p++;
	}
	if (*p == '-') {
		neg = 1;
		p++;
	} else if (*p == '+') {
		p++;
	}
	while (*p >= '0' && *p <= '9') {
		value = (value * 10) + (*p - '0');
		any = 1;
		p++;
	}
	if (!any) {
		return 0;
	}
	*out = neg ? -value : value;
	*cursor = p;
	return 1;
}

static int parse_u8_auto(const char **cursor, uint8_t *out)
{
	const char *p = *cursor;
	uint32_t value = 0;
	int base = 10;
	int any = 0;

	while (*p == ' ' || *p == '\t' || *p == ',') {
		p++;
	}
	if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
		base = 16;
		p += 2;
	}
	while (1) {
		uint8_t digit;
		if (*p >= '0' && *p <= '9') {
			digit = *p - '0';
		} else if (base == 16 && *p >= 'a' && *p <= 'f') {
			digit = 10 + (*p - 'a');
		} else if (base == 16 && *p >= 'A' && *p <= 'F') {
			digit = 10 + (*p - 'A');
		} else {
			break;
		}
		if (digit >= base) {
			break;
		}
		value = (value * base) + digit;
		any = 1;
		p++;
	}
	if (!any) {
		return 0;
	}
	*out = value & 0xffu;
	*cursor = p;
	return 1;
}

static uint8_t clamp_spikes(uint8_t bits)
{
	return bits & 0x0fu;
}

static void snn_emit_header(void)
{
#if SNN_OUTPUT_CSV
	uart_write_str("# snn_demo protocol v1 raw_q4_12\r\n");
	uart_write_str("# commands: H, C, D, M <measurement_raw>, S <measurement_raw> <spikes>, ?\r\n");
	uart_write_str("type,t,measurement,delta,input_spikes,position,velocity,cycles,raw,draw,feat,dfeat,m0,m1,m2,m3,m4,beta,isum,rsum,mclip,status\r\n");
#endif
}

static void snn_emit_sample_row(char type, uint32_t t, int16_t measurement, int16_t delta, uint8_t input_spikes)
{
	uint16_t pos_raw = snn_position_read();
	uint16_t vel_raw = snn_velocity_read();
	uint16_t cyc_raw = snn_cycles_read();
	uint16_t meas_raw = snn_debug_measurement_raw_read();
	uint16_t dmeas_raw = snn_debug_delta_raw_read();
	uint16_t feat_raw = snn_debug_measurement_feature_read();
	uint16_t dfeat_raw = snn_debug_delta_feature_read();
	uint16_t mem0_raw = snn_debug_membrane0_read();
	uint16_t mem1_raw = snn_debug_membrane1_read();
	uint16_t mem2_raw = snn_debug_membrane2_read();
	uint16_t mem3_raw = snn_debug_membrane3_read();
	uint16_t mem4_raw = snn_debug_membrane4_read();
	uint16_t beta_raw = snn_debug_beta_product_read();
	uint16_t isum_raw = snn_debug_input_sum_read();
	uint16_t rsum_raw = snn_debug_recurrent_sum_read();
	uint16_t mclip_raw = snn_debug_membrane_clip_read();
	uint16_t status_raw = (uint16_t)snn_status_read();

#if SNN_OUTPUT_CSV
	uart_write(type);
	uart_write(',');
	uart_write_dec((int32_t)t);
	uart_write(',');
	uart_write_dec((int32_t)measurement);
	uart_write(',');
	uart_write_dec((int32_t)delta);
	uart_write(',');
	uart_write_dec((int32_t)input_spikes);
	uart_write(',');
	uart_write_dec((int16_t)pos_raw);
	uart_write(',');
	uart_write_dec((int16_t)vel_raw);
	uart_write(',');
	uart_write_dec((int32_t)cyc_raw);
	uart_write(',');
	uart_write_dec((int16_t)meas_raw);
	uart_write(',');
	uart_write_dec((int16_t)dmeas_raw);
	uart_write(',');
	uart_write_dec((int16_t)feat_raw);
	uart_write(',');
	uart_write_dec((int16_t)dfeat_raw);
	uart_write(',');
	uart_write_dec((int16_t)mem0_raw);
	uart_write(',');
	uart_write_dec((int16_t)mem1_raw);
	uart_write(',');
	uart_write_dec((int16_t)mem2_raw);
	uart_write(',');
	uart_write_dec((int16_t)mem3_raw);
	uart_write(',');
	uart_write_dec((int16_t)mem4_raw);
	uart_write(',');
	uart_write_dec((int16_t)beta_raw);
	uart_write(',');
	uart_write_dec((int16_t)isum_raw);
	uart_write(',');
	uart_write_dec((int16_t)rsum_raw);
	uart_write(',');
	uart_write_dec((int16_t)mclip_raw);
	uart_write(',');
	uart_write_dec((int32_t)status_raw);
	uart_write_str("\r\n");
#else
	if (type == 'S') {
		uart_write_str("sample ");
	} else if (type == 'P') {
		uart_write_str("probe ");
	} else {
		uart_write_str("host ");
	}
	uart_write_dec((int32_t)t);
	uart_write_str(" in=");
	uart_write_q4_12(((uint16_t)measurement) & 0xffff);
	uart_write_str(" pos=");
	uart_write_q4_12(pos_raw);
	uart_write_str(" vel=");
	uart_write_q4_12(vel_raw);
	uart_write_str(" cyc=");
	uart_write_dec(cyc_raw);
#if SNN_DEMO_VERBOSE
	uart_write_str(" raw=");
	uart_write_q4_12(meas_raw);
	uart_write_str(" draw=");
	uart_write_q4_12(dmeas_raw);
	uart_write_str(" feat=");
	uart_write_q4_12(feat_raw);
	uart_write_str(" dfeat=");
	uart_write_q4_12(dfeat_raw);
	uart_write_str(" m0=");
	uart_write_q4_12(mem0_raw);
	uart_write_str(" m1=");
	uart_write_q4_12(mem1_raw);
	uart_write_str(" m2=");
	uart_write_q4_12(mem2_raw);
	uart_write_str(" m3=");
	uart_write_q4_12(mem3_raw);
	uart_write_str(" m4=");
	uart_write_q4_12(mem4_raw);
	uart_write_str(" spk=0x");
	uart_write_hex4(input_spikes);
	uart_write_str(" beta=");
	uart_write_q4_12(beta_raw);
	uart_write_str(" isum=");
	uart_write_q4_12(isum_raw);
	uart_write_str(" rsum=");
	uart_write_q4_12(rsum_raw);
	uart_write_str(" mclip=");
	uart_write_q4_12(mclip_raw);
	uart_write_str(" status=0x");
	uart_write_hex4(status_raw);
#endif
	uart_write_str("\r\n");
#endif
}

#if SNN_ENABLE_HOST_COMMANDS
static int host_mode = 0;
static int16_t host_prev_measurement = 0;
static uint32_t host_t = 0;
static char host_line[HOST_LINE_MAX];
static unsigned int host_line_len = 0;

static void run_host_sample(int16_t measurement, uint8_t input_spikes)
{
	int16_t delta = (int16_t)(measurement - host_prev_measurement);

	snn_push_override_sample(measurement, clamp_spikes(input_spikes));
	host_prev_measurement = measurement;
	snn_wait_done();
	snn_emit_sample_row('H', host_t++, measurement, delta, clamp_spikes(input_spikes));
}

static void handle_host_line(char *line)
{
	const char *p = line;
	int32_t measurement;
	uint8_t input_spikes;

	while (*p == ' ' || *p == '\t') {
		p++;
	}
	if (*p == '?' || *p == 'h') {
		uart_write_str("# mode=");
		uart_write_str(host_mode ? "host" : "demo");
		uart_write_str(" commands: H host, C clear+host, D demo, M measurement_raw, S measurement_raw spikes\r\n");
		return;
	}
	if (*p == 'H') {
		host_mode = 1;
		uart_write_str("# host mode\r\n");
		return;
	}
	if (*p == 'C' || *p == 'c') {
		snn_clear_state();
		host_prev_measurement = 0;
		host_t = 0;
		host_mode = 1;
		uart_write_str("# host state cleared\r\n");
		return;
	}
	if (*p == 'D' || *p == 'd') {
		host_mode = 0;
		uart_write_str("# returning to demo mode\r\n");
		return;
	}
	if (*p == 'M' || *p == 'm') {
		p++;
		if (!parse_int32(&p, &measurement)) {
			uart_write_str("# error: M requires measurement_raw\r\n");
			return;
		}
		host_mode = 1;
		input_spikes = encode_spikes((int16_t)measurement, host_prev_measurement);
		run_host_sample((int16_t)measurement, input_spikes);
		return;
	}
	if (*p == 'S' || *p == 's') {
		p++;
		if (!parse_int32(&p, &measurement) || !parse_u8_auto(&p, &input_spikes)) {
			uart_write_str("# error: S requires measurement_raw and spikes\r\n");
			return;
		}
		host_mode = 1;
		run_host_sample((int16_t)measurement, input_spikes);
		return;
	}
	if (*p != '\0') {
		uart_write_str("# error: unknown command\r\n");
	}
}

static void poll_host_commands(void)
{
	while (uart_read_nonblock()) {
		char c = uart_read();
		if (c == '\r' || c == '\n') {
			host_line[host_line_len] = '\0';
			if (host_line_len) {
				handle_host_line(host_line);
			}
			host_line_len = 0;
		} else if (host_line_len < (HOST_LINE_MAX - 1)) {
			host_line[host_line_len++] = c;
		} else {
			host_line_len = 0;
			uart_write_str("# error: command too long\r\n");
		}
	}
}
#endif

int main(void)
{
	uart_init();
	irq_setie(1);

	uart_write_str("\r\nsnn_demo: LIF tracking estimator\r\n");
	snn_emit_header();

#ifndef CSR_SNN_CONTROL_ADDR
	uart_write_str("ERROR: snn peripheral not present. Rebuild gateware with --with-snn-poc\r\n");
	return 1;
#endif

	snn_clear_state();

	while (1) {
		int16_t prev_measurement = 0;

#if SNN_ENABLE_HOST_COMMANDS
		poll_host_commands();
		if (host_mode) {
			delay_ms(10);
			continue;
		}
#endif

		uart_write_str("\r\nreset reservoir state\r\n");
		snn_clear_state();

		for (unsigned int i = 0; i < (sizeof(k_demo_measurements) / sizeof(k_demo_measurements[0])); i++) {
			uint8_t encoded_bits;
			int16_t delta;

			encoded_bits = encode_spikes(k_demo_measurements[i], prev_measurement);
			delta = (int16_t)(k_demo_measurements[i] - prev_measurement);
			snn_input_override_write(encoded_bits);
			snn_push_sample(k_demo_measurements[i]);
			prev_measurement = k_demo_measurements[i];
			snn_wait_done();
			snn_emit_sample_row('S', i, k_demo_measurements[i], delta, encoded_bits);

			delay_ms(150);
#if SNN_ENABLE_HOST_COMMANDS
			poll_host_commands();
			if (host_mode) {
				break;
			}
#endif
		}

#if SNN_ENABLE_HOST_COMMANDS
		if (host_mode) {
			continue;
		}
#endif

		uart_write_str("recurrent probe\r\n");
		snn_clear_state();
		for (unsigned int i = 0; i < 12; i++) {
			snn_push_override_sample(0, 0x4);
			snn_wait_done();
			snn_emit_sample_row('P', i, 0, 0, 0x4);

			delay_ms(120);
#if SNN_ENABLE_HOST_COMMANDS
			poll_host_commands();
			if (host_mode) {
				break;
			}
#endif
		}

		delay_ms(1200);
	}

	return 0;
}
