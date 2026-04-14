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

static void uart_write_str(const char *s)
{
	while (*s) {
		uart_write(*s++);
	}
}

#if SNN_DEMO_VERBOSE
static void uart_write_hex16(uint16_t v)
{
	for (int i = 3; i >= 0; i--) {
		uint8_t nib = (v >> (i * 4)) & 0xF;
		uart_write((nib < 10) ? ('0' + nib) : ('A' + (nib - 10)));
	}
}
#endif

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

int main(void)
{
	uart_init();
	irq_setie(1);

	uart_write_str("\r\nsnn_demo: LIF tracking estimator\r\n");

#ifndef CSR_SNN_CONTROL_ADDR
	uart_write_str("ERROR: snn peripheral not present. Rebuild gateware with --with-snn-poc\r\n");
	return 1;
#endif

	snn_clear_state();

	while (1) {
		int16_t prev_measurement = 0;

		uart_write_str("\r\nreset reservoir state\r\n");
		snn_clear_state();

		for (unsigned int i = 0; i < (sizeof(k_demo_measurements) / sizeof(k_demo_measurements[0])); i++) {
			uint16_t pos_raw;
			uint16_t vel_raw;
			uint16_t cyc_raw;
#if SNN_DEMO_VERBOSE
			uint16_t feat_raw;
			uint16_t dfeat_raw;
			uint16_t meas_raw;
			uint16_t dmeas_raw;
			uint16_t mem0_raw;
			uint16_t mem1_raw;
			uint16_t mem2_raw;
			uint16_t mem3_raw;
			uint16_t mem4_raw;
			uint16_t spk_raw;
			uint16_t beta_raw;
			uint16_t isum_raw;
			uint16_t rsum_raw;
			uint16_t mclip_raw;
#endif
			uint8_t encoded_bits;

			encoded_bits = encode_spikes(k_demo_measurements[i], prev_measurement);
			snn_input_override_write(encoded_bits);
			snn_push_sample(k_demo_measurements[i]);
			prev_measurement = k_demo_measurements[i];
			snn_wait_done();
			pos_raw = snn_position_read();
			vel_raw = snn_velocity_read();
			cyc_raw = snn_cycles_read();
#if SNN_DEMO_VERBOSE
			meas_raw = snn_debug_measurement_raw_read();
			dmeas_raw = snn_debug_delta_raw_read();
			feat_raw = snn_debug_measurement_feature_read();
			dfeat_raw = snn_debug_delta_feature_read();
			mem0_raw = snn_debug_membrane0_read();
			mem1_raw = snn_debug_membrane1_read();
			mem2_raw = snn_debug_membrane2_read();
			mem3_raw = snn_debug_membrane3_read();
			mem4_raw = snn_debug_membrane4_read();
			spk_raw = snn_debug_input_spikes_read();
			beta_raw = snn_debug_beta_product_read();
			isum_raw = snn_debug_input_sum_read();
			rsum_raw = snn_debug_recurrent_sum_read();
			mclip_raw = snn_debug_membrane_clip_read();
#endif

			uart_write_str("sample ");
			uart_write_dec((int32_t)i);
			uart_write_str(" in=");
			uart_write_q4_12(k_demo_measurements[i] & 0xffff);
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
			uart_write_hex16(spk_raw);
			uart_write_str(" beta=");
			uart_write_q4_12(beta_raw);
			uart_write_str(" isum=");
			uart_write_q4_12(isum_raw);
			uart_write_str(" rsum=");
			uart_write_q4_12(rsum_raw);
			uart_write_str(" mclip=");
			uart_write_q4_12(mclip_raw);
			uart_write_str(" status=0x");
			uart_write_hex16((uint16_t)snn_status_read());
#endif
			uart_write_str("\r\n");

			delay_ms(150);
		}

		uart_write_str("recurrent probe\r\n");
		snn_clear_state();
		for (unsigned int i = 0; i < 12; i++) {
			uint16_t pos_raw;
			uint16_t vel_raw;
			uint16_t cyc_raw;
			uint16_t mem4_raw;

			snn_push_override_sample(0, 0x4);
			snn_wait_done();
			pos_raw = snn_position_read();
			vel_raw = snn_velocity_read();
			cyc_raw = snn_cycles_read();
			mem4_raw = snn_debug_membrane4_read();

			uart_write_str("probe ");
			uart_write_dec((int32_t)i);
			uart_write_str(" inj=0x0004");
			uart_write_str(" pos=");
			uart_write_q4_12(pos_raw);
			uart_write_str(" vel=");
			uart_write_q4_12(vel_raw);
			uart_write_str(" m4=");
			uart_write_q4_12(mem4_raw);
			uart_write_str(" cyc=");
			uart_write_dec(cyc_raw);
			uart_write_str("\r\n");

			delay_ms(120);
		}

		delay_ms(1200);
	}

	return 0;
}
