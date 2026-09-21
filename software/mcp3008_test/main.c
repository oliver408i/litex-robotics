/* MCP3008 bring-up test.
 *
 * Three questions, in the order they can go wrong:
 *
 *   1. Is anything on the SPI bus at all?  The FPGA pulls MISO up, so an
 *      absent or unpowered part clocks in all ones and every channel reads
 *      1023.  All eight channels pinned at exactly 1023 is therefore reported
 *      as NOT ANSWERING rather than as a reading.
 *   2. Are the channels distinguishable?  Eight identical non-1023 values
 *      means the mux is not being addressed -- config byte or bit alignment.
 *   3. Is CH0 actually moving?  A min/max/mean window over a few hundred
 *      conversions: a floating input wanders, a grounded one sits, and a
 *      dead bus does neither.
 *
 * Build against a bitstream from targets/colorlight_i9/adc.py, then:
 *   .venv/bin/python tools/socboot.py --kernel software/mcp3008_test/mcp3008_test.bin \
 *       --expect "MCP3008 test complete" --timeout 90
 */
#include <stdint.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#ifndef CSR_MCP3008_BASE
#error "Build the SoC from targets/colorlight_i9/adc.py before compiling software/mcp3008_test."
#endif

static void log_char(char c)
{
	if(c == '\n')
		uart_write('\r');
	uart_write(c);
}

static void log_puts(const char *s)
{
	while(*s)
		log_char(*s++);
}

static void log_hex4(uint8_t value)
{
	value &= 0x0f;
	log_char(value < 10 ? '0' + value : 'a' + value - 10);
}

static void log_hex8(uint8_t value)
{
	log_hex4(value >> 4);
	log_hex4(value);
}

static void log_hex16(uint16_t value)
{
	log_hex8(value >> 8);
	log_hex8(value);
}

static void log_hex32(uint32_t value)
{
	log_hex16(value >> 16);
	log_hex16(value);
}

static void log_uint(uint32_t value)
{
	char buf[10];
	unsigned int i = 0;

	if(value == 0) {
		log_char('0');
		return;
	}

	while(value && i < sizeof(buf)) {
		buf[i++] = '0' + value % 10;
		value /= 10;
	}

	while(i)
		log_char(buf[--i]);
}

static void log_nl(void)
{
	log_char('\n');
	uart_sync();
}

/* VREF is tied to the 3V3 rail on this rig, so a count is 3300/1023 mV. */
static uint16_t adc_raw_to_mv(uint16_t raw)
{
	return (uint16_t)(((uint32_t)raw * 3300u + 511u) / 1023u);
}

static uint16_t mcp3008_sample_read(unsigned int ch)
{
	switch(ch) {
	case 0: return (uint16_t)mcp3008_sample_ch0_read();
	case 1: return (uint16_t)mcp3008_sample_ch1_read();
	case 2: return (uint16_t)mcp3008_sample_ch2_read();
	case 3: return (uint16_t)mcp3008_sample_ch3_read();
	case 4: return (uint16_t)mcp3008_sample_ch4_read();
	case 5: return (uint16_t)mcp3008_sample_ch5_read();
	case 6: return (uint16_t)mcp3008_sample_ch6_read();
	case 7: return (uint16_t)mcp3008_sample_ch7_read();
	default: return 0;
	}
}

static void dump_samples(const char *tag)
{
	log_puts(tag);
	log_puts(": busy="); log_uint(mcp3008_busy_read());
	log_puts(" update=0x"); log_hex8((uint8_t)mcp3008_update_mask_read());
	log_puts(" last="); log_uint(mcp3008_last_channel_read());
	log_puts(" count="); log_uint(mcp3008_sample_count_read());
	log_nl();

	for(unsigned int ch = 0; ch < 8; ch++) {
		uint16_t raw = mcp3008_sample_read(ch);
		log_puts("mcp3008: ch"); log_uint(ch);
		log_puts(" raw="); log_uint(raw);
		log_puts(" mv="); log_uint(adc_raw_to_mv(raw));
		log_nl();
	}
}

/* Q1+Q2: classify the eight parked values. */
static void verdict_channels(void)
{
	uint16_t first = mcp3008_sample_read(0);
	unsigned int all_max = 1, all_same = 1, all_zero = 1;

	for(unsigned int ch = 0; ch < 8; ch++) {
		uint16_t raw = mcp3008_sample_read(ch);
		if(raw != 1023) all_max = 0;
		if(raw != first) all_same = 0;
		if(raw != 0) all_zero = 0;
	}

	if(all_max) {
		log_puts("VERDICT: NOT ANSWERING -- all 8 channels read 1023, which is "
		         "the FPGA's MISO pull-up. Check power, CS (A2) and D_OUT (K4).");
	} else if(all_zero) {
		log_puts("VERDICT: SUSPECT -- every channel reads 0. D_OUT stuck low, or "
		         "SCLK (K5) / D_IN (B3) not reaching the part.");
	} else if(all_same) {
		log_puts("VERDICT: SUSPECT -- all 8 channels read the same non-rail value, "
		         "so the mux is probably not being addressed (config byte).");
	} else {
		log_puts("VERDICT: ANSWERING -- channels differ, the part is on the bus.");
	}
	log_nl();
}

/* Q3: is CH0 moving? Re-read the parked CSR after each conversion completes. */
static void window_ch0(void)
{
	const unsigned int n = 256;
	uint32_t sum = 0;
	uint16_t min = 0xffff, max = 0;

	mcp3008_channel_mask_write(0x01);	/* CH0 only, so every conversion is CH0 */
	mcp3008_sample_interval_write(0);	/* free run */

	for(unsigned int i = 0; i < n; i++) {
		uint32_t before = mcp3008_sample_count_read();
		while(mcp3008_sample_count_read() == before)
			;
		uint16_t raw = mcp3008_sample_read(0);
		sum += raw;
		if(raw < min) min = raw;
		if(raw > max) max = raw;
	}

	log_puts("mcp3008: ch0 over "); log_uint(n); log_puts(" conversions:");
	log_puts(" min="); log_uint(min);
	log_puts(" max="); log_uint(max);
	log_puts(" mean="); log_uint(sum / n);
	log_puts(" p-p="); log_uint((uint32_t)(max - min));
	log_nl();

	mcp3008_channel_mask_write(0xff);
}

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("MCP3008 bring-up test"); log_nl();
	log_puts("CSR mcp3008 base=0x"); log_hex32(CSR_MCP3008_BASE); log_nl();
	log_puts("sys clock="); log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();
#ifdef MCP3008_SCLK_FREQ
	log_puts("adc SCLK="); log_uint(MCP3008_SCLK_FREQ); log_puts(" Hz (div=");
	log_uint(MCP3008_SCLK_DIV); log_puts(")"); log_nl();
#endif

	mcp3008_enable_write(0);
	mcp3008_channel_mask_write(0xff);
	mcp3008_sample_interval_write(0);
	mcp3008_clear_update_write(0xff);
	mcp3008_enable_write(1);

	for(unsigned int i = 0; i < 5; i++) {
		busy_wait(20);
		dump_samples("mcp3008");
		mcp3008_clear_update_write(0xff);
	}

	verdict_channels();
	window_ch0();

	log_puts("MCP3008 test complete"); log_nl();

	while(1)
		;

	return 0;
}
