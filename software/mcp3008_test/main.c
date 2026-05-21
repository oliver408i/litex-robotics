#include <stdint.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#ifndef CSR_MCP3008_BASE
#error "Build the SoC with --with-mcp3008-core before compiling software/mcp3008_test."
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

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("MCP3008 Verilog core test"); log_nl();
	log_puts("CSR uart    base=0x"); log_hex32(CSR_UART_BASE); log_nl();
	log_puts("CSR mcp3008 base=0x"); log_hex32(CSR_MCP3008_BASE); log_nl();
	log_puts("clock="); log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();

	mcp3008_enable_write(0);
	mcp3008_channel_mask_write(0xff);
	mcp3008_sample_interval_write(0);
	mcp3008_clear_update_write(0xff);
	mcp3008_enable_write(1);

	for(unsigned int i = 0; i < 10; i++) {
		busy_wait(20);
		dump_samples("mcp3008");
		mcp3008_clear_update_write(0xff);
	}

	log_puts("MCP3008 test complete"); log_nl();

	while(1)
		;

	return 0;
}
