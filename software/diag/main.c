#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/uart.h>

static volatile uint8_t timer0_fired;
static uint8_t timer0_isr_attached;

static void timer0_isr(void)
{
	timer0_ev_pending_write(1);
	timer0_fired = 1;
}

static void uart_write_str(const char *s)
{
	while (*s) {
		uart_write(*s++);
	}
}

static void uart_write_dec(uint32_t v)
{
	char buf[11];
	int i = 10;
	buf[i--] = '\0';
	if (v == 0) {
		buf[i] = '0';
		uart_write_str(&buf[i]);
		return;
	}
	while (v && i >= 0) {
		buf[i--] = '0' + (v % 10);
		v /= 10;
	}
	uart_write_str(&buf[i + 1]);
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

	if (irq_attach && !timer0_isr_attached) {
		irq_attach(TIMER0_INTERRUPT, timer0_isr);
		irq_setmask(irq_getmask() | (1 << TIMER0_INTERRUPT));
		timer0_ev_enable_write(1);
		timer0_isr_attached = 1;
	}

	timer0_en_write(0);
	timer0_reload_write(0);
	timer0_load_write((uint32_t)ticks);
	timer0_ev_pending_write(1);
	timer0_fired = 0;
	timer0_en_write(1);

	while (!timer0_fired) {
		__asm__ volatile("wfi");
	}
#else
	volatile uint32_t spin = ms * (CONFIG_CLOCK_FREQUENCY / 20000u);
	while (spin--) {
		__asm__ volatile("nop");
	}
#endif
}

static uint16_t adc_raw_to_mv(uint16_t raw)
{
	uint32_t mv = ((uint32_t)raw * 3300u + 511u) / 1023u;
	if (mv > 0xffffu) {
		mv = 0xffffu;
	}
	return (uint16_t)mv;
}

static void print_presence(const char *name, int present)
{
	uart_write_str("  ");
	uart_write_str(name);
	uart_write_str(": ");
	uart_write_str(present ? "OK" : "MISSING");
	uart_write_str("\r\n");
}

static void run_led_check(void)
{
#ifdef CSR_LEDS_BASE
	uart_write_str("LEDs: chase (user_led[1..4])...\r\n");
	for (uint8_t rep = 0; rep < 3; rep++) {
		for (uint8_t i = 0; i < 4; i++) {
			leds_out_write((uint32_t)(1u << i));
			delay_ms(120);
		}
	}
	leds_out_write(0x0);
	uart_write_str("LEDs: OK\r\n");
#else
	uart_write_str("LEDs: MISSING\r\n");
#endif
}

static void run_timer_check(void)
{
#ifdef CSR_TIMER0_BASE
	uart_write_str("Timer0: delay test...\r\n");
	delay_ms(50);
	uart_write_str("Timer0: OK\r\n");
#else
	uart_write_str("Timer0: MISSING\r\n");
#endif
}

static void run_mcp3008_check(void)
{
#ifdef CSR_MCP3008_BASE
	uint32_t timeout_ms = 400;
	uint8_t update_mask = 0;

	uart_write_str("MCP3008: sampling CH0-CH7...\r\n");
	mcp3008_enable_write(1);
	mcp3008_channel_mask_write(0xFF);
	mcp3008_sample_interval_write((uint32_t)(CONFIG_CLOCK_FREQUENCY / 100u));
	mcp3008_clear_update_write(0xFF);

	for (uint32_t i = 0; i < timeout_ms; i++) {
		update_mask = (uint8_t)mcp3008_update_mask_read();
		if (update_mask == 0xFF) {
			break;
		}
		delay_ms(1);
	}

	if (update_mask == 0) {
		uart_write_str("MCP3008: TIMEOUT (no update)\r\n");
		return;
	}

	for (uint8_t ch = 0; ch < 8; ch++) {
		uint16_t raw;
		uint16_t mv;

		switch (ch) {
		case 0: raw = (uint16_t)mcp3008_sample_ch0_read(); break;
		case 1: raw = (uint16_t)mcp3008_sample_ch1_read(); break;
		case 2: raw = (uint16_t)mcp3008_sample_ch2_read(); break;
		case 3: raw = (uint16_t)mcp3008_sample_ch3_read(); break;
		case 4: raw = (uint16_t)mcp3008_sample_ch4_read(); break;
		case 5: raw = (uint16_t)mcp3008_sample_ch5_read(); break;
		case 6: raw = (uint16_t)mcp3008_sample_ch6_read(); break;
		case 7: raw = (uint16_t)mcp3008_sample_ch7_read(); break;
		default: raw = 0; break;
		}

		mv = adc_raw_to_mv(raw);
		uart_write_str("  CH");
		uart_write_dec(ch);
		uart_write_str(": raw=");
		uart_write_dec(raw);
		uart_write_str(" mv=");
		uart_write_dec(mv);
		if ((update_mask & (1u << ch)) == 0) {
			uart_write_str(" (stale)");
		}
		uart_write_str("\r\n");
	}
	mcp3008_clear_update_write(0xFF);
#else
	uart_write_str("MCP3008: MISSING\r\n");
#endif
}

int main(void)
{
	uart_init();
	irq_setie(1);

	uart_write_str("diag: IcePi Zero peripheral check\r\n");
	uart_write_str("Clock Hz: ");
	uart_write_dec(CONFIG_CLOCK_FREQUENCY);
	uart_write_str("\r\n");
	uart_write_str("Note: SDRAM test skipped.\r\n\r\n");

	uart_write_str("CSR presence:\r\n");
#ifdef CSR_TIMER0_BASE
	print_presence("timer0", 1);
#else
	print_presence("timer0", 0);
#endif
#ifdef CSR_LEDS_BASE
	print_presence("leds", 1);
#else
	print_presence("leds", 0);
#endif
#ifdef CSR_MCP3008_BASE
	print_presence("mcp3008", 1);
#else
	print_presence("mcp3008", 0);
#endif
#ifdef CSR_SPIFLASH_CORE_BASE
	print_presence("spiflash_core", 1);
#else
	print_presence("spiflash_core", 0);
#endif
#ifdef CSR_SPIFLASH_BASE
	print_presence("spiflash", 1);
#else
	print_presence("spiflash", 0);
#endif

	uart_write_str("\r\nRunning checks...\r\n");
	run_timer_check();
	run_led_check();
	run_mcp3008_check();

	uart_write_str("\r\nDiag complete. Entering blinky.\r\n");

	while (1) {
#ifdef CSR_LEDS_BASE
		for (uint8_t i = 0; i < 4; i++) {
			leds_out_write((uint32_t)(1u << i));
			delay_ms(500);
		}
#else
		__asm__ volatile("wfi");
#endif
	}

	return 0;
}
