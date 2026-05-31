/* TEMPORARY ring-oscillator demo firmware.
 *
 * Streams the measured ring-oscillator frequency over UART. Build the matching
 * gateware with icepi_zero_ringosc.py first. Watch the frequency wander when
 * you warm or cool the FPGA -- that drift is the whole point of the experiment.
 */
#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/uart.h>

#include "log.h"

/* These must match the RingMonitor parameters in icepi_zero_ringosc.py.
 * gate_cycles = sys_clk_freq * gate_ms / 1000, and the counter sees the ring
 * divided by RING_PRESCALE (= 2**prescale_bits). */
#define RING_GATE_MS  10u
#define RING_PRESCALE 256u  /* prescale_bits = 8 */

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

/* Print v as "W.FFF MHz" given a frequency in Hz. */
static void log_mhz(uint64_t hz)
{
	uint32_t whole = (uint32_t)(hz / 1000000u);
	uint32_t frac  = (uint32_t)((hz % 1000000u) / 1000u); /* milli-MHz = kHz */

	log_uint(whole);
	log_char('.');
	log_char('0' + (frac / 100) % 10);
	log_char('0' + (frac / 10) % 10);
	log_char('0' + frac % 10);
	log_puts(" MHz");
}

int main(void)
{
	uart_init();
	irq_setie(1);   /* libbase UART is interrupt-driven; without this the TX
	                   ring buffer fills and uart_write() hangs mid-string. */

	log_puts("\r\nringosc_demo: ring-oscillator frequency monitor\r\n");

#ifndef CSR_RINGOSC_COUNT_ADDR
	log_puts("ERROR: ringosc peripheral missing. Build with icepi_zero_ringosc.py\r\n");
	return 1;
#else
	const uint32_t gate_cycles = (uint32_t)CONFIG_CLOCK_FREQUENCY / 1000u * RING_GATE_MS;

	ringosc_enable_write(1);
	while (!ringosc_valid_read()) {
		/* wait for the first full measurement window */
	}

	uint32_t cmin = 0xffffffffu;
	uint32_t cmax = 0;

#ifdef CSR_RINGOSC_HEATER_ADDR
	int heater = 0;
	ringosc_heater_write(0);
	log_puts("commands: 'h' toggle heater, 'r' reset min/max\r\n");
#endif

	while (1) {
#ifdef CSR_RINGOSC_HEATER_ADDR
		if (uart_read_nonblock()) {
			char c = uart_read();
			if (c == 'h') {
				heater = !heater;
				ringosc_heater_write(heater);
				cmin = 0xffffffffu;       /* drop stale baseline */
				cmax = 0;
				log_puts(heater ? "# heater ON\r\n" : "# heater OFF\r\n");
			} else if (c == 'r') {
				cmin = 0xffffffffu;
				cmax = 0;
			}
		}
#endif

		uint32_t count = ringosc_count_read();
		uint64_t hz = (uint64_t)count * RING_PRESCALE * CONFIG_CLOCK_FREQUENCY / gate_cycles;

		if (count < cmin) {
			cmin = count;
		}
		if (count > cmax) {
			cmax = count;
		}

		log_puts("count="); log_uint(count);
		log_puts("  ~"); log_mhz(hz);
#ifdef CSR_RINGOSC_HEATER_ADDR
		log_puts(heater ? "  HEAT" : "      ");
#endif
		log_puts("  [min="); log_uint(cmin);
		log_puts(" max="); log_uint(cmax);
		log_puts(" spread="); log_uint(cmax - cmin);
		log_puts("]");
		log_nl();

		delay_ms(250);
	}

	return 0;
#endif
}
