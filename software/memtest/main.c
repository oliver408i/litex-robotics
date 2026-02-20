#include <stdint.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/uart.h>

#ifndef MEMTEST_OFFSET
#define MEMTEST_OFFSET 0x00100000u /* Additional offset into MAIN_RAM */
#endif

#ifndef MEMTEST_LEN
#define MEMTEST_LEN 0u /* 0 = test until end of MAIN_RAM */
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

static void print_range(uint32_t start, uint32_t end)
{
	uart_write_str("Range 0x");
	uart_write_hex32(start);
	uart_write_str(" - 0x");
	uart_write_hex32(end);
	uart_write_str(" (bytes: ");
	uart_write_dec(end - start);
	uart_write_str(")\r\n");
}

static uint32_t pattern_for_addr(uint32_t addr)
{
	return addr ^ 0xAAAAAAAAu;
}

static uint32_t align_up(uint32_t v, uint32_t align)
{
	return (v + (align - 1)) & ~(align - 1);
}

int main(void)
{
	uart_init();
	irq_setie(1);

	extern uint32_t _erodata;
	uint32_t code_end = (uint32_t)&_erodata;
	uint32_t base_min = align_up(code_end, 0x1000u);
	uint32_t base = MAIN_RAM_BASE + MEMTEST_OFFSET;
	uint32_t end  = MAIN_RAM_BASE + MAIN_RAM_SIZE;
	uint32_t len  = MEMTEST_LEN ? MEMTEST_LEN : (end - base);
	uint32_t stop = base + len;

	uart_write_str("memtest: software-only RAM tester\r\n");
	uart_write_str("MAIN_RAM_BASE=0x");
	uart_write_hex32(MAIN_RAM_BASE);
	uart_write_str(" MAIN_RAM_SIZE=0x");
	uart_write_hex32(MAIN_RAM_SIZE);
	uart_write_str("\r\n");
	uart_write_str("code_end=0x");
	uart_write_hex32(code_end);
	uart_write_str("\r\n");

	if (base < base_min) {
		base = base_min;
	}
	stop = MEMTEST_LEN ? (base + MEMTEST_LEN) : end;

	if (base >= end || stop > end || (base & 3) || (stop & 3)) {
		uart_write_str("memtest: invalid range\r\n");
		return 1;
	}

	print_range(base, stop);

	volatile uint32_t *p = (volatile uint32_t *)base;
	uint32_t words = (stop - base) / 4u;
	uint32_t progress_step = words / 16u;
	if (progress_step == 0) progress_step = 1;

	uart_write_str("Pass 1: write pattern\r\n");
	for (uint32_t i = 0; i < words; i++) {
		uint32_t addr = base + (i * 4u);
		p[i] = pattern_for_addr(addr);
		if ((i % progress_step) == 0) {
			uart_write_str("  write @ 0x");
			uart_write_hex32(addr);
			uart_write_str("\r\n");
		}
	}

	uart_write_str("Pass 2: verify pattern\r\n");
	for (uint32_t i = 0; i < words; i++) {
		uint32_t addr = base + (i * 4u);
		uint32_t exp  = pattern_for_addr(addr);
		uint32_t got  = p[i];
		if (got != exp) {
			uart_write_str("FAIL @ 0x");
			uart_write_hex32(addr);
			uart_write_str(" got 0x");
			uart_write_hex32(got);
			uart_write_str(" exp 0x");
			uart_write_hex32(exp);
			uart_write_str("\r\n");
			return 2;
		}
		if ((i % progress_step) == 0) {
			uart_write_str("  verify @ 0x");
			uart_write_hex32(addr);
			uart_write_str("\r\n");
		}
	}

	uart_write_str("Pass 3: write inverse pattern\r\n");
	for (uint32_t i = 0; i < words; i++) {
		uint32_t addr = base + (i * 4u);
		p[i] = ~pattern_for_addr(addr);
		if ((i % progress_step) == 0) {
			uart_write_str("  write @ 0x");
			uart_write_hex32(addr);
			uart_write_str("\r\n");
		}
	}

	uart_write_str("Pass 4: verify inverse pattern\r\n");
	for (uint32_t i = 0; i < words; i++) {
		uint32_t addr = base + (i * 4u);
		uint32_t exp  = ~pattern_for_addr(addr);
		uint32_t got  = p[i];
		if (got != exp) {
			uart_write_str("FAIL @ 0x");
			uart_write_hex32(addr);
			uart_write_str(" got 0x");
			uart_write_hex32(got);
			uart_write_str(" exp 0x");
			uart_write_hex32(exp);
			uart_write_str("\r\n");
			return 3;
		}
		if ((i % progress_step) == 0) {
			uart_write_str("  verify @ 0x");
			uart_write_hex32(addr);
			uart_write_str("\r\n");
		}
	}

	uart_write_str("memtest: PASS\r\n");
	return 0;
}
