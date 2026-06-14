/* PISC sequencer-core bring-up firmware.
 *
 * Loads small hand-assembled PISC programs into the core's instruction memory
 * over the CSR bus, runs them, and checks the results -- the Phase-2 milestone
 * for verilog/pisc.v + gateware/pisc.py. Load it like any other test image:
 *   litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/pisc_test/pisc_test.bin
 *
 * Programs are the assembled output of tools/pisc_asm.py (see docs/pisc_isa.md);
 * the source is in the comment above each word array. Three tests cover the
 * three things the core has to get right to be useful:
 *   1. compute + control flow   (sum 1..10 -> 55, latched in r7/result)
 *   2. driving output ports     (SETB/CLRB pulse -> gpio_out)
 *   3. reading input ports      (IN -> result mirrors gpio_in)
 */
#include <stdint.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

/* ---- tiny UART logging ------------------------------------------------- */
static void log_char(char c) { if (c == '\n') uart_write('\r'); uart_write(c); }
static void log_puts(const char *s) { while (*s) log_char(*s++); }
static void log_nl(void) { log_char('\n'); uart_sync(); }
static void log_hex4(uint8_t v) { v &= 0x0f; log_char(v < 10 ? '0' + v : 'a' + v - 10); }
static void log_hex16(uint16_t v)
{
	log_hex4(v >> 12); log_hex4(v >> 8); log_hex4(v >> 4); log_hex4(v);
}
static void log_uint(uint32_t v)
{
	char buf[10];
	unsigned int i = 0;
	if (v == 0) { log_char('0'); return; }
	while (v && i < sizeof(buf)) { buf[i++] = '0' + v % 10; v /= 10; }
	while (i) log_char(buf[--i]);
}

#ifdef CSR_PISC_BASE
/* ---- PISC driver ------------------------------------------------------- */
/* control bit0 = run (pulse), bit1 = clr (pulse); status bit1 = halted. */
static inline void pisc_clr(void) { pisc_control_write(2u); }
static inline void pisc_go(void)  { pisc_control_write(1u); }
static inline int  pisc_halted(void) { return (pisc_status_read() >> 1) & 1u; }

static void pisc_load(const uint16_t *prog, unsigned int n)
{
	for (unsigned int i = 0; i < n; i++) {
		pisc_imem_addr_write(i);
		pisc_imem_data_write(prog[i]);
		pisc_imem_ctl_write(1u);          /* .we pulse */
	}
}

/* Clear, load, run to HLT, return result (= r7 at HLT). */
static uint16_t pisc_exec(const uint16_t *prog, unsigned int n)
{
	pisc_clr();
	pisc_load(prog, n);
	pisc_go();
	while (!pisc_halted())
		;
	return (uint16_t)pisc_result_read();
}

/* sum 1..10 -> r7 = 55
 *      LDI r1,0 / LDI r2,1 / LDI r3,11
 * loop:ADD r1,r1,r2 / ADDI r2,r2,1 / BNE r2,r3,loop / MOV r7,r1 / HLT */
static const uint16_t prog_sum[] = {
	0x3200, 0x3401, 0x360b, 0x1250, 0x3481, 0x86be, 0x1e40, 0xf000,
};

/* output pulse on port 0 bit 0: SETB / CLRB / SETB / HLT (ends high). */
static const uint16_t prog_pulse[] = {
	0xb000, 0xc000, 0xb000, 0xf000,
};

/* read input port 0 into r7: IN r1,0 / MOV r7,r1 / HLT. */
static const uint16_t prog_in[] = {
	0xa200, 0x1e40, 0xf000,
};

static void report(const char *name, uint32_t got, uint32_t want)
{
	log_puts("  "); log_puts(name); log_puts(": got 0x"); log_hex16(got);
	log_puts(" ("); log_uint(got); log_puts(")  want 0x"); log_hex16(want);
	log_puts(got == want ? "  PASS" : "  *** FAIL ***");
	log_nl();
}

static void run_tests(void)
{
	log_puts("PISC self-test:"); log_nl();

	/* 1. compute + control flow */
	report("sum 1..10 -> r7", pisc_exec(prog_sum, 8), 55);

	/* 2. drive output ports (check gpio_out, not result) */
	(void)pisc_exec(prog_pulse, 4);
	report("gpio_out[0] after pulse", pisc_gpio_out_read() & 1u, 1);

	/* 3. read input ports */
	pisc_gpio_in_write(0x1234u);
	report("IN port0 -> r7", pisc_exec(prog_in, 3), 0x1234);
}
#endif /* CSR_PISC_BASE */

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif
	log_nl();
	log_puts("pisc_test firmware up. sys clk = ");
	log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();

#ifdef CSR_PISC_BASE
	run_tests();
	log_puts("done. press any key to re-run."); log_nl();
	for (;;) {
		(void)uart_read();
		run_tests();
		log_puts("done. press any key to re-run."); log_nl();
	}
#else
	log_puts("ERROR: this SoC has no PISC peripheral (CSR_PISC_BASE undefined)."); log_nl();
	for (;;)
		;
#endif
	return 0;
}
