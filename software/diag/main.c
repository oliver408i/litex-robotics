/* Generic hardware-bring-up diagnostic console.
 *
 * A standalone firmware (load it like any other test image, e.g.
 *   litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/diag/diag.bin)
 * that drops you into a single-keystroke menu. Each menu entry is a plain
 * void(void) routine; add a function and one table row to grow the console.
 * Routines are #ifdef-guarded on the CSRs they need, so this one main.c
 * compiles against any SoC top -- entries whose hardware is absent simply
 * don't appear in the menu.
 *
 * Shipped with one diagnostic: the 74HC595 SER->shift->RCLK->Qd loopback
 * (needs add_sr595_loopback() in the SoC top, i.e. icepi_zero_all.py). It is
 * the no-scope way to prove the expander's shift path is reliable after the
 * sr595.py output-register fix -- run 't' and watch for a zero error count.
 */
#include <stdint.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

/* ---- tiny UART logging (self-contained; no common/ deps) --------------- */
/* The hex helpers are part of the template toolkit; mark them maybe-unused so
 * a console that only prints decimals doesn't warn. */
#define DIAG_MAYBE_UNUSED __attribute__((unused))
static void log_char(char c) { if(c == '\n') uart_write('\r'); uart_write(c); }
static void log_puts(const char *s) { while(*s) log_char(*s++); }
static DIAG_MAYBE_UNUSED void log_hex4(uint8_t v) { v &= 0x0f; log_char(v < 10 ? '0' + v : 'a' + v - 10); }
static DIAG_MAYBE_UNUSED void log_hex8(uint8_t v) { log_hex4(v >> 4); log_hex4(v); }
static DIAG_MAYBE_UNUSED void log_hex16(uint16_t v) { log_hex8(v >> 8); log_hex8(v); }
static DIAG_MAYBE_UNUSED void log_hex32(uint32_t v) { log_hex16(v >> 16); log_hex16(v); }
static void log_nl(void) { log_char('\n'); uart_sync(); }

static void log_uint(uint32_t v)
{
	char buf[10];
	unsigned int i = 0;
	if(v == 0) { log_char('0'); return; }
	while(v && i < sizeof(buf)) { buf[i++] = '0' + v % 10; v /= 10; }
	while(i) log_char(buf[--i]);
}

/* Block until a key is pressed; echo it back. */
static char read_key(void)
{
	char c = uart_read();
	log_char(c);
	log_nl();
	return c;
}

/* =======================================================================
 * Diagnostics. Each is a void(void); register it in commands[] below.
 * Guard on the CSR(s) it needs so this file builds against any SoC top.
 * ======================================================================= */

#if defined(CSR_SR595_TEST_BASE) && defined(CSR_SR595_LOOP_BASE)
/* One shift takes ~45 us at the current 200 kHz SRCLK (18 half-periods @2.5 us
 * + FSM detect + the output-register stage). 200 us is generous settle margin
 * before readback; bump it if the gateware's SRCLK is ever slowed further. */
#define SR595_SETTLE_US 200

/* Drive Qd to `bit`, wait for the shift to land, return the looped-back level. */
static unsigned int sr595_drive_read(unsigned int bit)
{
	sr595_test_out_write(bit & 1u);
	busy_wait_us(SR595_SETTLE_US);
	return sr595_loop_in_read() & 1u;
}

/* Steady-state toggle test: alternate Qd 0/1 and confirm the loopback tracks.
 * A clean shift path reads back every level; a glitchy SRCLK/RCLK (the bug the
 * sr595.py output registers fix) drops or mis-latches bits -> nonzero errors. */
static void diag_sr595_toggle(void)
{
	const unsigned int iters = 2000;
	unsigned int errors = 0, first_fail = 0;

	log_puts("sr595: toggle loopback, "); log_uint(iters);
	log_puts(" iterations..."); log_nl();

	for(unsigned int i = 0; i < iters; i++) {
		unsigned int want = i & 1u;
		unsigned int got  = sr595_drive_read(want);
		if(got != want) {
			if(errors == 0) first_fail = i;
			errors++;
		}
	}

	log_puts("  errors = "); log_uint(errors);
	log_puts(" / ");          log_uint(iters);
	if(errors) { log_puts("  FIRST FAIL @ iter "); log_uint(first_fail); }
	else         log_puts("  PASS");
	log_nl();
}

/* Hold Qd at a fixed level and idle so you can meter the physical Qd pin (and
 * the IO10 jumper) with a multimeter. Prints the looped-back level once a
 * second; press any key to return to the menu. This is the test that tells a
 * dead/disconnected path apart from a glitchy one: if metered Qd tracks the
 * held level but the loop readback doesn't, the jumper/input is the problem;
 * if metered Qd itself doesn't move, the shift path isn't reaching the '595. */
static void diag_sr595_hold(unsigned int level)
{
	sr595_test_out_write(level & 1u);
	busy_wait_us(SR595_SETTLE_US);
	log_puts("holding Qd = "); log_uint(level & 1u);
	log_puts("  -- meter the Qd pin; press any key to stop"); log_nl();
	while(!uart_read_nonblock()) {
		log_puts("  loop reads "); log_uint(sr595_loop_in_read() & 1u); log_nl();
		busy_wait(1000);
	}
	(void)uart_read();   /* consume the keypress */
}

static void diag_sr595_hold_low(void)  { diag_sr595_hold(0); }
static void diag_sr595_hold_high(void) { diag_sr595_hold(1); }

/* Slow raw sweep: drive 0 then 1 with a long settle and print the raw readback
 * each way, ten times. "w0->r0 w1->r1" every line = path is clean; a column
 * stuck at one value localises the fault without a scope. */
static void diag_sr595_raw(void)
{
	for(unsigned int i = 0; i < 10; i++) {
		sr595_test_out_write(0); busy_wait(1);
		unsigned int r0 = sr595_loop_in_read() & 1u;
		sr595_test_out_write(1); busy_wait(1);
		unsigned int r1 = sr595_loop_in_read() & 1u;
		log_puts("  w0->r"); log_uint(r0);
		log_puts(" w1->r");  log_uint(r1); log_nl();
	}
}

/* Continuous-activity test: spin writing an ever-changing value so SR595 never
 * stops reshifting. SER/SRCLK/RCLK then carry a steady duty cycle a multimeter
 * can read as an average voltage -- the only way to check these dynamic lines
 * without a scope. Press any key to stop, then meter the 595's pins:
 *   SRCLK (pin 11): clearly non-zero, ~1-1.6 V (pulsing). Flat 0 V or flat
 *                   3.3 V => the FPGA isn't toggling it (loose jumper / wrong
 *                   pin / gateware not reflashed).
 *   RCLK  (pin 12): lower but still non-zero. Dead flat => latch never pulses,
 *                   which is exactly what leaves outputs stale & boot-random.
 *   SER   (pin 14): mid-ish and wandering as the data bit changes.
 * Wiggle each jumper while metering; a jump in the reading is your bad joint. */
static void diag_sr595_activity(void)
{
	log_puts("shifting continuously -- meter 595 pins 11/12/14, "
	         "wiggle jumpers; press any key to stop"); log_nl();
	unsigned int v = 0;
	while(!uart_read_nonblock()) {
		/* Always differ from last write so the FSM keeps reshifting. */
		sr595_test_out_write(v & 1u);
		v++;
	}
	(void)uart_read();
	sr595_test_out_write(0);
	log_puts("stopped."); log_nl();
}

/* Stress test: hammer Qd with back-to-back writes (no settle between) to land
 * CSR writes mid-shift, then settle once and confirm the register converged on
 * the last value. Exercises SR595's "value changed again -> reshift" path. */
static void diag_sr595_stress(void)
{
	const unsigned int rounds = 500;
	unsigned int errors = 0;

	log_puts("sr595: mid-shift retrigger stress, "); log_uint(rounds);
	log_puts(" rounds..."); log_nl();

	for(unsigned int r = 0; r < rounds; r++) {
		/* Spray writes with no wait; final intended level = (r & 1). */
		for(unsigned int k = 0; k < 8; k++)
			sr595_test_out_write((k ^ r) & 1u);
		sr595_test_out_write(r & 1u);

		busy_wait_us(SR595_SETTLE_US);
		if((sr595_loop_in_read() & 1u) != (r & 1u))
			errors++;
	}

	log_puts("  converged errors = "); log_uint(errors);
	log_puts(" / ");                    log_uint(rounds);
	log_puts(errors ? "  FAIL" : "  PASS");
	log_nl();
}
#endif /* SR595 loopback */

/* ---- example template entry (always present) --------------------------- */
static void diag_clocks(void)
{
	log_puts("sys clk = "); log_uint(CONFIG_CLOCK_FREQUENCY);
	log_puts(" Hz"); log_nl();
}

/* =======================================================================
 * Command table. key -> {label, routine}.
 * ======================================================================= */
typedef void (*diag_fn)(void);

struct diag_cmd {
	char         key;
	const char  *name;
	diag_fn      fn;
};

static const struct diag_cmd commands[] = {
#if defined(CSR_SR595_TEST_BASE) && defined(CSR_SR595_LOOP_BASE)
	{ 't', "74HC595 Qd loopback toggle test",   diag_sr595_toggle },
	{ 's', "74HC595 mid-shift retrigger stress", diag_sr595_stress },
	{ 'r', "74HC595 slow raw sweep (w0/w1->read)", diag_sr595_raw   },
	{ 'p', "74HC595 continuous shift (meter SER/SRCLK/RCLK)", diag_sr595_activity },
	{ 'l', "74HC595 hold Qd LOW  (meter the pin)", diag_sr595_hold_low  },
	{ 'h', "74HC595 hold Qd HIGH (meter the pin)", diag_sr595_hold_high },
#endif
	{ 'c', "print clocks",                       diag_clocks       },
};

#define N_COMMANDS (sizeof(commands) / sizeof(commands[0]))

static void print_menu(void)
{
	log_nl();
	log_puts("=== diag console ==="); log_nl();
	for(unsigned int i = 0; i < N_COMMANDS; i++) {
		log_puts("  ["); log_char(commands[i].key); log_puts("] ");
		log_puts(commands[i].name); log_nl();
	}
	log_puts("  [?] menu"); log_nl();
	log_puts("> "); uart_sync();
}

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_nl();
	log_puts("diag firmware up. sys clk = ");
	log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();
	print_menu();

	for(;;) {
		char c = read_key();
		if(c == '?' || c == '\r' || c == '\n') { print_menu(); continue; }

		const struct diag_cmd *cmd = 0;
		for(unsigned int i = 0; i < N_COMMANDS; i++)
			if(commands[i].key == c) { cmd = &commands[i]; break; }

		if(cmd) {
			log_puts("--- "); log_puts(cmd->name); log_puts(" ---"); log_nl();
			cmd->fn();
		} else {
			log_puts("unknown key '"); log_char(c); log_puts("' (? for menu)");
			log_nl();
		}
		log_puts("> "); uart_sync();
	}

	return 0;
}
