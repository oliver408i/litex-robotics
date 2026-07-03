/* IcePi Zero ESP32-C3 link CONTINUITY DIAGNOSTIC -- FPGA side (SRAM-resident).
 *
 * The C3<->FPGA SPI link has never reliably passed a byte though both sides look
 * healthy. This firmware throws away the SPI stack and drives the same five balls
 * as raw bidirectional GPIO (gateware add_c3_diag / LiteX GPIOTristate, CSR
 * "c3diag") so we can prove signal continuity -- and its direction -- wire by
 * wire, with no shift register, clock domain, or protocol in the way.
 *
 * Bit map (matches gateware _c3_diag_io):
 *   bit0 SCLK/P3   bit1 MOSI/L1   bit2 MISO/M2   bit3 CS#/J3   bit4 READY/R1
 *
 * Two modes, toggled by pressing ANY KEY in litex_term:
 *   READ  (default): all pins high-Z inputs. Prints each pin's live level on
 *                    change + a 1 Hz heartbeat. Run this while the C3 DRIVES ->
 *                    proves C3->FPGA continuity on every wire (incl. the M2/R1
 *                    input pads). A clean incrementing 0..31 count = all 5 good.
 *   DRIVE          : all pins outputs, driven with a binary counter so pin i
 *                    toggles at rate 2^i (SCLK fastest, READY slowest). Run this
 *                    while the C3 READS -> proves FPGA->C3 continuity + that each
 *                    FPGA output pad can actually drive its wire.
 *
 * CONTENTION: only ONE side may drive a wire at a time. Default is READ so the
 * out-of-box pairing (C3 drives, FPGA reads) is safe. Only flip to DRIVE once the
 * C3 side is reading. Both-READ is harmless; both-DRIVE fights on every wire.
 *
 * Serial-boot over UART (SDRAM is dead so load to main_ram BRAM):
 *   litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/c3_diag/c3_diag.bin
 */
#include <stdint.h>
#include <generated/csr.h>

#define M_SCLK  (1 << 0)
#define M_MOSI  (1 << 1)
#define M_MISO  (1 << 2)
#define M_CS    (1 << 3)
#define M_READY (1 << 4)
#define M_ALL   0x1F

/* ---- polled UART (this firmware never enables IRQs) ------------------------ */
static void log_char(char c)
{
	if (c == '\n') {
		while (uart_txfull_read())
			;
		uart_rxtx_write('\r');
	}
	while (uart_txfull_read())
		;
	uart_rxtx_write(c);
}
static void log_puts(const char *s) { while (*s) log_char(*s++); }

/* Non-blocking: return the next command byte from the host, or 0 if none.
 * We select mode by explicit char ('d'/'r'), NOT a toggle: a toggle bounces
 * uncontrollably if the console streams the key (autorepeat), which is exactly
 * what stopped the FPGA from *staying* in DRIVE. An explicit command sticks no
 * matter how many bytes arrive -- repeats of the same char are idempotent and
 * any other byte is ignored. */
static int poll_cmd(void)
{
	if (uart_rxempty_read())
		return 0;
	return uart_rxtx_read();
}

/* Rough spin delay (~ms); no timer dependency. Calibrated loosely for 50 MHz
 * sys -- exact timing does not matter for a hand-observed continuity test. */
static void delay_ms(uint32_t ms)
{
	volatile uint32_t i;
	while (ms--)
		for (i = 0; i < 8000; i++)
			;
}

/* Print a 5-bit pin state as named levels: "raw=0x05 SCLK=1 MOSI=0 MISO=1 CS=0 RDY=0" */
static void print_state(const char *tag, uint32_t v)
{
	static const char hexd[] = "0123456789abcdef";
	log_puts(tag);
	log_puts(" raw=0x");
	log_char(hexd[(v >> 4) & 0xf]);
	log_char(hexd[v & 0xf]);
	log_puts("  SCLK="); log_char((v & M_SCLK)  ? '1' : '0');
	log_puts(" MOSI=");  log_char((v & M_MOSI)  ? '1' : '0');
	log_puts(" MISO=");  log_char((v & M_MISO)  ? '1' : '0');
	log_puts(" CS=");    log_char((v & M_CS)    ? '1' : '0');
	log_puts(" RDY=");   log_char((v & M_READY) ? '1' : '0');
	log_char('\n');
}

int main(void)
{
	log_puts("\nc3_diag (FPGA side) up -- raw GPIO on the C3 SPI balls.\n");
	log_puts("bits: 0=SCLK/P3 1=MOSI/L1 2=MISO/M2 3=CS#/J3 4=READY/R1\n");
	log_puts("mode=READ (all high-Z inputs). Type 'd'=DRIVE, 'r'=READ (repeats OK, sticks).\n");
	log_puts("  READ : C3 should DRIVE -> watch for a clean 0..31 count (all wires good).\n");
	log_puts("  DRIVE: C3 should READ  -> we drive a 2^i-per-pin counter on all 5.\n\n");

	int drive = 0;                 /* start in READ (safe: no contention) */
	uint32_t last_in = 0xff;       /* force first print */
	uint32_t hb = 0;               /* heartbeat divider */
	uint32_t tick = 0;             /* DRIVE counter */

	c3diag_oe_write(0);            /* all inputs */
	c3diag_out_write(0);

	for (;;) {
		int cmd = poll_cmd();
		if ((cmd == 'd' || cmd == 'D') && !drive) {
			drive = 1;
			c3diag_oe_write(M_ALL);
			log_puts("\n-> DRIVE mode: driving all 5 pins (2^i counter). "
			         "Make sure the C3 is READING (not driving)!\n");
			last_in = 0xff; tick = 0; hb = 0;
		} else if ((cmd == 'r' || cmd == 'R') && drive) {
			drive = 0;
			c3diag_oe_write(0);
			c3diag_out_write(0);
			log_puts("\n-> READ mode: all pins high-Z. Have the C3 DRIVE now.\n");
			last_in = 0xff; tick = 0; hb = 0;
		}
		/* any other byte (incl. a repeat of the current mode's key) is ignored */

		if (drive) {
			/* Binary counter: pin i toggles at rate 2^i. A reader sees a clean
			 * incrementing 0..31; a stuck wire = a bit that never moves; a short
			 * = two bits that always move together. */
			c3diag_out_write(tick & M_ALL);
			if ((tick & M_ALL) == 0)
				print_state("DRIVE step wraps; now driving", 0);
			tick++;
			delay_ms(120);         /* SCLK bit ~4 Hz -- eyeball-able, full cycle ~4 s */
		} else {
			uint32_t v = c3diag_in_read() & M_ALL;
			if (v != last_in) {
				print_state("IN change:", v);
				last_in = v;
			}
			if (++hb >= 100) {     /* ~1 s heartbeat so a dead-quiet bus is obvious */
				print_state("IN hb:", v);
				hb = 0;
			}
			delay_ms(10);
		}
	}
	return 0;
}
