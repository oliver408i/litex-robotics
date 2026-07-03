/* IcePi Zero ESP32-C3 SPI flash-loader -- FPGA side (SRAM-resident).
 *
 * The FPGA is an SPI slave (gateware/spi_slave.py, CSR "c3"); the ESP32-C3
 * master streams a small command protocol over the dedicated C3 bus. This
 * firmware drains the slave RX FIFO, parses commands, and drives the LiteSPI
 * master (flash_w25q.c) to erase/program/verify the SPI NOR. The C3 owns image
 * staging + whole-image CRC, so there is NO on-FPGA image buffer and NO SDRAM
 * dependency -- the loader runs entirely from on-chip SRAM (SDRAM is down).
 *
 * Flow control: the BUSY/READY pad (IO4, the c3 `ready` CSR) is held LOW while a
 * slow flash op runs so the C3 pauses; raised when ready to accept again. Each
 * command pushes a reply (status [+ payload]) to the TX FIFO; the C3 reads it
 * once READY is high again.
 *
 * Bring-up: serial-boot over UART (SDRAM is down so load to SRAM):
 *   litex_term /dev/ttyUSB0 --speed 1000000 --kernel c3_loader.bin --kernel-adr 0x10000000
 *
 * Protocol v1 (multibyte little-endian). See docs/c3_loader.md.
 *   0x00 NOP                        -> (none) -- the dummy/read byte: the C3
 *                                      clocks 0x00s to shift a queued reply out
 *                                      on MISO; firmware ignores them so they
 *                                      are not mistaken for commands.
 *   0x01 PING                       -> 0xA5, jedec[4]
 *   0x02 ERASE   off[4] len[4]      -> status
 *   0x03 PROGRAM off[4] n[2] d[n]   -> status      (n <= 256)
 *   0x04 CRC     off[4] len[4]      -> status, crc32[4]
 *   0x05 REBOOT                     -> (ctrl_reset; no reply)
 */
#include <stdint.h>
#include <libbase/crc.h>
#include <generated/csr.h>
#include <generated/mem.h>

#define CMD_NOP     0x00
#define CMD_PING    0x01
#define CMD_ERASE   0x02
#define CMD_PROGRAM 0x03
#define CMD_CRC     0x04
#define CMD_REBOOT  0x05

#define ST_OK       0x00
#define ST_BAD_CMD  0x01
#define ST_BAD_ARG  0x02

#include "flash_w25q.h"

/* ---- UART logging (bring-up visibility) -----------------------------------
 * Polled writes straight to the UART CSRs -- NOT libbase uart_write(), which is
 * interrupt-driven and would block forever here (this firmware is pure-polling
 * and never enables IRQs / installs an isr()). */
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
static void log_hex(uint32_t v)
{
	log_puts("0x");
	for (int i = 28; i >= 0; i -= 4)
		log_char("0123456789abcdef"[(v >> i) & 0xf]);
}

/* ---- C3 SPI-slave transport ---------------------------------------------- */
/* Blocking byte read from the RX FIFO (bytes the C3 master clocked in). */
static uint8_t rx_byte(void)
{
	uint32_t t = 0;
	while (c3_rxempty_read()) {
		if (++t == 50000000) { log_puts("STUCK:rx_empty\n"); t = 0; }
	}
	return c3_rxtx_read();
}

/* Blocking byte push to the TX FIFO (clocked back out on MISO). */
static void tx_byte(uint8_t b)
{
	uint32_t t = 0;
	while (c3_txfull_read()) {
		if (++t == 50000000) { log_puts("STUCK:tx_full\n"); t = 0; }
	}
	c3_rxtx_write(b);
}

static uint32_t rx_u16(void) { uint32_t v = rx_byte(); v |= rx_byte() << 8; return v; }
static uint32_t rx_u32(void)
{
	uint32_t v = rx_byte();
	v |= (uint32_t)rx_byte() << 8;
	v |= (uint32_t)rx_byte() << 16;
	v |= (uint32_t)rx_byte() << 24;
	return v;
}
static void tx_u32(uint32_t v) { for (int i = 0; i < 32; i += 8) tx_byte((v >> i) & 0xff); }

static void ready(int r) { c3_ready_write(r ? 1 : 0); }

/* 256-byte page staging buffer (one PROGRAM command's worth). */
static uint8_t pagebuf[FLASH_PAGE_SIZE];

static void handle_program(void)
{
	uint32_t off = rx_u32();
	uint32_t n   = rx_u16();
	if (n > FLASH_PAGE_SIZE) {
		/* Drain the bogus payload so the stream stays framed, then NAK. */
		for (uint32_t i = 0; i < n; i++) (void)rx_byte();
		ready(0); tx_byte(ST_BAD_ARG); ready(1);
		return;
	}
	for (uint32_t i = 0; i < n; i++)
		pagebuf[i] = rx_byte();
	ready(0);                       /* busy: flash program in flight */
	flash_program(off, pagebuf, n);
	tx_byte(ST_OK);
	ready(1);
}

int main(void)
{
	ready(0);                       /* hold the C3 off until we are up */

	log_puts("\nc3_loader (FPGA side) up. flash JEDEC ");
	uint32_t id = flash_jedec_id();
	log_hex(id);
	log_puts(id == 0x00EF4018 ? " (W25Q128)\n" : " (UNEXPECTED)\n");
	log_puts("waiting for ESP32-C3 commands on the c3 SPI link...\n");

	/* Do NOT set ready=1 here. READY must only go high after reply bytes have
	 * been pushed to the TX FIFO. A premature ready=1 lets the C3 read an
	 * empty TX FIFO and get all-zeros on the first ping after reset. */

	for (;;) {
		uint8_t op = rx_byte();
		/* Log NON-zero bytes only. Logging is ~160us/line at 1Mbaud, so logging
		 * every 0x00 (NOP / noise) would back up the RX FIFO and stall command
		 * processing -- which was breaking the PING->reply timing. Draining a
		 * 0x00 silently is a ~ns FIFO pop, keeping the firmware responsive. */
		if (op != 0x00) { log_puts("rx "); log_hex(op); log_char('\n'); }
		switch (op) {
		case CMD_NOP:           /* dummy/read byte -- shift a reply out, ignore */
			log_char('.');
			break;
		case CMD_PING:
			ready(0);
			tx_byte(0xA5);
			tx_u32(id);
			ready(1);
			log_puts("tx ping: 0xA5 jedec="); log_hex(id); log_puts(" (ready=1)\n");
			break;
		case CMD_ERASE: {
			uint32_t off = rx_u32();
			uint32_t len = rx_u32();
			ready(0);
			flash_erase_range(off, len);
			tx_byte(ST_OK);
			ready(1);
			break;
		}
		case CMD_PROGRAM:
			handle_program();
			break;
		case CMD_CRC: {
			uint32_t off = rx_u32();
			uint32_t len = rx_u32();
			ready(0);
			uint32_t c = crc32((const unsigned char *)(SPIFLASH_BASE + off), len);
			tx_byte(ST_OK);
			tx_u32(c);
			ready(1);
			break;
		}
		case CMD_REBOOT:
			log_puts("reboot\n");
			ctrl_reset_write(1);
			break;
		default:
			ready(0);
			tx_byte(ST_BAD_CMD);
			ready(1);
			break;
		}
	}
	return 0;
}
