/* IcePi Zero ESP32-C3 SPIBone flash loader -- FPGA side (SRAM-resident).
 *
 * The post-WINC loader rebuilt on the verified SPIBone transport. The ESP32-C3
 * (Wishbone master over SPI) writes a command + page data into an UNCACHED
 * mailbox RAM (gateware add_c3_mailbox @ 0x90000000) and rings a doorbell; this
 * firmware polls the mailbox and drives the LiteSPI MASTER (flash_w25q.c) to
 * erase/program/verify the SPI NOR. See icepi_zero_c3flash.py / docs/c3_loader.md.
 *
 * XIP-safety: this runs from main_ram (BRAM), never fetching through the flash
 * mmap, so it is safe to issue LiteSPI master commands (which knock the flash out
 * of continuous-read mode -- see flash_w25q.h / docs/boot_chain.md).
 *
 * Bring-up: serial-boot to main_ram (SDRAM is down):
 *   litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/c3_flash/c3_flash.bin
 *
 * Mailbox layout (shared with software/c3_flash_esp), all little-endian u32:
 *   +0x00 cmd     0=idle/done, else opcode (written LAST by the C3 = doorbell)
 *   +0x04 arg0    flash offset (ERASE/PROGRAM/CRC)
 *   +0x08 arg1    length / byte count
 *   +0x0C status  result code (ST_*), written by firmware before clearing cmd
 *   +0x10 result  JEDEC id (PING) or CRC32 (CRC)
 *   +0x40 data[]  up to 256 payload bytes for PROGRAM
 * Handshake: C3 writes arg0/arg1(+data) then cmd; firmware runs it, writes
 * result+status, then writes cmd=0. C3 polls cmd==0, then reads status+result.
 */
#include <stdint.h>
#include <system.h>            /* flush_cpu_dcache() */
#include <libbase/crc.h>
#include <generated/csr.h>
#include <generated/mem.h>

#include "flash_w25q.h"

/* Opcodes (mirror docs/c3_loader.md). */
#define CMD_PING     0x01
#define CMD_ERASE    0x02
#define CMD_PROGRAM  0x03
#define CMD_CRC      0x04
#define CMD_REBOOT   0x05

/* Status codes. */
#define ST_OK        0x00
#define ST_BAD_CMD   0x01
#define ST_BAD_ARG   0x02

/* Uncached mailbox (must match gateware add_c3_mailbox origin + the C3 side). */
#define MBX_BASE     0x90000000u
#define MBX(off)     (*(volatile uint32_t *)(MBX_BASE + (off)))
#define MBX_CMD      0x00
#define MBX_ARG0     0x04
#define MBX_ARG1     0x08
#define MBX_STATUS   0x0C
#define MBX_RESULT   0x10
#define MBX_DATA     0x40      /* byte offset of the 256-byte payload buffer */

/* ---- UART logging (bring-up visibility on the FTDI console) ----------------
 * Polled writes straight to the UART CSRs -- NOT libbase uart_write() (that is
 * IRQ-driven and this firmware never enables IRQs). */
static void log_char(char c)
{
	if (c == '\n') { while (uart_txfull_read()) ; uart_rxtx_write('\r'); }
	while (uart_txfull_read()) ;
	uart_rxtx_write(c);
}
static void log_puts(const char *s) { while (*s) log_char(*s++); }
static void log_hex(uint32_t v)
{
	log_puts("0x");
	for (int i = 28; i >= 0; i -= 4)
		log_char("0123456789abcdef"[(v >> i) & 0xf]);
}

int main(void)
{
	MBX(MBX_CMD) = 0;               /* idle before we advertise readiness */

	log_puts("\nc3_flash (SPIBone loader) up. flash JEDEC ");
	uint32_t id = flash_jedec_id();
	log_hex(id);
	log_puts(id == 0x00EF4018 ? " (W25Q128)\n" : " (UNEXPECTED)\n");
	log_puts("waiting for C3 mailbox commands @0x90000000...\n");

	for (;;) {
		uint32_t cmd = MBX(MBX_CMD);
		if (cmd == 0)
			continue;              /* idle: no doorbell */

		uint32_t status = ST_OK, result = 0;
		switch (cmd) {
		case CMD_PING:
			result = id;
			break;
		case CMD_ERASE:
			flash_erase_range(MBX(MBX_ARG0), MBX(MBX_ARG1));
			break;
		case CMD_PROGRAM: {
			uint32_t off = MBX(MBX_ARG0);
			uint32_t n   = MBX(MBX_ARG1);
			if (n > FLASH_PAGE_SIZE) { status = ST_BAD_ARG; break; }
			/* Program straight from the uncached mailbox buffer. */
			flash_program(off, (const uint8_t *)(MBX_BASE + MBX_DATA), n);
			break;
		}
		case CMD_CRC:
			/* Invalidate the D-cache first: a prior CRC pulled the old flash
			 * contents into cache; without this the read-back verify returns
			 * stale data and never sees what PROGRAM just wrote. */
			flush_cpu_dcache();
			result = crc32((const unsigned char *)(SPIFLASH_BASE + MBX(MBX_ARG0)),
			               MBX(MBX_ARG1));
			break;
		case CMD_REBOOT:
			log_puts("reboot\n");
			ctrl_reset_write(1);
			break;
		default:
			status = ST_BAD_CMD;
			break;
		}

		log_puts("cmd "); log_hex(cmd);
		log_puts(" -> status "); log_hex(status);
		log_puts(" result "); log_hex(result); log_char('\n');

		MBX(MBX_RESULT) = result;
		MBX(MBX_STATUS) = status;
		MBX(MBX_CMD)    = 0;       /* signal done LAST (result+status already set) */
	}
	return 0;
}
