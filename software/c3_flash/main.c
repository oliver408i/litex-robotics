/* IcePi Zero ESP32-C3 SPIBone flash loader -- FPGA side (SRAM/SDRAM-resident).
 *
 * The post-WINC loader rebuilt on the verified SPIBone transport. The ESP32-C3
 * (Wishbone master over SPI) writes a command + page data into an UNCACHED
 * mailbox RAM (gateware add_c3_mailbox @ 0x90000000) and rings a doorbell; this
 * firmware polls the mailbox and drives the LiteSPI MASTER (flash_w25q.c) to
 * erase/program/verify the SPI NOR. See icepi_zero_c3flash.py / docs/c3_loader.md.
 *
 * XIP-safety: this runs from main_ram (SDRAM, not flash), never fetching
 * through the flash mmap, so it is safe to issue LiteSPI master commands
 * (which knock the flash out of continuous-read mode -- see flash_w25q.h /
 * docs/boot_chain.md).
 *
 * Boot-manager triage: resident-by-default (opposite of winc_loader's
 * resident-unless-told-to-stay). On every boot, main() checks the sticky
 * boot_ctl flag (gateware add_boot_flag, docs/c3_loader.md); if it reads
 * BOOT_APP_MAGIC, it's cleared and try_chain_boot() copies the app slot
 * (FLASH_APP_OFFSET) into main_ram and jumps -- a one-shot request, since the
 * flag is already 0 by the time the app (or a subsequent reset) runs, so the
 * *next* reset lands back in this resident loader with no extra bookkeeping.
 * Otherwise (the common case -- default polarity, flag clear) this falls
 * straight into the resident mailbox loop below. Triggered by the host via
 * the CMD_BOOT_APP mailbox opcode (flash.py --boot-app).
 *
 * Bring-up: serial-boot to main_ram:
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
 * CMD_BOOT_APP is the exception: no reply, since it resets before it could
 * clear the doorbell -- the C3 side fires it and moves on (see main.cpp).
 */
#include <stdint.h>
#include <system.h>            /* flush_cpu_dcache(), flush_cpu_icache() */
#include <libbase/crc.h>
#include <libbase/uart.h>      /* uart_sync() */
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>     /* FLASH_APP_OFFSET, when the SoC provides it */

#include "flash_w25q.h"

/* Opcodes (mirror docs/c3_loader.md). */
#define CMD_PING     0x01
#define CMD_ERASE    0x02
#define CMD_PROGRAM  0x03
#define CMD_CRC      0x04
#define CMD_REBOOT   0x05
#define CMD_BOOT_APP 0x06   /* clear stay flag + reset -> chain-boot the app */
#define CMD_STAY     0x07   /* set stay flag + reset -> stay resident (flag path) */

/* Status codes. */
#define ST_OK        0x00
#define ST_BAD_CMD   0x01
#define ST_BAD_ARG   0x02

/* boot_ctl flag value that requests staying in the loader. This is the SECONDARY
 * (software) path -- the primary boot-mode signal is the IO4/R1 strap driven by
 * the C3's GPIO10 (see stay_requested()). Distinct arbitrary magic; the flag is
 * reset_less so it survives a soft/C3 reset and reads 0 at power-on. */
#define STAY_IN_LOADER_MAGIC 0x57A9F1A5u

#ifndef FLASH_APP_OFFSET   /* soc.h constant when the SoC provides it */
#define FLASH_APP_OFFSET 0x280000
#endif

/* SRAM-resident copy stub (chain_stub.S) -- never returns. */
extern void chain_stub(const void *src, void *dst, uint32_t len, uint32_t entry);

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

/* Validate + chain-boot the app .fbi at FLASH_APP_OFFSET; returns only if
 * there is no valid image (stays resident). Ported from
 * software/winc_loader/main.c's function of the same name -- the copy runs
 * from the SRAM-resident chain_stub because the app lands exactly where this
 * loader executes (main_ram). */
static void try_chain_boot(void)
{
	const uint8_t *img = (const uint8_t *)(SPIFLASH_BASE + FLASH_APP_OFFSET);

	flush_cpu_dcache();   /* the slot may have just been reflashed */
	uint32_t len = ((const volatile uint32_t *)img)[0];
	uint32_t crc = ((const volatile uint32_t *)img)[1];

	if (len == 0 || len == 0xffffffff ||
	    len > SPIFLASH_SIZE - FLASH_APP_OFFSET - 8) {
		log_puts("no app image at flash "); log_hex(FLASH_APP_OFFSET);
		log_puts(" -- staying resident\n");
		return;
	}
	if (crc32(img + 8, len) != crc) {
		log_puts("app image CRC mismatch -- staying resident\n");
		return;
	}

	log_puts("chain-booting app (");
	log_hex(len);
	log_puts(" B @"); log_hex(FLASH_APP_OFFSET); log_puts(")\n");
	/* visual separator (matches winc_loader/the BIOS "Liftoff!"): everything
	 * below is app, not loader. */
	log_puts("--============== \e[1mapp\e[0m ===============--\n");
	flush_cpu_icache();
	flush_cpu_dcache();
	chain_stub(img + 8, (void *)MAIN_RAM_BASE, (len + 3) & ~3u, MAIN_RAM_BASE);
	__builtin_unreachable();
}

/* Should the loader stay resident (for flashing) instead of chain-booting the
 * app? Primary signal: the boot-mode strap on IO4/R1 (loader_stay GPIOIn),
 * driven by the C3's GPIO10 -- pulled up, so high = boot the app (default), and
 * the C3 pulls it LOW to request staying. Secondary path: the reset_less
 * boot_ctl flag set to STAY_IN_LOADER_MAGIC (consumed here), for a software-
 * triggered stay when the strap isn't used. See docs/c3_loader.md. */
static int stay_requested(void)
{
	int stay = ((loader_stay_in_read() & 1u) == 0u);   /* strap asserted low */
	if (boot_ctl_flag_read() == STAY_IN_LOADER_MAGIC) {
		boot_ctl_flag_write(0);                        /* consume one-shot */
		stay = 1;
	}
	return stay;
}

int main(void)
{
	MBX(MBX_CMD) = 0;               /* idle before we advertise readiness */

	log_puts("\nc3_flash (SPIBone loader) up. flash JEDEC ");
	uint32_t id = flash_jedec_id();
	log_hex(id);
	log_puts(id == 0x00EF4018 ? " (W25Q128)\n" : " (UNEXPECTED)\n");

	/* Resident-only when asked (strap or flag); otherwise chain-boot the app.
	 * This is the auto-boot polarity: a normal power-on runs the app, and you
	 * enter the loader deliberately (C3 pulls the IO4 strap low, then resets). */
	if (stay_requested()) {
		log_puts("stay requested -- resident loader\n");
	} else {
		log_puts("no stay request -- ");
		try_chain_boot();   /* chain-boots the app; returns only without a valid image */
	}

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
		case CMD_BOOT_APP:
			/* Clear any stay flag and reboot. With the strap released (C3
			 * GPIO10 high) the loader chain-boots the app next boot. */
			log_puts("boot-app requested\n");
			uart_sync();
			boot_ctl_flag_write(0);
			ctrl_reset_write(1);
			break;
		case CMD_STAY:
			/* Software "stay in loader" request (secondary to the IO4 strap):
			 * set the flag and reboot; stay_requested() consumes it next boot. */
			log_puts("stay requested (flag)\n");
			uart_sync();
			boot_ctl_flag_write(STAY_IN_LOADER_MAGIC);
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
