/* W25Q128JV erase/program via the LiteSPI master (see flash_w25q.h for the
 * XIP-safety contract). CSR access pattern follows liblitespi/spiflash.c:
 * byte-wide single-lane transfers, software-held CS. The mmap (XIP) port
 * simply stalls while the master holds CS -- harmless, nothing reads it. */
#include <stdint.h>

#include <generated/csr.h>

#include "flash_w25q.h"

#ifndef CSR_SPIFLASH_MASTER_CS_ADDR
#error "SoC has no LiteSPI master -- build icepi_zero_winc.py with --flash-master."
#endif

/* W25Q128JV command set */
#define CMD_JEDEC_ID     0x9f
#define CMD_READ_STATUS  0x05
#define CMD_WRITE_ENABLE 0x06
#define CMD_PAGE_PROGRAM 0x02
#define CMD_ERASE_4K     0x20
#define CMD_ERASE_64K    0xd8

static void master_begin(void)
{
	/* Drain any stale RX entries, then 8-bit single-lane transfers. */
	while(spiflash_master_status_read() &
	      (1u << CSR_SPIFLASH_MASTER_STATUS_RX_READY_OFFSET))
		(void)spiflash_master_rxtx_read();
	spiflash_master_phyconfig_write(
		(8u << CSR_SPIFLASH_MASTER_PHYCONFIG_LEN_OFFSET) |
		(1u << CSR_SPIFLASH_MASTER_PHYCONFIG_WIDTH_OFFSET) |
		(1u << CSR_SPIFLASH_MASTER_PHYCONFIG_MASK_OFFSET));
	spiflash_master_cs_write(1);
}

static void master_end(void)
{
	spiflash_master_cs_write(0);
}

static uint8_t xfer(uint8_t b)
{
	while(!(spiflash_master_status_read() &
	        (1u << CSR_SPIFLASH_MASTER_STATUS_TX_READY_OFFSET)))
		;
	spiflash_master_rxtx_write(b);
	while(!(spiflash_master_status_read() &
	        (1u << CSR_SPIFLASH_MASTER_STATUS_RX_READY_OFFSET)))
		;
	return (uint8_t)spiflash_master_rxtx_read();
}

static uint8_t read_status(void)
{
	uint8_t s;
	master_begin();
	(void)xfer(CMD_READ_STATUS);
	/* liblitespi reads SR several times before it is stable; keep the quirk. */
	(void)xfer(0);
	(void)xfer(0);
	s = xfer(0);
	master_end();
	return s;
}

static void wait_wip(void)
{
	while(read_status() & 1)   /* BUSY/WIP bit */
		;
}

static void write_enable(void)
{
	master_begin();
	(void)xfer(CMD_WRITE_ENABLE);
	master_end();
}

uint32_t flash_jedec_id(void)
{
	uint32_t id;
	master_begin();
	(void)xfer(CMD_JEDEC_ID);
	id  = (uint32_t)xfer(0) << 16;
	id |= (uint32_t)xfer(0) << 8;
	id |= (uint32_t)xfer(0);
	master_end();
	return id;
}

static void erase_cmd(uint8_t op, uint32_t addr)
{
	write_enable();
	master_begin();
	(void)xfer(op);
	(void)xfer((uint8_t)(addr >> 16));
	(void)xfer((uint8_t)(addr >> 8));
	(void)xfer((uint8_t)addr);
	master_end();
	wait_wip();
}

void flash_erase_range(uint32_t off, uint32_t len)
{
	/* Cover len with the minimal 4 KB-sector span; never erase past it (the
	 * next flash region -- bitstream/BIOS/fw -- may start right behind). */
	uint32_t pos = off;
	uint32_t end = off + ((len + FLASH_SECTOR_SIZE - 1) & ~(uint32_t)(FLASH_SECTOR_SIZE - 1));

	while(pos < end) {
		if(((pos & (FLASH_BLOCK_SIZE - 1)) == 0) && (end - pos) >= FLASH_BLOCK_SIZE) {
			erase_cmd(CMD_ERASE_64K, pos);
			pos += FLASH_BLOCK_SIZE;
		} else {
			erase_cmd(CMD_ERASE_4K, pos);
			pos += FLASH_SECTOR_SIZE;
		}
	}
}

void flash_program(uint32_t off, const uint8_t *src, uint32_t len)
{
	while(len) {
		uint32_t n = FLASH_PAGE_SIZE - (off & (FLASH_PAGE_SIZE - 1));
		if(n > len)
			n = len;

		write_enable();
		master_begin();
		(void)xfer(CMD_PAGE_PROGRAM);
		(void)xfer((uint8_t)(off >> 16));
		(void)xfer((uint8_t)(off >> 8));
		(void)xfer((uint8_t)off);
		for(uint32_t i = 0; i < n; i++)
			(void)xfer(src[i]);
		master_end();
		wait_wip();

		off += n;
		src += n;
		len -= n;
	}
}
