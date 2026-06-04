/* W25Q128JV erase/program over the LiteSPI master CSRs.
 *
 * SAFE ONLY from code running entirely out of SDRAM: a master command knocks
 * the flash out of the continuous-read mode the XIP path uses, so nothing may
 * fetch/read through the flash mmap while these run (see docs/xip_bios.md).
 * Self-contained rather than liblitespi because that lacks 64K block erase
 * and printf-spams a per-byte verify; readback verify here is a single CRC
 * over the mmap window done by the caller.
 */
#ifndef FLASH_W25Q_H
#define FLASH_W25Q_H

#include <stdint.h>

#define FLASH_SECTOR_SIZE 0x1000   /* 4 KB erase granularity (and alignment unit) */
#define FLASH_BLOCK_SIZE  0x10000  /* 64 KB block erase -- ~4x faster per byte    */
#define FLASH_PAGE_SIZE   256

/* JEDEC ID as 0x00MMTTCC (W25Q128JV = 0x00EF4018). */
uint32_t flash_jedec_id(void);

/* Erase [off, off+len) rounded up to 4 KB; off must be 4 KB-aligned.
 * Uses 64 KB block erases where they fit inside the rounded range. */
void flash_erase_range(uint32_t off, uint32_t len);

/* Page-program len bytes from src to flash offset off (any alignment). */
void flash_program(uint32_t off, const uint8_t *src, uint32_t len);

#endif
