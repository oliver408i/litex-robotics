/* Memory-backed WAD file backend for the IcePi Zero (no filesystem).
 *
 * DOOM's w_file abstraction (w_file.c) always calls `stdc_wad_file.OpenFile`.
 * We provide that symbol here, backed by the WAD living in SPI flash and read
 * through the XIP mmap window -- zero copy: W_Read memcpy's lump bytes straight
 * out of flash, and W_CacheLump copies them once into the zone heap.
 *
 * The shareware DOOM1.WAD is flashed to flash offset 0x400000, which the LiteSPI
 * XIP region maps to CPU address SPIFLASH_BASE + 0x400000. Override the address
 * or length with -DDOOM_WAD_ADDR / -DDOOM_WAD_LEN if the layout changes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <generated/mem.h>

#include "doomtype.h"
#include "w_file.h"

#ifndef DOOM_WAD_ADDR
#define DOOM_WAD_ADDR (SPIFLASH_BASE + 0x400000u)
#endif
#ifndef DOOM_WAD_LEN
#define DOOM_WAD_LEN  4196020u        /* shareware DOOM1.WAD */
#endif

wad_file_class_t stdc_wad_file;       /* defined at bottom; forward for handle */

static int is_wad_id(const byte *p)
{
    return (memcmp(p, "IWAD", 4) == 0) || (memcmp(p, "PWAD", 4) == 0);
}

static wad_file_t *IcePi_OpenFile(char *path)
{
    (void)path;
    byte *base = (byte *)(uintptr_t)DOOM_WAD_ADDR;

    /* Diagnostic: show exactly what is at the WAD address in flash. */
    printf("wad@%p: %02x %02x %02x %02x  '%c%c%c%c'\n",
           base, base[0], base[1], base[2], base[3],
           base[0] >= 32 && base[0] < 127 ? base[0] : '.',
           base[1] >= 32 && base[1] < 127 ? base[1] : '.',
           base[2] >= 32 && base[2] < 127 ? base[2] : '.',
           base[3] >= 32 && base[3] < 127 ? base[3] : '.');

    unsigned int len = DOOM_WAD_LEN;

    /* Accept either a raw WAD (IWAD/PWAD at offset 0) or an .fbi-wrapped one
     * (8-byte LiteX length+CRC header, WAD id at offset 8). */
    if (!is_wad_id(base) && is_wad_id(base + 8)) {
        printf("wad: .fbi wrapper detected, skipping 8-byte header\n");
        base += 8;
        len  -= 8;
    }

    wad_file_t *w = malloc(sizeof(wad_file_t));
    if (w == NULL)
        return NULL;
    w->file_class = &stdc_wad_file;
    w->mapped     = base;
    w->length     = len;
    return w;
}

static void IcePi_CloseFile(wad_file_t *w)
{
    free(w);
}

static size_t IcePi_Read(wad_file_t *w, unsigned int offset,
                         void *buffer, size_t buffer_len)
{
    if (offset >= w->length)
        return 0;
    if (offset + buffer_len > w->length)
        buffer_len = w->length - offset;
    memcpy(buffer, w->mapped + offset, buffer_len);
    return buffer_len;
}

wad_file_class_t stdc_wad_file =
{
    IcePi_OpenFile,
    IcePi_CloseFile,
    IcePi_Read,
};
