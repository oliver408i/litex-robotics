/* Minimal bare-metal POSIX stubs for picolibc on the IcePi Zero SoC.
 *
 * There is no filesystem, so file ops fail gracefully (open -> -1), which makes
 * fopen() return NULL -- DOOM tolerates a missing config/savegame and uses
 * defaults. The WAD does NOT go through here; it is served by the memory-backed
 * w_file backend (w_file_icepi.c).
 *
 * NOT defined here (provided by picolibc / LiteX libc):
 *   sbrk       -> picolibc picosbrk.c, bounded by __heap_start/__heap_end (linker.ld)
 *   _exit,kill,getpid -> litex/soc/software/libc/missing.c
 * picolibc's tinystdio calls the bare POSIX names (open/read/write/...), so
 * these are defined WITHOUT the leading underscore. */
#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <generated/csr.h>

/* console out: route stdio to the LiteX UART (polled). */
static void uart_putc(char c)
{
    while (uart_txfull_read());
    uart_rxtx_write((unsigned char)c);
}

ssize_t write(int fd, const void *buf, size_t n)
{
    (void)fd;
    const char *p = (const char *)buf;
    for (size_t i = 0; i < n; i++) {
        if (p[i] == '\n')
            uart_putc('\r');
        uart_putc(p[i]);
    }
    return (ssize_t)n;
}

ssize_t read(int fd, void *buf, size_t n) { (void)fd; (void)buf; (void)n; return 0; }
int     open(const char *path, int flags, ...) { (void)path; (void)flags; errno = ENOENT; return -1; }
int     close(int fd) { (void)fd; return 0; }
off_t   lseek(int fd, off_t off, int whence) { (void)fd; (void)off; (void)whence; return (off_t)-1; }
int     unlink(const char *path) { (void)path; errno = ENOENT; return -1; }
int     rename(const char *from, const char *to) { (void)from; (void)to; errno = ENOENT; return -1; }
int     mkdir(const char *path, mode_t mode) { (void)path; (void)mode; errno = EACCES; return -1; }
int     isatty(int fd) { (void)fd; return 1; }
