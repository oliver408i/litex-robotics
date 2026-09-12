/* Board-agnostic RV32 smoke test.
 *
 * Deliberately depends on nothing but the CPU, the UART and main_ram, so it
 * runs on any SoC this repo builds -- IcePi Zero or Colorlight i9 -- by
 * pointing BUILD_DIR at that board's build tree:
 *
 *     make -C software/rv32_test BUILD_DIR=/path/to/build
 *     litex_term /dev/icelink-uart --speed 115200 --kernel software/rv32_test/rv32_test.bin
 *
 * What a clean run actually proves, in order:
 *
 *   1. serial boot landed the image in main_ram and jumped to it -- so the CPU
 *      is fetching instructions out of SDRAM, not just out of ROM. On a board
 *      whose SDRAM PHY is wrong this never prints at all.
 *   2. the integer and multiply/divide paths give known-good answers.
 *   3. main_ram survives a write/read-back sweep at three access widths, done
 *      from C with the cache in play -- which is a different stress from the
 *      BIOS memtest and catches byte/halfword lane problems the BIOS misses.
 */
#include <stdint.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

/* ---- minimal output (no printf: keeps the image small and dependency-free) */
static void put(char c)        { if (c == '\n') uart_write('\r'); uart_write(c); }
static void puts_(const char *s) { while (*s) put(*s++); }
static void put_u32(uint32_t v)
{
    char t[11]; int n = 0;
    if (!v) t[n++] = '0';
    while (v) { t[n++] = (char)('0' + v % 10u); v /= 10u; }
    while (n) put(t[--n]);
}
static void put_hex(uint32_t v)
{
    static const char h[] = "0123456789abcdef";
    puts_("0x");
    for (int i = 28; i >= 0; i -= 4) put(h[(v >> i) & 0xf]);
}

static int failures;
static void check(const char *what, uint32_t got, uint32_t want)
{
    puts_(got == want ? "  ok   " : "  FAIL ");
    puts_(what);
    if (got != want) {
        puts_("  got "); put_hex(got);
        puts_(" want "); put_hex(want);
        failures++;
    }
    put('\n');
}

/* ---- integer datapath ---------------------------------------------------- */
/* volatile inputs so the compiler cannot constant-fold the whole thing away
 * and leave us testing nothing. */
static volatile uint32_t a = 0x12345678u, b = 0x9abcdef0u;
static volatile int32_t  sa = -1234567, sb = 76543;

static void test_alu(void)
{
    puts_("integer datapath\n");
    check("add",     a + b,            0xacf13568u);
    check("sub",     b - a,            0x88888878u);
    check("and",     a & b,            0x12345670u);
    check("or",      a | b,            0x9abcdef8u);
    check("xor",     a ^ b,            0x88888888u);
    check("sll",     a << 5,           0x468acf00u);
    check("srl",     b >> 7,           0x013579bdu);
    check("sra",     (uint32_t)(sa >> 3), 0xfffda52fu);
    check("mul",     a * b,            0x242d2080u);
    check("mulhu",   (uint32_t)(((uint64_t)a * (uint64_t)b) >> 32), 0x0b00ea4eu);
    check("divu",    b / a,            8u);
    check("remu",    b % a,            0x091a2b30u);
    check("div",     (uint32_t)(sa / sb), 0xfffffff0u);
    check("rem",     (uint32_t)(sa % sb), 0xffffd969u);
}

/* ---- main_ram ------------------------------------------------------------ */
/* Test a window well above where this image is loaded, so we never overwrite
 * ourselves. The image lands at MAIN_RAM_BASE. */
#define TEST_OFF   (1u << 20)          /* 1 MiB in */
#define TEST_WORDS (16u * 1024u)       /* 64 KiB */

static uint32_t prng(uint32_t x)       /* xorshift32: cheap, no tables */
{
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return x;
}

static void test_main_ram(void)
{
    volatile uint32_t *w = (volatile uint32_t *)(MAIN_RAM_BASE + TEST_OFF);
    uint32_t bad = 0, seed;

    puts_("main_ram @ "); put_hex(MAIN_RAM_BASE + TEST_OFF);
    puts_(", "); put_u32(TEST_WORDS * 4u / 1024u); puts_(" KiB\n");

    /* 32-bit sweep with a pseudorandom pattern -- a constant would not catch a
     * stuck address line, and the sequence is reproducible from the seed. */
    seed = 0xa5a5a5a5u;
    for (uint32_t i = 0; i < TEST_WORDS; i++) { seed = prng(seed); w[i] = seed; }
    flush_cpu_dcache();
#ifdef CONFIG_L2_SIZE
    flush_l2_cache();
#endif
    seed = 0xa5a5a5a5u;
    for (uint32_t i = 0; i < TEST_WORDS; i++) {
        seed = prng(seed);
        if (w[i] != seed && bad++ == 0) {
            puts_("  first bad word at index "); put_u32(i);
            puts_(" got "); put_hex(w[i]); puts_(" want "); put_hex(seed); put('\n');
        }
    }
    check("word read/write", bad, 0);

    /* Byte and halfword lanes. The BIOS memtest is word-only, so a broken
     * byte-enable path survives it and shows up here. */
    volatile uint8_t *bp = (volatile uint8_t *)w;
    for (uint32_t i = 0; i < 1024; i++) bp[i] = (uint8_t)(i * 7u + 1u);
    flush_cpu_dcache();
    bad = 0;
    for (uint32_t i = 0; i < 1024; i++) if (bp[i] != (uint8_t)(i * 7u + 1u)) bad++;
    check("byte lanes", bad, 0);

    volatile uint16_t *hp = (volatile uint16_t *)w;
    for (uint32_t i = 0; i < 512; i++) hp[i] = (uint16_t)(i * 1103u + 7u);
    flush_cpu_dcache();
    bad = 0;
    for (uint32_t i = 0; i < 512; i++) if (hp[i] != (uint16_t)(i * 1103u + 7u)) bad++;
    check("halfword lanes", bad, 0);
}

int main(void)
{
    uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
    /* uart_write() is the IRQ-driven ring-buffer version whenever the CPU has
     * interrupts, and it blocks forever once the ring fills unless the UART ISR
     * is actually running. Without this the firmware prints a handful of
     * characters and wedges. Same pattern as every other firmware here. */
    irq_setie(1);
#endif

    puts_("\n=== rv32_test ===\n");
    puts_("running from main_ram at "); put_hex(MAIN_RAM_BASE);
    puts_(", sys clk "); put_u32(CONFIG_CLOCK_FREQUENCY / 1000000u); puts_(" MHz\n\n");

    test_alu();
    put('\n');
    test_main_ram();

    put('\n');
    if (failures) { puts_("FAILURES: "); put_u32((uint32_t)failures); put('\n'); }
    else          puts_("ALL PASS\n");

    puts_("\nechoing (this also proves UART rx); reset the board to stop\n");
    for (;;) put(uart_read());
}
