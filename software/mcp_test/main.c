/* MCP23S17 SPI GPIO-expander bring-up firmware.
 *
 * The hardware half is icepi_zero_mcp.py (BaseSoC + add_mcp_expander): the
 * MCP23S17 sits on the shared aux SPI bus on a 4th chip-select (AUX_CS_IOX,
 * IO11/G2) with reset on IO10/L2 (iox_reset) and INTA on IO22/P2 (iox_inta).
 * This replaces the retired ATWINC1500. There is no WiFi OTA path yet, so load
 * it over UART:
 *   litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/mcp_test/mcp_test.bin
 *
 * Sequence:
 *   1. IMU WHO_AM_I  -- proves sclk/mosi/miso + the AuxSPIMaster datapath
 *                       independently of the expander (cross-check).
 *   2. reset + probe -- release the expander, echo-test a scratch register.
 *   3. GPA7 loopback -- closed loop IO24/L1 <-> GPA7, both directions.
 *   4. output walk   -- GPA as outputs, walk a 1 across OLATA (meter/LED check).
 *   5. input watch   -- GPB as pulled-up inputs, interrupt-on-change enabled;
 *                       poll GPIOB + the INTA sideband and log on change.
 *
 * Logging is the project's uart-backed log.c (software/common) -- there is no
 * working printf in this firmware.
 */
#include <stdint.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "log.h"
#include "aux_spi.h"
#include "mcp23s17.h"

/* ---- IMU bus-health cross-check (raw, no full driver) ------------------- */
#define IMU_WHO_AM_I 0x0Fu
static uint8_t imu_whoami(void)
{
    uint8_t v;
    aux_spi_select(&AUX_IMU);
    (void)aux_spi_xfer8(0x80u | IMU_WHO_AM_I);   /* bit7 = read */
    v = aux_spi_xfer8(0x00u);
    aux_spi_deselect();
    return v;
}

static void bus_health(void)
{
    uint8_t who = imu_whoami();
    log_puts("IMU WHO_AM_I = 0x"); log_hex8(who); log_char(' ');
    if (who == 0x69u || who == 0x6Au)
        log_puts("(OK -- aux bus datapath good)");
    else
        log_puts("(unexpected; IMU absent or bus issue -- expander result below "
                 "still tells us about the IOX side)");
    log_nl();
}

/* ---- GPA7 loopback: IO24/L1 (FPGA tristate) <-> MCP23S17 GPA7 -----------
 * The strongest bring-up check: a closed loop through the expander's real GPIO
 * silicon, both directions. Requires the gpa7_loop fixture in gateware. The
 * drive order is chosen so the two ends never drive the wire at once. */
#ifndef CSR_GPA7_LOOP_BASE
#error "Build icepi_zero_mcp.py with add_gpa7_loopback() for the GPA7 loopback test."
#endif
#define GPA7 7u
static void log_dir(const char *tag, int drove, int got)
{
    log_puts(tag); log_uint((uint32_t)drove);
    log_puts(", read "); log_uint((uint32_t)got);
    log_puts(got == drove ? "  OK" : "  MISMATCH"); log_nl();
}

static int gpa7_loopback(void)
{
    int ok = 1;

    /* Direction A: expander drives GPA7 (output), FPGA reads IO24 (input). */
    gpa7_loop_oe_write(0);                                  /* FPGA pin = Hi-Z input */
    mcp23s17_write(MCP_IODIRA, (uint8_t)~(1u << GPA7));     /* GPA7 output, GPA0-6 input */
    for (int v = 0; v <= 1; v++) {
        mcp23s17_write(MCP_OLATA, (uint8_t)(v << GPA7));
        busy_wait(1);
        int got = (int)(gpa7_loop_in_read() & 1u);
        log_dir("  A exp->fpga: drove ", v, got);
        if (got != v) ok = 0;
    }

    /* Direction B: FPGA drives IO24, expander reads GPA7 over SPI.
     * Make the expander an input BEFORE enabling the FPGA driver (no contention). */
    mcp23s17_write(MCP_IODIRA, 0xFFu);                     /* GPA all inputs */
    gpa7_loop_oe_write(1);                                 /* now FPGA drives the wire */
    for (int v = 0; v <= 1; v++) {
        gpa7_loop_out_write((uint32_t)v);
        busy_wait(1);
        int got = (int)((mcp23s17_read(MCP_GPIOA) >> GPA7) & 1u);
        log_dir("  B fpga->exp: drove ", v, got);
        if (got != v) ok = 0;
    }

    /* Safe idle: both ends inputs. */
    gpa7_loop_oe_write(0);
    mcp23s17_write(MCP_IODIRA, 0xFFu);
    return ok;
}

/* ---- output walk: drive a single 1 across GPA, verify via GPIOA ---------- */
static void output_walk(void)
{
    mcp23s17_write(MCP_IODIRA, 0x00u);   /* GPA all outputs */
    log_puts("GPA output walk: ");
    for (unsigned bit = 0; bit < 8; bit++) {
        uint8_t want = (uint8_t)(1u << bit);
        mcp23s17_write(MCP_OLATA, want);
        uint8_t got = mcp23s17_read(MCP_GPIOA);
        log_hex8(got);
        if (got != want) log_char('!');
        log_char(' ');
        busy_wait(50);   /* slow enough to see on an LED / catch on a meter */
    }
    mcp23s17_write(MCP_OLATA, 0x00u);
    log_nl();
}

/* ---- input watch: GPB pulled-up inputs, interrupt-on-change -------------- */
static void input_watch_setup(void)
{
    mcp23s17_write(MCP_IODIRB,   0xFFu);  /* GPB all inputs        */
    mcp23s17_write(MCP_GPPUB,    0xFFu);  /* 100k pull-ups on      */
    mcp23s17_write(MCP_INTCONB,  0x00u);  /* compare to previous   */
    mcp23s17_write(MCP_GPINTENB, 0xFFu);  /* interrupt-on-change   */
    (void)mcp23s17_read(MCP_GPIOB);       /* clear any pending int */
}

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);
#endif
    uart_init();

    log_puts("\n=== MCP23S17 aux-bus bring-up ===\n");
    log_puts("aux_spi nominal "); log_uint(IOX_SPI_DEFAULT_FREQUENCY);
    log_puts(" Hz, IOX cs["); log_uint(AUX_CS_IOX);
    log_puts("], reset IO10, INTA IO22"); log_nl();

    bus_health();

    mcp23s17_reset();
    if (!mcp23s17_probe()) {
        log_puts("PROBE FAIL: register echo mismatch -- check CS(IO11), reset(IO10), "
                 "wiring, or A2:A0 strap (expected 000)."); log_nl();
        /* Don't spin silently; keep retrying the probe. */
        while (1) {
            busy_wait(1000);
            mcp23s17_reset();
            log_puts("retry probe: ");
            log_puts(mcp23s17_probe() ? "OK now" : "still failing");
            log_nl();
        }
    }
    log_puts("PROBE OK: register echo round-trips -- SPI link to MCP23S17 is good."); log_nl();

    log_puts("GPA7 loopback (IO24/L1 <-> GPA7):"); log_nl();
    log_puts(gpa7_loopback() ? "LOOPBACK PASS -- expander GPIO good both directions."
                             : "LOOPBACK FAIL -- check the IO24<->GPA7 jumper.");
    log_nl();

    output_walk();
    input_watch_setup();

    log_puts("Input watch on GPB (pulled up; tie a pin low to trigger). On change:"); log_nl();
    uint8_t last = mcp23s17_read(MCP_GPIOB);
    log_puts("GPB initial = 0x"); log_hex8(last); log_nl();
    while (1) {
        uint8_t now  = mcp23s17_read(MCP_GPIOB);
        int     inta = mcp23s17_inta_asserted();
        if (now != last || inta) {
            uint8_t intf = mcp23s17_read(MCP_INTFB);
            uint8_t cap  = mcp23s17_read(MCP_INTCAPB);  /* read clears INTA */
            log_puts("GPB=0x");  log_hex8(now);
            log_puts(" INTF=0x"); log_hex8(intf);
            log_puts(" CAP=0x");  log_hex8(cap);
            log_puts(" INTA=");   log_uint((uint32_t)inta);
            log_nl();
            last = now;
        }
        busy_wait(20);
    }
    return 0;
}
