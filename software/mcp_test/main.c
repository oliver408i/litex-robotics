/* MCP23S17 SPI GPIO-expander bring-up firmware.
 *
 * The hardware half is icepi_zero_mcp.py (C3FlashSoC's shape + add_mcp_expander):
 * the MCP23S17 sits on the shared aux SPI bus on a 3rd chip-select (AUX_CS_IOX,
 * IO17/R3) with reset on IO10/L2 (iox_reset) and INTA on IO22/P2 (iox_inta).
 * This replaces the retired ATWINC1500. Runs as the chain-booted app behind
 * the resident C3 loader (software/c3_flash) -- no JTAG/litex_term needed:
 *   flash.py --app software/mcp_test/mcp_test.bin --port /dev/ttyACM0
 *   flash.py --boot-app --port /dev/ttyACM0
 *
 * Sequence:
 *   1. IMU WHO_AM_I  -- proves sclk/mosi/miso + the AuxSPIMaster datapath
 *                       independently of the expander (cross-check).
 *   2. reset + probe -- release the expander, echo-test a scratch register.
 *   3. output walk   -- GPA as outputs, walk a 1 across OLATA (meter/LED check).
 *   4. input watch   -- GPB as pulled-up inputs, interrupt-on-change enabled;
 *                       poll GPIOB + the INTA sideband and log on change.
 *
 * NOTE: the GPA7 loopback closed-loop test (IO24/L1 <-> GPA7) is dropped here
 * -- IO24/L1 is now the C3 link's MOSI line, a real pin conflict with the
 * loopback bench fixture. See icepi_zero_mcp.py's docstring.
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
        log_puts("PROBE FAIL: register echo mismatch -- check CS(IO17), reset(IO10), "
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
