#include <stdint.h>
#include <stdbool.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <system.h>          /* busy_wait (ms) */

#include "aux_spi.h"
#include "mcp23s17.h"

#ifndef CSR_IOX_RESET_BASE
#error "MCP23S17 driver needs the iox_reset/iox_inta CSRs -- build a SoC with the expander (add_mcp_expander, or add_aux_imu with_iox=True: icepi_zero_mcp.py / icepi_zero_mnist_lcd.py)."
#endif

/* SPI control byte: [0100 A2 A1 A0 R/W]. Hardware address pins strapped to 000
 * on this board, so write = 0x40, read = 0x41 (the LSB is the R/W bit). */
#define MCP_OPCODE_WRITE 0x40u
#define MCP_OPCODE_READ  0x41u

void mcp23s17_reset(void)
{
    /* RESET is active low on iox_reset (GPIOOut). It powers up at 0 (held in
     * reset); drive a clean low->high pulse so a warm restart also re-inits. */
    iox_reset_out_write(0);   /* assert reset  */
    busy_wait(1);             /* >> 1 us min reset pulse width */
    iox_reset_out_write(1);   /* release       */
    busy_wait(1);             /* settle before first access    */
}

void mcp23s17_write(uint8_t reg, uint8_t val)
{
    aux_spi_select(&AUX_IOX);
    (void)aux_spi_xfer8(MCP_OPCODE_WRITE);
    (void)aux_spi_xfer8(reg);
    (void)aux_spi_xfer8(val);
    aux_spi_deselect();
}

uint8_t mcp23s17_read(uint8_t reg)
{
    uint8_t v;
    aux_spi_select(&AUX_IOX);
    (void)aux_spi_xfer8(MCP_OPCODE_READ);
    (void)aux_spi_xfer8(reg);
    v = aux_spi_xfer8(0x00u);
    aux_spi_deselect();
    return v;
}

/* BANK=0 auto-increment: a 2-byte burst from reg_a writes A then B. */
void mcp23s17_write16(uint8_t reg_a, uint16_t val)
{
    aux_spi_select(&AUX_IOX);
    (void)aux_spi_xfer8(MCP_OPCODE_WRITE);
    (void)aux_spi_xfer8(reg_a);
    (void)aux_spi_xfer8((uint8_t)(val & 0xffu));        /* -> A */
    (void)aux_spi_xfer8((uint8_t)((val >> 8) & 0xffu)); /* -> B */
    aux_spi_deselect();
}

uint16_t mcp23s17_read16(uint8_t reg_a)
{
    uint8_t a, b;
    aux_spi_select(&AUX_IOX);
    (void)aux_spi_xfer8(MCP_OPCODE_READ);
    (void)aux_spi_xfer8(reg_a);
    a = aux_spi_xfer8(0x00u);   /* A */
    b = aux_spi_xfer8(0x00u);   /* B */
    aux_spi_deselect();
    return (uint16_t)(a | (b << 8));
}

bool mcp23s17_probe(void)
{
    /* IPOLB is a plain R/W register that doesn't move pins, so it's a safe
     * scratch for an echo test. Walk a few patterns through it, then restore. */
    static const uint8_t pats[] = { 0x55u, 0xAAu, 0x00u, 0xFFu };
    for (unsigned i = 0; i < sizeof(pats); i++) {
        mcp23s17_write(MCP_IPOLB, pats[i]);
        if (mcp23s17_read(MCP_IPOLB) != pats[i]) {
            mcp23s17_write(MCP_IPOLB, 0x00u);   /* leave it at POR default */
            return false;
        }
    }
    mcp23s17_write(MCP_IPOLB, 0x00u);
    return true;
}

int mcp23s17_inta_asserted(void)
{
    /* iox_inta is GPIOIn (active-low default); 0 on the pin = asserted. */
    return (iox_inta_in_read() & 1u) ? 0 : 1;
}
