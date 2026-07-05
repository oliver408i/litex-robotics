#ifndef MCP_MCP23S17_H_
#define MCP_MCP23S17_H_

#include <stdint.h>
#include <stdbool.h>

/* MCP23S17 16-bit SPI GPIO expander on the shared aux bus (AUX_IOX, cs[2]).
 *
 * Wiring (board refactor 2026-06-25, replaces the retired ATWINC1500):
 *   CS    = IO17/R3  -> aux_spi cs[2] (AUX_CS_IOX)
 *   RESET = IO10/L2  -> iox_reset GPIOOut (active low; 0 = held in reset)
 *   INTA  = IO22/P2  -> iox_inta  GPIOIn  (active low by default; with IRQ)
 *
 * SPI framing: a control byte [0100 A2 A1 A0 R/W], then the register address,
 * then data byte(s). Hardware address pins A2:A0 are strapped to 000 for this
 * board -> opcodes 0x40 (write) / 0x41 (read). With IOCON.BANK=0 (POR default)
 * the A/B registers interleave and auto-increment, so a 16-bit access is just
 * a 2-byte burst starting at the A register. Requires the aux bus + iox_*
 * CSRs in gateware (icepi_zero_mcp.py / add_mcp_expander). */

/* ---- register addresses (IOCON.BANK = 0, the power-on default) ----------- */
#define MCP_IODIRA   0x00u   /* 1 = input (POR 0xFF)               */
#define MCP_IODIRB   0x01u
#define MCP_IPOLA    0x02u   /* input polarity invert             */
#define MCP_IPOLB    0x03u
#define MCP_GPINTENA 0x04u   /* interrupt-on-change enable        */
#define MCP_GPINTENB 0x05u
#define MCP_DEFVALA  0x06u
#define MCP_DEFVALB  0x07u
#define MCP_INTCONA  0x08u   /* 0 = compare to previous value     */
#define MCP_INTCONB  0x09u
#define MCP_IOCON    0x0Au   /* config (mirrored at 0x0B)         */
#define MCP_GPPUA    0x0Cu   /* 100k pull-up enable               */
#define MCP_GPPUB    0x0Du
#define MCP_INTFA    0x0Eu   /* interrupt flags (read-only)       */
#define MCP_INTFB    0x0Fu
#define MCP_INTCAPA  0x10u   /* captured GPIO at interrupt (RO; read clears) */
#define MCP_INTCAPB  0x11u
#define MCP_GPIOA    0x12u   /* live port read / OLAT write       */
#define MCP_GPIOB    0x13u
#define MCP_OLATA    0x14u   /* output latch                      */
#define MCP_OLATB    0x15u

/* ---- IOCON bits ---------------------------------------------------------- */
#define MCP_IOCON_BANK   (1u << 7)
#define MCP_IOCON_MIRROR (1u << 6)   /* OR INTA/INTB into one line        */
#define MCP_IOCON_SEQOP  (1u << 5)   /* 1 = disable address auto-increment */
#define MCP_IOCON_DISSLW (1u << 4)
#define MCP_IOCON_HAEN   (1u << 3)   /* enable hardware address pins      */
#define MCP_IOCON_ODR    (1u << 2)   /* 1 = INT open-drain                */
#define MCP_IOCON_INTPOL (1u << 1)   /* 1 = INT active-high (when ODR=0)  */

/* Release the expander from hardware reset (pulse iox_reset low->high) and let
 * it settle. After this the registers are at their POR defaults. */
void    mcp23s17_reset(void);

/* Single 8-bit register write / read over the aux bus. */
void    mcp23s17_write(uint8_t reg, uint8_t val);
uint8_t mcp23s17_read(uint8_t reg);

/* 16-bit paired access (A reg then B reg via auto-increment; BANK=0). */
void    mcp23s17_write16(uint8_t reg_a, uint16_t val);  /* low byte -> A, high -> B */
uint16_t mcp23s17_read16(uint8_t reg_a);

/* Link self-test: writes walking patterns to a scratch register (IPOLB) and
 * reads them back. Returns true if every pattern round-trips. The MCP23S17 has
 * no WHO_AM_I, so this register echo is the bring-up "is it alive" check. */
bool    mcp23s17_probe(void);

/* Read the INTA sideband level via the iox_inta GPIO CSR (1 = idle/high,
 * 0 = asserted, for the default active-low push-pull config). */
int     mcp23s17_inta_asserted(void);

#endif /* MCP_MCP23S17_H_ */
