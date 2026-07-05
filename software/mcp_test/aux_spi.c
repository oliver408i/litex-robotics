#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>

#include "log.h"
#include "aux_spi.h"

#ifndef CSR_AUX_SPI_BASE
#error "Build the SoC from icepi_zero_mcp.py (provides aux_spi) before compiling software/mcp_test."
#endif

/* MCP23S17 SPI GPIO expander on the shared aux bus, chip-select AUX_CS_IOX.
 *
 * SCLK: bring-up runs at the gateware default (1 MHz). The MCP23S17 handles up
 * to 10 MHz, this bus tops out at sys/2 = 25 MHz (div=1); once the link is
 * proven, raise via the aux_spi clk_divider CSR (no gateware rebuild). */
#define IOX_SPI_HZ 1000000
const aux_spi_dev_t AUX_IOX = {
    .cs_mask = (uint8_t)(1u << AUX_CS_IOX),
    .div     = (uint8_t)AUX_SPI_DIV_FOR_HZ(IOX_SPI_HZ),
    .mode    = 0,
};

/* LSM6DS3 IMU on the same bus (cs[0]) -- bus-health reference: a passing
 * WHO_AM_I proves sclk/mosi/miso + the AuxSPIMaster datapath, isolating an
 * expander failure to the expander side. 1 MHz is inside the LSM6DS3 limit. */
const aux_spi_dev_t AUX_IMU = {
    .cs_mask = (uint8_t)(1u << AUX_CS_IMU),
    .div     = (uint8_t)AUX_SPI_DIV_FOR_HZ(1000000),
    .mode    = 0,
};

void aux_spi_select(const aux_spi_dev_t *dev)
{
    aux_spi_clk_divider_write(dev->div);
    aux_spi_cs_write(dev->cs_mask);   /* assert; held until aux_spi_deselect() */
}

void aux_spi_deselect(void)
{
    aux_spi_cs_write(0);              /* all cs_n high */
}

uint32_t aux_spi_xfer_count;   /* diagnostics: total bytes clocked on the bus */

uint8_t aux_spi_xfer8(uint8_t out)
{
    uint32_t guard = 0;
    aux_spi_xfer_count++;
    aux_spi_mosi_write(out);
    aux_spi_start_write(1);                       /* doorbell: launch transfer */
    while ((aux_spi_status_read() & 1u) == 0u) {  /* wait for ready */
        /* Tripwire: one byte at div<=255 is <3us; 1M polls means the core is
         * wedged. Scream instead of hanging the firmware silently. */
        if (++guard == 1000000u) {
            log_puts("aux_spi: XFER TIMEOUT (core stuck, status=0)"); log_nl();
            return 0xFF;
        }
    }
    return (uint8_t)aux_spi_miso_read();
}
