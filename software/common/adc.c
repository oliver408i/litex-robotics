#include <generated/soc.h>   /* AUX_CS_MCP, CONFIG_CLOCK_FREQUENCY */

#include "adc.h"
#include "aux_spi.h"

/* MCP3008 on the shared aux bus. The divider is a compile-time constant
 * (AUX_SPI_DIV_FOR_HZ is a pure macro over CONFIG_CLOCK_FREQUENCY), so the
 * device descriptor is static const like AUX_WINC / AUX_IMU. */
static const aux_spi_dev_t AUX_MCP = {
    .cs_mask = (uint8_t)(1u << AUX_CS_MCP),
    .div     = (uint8_t)AUX_SPI_DIV_FOR_HZ(ADC_MCP_SPI_HZ),
    .mode    = 0,
};

uint16_t adc_mcp3008_read(uint8_t channel)
{
    /* MCP3008 single-ended frame (3 bytes, MSB-first):
     *   out: 0x01                       -> 7 leading zeros + start bit
     *        0x80 | (channel << 4)      -> SGL/DIFF=1 (single-ended) + D2D1D0
     *        0x00                       -> clock the result out
     * The 10-bit result lands in the low 2 bits of the 2nd return byte
     * (bits 9:8) and all of the 3rd (bits 7:0). */
    aux_spi_select(&AUX_MCP);
    (void)aux_spi_xfer8(0x01);
    uint8_t hi = aux_spi_xfer8((uint8_t)(0x80u | ((channel & 0x07u) << 4)));
    uint8_t lo = aux_spi_xfer8(0x00);
    aux_spi_deselect();

    return (uint16_t)(((uint16_t)(hi & 0x03u) << 8) | lo);
}

uint32_t adc_mcp3008_read_mv(uint8_t channel)
{
    uint32_t code = adc_mcp3008_read(channel);
    /* code (<=1023) * 3300 fits in uint32 with room to spare. */
    return code * ADC_VREF_MV / ADC_MCP_MAX_CODE;
}
