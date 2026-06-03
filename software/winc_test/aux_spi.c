#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>

#include "aux_spi.h"

#ifndef CSR_AUX_SPI_BASE
#error "Build the SoC from icepi_zero_winc.py (provides aux_spi) before compiling software/winc_test."
#endif

/* WINC1500 on the shared aux bus, chip-select index AUX_CS_WINC.
 *
 * SCLK: bring-up ran at the gateware default (1 MHz); now that the link is
 * proven we run at 12.5 MHz (div=2 @ 50 MHz sys). The WINC handles up to
 * ~48 MHz, this bus tops out at sys/2 = 25 MHz (div=1) -- try that next if
 * 12.5 MHz is clean. Purely a runtime divider change, no gateware rebuild. */
#define WINC_SPI_HZ 12500000   /* div=2. TESTED CEILING for this wiring:
                                * div=1 (25 MHz) corrupts MISO reads (garbled
                                * version strings, spi_data_read failures) --
                                * the MISO round trip doesn't fit the 20 ns
                                * sample window. Going faster needs shorter
                                * wires or a late-sample option in the SPI
                                * core, not just a divider change. */
const aux_spi_dev_t AUX_WINC = {
    .cs_mask = (uint8_t)(1u << AUX_CS_WINC),
    .div     = (uint8_t)AUX_SPI_DIV_FOR_HZ(WINC_SPI_HZ),
    .mode    = 0,
};

/* LSM6DS3 IMU on the same bus (cs[1]) -- used as a bus-health reference:
 * a passing WHO_AM_I proves sclk/mosi/miso + the AuxSPIMaster datapath, which
 * isolates WINC failures to the WINC side (power, sidebands, fw). 1 MHz is
 * comfortably inside the LSM6DS3's 10 MHz limit. */
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
            printf("aux_spi: XFER TIMEOUT (core stuck, status=0)\n");
            return 0xFF;
        }
    }
    return (uint8_t)aux_spi_miso_read();
}
