#ifndef WINC_AUX_SPI_H_
#define WINC_AUX_SPI_H_

#include <stdint.h>

/* Shared sensor/aux SPI bus HAL (AuxSPIMaster in gateware/aux_spi.py).
 *
 * One byte per transfer, full duplex, with a runtime SCLK divider and
 * software-held chip-selects. Each device on the bus is described by an
 * aux_spi_dev_t; aux_spi_select() programs that device's clock and asserts its
 * CS, held until aux_spi_deselect(). This is the single source of truth for
 * per-device bus settings (WINC today; IMU / MCP3008 slot in the same way). */

typedef struct {
    uint8_t cs_mask;   /* bit i (= 1u<<AUX_CS_*) drives cs_n[i] low */
    uint8_t div;       /* SCLK half-bit period in sys cycles (>=1); SCLK = sysclk/(2*div) */
    uint8_t mode;      /* SPI mode 0..3 (informational: core is fixed mode 0 for now) */
} aux_spi_dev_t;

/* Half-bit divider for a target SCLK in Hz, rounded up so actual SCLK <= target,
 * clamped to >= 1. CONFIG_CLOCK_FREQUENCY comes from generated/soc.h. */
#define AUX_SPI_DIV_FOR_HZ(hz) \
    ( (CONFIG_CLOCK_FREQUENCY) < (2u * (hz)) ? 1u \
      : ((CONFIG_CLOCK_FREQUENCY) + (2u * (hz)) - 1u) / (2u * (hz)) )

/* The ATWINC1500 on the shared bus (cs index + initial clock from the SoC). */
extern const aux_spi_dev_t AUX_WINC;

/* LSM6DS3 IMU on the same bus (cs[1]); bus-health reference for diagnostics. */
extern const aux_spi_dev_t AUX_IMU;

/* Program the device's SCLK divider and assert its CS (held until deselect). */
void    aux_spi_select(const aux_spi_dev_t *dev);

/* Deassert all chip-selects (end of a multi-byte transaction). */
void    aux_spi_deselect(void);

/* One 8-bit full-duplex transfer; returns the MISO byte. CS must already be
 * asserted via aux_spi_select(). */
uint8_t aux_spi_xfer8(uint8_t out);

#endif /* WINC_AUX_SPI_H_ */
