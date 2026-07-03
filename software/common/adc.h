#ifndef COMMON_ADC_H_
#define COMMON_ADC_H_

#include <stdint.h>

/* MCP3008 8-channel 10-bit SPI ADC on the shared aux bus (cs[2] = AUX_CS_MCP).
 *
 * Single-ended reads only (the power brick's voltage/current pins are
 * referenced to system ground). The aux_spi HAL owns the bus; this module just
 * frames the MCP3008 3-byte single-ended transaction. No init call is needed --
 * aux_spi_select() programs the SCLK divider per transfer.
 *
 * Requires the aux bus in gateware (add_aux_imu / add_winc_aux): AUX_CS_MCP and
 * CONFIG_CLOCK_FREQUENCY come from generated/soc.h, so include that before this.
 */

/* MCP3008: 10-bit -> full-scale code 1023. */
#define ADC_MCP_MAX_CODE 1023u

/* SCLK for the MCP3008. Conservative 1 MHz: the part does ~1.35 MHz at 3.3 V
 * VDD for full accuracy, and the bus is shared, so 1 MHz leaves margin. */
#define ADC_MCP_SPI_HZ 1000000u

/* ADC reference voltage in millivolts. CALIBRATION-CRITICAL and board-specific:
 * this MUST match how the MCP3008 VREF pin is actually wired. The Pixhawk power
 * port presents 0-3.3 V analog, so VREF = 3.3 V uses the full ADC range and is
 * the expected wiring. If this board ties MCP3008 VREF to 5 V instead, change
 * this to 5000 (and resolution drops, since the signal only spans 0-3.3 V).
 * Confirm against the board before trusting the telemetry. */
#define ADC_VREF_MV 3300u

/* One single-ended conversion. channel = 0..7. Returns the raw 10-bit code
 * (0..1023). Asserts/deasserts AUX_CS_MCP internally. */
uint16_t adc_mcp3008_read(uint8_t channel);

/* Same, converted to millivolts at the input pin using ADC_VREF_MV. */
uint32_t adc_mcp3008_read_mv(uint8_t channel);

#endif /* COMMON_ADC_H_ */
