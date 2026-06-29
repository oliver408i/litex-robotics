#ifndef COMMON_POWER_H_
#define COMMON_POWER_H_

#include <stdint.h>

/* CubePilot / Hex "Power Brick Mini" telemetry via the MCP3008 aux ADC.
 *
 * The brick powers the whole board and exposes two ground-referenced analog
 * sense outputs (0-3.3 V by the Pixhawk connector standard):
 *   - VOLTAGE sense -> MCP3008 channel 6
 *   - CURRENT sense -> MCP3008 channel 5
 *
 * Scaling uses the ArduPilot firmware defaults for this module (not forum
 * lore): BATT_VOLT_MULT = 12.02, BATT_AMP_PERVLT = 39.877, BATT_AMP_OFFSET = 0.
 * Current sensing is shunt-based and unreliable below ~3-10 A, so treat small
 * current/power readings as approximate. Integer fixed-point throughout (no
 * soft-float). All math fits uint32 except instantaneous power and the energy
 * accumulators, which use uint64 internally.
 */

/* Wiring (per the user's harness): voltage on ch6, current on ch5. */
#define POWER_ADC_CH_VOLTAGE 6u
#define POWER_ADC_CH_CURRENT 5u

typedef struct {
    uint16_t v_code;      /* raw MCP3008 code, voltage channel */
    uint16_t i_code;      /* raw MCP3008 code, current channel */
    uint32_t bus_mV;      /* battery/bus voltage, millivolts */
    uint32_t current_mA;  /* load current, milliamps */
    uint32_t power_mW;    /* instantaneous power, milliwatts */
} power_reading_t;

/* Sample both channels once and fill *out with raw codes + converted values. */
void power_read(power_reading_t *out);

/* Integrate consumed charge/energy. Call periodically with the wall-clock
 * milliseconds elapsed since the previous call, passing that interval's
 * reading (rectangular integration). Accuracy tracks how regularly you call. */
void power_accumulate(const power_reading_t *r, uint32_t dt_ms);

/* Accumulated charge (mAh) and energy (mWh) since the last reset. */
uint32_t power_charge_mAh(void);
uint32_t power_energy_mWh(void);

/* Zero the charge/energy accumulators (e.g. at the start of a logging run). */
void power_reset_accumulators(void);

/* ---- Battery state-of-charge (voltage-only) ------------------------------ */
/* Current sense is unusable at this load (board draws < 1 ADC LSB ≈ 128 mA),
 * so SoC comes from pack voltage alone. */
#define BATTERY_CELLS 2          /* 2S LiPo */

/* Rough SoC percent (0..100) from pack voltage, via a per-cell LiPo discharge
 * curve (LiPo is flat through the middle, so a linear map would lie). Voltage
 * sags under load, so this reads low while drawing and recovers at rest --
 * good enough for a field "how much left" gauge, not a coulomb-counting fuel
 * gauge (which this sensor can't support anyway). */
uint8_t battery_soc_pct(uint32_t bus_mV);

#endif /* COMMON_POWER_H_ */
