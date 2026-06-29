#include "power.h"
#include "adc.h"

/* Power Brick Mini scaling, fixed-point. See power.h for provenance.
 *   bus_mV     = v_pin_mV * 12.02
 *   current_mA = (i_pin_mV - offset) * 39.877      (offset = 0)
 * Kept as integer ratios so we never touch soft-float. */
/* Voltage multiplier EMPIRICALLY CALIBRATED for this unit, not the datasheet
 * 12.02: a known 7.51 V (multimeter) read raw code 240 -> V_pin = 774 mV ->
 * effective divider 7510/774 = 9.70. The divider is ratiometric (0 V -> code 0),
 * so this single point fixes the slope. Re-trim here if a second known voltage
 * disagrees, or if VREF (adc.h ADC_VREF_MV) turns out not to be 3.3 V. */
#define PB_VOLT_MULT_NUM     970u   /* 9.70 -> *970 / 100 */
#define PB_VOLT_MULT_DEN     100u
#define PB_AMP_PERVLT_NUM  39877u   /* 39.877 -> *39877 / 1000 */
#define PB_AMP_PERVLT_DEN   1000u
#define PB_AMP_OFFSET_MV       0u   /* BATT_AMP_OFFSET, in pin millivolts */

static uint64_t accum_mA_ms;  /* milliamp-milliseconds  -> mAh = /3,600,000 */
static uint64_t accum_mW_ms;  /* milliwatt-milliseconds -> mWh = /3,600,000 */

void power_read(power_reading_t *out)
{
    out->v_code = adc_mcp3008_read(POWER_ADC_CH_VOLTAGE);
    out->i_code = adc_mcp3008_read(POWER_ADC_CH_CURRENT);

    /* Pin millivolts (<= 3300), both fit comfortably in uint32 after scaling. */
    uint32_t v_pin_mV = (uint32_t)out->v_code * ADC_VREF_MV / ADC_MCP_MAX_CODE;
    uint32_t i_pin_mV = (uint32_t)out->i_code * ADC_VREF_MV / ADC_MCP_MAX_CODE;

    out->bus_mV = v_pin_mV * PB_VOLT_MULT_NUM / PB_VOLT_MULT_DEN;

    uint32_t i_eff_mV = (i_pin_mV > PB_AMP_OFFSET_MV)
                      ? (i_pin_mV - PB_AMP_OFFSET_MV) : 0u;
    out->current_mA = i_eff_mV * PB_AMP_PERVLT_NUM / PB_AMP_PERVLT_DEN;

    /* mV * mA can exceed uint32 (≈39 V * 130 A), so widen for the product. */
    out->power_mW = (uint32_t)(((uint64_t)out->bus_mV * out->current_mA) / 1000u);
}

void power_accumulate(const power_reading_t *r, uint32_t dt_ms)
{
    accum_mA_ms += (uint64_t)r->current_mA * dt_ms;
    accum_mW_ms += (uint64_t)r->power_mW   * dt_ms;
}

uint32_t power_charge_mAh(void)
{
    return (uint32_t)(accum_mA_ms / 3600000ull);  /* mA*ms -> mAh */
}

uint32_t power_energy_mWh(void)
{
    return (uint32_t)(accum_mW_ms / 3600000ull);  /* mW*ms -> mWh */
}

void power_reset_accumulators(void)
{
    accum_mA_ms = 0;
    accum_mW_ms = 0;
}

/* Per-cell LiPo resting-voltage -> SoC, descending in mV. Standard 1S LiPo
 * discharge knee: ~4.20 V full, flat plateau around 3.7-3.85 V, steep drop
 * below ~3.5 V to a 3.0 V floor. Linear-interpolated between points. */
typedef struct { uint16_t mv; uint8_t pct; } soc_point_t;

static const soc_point_t lipo_cell_curve[] = {
    {4200, 100}, {4150,  95}, {4110,  90}, {4080,  85}, {4020,  80},
    {3980,  75}, {3950,  70}, {3910,  65}, {3870,  60}, {3850,  55},
    {3840,  50}, {3820,  45}, {3800,  40}, {3790,  35}, {3770,  30},
    {3750,  25}, {3730,  20}, {3710,  15}, {3690,  10}, {3610,   5},
    {3300,   0},
};

uint8_t battery_soc_pct(uint32_t bus_mV)
{
    uint32_t cell_mV = bus_mV / BATTERY_CELLS;
    const int n = (int)(sizeof(lipo_cell_curve) / sizeof(lipo_cell_curve[0]));

    if (cell_mV >= lipo_cell_curve[0].mv)     return 100;
    if (cell_mV <= lipo_cell_curve[n - 1].mv) return 0;

    for (int i = 1; i < n; i++) {
        if (cell_mV >= lipo_cell_curve[i].mv) {
            const soc_point_t *hi = &lipo_cell_curve[i - 1];
            const soc_point_t *lo = &lipo_cell_curve[i];
            uint32_t span = (uint32_t)(hi->mv - lo->mv);
            uint32_t frac = cell_mV - lo->mv;
            return (uint8_t)(lo->pct + (hi->pct - lo->pct) * frac / span);
        }
    }
    return 0;
}
