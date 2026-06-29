#ifndef COMMON_LSM6DS3_H_
#define COMMON_LSM6DS3_H_

#include <stdint.h>
#include <stdbool.h>

/* LSM6DS3 6-axis IMU on the shared aux bus (AUX_IMU, cs[1]).
 *
 * Register map + the gyro-then-accel sample layout come from the validated
 * imu-bringup probe (software/imu_test on the imu-bringup branch). Configured
 * here for the data logger: 104 Hz ODR, +-8 g / +-500 dps (wide enough not to
 * clip pothole / track-joint transients), FIFO continuous mode so samples
 * buffer in hardware -- the logger shares the CPU with LVGL, so a slow render
 * frame must not drop IMU samples.
 *
 * Requires the aux bus in gateware (add_winc_aux / add_aux_imu); AUX_IMU and
 * AUX_CS_IMU come from aux_spi / generated headers.
 */

#define LSM6DS3_WHO_AM_I_69  0x69u   /* LSM6DS3                    */
#define LSM6DS3_WHO_AM_I_6A  0x6Au   /* LSM6DS3TR-C / LSM6DSL      */

/* Configured ranges (for the host-side consumer / file header). */
#define LSM6DS3_ODR_HZ        104u
#define LSM6DS3_ACCEL_FS_G      8u   /* +-8 g    -> 0.244 mg/LSB   */
#define LSM6DS3_GYRO_FS_DPS   500u   /* +-500 dps -> 17.50 mdps/LSB */

/* One sample set: raw int16 counts, in device FIFO order. */
typedef struct {
	int16_t gx, gy, gz;   /* gyro  */
	int16_t ax, ay, az;   /* accel */
} lsm6ds3_sample_t;

/* Sticky count of FIFO overruns seen (alignment was re-synced each time). */
extern volatile uint32_t lsm6ds3_overruns;

/* Read WHO_AM_I (returns the HAL's 0xFF on a stuck bus). */
uint8_t lsm6ds3_whoami(void);

/* Probe + configure for logging. Returns true if WHO_AM_I matched a known id
 * and the config + FIFO were programmed. */
bool lsm6ds3_init_logging(void);

/* Drain up to max_out *complete* sample sets from the hardware FIFO into out[].
 * Returns the number written (0 if none ready). Never blocks. On a detected
 * FIFO overrun it re-syncs the pattern and returns 0 for that call. */
unsigned lsm6ds3_fifo_drain(lsm6ds3_sample_t *out, unsigned max_out);

#endif /* COMMON_LSM6DS3_H_ */
