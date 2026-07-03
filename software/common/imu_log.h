#ifndef COMMON_IMU_LOG_H_
#define COMMON_IMU_LOG_H_

#include <stdint.h>
#include <stdbool.h>

/* IMU logging engine: LSM6DS3 FIFO -> RAM buffer -> FAT32 file on the SDIO SD
 * card, written as timestamped tagged records (forward-compatible with a future
 * UART GPS -- GPS fixes become a second record type on the same timebase).
 *
 * Driven from a shared main loop (it coexists with LVGL): call imu_log_poll()
 * often; it drains the hardware FIFO and flushes in bounded chunks, so it never
 * blocks the UI for long. Requires a SoC with the SD core (add_sdcard) AND the
 * aux bus (AUX_IMU), and a write-enabled FatFs (ffconf.h FF_FS_READONLY 0).
 *
 * On-card format: a 20-byte file header (magic "IMLG", version, ODR, ranges,
 * sys clock) then a stream of records, each a 6-byte header
 * {uint32 t_ms, uint8 type, uint8 len} + payload. IMU payload = 6x int16
 * (gx,gy,gz,ax,ay,az), device order. A host script demuxes to numpy/CSV.
 */

typedef enum {
	IMU_LOG_IDLE = 0,
	IMU_LOG_RUNNING,
	IMU_LOG_ERROR,
} imu_log_state_t;

typedef struct {
	imu_log_state_t state;
	bool     imu_ok;        /* WHO_AM_I matched              */
	int      last_err;      /* last FatFs FRESULT (0 = ok)   */
	uint32_t samples;       /* IMU sample sets written       */
	uint32_t bytes;         /* file bytes written            */
	uint32_t overruns;      /* FIFO overruns observed        */
	char     filename[16];  /* open log file (8.3)           */
} imu_log_status_t;

/* Record type tags. */
#define IMU_REC_IMU   1u
#define IMU_REC_GPS   2u   /* reserved: future UART GPS fix  */
#define IMU_REC_MARK  3u   /* reserved: user event marker    */

/* Bring up IMU + SD + FAT, open a fresh LOGNNNNN.BIN, write the header.
 * now_ms is the caller's monotonic millisecond clock (record timebase origin).
 * Returns 0 on success, <0 on failure (state -> IMU_LOG_ERROR). */
int  imu_log_start(uint32_t now_ms);

/* Drain the FIFO into the buffer and flush full chunks. Call frequently;
 * now_ms stamps records (ms since imu_log_start). No-op unless RUNNING.
 * Returns nonzero if it performed SD I/O (a flush) this call -- the SD driver
 * uses busy_wait(), which clobbers timer0, so a caller using timer0 as a clock
 * (e.g. an LVGL tick) must re-sync its timebase afterwards. */
int imu_log_poll(uint32_t now_ms);

/* Flush the remainder and close the file. */
void imu_log_stop(void);

const imu_log_status_t *imu_log_get_status(void);

#endif /* COMMON_IMU_LOG_H_ */
