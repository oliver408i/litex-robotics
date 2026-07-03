#ifndef COMMON_GPS_NMEA_H_
#define COMMON_GPS_NMEA_H_

#include <stdint.h>
#include <stdbool.h>

/* NMEA GPS reader for the logger's second hardware UART ("gps", rx=IO5/E1,
 * tx=IO11/G2; see gateware add_gps_uart). Bring-up first: pull the module's
 * NMEA stream off the UART, parse the two sentences that matter for a fix
 * (GGA = fix quality + sats + altitude, RMC = valid flag + position + speed +
 * date/time), and expose a status struct the LCD can show. Logging GPS fixes
 * (IMU_REC_GPS, already reserved in imu_log.h) comes once data is confirmed.
 *
 * Polled, non-blocking: call gps_nmea_poll() often from the shared main loop
 * (alongside imu_log_poll / lv_timer_handler). It drains the UART RX FIFO,
 * assembles full sentences, verifies the *NN checksum, and updates the status.
 * Talker-agnostic: matches GGA/RMC regardless of talker id (GP/GN/GL/GA).
 *
 * Lat/lon are kept as signed microdegrees (1e-6 deg; S/W negative) so the whole
 * path stays integer -- no soft-float on this VexRiscv. The raw last sentence
 * is retained for on-screen bring-up debugging.
 */

typedef struct {
	bool     valid;          /* RMC status 'A' (nav data valid)            */
	uint8_t  fix_quality;    /* GGA field 6: 0=no fix,1=GPS,2=DGPS,...      */
	uint8_t  sats;           /* GGA field 7: satellites used in the fix     */
	int32_t  lat_udeg;       /* latitude,  microdegrees (+N / -S)           */
	int32_t  lon_udeg;       /* longitude, microdegrees (+E / -W)           */
	int32_t  alt_dm;         /* GGA altitude (MSL), decimeters              */
	uint32_t time_hms;       /* UTC time as hhmmss (RMC/GGA field)          */
	uint32_t date_dmy;       /* UTC date as ddmmyy (RMC)                    */
	uint16_t speed_mknots;   /* RMC ground speed, milli-knots               */

	/* Link / parse health (bring-up + sanity). */
	uint32_t bytes;          /* bytes read off the UART                     */
	uint32_t sentences;      /* well-formed sentences (checksum ok)         */
	uint32_t gga;            /* GGA sentences parsed                        */
	uint32_t rmc;            /* RMC sentences parsed                        */
	uint32_t csum_errors;    /* sentences dropped on a bad checksum         */
	uint32_t overflows;      /* sentences dropped: line buffer overrun      */
	uint32_t last_ms;        /* caller now_ms when the last sentence landed */

	char     last[84];       /* last complete sentence (NUL-terminated)     */
} gps_status_t;

/* Reset counters/state and clear the UART RX FIFO. now_ms is the caller's
 * monotonic millisecond clock (same one fed to gps_nmea_poll / imu_log). */
void gps_nmea_init(uint32_t now_ms);

/* Drain the UART, assemble + parse complete sentences. Non-blocking; stamps
 * last_ms with now_ms on each accepted sentence. Call frequently. */
void gps_nmea_poll(uint32_t now_ms);

const gps_status_t *gps_nmea_get_status(void);

/* True if a well-formed sentence arrived within timeout_ms of now_ms (i.e. the
 * module is wired and talking), independent of whether it has a position fix. */
bool gps_nmea_link_alive(uint32_t now_ms, uint32_t timeout_ms);

#endif /* COMMON_GPS_NMEA_H_ */
