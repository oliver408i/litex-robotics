#include <string.h>

#include <generated/soc.h>      /* CONFIG_CLOCK_FREQUENCY */

#include "ff.h"                 /* FatFs (libfatfs)            */
#include "sdcard.h"             /* fatfs_set_ops_sdcard (liblitesdcard) */

#include "imu_log.h"
#include "lsm6ds3.h"

/* Bring-up breadcrumbs (off by default; build with -DIMU_LOG_DEBUG). The SD
 * init can block on a non-responding card, so these localize where a hang in
 * imu_log_start() actually happens. */
#ifdef IMU_LOG_DEBUG
#include "log.h"
#define ILOG(s)      do { log_puts(s); log_nl(); } while (0)
#define ILOGV(s, v)  do { log_puts(s); log_uint((unsigned)(v)); log_nl(); } while (0)
#else
#define ILOG(s)
#define ILOGV(s, v)
#endif

/* ---- on-card format ----------------------------------------------------- */
typedef struct __attribute__((packed)) {
	char     magic[4];      /* "IMLG"                                  */
	uint16_t version;       /* 1                                       */
	uint16_t odr_hz;        /* 104                                     */
	uint16_t accel_fs_g;    /* 8                                       */
	uint16_t gyro_fs_dps;   /* 500                                     */
	uint32_t clk_hz;        /* CONFIG_CLOCK_FREQUENCY (timebase ref)   */
	uint32_t reserved;
} imlg_hdr_t;

typedef struct __attribute__((packed)) {
	uint32_t t_ms;          /* ms since imu_log_start                  */
	uint8_t  type;          /* IMU_REC_*                               */
	uint8_t  len;           /* payload bytes                           */
} rec_hdr_t;

#define IMU_PAYLOAD_BYTES 12u   /* 6x int16: gx,gy,gz,ax,ay,az         */

/* ---- engine state ------------------------------------------------------- */
#define LOG_BUF_SZ   4096u      /* RAM staging buffer                  */
#define DRAIN_MAX      64u      /* sample sets pulled per poll         */
#define SYNC_EVERY      8u      /* f_sync() every N chunk flushes      */

static FATFS    s_fs;
static FIL      s_fil;
static imu_log_status_t s_st;
static uint32_t s_start_ms;
static uint8_t  s_buf[LOG_BUF_SZ];
static unsigned s_len;
static unsigned s_flushes;

static void set_err(int fr)
{
	s_st.last_err = fr;
	s_st.state    = IMU_LOG_ERROR;
}

static int flush_buf(void)
{
	if (s_len == 0)
		return 0;
	UINT bw = 0;
	FRESULT fr = f_write(&s_fil, s_buf, s_len, &bw);
	if (fr != FR_OK || bw != s_len) {
		set_err((int)fr);
		return -1;
	}
	s_st.bytes += bw;
	s_len = 0;
	if (++s_flushes >= SYNC_EVERY) {
		s_flushes = 0;
		f_sync(&s_fil);          /* bound data-loss on a power cut */
	}
	return 0;
}

/* Open the first free LOGNNNNN.BIN (8.3). Avoids f_stat (removed at
 * FF_FS_MINIMIZE 1) by letting FA_CREATE_NEW return FR_EXIST. */
static int open_new_file(void)
{
	for (unsigned i = 0; i < 100000u; i++) {
		char *p = s_st.filename;
		p[0] = 'L'; p[1] = 'O'; p[2] = 'G';
		p[3] = '0' + (i / 10000u) % 10u;
		p[4] = '0' + (i / 1000u)  % 10u;
		p[5] = '0' + (i / 100u)   % 10u;
		p[6] = '0' + (i / 10u)    % 10u;
		p[7] = '0' +  i           % 10u;
		p[8] = '.'; p[9] = 'B'; p[10] = 'I'; p[11] = 'N'; p[12] = 0;

		FRESULT fr = f_open(&s_fil, s_st.filename, FA_WRITE | FA_CREATE_NEW);
		if (fr == FR_OK)
			return 0;
		if (fr != FR_EXIST) {
			set_err((int)fr);
			return -1;
		}
	}
	set_err((int)FR_DENIED);
	return -1;
}

int imu_log_start(uint32_t now_ms)
{
	memset(&s_st, 0, sizeof(s_st));
	s_len      = 0;
	s_flushes  = 0;
	s_start_ms = now_ms;

	ILOG("imu_log: init IMU (aux SPI)...");
	s_st.imu_ok = lsm6ds3_init_logging();
	ILOGV("imu_log: imu_ok=", s_st.imu_ok);
	if (!s_st.imu_ok) {
		s_st.state    = IMU_LOG_ERROR;
		s_st.last_err = -1;
		return -1;
	}

	/* Mount (opt=1 forces immediate mount -> disk_initialize -> sdcard_init). */
	ILOG("imu_log: f_mount (SD init)...");
	fatfs_set_ops_sdcard();
	FRESULT fr = f_mount(&s_fs, "", 1);
	ILOGV("imu_log: f_mount fr=", fr);
	if (fr != FR_OK) {
		set_err((int)fr);
		return -1;
	}
	ILOG("imu_log: open_new_file...");
	if (open_new_file() != 0)
		return -1;
	ILOGV("imu_log: file opened, header write... bytes=", sizeof(imlg_hdr_t));

	imlg_hdr_t h;
	memcpy(h.magic, "IMLG", 4);
	h.version     = 1;
	h.odr_hz      = LSM6DS3_ODR_HZ;
	h.accel_fs_g  = LSM6DS3_ACCEL_FS_G;
	h.gyro_fs_dps = LSM6DS3_GYRO_FS_DPS;
	h.clk_hz      = CONFIG_CLOCK_FREQUENCY;
	h.reserved    = 0;

	UINT bw = 0;
	fr = f_write(&s_fil, &h, sizeof(h), &bw);
	if (fr != FR_OK || bw != sizeof(h)) {
		set_err((int)fr);
		return -1;
	}
	s_st.bytes += bw;
	f_sync(&s_fil);

	ILOG("imu_log: RUNNING");
	s_st.state = IMU_LOG_RUNNING;
	return 0;
}

int imu_log_poll(uint32_t now_ms)
{
	int did_io = 0;

	if (s_st.state != IMU_LOG_RUNNING)
		return 0;

	lsm6ds3_sample_t samp[DRAIN_MAX];
	unsigned n = lsm6ds3_fifo_drain(samp, DRAIN_MAX);
	s_st.overruns = lsm6ds3_overruns;
	if (n == 0)
		return 0;

	uint32_t t = now_ms - s_start_ms;
	for (unsigned i = 0; i < n; i++) {
		if (s_len + sizeof(rec_hdr_t) + IMU_PAYLOAD_BYTES > LOG_BUF_SZ) {
			did_io = 1;
			if (flush_buf() != 0)
				return did_io;
		}
		rec_hdr_t rh = { .t_ms = t, .type = (uint8_t)IMU_REC_IMU,
		                 .len = (uint8_t)IMU_PAYLOAD_BYTES };
		memcpy(s_buf + s_len, &rh, sizeof(rh));
		s_len += sizeof(rh);
		/* lsm6ds3_sample_t is 6 packed int16 in device order -> 12 bytes. */
		memcpy(s_buf + s_len, &samp[i], IMU_PAYLOAD_BYTES);
		s_len += IMU_PAYLOAD_BYTES;
		s_st.samples++;
	}
	return did_io;
}

void imu_log_stop(void)
{
	if (s_st.state != IMU_LOG_RUNNING)
		return;
	flush_buf();
	f_sync(&s_fil);
	f_close(&s_fil);
	s_st.state = IMU_LOG_IDLE;
}

const imu_log_status_t *imu_log_get_status(void) { return &s_st; }
