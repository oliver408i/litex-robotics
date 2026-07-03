#include <generated/soc.h>   /* AUX_CS_IMU */

#include "lsm6ds3.h"
#include "aux_spi.h"

/* ---- register map (validated subset + FIFO set) ------------------------- */
#define REG_FIFO_CTRL1   0x06   /* FTH[7:0] watermark (unused; we poll level) */
#define REG_FIFO_CTRL2   0x07
#define REG_FIFO_CTRL3   0x08   /* DEC_FIFO_GYRO[5:3], DEC_FIFO_XL[2:0]       */
#define REG_FIFO_CTRL4   0x09
#define REG_FIFO_CTRL5   0x0A   /* ODR_FIFO[6:3], FIFO_MODE[2:0]              */
#define REG_WHO_AM_I     0x0F
#define REG_CTRL1_XL     0x10
#define REG_CTRL2_G      0x11
#define REG_CTRL3_C      0x12
#define REG_FIFO_STATUS1 0x3A   /* DIFF_FIFO[7:0]                            */
#define REG_FIFO_STATUS2 0x3B   /* [6]=OVER_RUN, [3:0]=DIFF_FIFO[11:8]       */
#define REG_FIFO_DATA_L  0x3E   /* FIFO_DATA_OUT_L (0x3E) / _H (0x3F)        */

/* ---- config values ------------------------------------------------------ */
/*  CTRL3_C  = 0x44  BDU=1 (coherent hi/lo), IF_INC=1 (auto-increment) -- validated.
 *  CTRL1_XL = 0x4C  accel ODR 104 Hz (0100<<4), FS +-8 g  (11<<2).
 *  CTRL2_G  = 0x44  gyro  ODR 104 Hz (0100<<4), FS 500 dps (01<<2).
 *  FIFO_CTRL3 = 0x09  no decimation: DEC_FIFO_GYRO=001(<<3), DEC_FIFO_XL=001.
 *  FIFO_CTRL5 = 0x26  ODR_FIFO 104 Hz (0100<<3) + FIFO_MODE continuous (110).
 *  FIFO bypass (mode 000) flushes + re-aligns the pattern to a fresh Gx. */
#define CFG_CTRL3_C      0x44u
#define CFG_CTRL1_XL     0x4Cu
#define CFG_CTRL2_G      0x44u
#define CFG_FIFO_CTRL3   0x09u
#define CFG_FIFO_CONT    0x26u
#define CFG_FIFO_BYPASS  0x00u

/* Words (16-bit) per sample set in the FIFO: Gx,Gy,Gz,Ax,Ay,Az. */
#define FIFO_WORDS_PER_SAMPLE 6u

volatile uint32_t lsm6ds3_overruns;

/* ---- aux-bus register access (read bit = 0x80; auto-increment burst) ---- */
static void imu_read(uint8_t reg, uint8_t *buf, unsigned n)
{
	aux_spi_select(&AUX_IMU);
	(void)aux_spi_xfer8(0x80u | reg);
	for (unsigned i = 0; i < n; i++)
		buf[i] = aux_spi_xfer8(0x00u);
	aux_spi_deselect();
}

static uint8_t imu_rd1(uint8_t reg)
{
	uint8_t v;
	imu_read(reg, &v, 1);
	return v;
}

static void imu_wr1(uint8_t reg, uint8_t val)
{
	aux_spi_select(&AUX_IMU);
	(void)aux_spi_xfer8(reg & 0x7fu);    /* bit7 = 0: write */
	(void)aux_spi_xfer8(val);
	aux_spi_deselect();
}

uint8_t lsm6ds3_whoami(void) { return imu_rd1(REG_WHO_AM_I); }

bool lsm6ds3_init_logging(void)
{
	uint8_t who = imu_rd1(REG_WHO_AM_I);
	if (who != LSM6DS3_WHO_AM_I_69 && who != LSM6DS3_WHO_AM_I_6A)
		return false;

	imu_wr1(REG_CTRL3_C,  CFG_CTRL3_C);
	imu_wr1(REG_CTRL1_XL, CFG_CTRL1_XL);
	imu_wr1(REG_CTRL2_G,  CFG_CTRL2_G);
	imu_wr1(REG_FIFO_CTRL3, CFG_FIFO_CTRL3);
	/* Bypass then continuous: flush and start the pattern at a known Gx. */
	imu_wr1(REG_FIFO_CTRL5, CFG_FIFO_BYPASS);
	imu_wr1(REG_FIFO_CTRL5, CFG_FIFO_CONT);
	lsm6ds3_overruns = 0;
	return true;
}

/* One 16-bit FIFO word (L then H pops the next entry). */
static int16_t fifo_pop_word(void)
{
	uint8_t b[2];
	imu_read(REG_FIFO_DATA_L, b, 2);
	return (int16_t)(b[0] | (b[1] << 8));
}

unsigned lsm6ds3_fifo_drain(lsm6ds3_sample_t *out, unsigned max_out)
{
	uint8_t s2 = imu_rd1(REG_FIFO_STATUS2);

	/* On overrun the oldest entries were discarded word-wise, so the 6-word
	 * pattern phase can no longer be trusted -- re-sync and skip this round.
	 * NOTE: FIFO_STATUS2 bit layout (OVER_RUN bit6, DIFF_FIFO[11:8] in [3:0])
	 * should be confirmed against the LSM6DS3 datasheet on first bring-up. */
	if (s2 & 0x40u) {
		lsm6ds3_overruns++;
		imu_wr1(REG_FIFO_CTRL5, CFG_FIFO_BYPASS);
		imu_wr1(REG_FIFO_CTRL5, CFG_FIFO_CONT);
		return 0;
	}

	unsigned words = ((unsigned)(s2 & 0x0Fu) << 8) | imu_rd1(REG_FIFO_STATUS1);
	unsigned avail = words / FIFO_WORDS_PER_SAMPLE;   /* complete sample sets */
	if (avail > max_out)
		avail = max_out;

	for (unsigned i = 0; i < avail; i++) {
		out[i].gx = fifo_pop_word();
		out[i].gy = fifo_pop_word();
		out[i].gz = fifo_pop_word();
		out[i].ax = fifo_pop_word();
		out[i].ay = fifo_pop_word();
		out[i].az = fifo_pop_word();
	}
	return avail;
}
