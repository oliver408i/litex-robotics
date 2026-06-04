#include <stdint.h>

#include <irq.h>
#include <libbase/uart.h>
#include <system.h>
#include <generated/csr.h>
#include <generated/soc.h>

#ifndef CSR_IMU_SPI_BASE
#error "Build the SoC from icepi_zero_imu.py (provides imu_spi) before compiling software/imu_test."
#endif

/* ---- LSM6DS3 register map (subset used for bring-up) ------------------- */
#define LSM6DS3_WHO_AM_I   0x0F  /* expect 0x69 (0x6A on the TR-C variant)  */
#define LSM6DS3_CTRL1_XL   0x10  /* accel ODR / full-scale                  */
#define LSM6DS3_CTRL2_G    0x11  /* gyro  ODR / full-scale                  */
#define LSM6DS3_CTRL3_C    0x12  /* BDU / IF_INC (register auto-increment)  */
#define LSM6DS3_STATUS_REG 0x1E  /* bit0 XLDA (accel), bit1 GDA (gyro)      */
#define LSM6DS3_OUTX_L_G   0x22  /* gyro X..Z (6B) then accel X..Z (6B)     */

#define LSM6DS3_WHO_AM_I_EXPECTED 0x69

/* imu_spi chip-selects: cs[0] = IMU, cs[1] = MCP3008 (parked). */
#define IMU_CS_SEL 0x1u
/* _cs CSR: sel at bit0, mode at bit16 (1 = manual CS held by firmware). */
#define IMU_CS_MANUAL_ASSERT ((1u << 16) | IMU_CS_SEL)
#define IMU_CS_RELEASE       0u

/* ---- tiny UART logging (self-contained; no common/ deps) --------------- */
static void log_char(char c) { if(c == '\n') uart_write('\r'); uart_write(c); }
static void log_puts(const char *s) { while(*s) log_char(*s++); }
static void log_hex4(uint8_t v) { v &= 0x0f; log_char(v < 10 ? '0' + v : 'a' + v - 10); }
static void log_hex8(uint8_t v) { log_hex4(v >> 4); log_hex4(v); }
static void log_hex16(uint16_t v) { log_hex8(v >> 8); log_hex8(v); }
static void log_hex32(uint32_t v) { log_hex16(v >> 16); log_hex16(v); }
static void log_nl(void) { log_char('\n'); uart_sync(); }

static void log_uint(uint32_t v)
{
	char buf[10];
	unsigned int i = 0;
	if(v == 0) { log_char('0'); return; }
	while(v && i < sizeof(buf)) { buf[i++] = '0' + v % 10; v /= 10; }
	while(i) log_char(buf[--i]);
}

static void log_int(int32_t v)
{
	if(v < 0) { log_char('-'); v = -v; }
	log_uint((uint32_t)v);
}

/* ---- SPIMaster primitives ---------------------------------------------- */
/* One 8-bit full-duplex transfer; returns the byte clocked in on MISO. */
static uint8_t spi_xfer8(uint8_t out)
{
	imu_spi_mosi_write(out);
	/* control: length[15:8] = 8 bits, start[0] = 1 (auto-clearing pulse). */
	imu_spi_control_write((8u << 8) | 1u);
	while((imu_spi_status_read() & 1u) == 0)  /* wait for done */
		;
	return (uint8_t)imu_spi_miso_read();
}

/* Burst read of `n` bytes starting at `reg` (auto-increment); CS held low
 * across the whole frame via manual-CS mode. */
static void imu_read(uint8_t reg, uint8_t *buf, unsigned int n)
{
	imu_spi_cs_write(IMU_CS_MANUAL_ASSERT);
	spi_xfer8(0x80u | reg);            /* bit7 = 1: read */
	for(unsigned int i = 0; i < n; i++)
		buf[i] = spi_xfer8(0x00u);
	imu_spi_cs_write(IMU_CS_RELEASE);
}

static uint8_t imu_read_reg(uint8_t reg)
{
	uint8_t v;
	imu_read(reg, &v, 1);
	return v;
}

static void imu_write_reg(uint8_t reg, uint8_t val)
{
	imu_spi_cs_write(IMU_CS_MANUAL_ASSERT);
	spi_xfer8(reg & 0x7fu);            /* bit7 = 0: write */
	spi_xfer8(val);
	imu_spi_cs_write(IMU_CS_RELEASE);
}

/* ---- bring-up sequence ------------------------------------------------- */
static void dump_ctrl_regs(void)
{
	for(uint8_t r = 0x10; r <= 0x1A; r++) {
		log_puts("  reg 0x"); log_hex8(r);
		log_puts(" = 0x");    log_hex8(imu_read_reg(r));
		log_nl();
	}
}

static void stream_samples(unsigned int count)
{
	uint8_t raw[12];

	for(unsigned int i = 0; i < count; i++) {
		/* wait until both accel + gyro have fresh data */
		while((imu_read_reg(LSM6DS3_STATUS_REG) & 0x03u) != 0x03u)
			;
		imu_read(LSM6DS3_OUTX_L_G, raw, sizeof(raw));

		int16_t gx = (int16_t)(raw[0]  | (raw[1]  << 8));
		int16_t gy = (int16_t)(raw[2]  | (raw[3]  << 8));
		int16_t gz = (int16_t)(raw[4]  | (raw[5]  << 8));
		int16_t ax = (int16_t)(raw[6]  | (raw[7]  << 8));
		int16_t ay = (int16_t)(raw[8]  | (raw[9]  << 8));
		int16_t az = (int16_t)(raw[10] | (raw[11] << 8));

		log_puts("a=["); log_int(ax); log_char(','); log_int(ay); log_char(',');
		log_int(az); log_puts("] g=["); log_int(gx); log_char(','); log_int(gy);
		log_char(','); log_int(gz); log_char(']');
		/* +-2g => 0.061 mg/LSB; show |a| axis-by-axis as milli-g for sanity */
		log_puts("  (az_mg="); log_int((int32_t)az * 61 / 1000); log_char(')');
		log_nl();
	}
}

int main(void)
{
	uart_init();
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setie(1);
#endif

	log_puts("LSM6DS3 IMU bring-up (SPIMaster)"); log_nl();
	log_puts("CSR uart    base=0x"); log_hex32(CSR_UART_BASE);    log_nl();
	log_puts("CSR imu_spi base=0x"); log_hex32(CSR_IMU_SPI_BASE); log_nl();
	log_puts("sys clk="); log_uint(CONFIG_CLOCK_FREQUENCY); log_puts(" Hz"); log_nl();
	log_puts("spi clk="); log_uint(IMU_SPI_FREQUENCY);      log_puts(" Hz"); log_nl();

	uint8_t who = imu_read_reg(LSM6DS3_WHO_AM_I);
	log_puts("WHO_AM_I = 0x"); log_hex8(who);
	if(who == LSM6DS3_WHO_AM_I_EXPECTED) {
		log_puts("  OK");
	} else if(who == 0x6A) {
		log_puts("  (looks like LSM6DS3TR-C / LSM6DSL -- adjust expected id)");
	} else {
		log_puts("  MISMATCH -- check wiring/CS/SPI mode");
	}
	log_nl();

	if(who != LSM6DS3_WHO_AM_I_EXPECTED && who != 0x6A) {
		log_puts("Halting before configuring an unidentified device."); log_nl();
		while(1) ;
	}

	/* CTRL3_C: BDU=1 (coherent hi/lo), IF_INC=1 (burst auto-increment). */
	imu_write_reg(LSM6DS3_CTRL3_C, 0x44);
	/* CTRL1_XL = 0x60: accel ODR 104 Hz, FS +-2g. */
	imu_write_reg(LSM6DS3_CTRL1_XL, 0x60);
	/* CTRL2_G  = 0x60: gyro  ODR 104 Hz, FS 250 dps. */
	imu_write_reg(LSM6DS3_CTRL2_G, 0x60);

	log_puts("control registers after config:"); log_nl();
	dump_ctrl_regs();

	log_puts("streaming raw samples (expect az ~ +1g face-up):"); log_nl();
	stream_samples(40);

	log_puts("IMU bring-up complete"); log_nl();
	while(1)
		;

	return 0;
}
