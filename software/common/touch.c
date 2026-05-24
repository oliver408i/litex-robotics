#include "touch.h"
#include "log.h"

#include <generated/csr.h>
#include <generated/mem.h>

/* ---- HW I2C CSR layout ------------------------------------------------
 * litex.soc.cores.i2c.I2CMaster is wishbone-mapped at CTP_I2C_BASE.
 *   offset 0: XFER  - write to issue commands (start/stop/read/write);
 *                     read for received data byte + status (slave ACK,
 *                     idle flag).
 *   offset 1: CONFIG - SCL clock divider. */
#define CTP_I2C_REG(off)  (*(volatile uint32_t *)(CTP_I2C_BASE + ((off) << 2)))
#define I2C_XFER          0
#define I2C_CONFIG        1

#define I2C_ACK    (1u << 8)
#define I2C_READ   (1u << 9)
#define I2C_WRITE  (1u << 10)
#define I2C_START  (1u << 11)
#define I2C_STOP   (1u << 12)
#define I2C_IDLE   (1u << 13)

void touch_init(void)
{
	/* SCL = sys_clk / (2 * (load + 1)) by the I2CMasterMachine timing.
	 * Target 100 kHz, derived at runtime from CONFIG_CLOCK_FREQUENCY. */
	uint32_t load = (CONFIG_CLOCK_FREQUENCY / (2u * 100000u)) - 1u;
	CTP_I2C_REG(I2C_CONFIG) = load;
}

void touch_i2c_wait_idle(void)
{
	while(!(CTP_I2C_REG(I2C_XFER) & I2C_IDLE))
		;
}

int touch_i2c_addr(uint8_t addr7, int read)
{
	touch_i2c_wait_idle();
	CTP_I2C_REG(I2C_XFER) = I2C_START;
	touch_i2c_wait_idle();
	uint32_t byte = (uint32_t)((addr7 << 1) | (read ? 1u : 0u));
	CTP_I2C_REG(I2C_XFER) = I2C_WRITE | byte;
	touch_i2c_wait_idle();
	return (CTP_I2C_REG(I2C_XFER) & I2C_ACK) != 0;
}

int touch_i2c_write_byte(uint8_t b)
{
	touch_i2c_wait_idle();
	CTP_I2C_REG(I2C_XFER) = I2C_WRITE | (uint32_t)b;
	touch_i2c_wait_idle();
	return (CTP_I2C_REG(I2C_XFER) & I2C_ACK) != 0;
}

uint8_t touch_i2c_read_byte(int ack)
{
	touch_i2c_wait_idle();
	CTP_I2C_REG(I2C_XFER) = I2C_READ | (ack ? I2C_ACK : 0u);
	touch_i2c_wait_idle();
	return (uint8_t)(CTP_I2C_REG(I2C_XFER) & 0xff);
}

void touch_i2c_stop(void)
{
	touch_i2c_wait_idle();
	CTP_I2C_REG(I2C_XFER) = I2C_STOP;
	touch_i2c_wait_idle();
}

bool ft6336u_present(void)
{
	int ack = touch_i2c_addr(FT6336U_I2C_ADDR, 0);
	touch_i2c_stop();
	return ack != 0;
}

bool ft6336u_read(uint8_t reg, uint8_t *buf, unsigned int len)
{
	if(!touch_i2c_addr(FT6336U_I2C_ADDR, 0)) { touch_i2c_stop(); return false; }
	if(!touch_i2c_write_byte(reg))           { touch_i2c_stop(); return false; }
	if(!touch_i2c_addr(FT6336U_I2C_ADDR, 1)) { touch_i2c_stop(); return false; }
	for(unsigned int i = 0; i < len; i++)
		buf[i] = touch_i2c_read_byte(i < (len - 1));
	touch_i2c_stop();
	return true;
}

void ft6336u_probe(void)
{
	uint8_t chip_id = 0, vendor_id = 0, fw_ver = 0;

	log_puts("ft6336u: probing 0x"); log_hex8(FT6336U_I2C_ADDR); log_nl();
	if(!ft6336u_present()) {
		log_puts("ft6336u: NO ACK at slave addr"); log_nl();
		return;
	}
	(void)ft6336u_read(0xa3, &chip_id,   1);
	(void)ft6336u_read(0xa8, &vendor_id, 1);
	(void)ft6336u_read(0xa6, &fw_ver,    1);
	log_puts("ft6336u: chip_id=0x");  log_hex8(chip_id);
	log_puts(" vendor_id=0x");        log_hex8(vendor_id);
	log_puts(" fw_ver=0x");           log_hex8(fw_ver);
	log_nl();
}
