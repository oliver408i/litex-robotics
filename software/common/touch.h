#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <generated/mem.h>

#ifndef CTP_I2C_BASE
#error "touch.h: build the SoC with --with-lcd (ctp_i2c missing)"
#endif

#define FT6336U_I2C_ADDR  0x38

/* ---- Hardware I2C master ---------------------------------------------- */

/* Configure the SCL clock divider (target: ~100 kHz at sys_clk).
 * Call once at boot before any other touch.h function. */
void touch_init(void);

/* Low-level I2C primitives. Most callers don't need these; ft6336u_*
 * below covers the touch IC. Use these to talk to other slaves that
 * might share the ctp_i2c bus (or replace the touch IC entirely). */
void touch_i2c_wait_idle(void);
int  touch_i2c_addr(uint8_t addr7, int read);     /* START + addr byte; returns slave ACK */
int  touch_i2c_write_byte(uint8_t b);             /* returns slave ACK */
uint8_t touch_i2c_read_byte(int ack);             /* ack=1 -> ACK (more bytes follow) */
void touch_i2c_stop(void);

/* ---- FT6336U helpers --------------------------------------------------- */

/* Address-poll the touch IC. Returns true if the FT6336U ACKs. */
bool ft6336u_present(void);

/* Read `len` bytes from FT6336U register `reg` into `buf`.
 * Returns true on success, false if any I2C step NACKed. */
bool ft6336u_read(uint8_t reg, uint8_t *buf, unsigned int len);

/* Probe + log chip_id / vendor_id / fw_ver via log_puts/log_hex8.
 * Useful at boot to confirm the IC is alive. */
void ft6336u_probe(void);
