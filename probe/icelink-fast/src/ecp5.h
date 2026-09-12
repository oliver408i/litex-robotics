/* ECP5 SRAM configuration over JTAG.
 *
 * Split into begin / data / end because a 45F bitstream is ~1.4 MB and the
 * probe has 20 KB of RAM: the whole thing has to be shifted straight from the
 * USB FIFO to TDI as it arrives, never buffered. Between begin() and end() the
 * TAP sits in Shift-DR and data() can be called as many times as the host
 * needs.
 *
 * Sequence per the ECP5 sysCONFIG usage guide and cross-checked against
 * apollo/ecp5.py: REFRESH, READ_ID, preload, ISC_ENABLE, ISC_ERASE,
 * SET_WORKING_ADDRESS, BITSTREAM_BURST, ISC_DISABLE, then STATUS bit 8 (DONE).
 */
#ifndef ECP5_H
#define ECP5_H

#include <stdint.h>

#define ECP5_STATUS_DONE (1u << 8)

/* ECP5 JTAG instructions. */
#define ECP5_READ_ID              0xE0u
#define ECP5_USERCODE             0xC0u
#define ECP5_LSC_READ_STATUS      0x3Cu
#define ECP5_LSC_REFRESH          0x79u
#define ECP5_LSC_PRELOAD          0x1Cu
#define ECP5_ISC_ENABLE           0xC6u
#define ECP5_ISC_ERASE            0x0Eu
#define ECP5_LSC_SET_WORKING_ADDR 0x46u
#define ECP5_LSC_BITSTREAM_BURST  0x7Au
#define ECP5_ISC_DISABLE          0x26u
#define ECP5_ISC_NOOP             0xFFu

typedef enum {
    ECP5_OK = 0,
    ECP5_ERR_IDCODE,     /* no plausible part on the chain */
    ECP5_ERR_NOT_DONE,   /* burst finished but DONE never came up */
} ecp5_err_t;

/* Run everything up to and including entering Shift-DR under
 * LSC_BITSTREAM_BURST. After this returns ECP5_OK the TAP is parked ready for
 * bitstream bytes. `status_after_erase` (optional) reports the status register
 * once the SRAM is erased -- DONE should have dropped there, which is the
 * cheapest proof the sequence is actually taking effect. */
ecp5_err_t ecp5_config_begin(uint32_t *idcode, uint32_t *status_after_erase);

/* Shift bitstream bytes, MSB-first. Call repeatedly; stays in Shift-DR. */
void ecp5_config_data(const uint8_t *buf, uint32_t n);

/* Leave Shift-DR, settle, disable ISC, and check DONE. */
ecp5_err_t ecp5_config_end(uint32_t *final_status);

#endif /* ECP5_H */
