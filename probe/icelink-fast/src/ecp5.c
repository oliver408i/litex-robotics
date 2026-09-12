/* ECP5 SRAM configuration over JTAG -- see ecp5.h for why it is split in three. */
#include "apm32f103.h"
#include "ecp5.h"

/* From jtag.c. */
void     jtag_tap_reset(void);
void     jtag_shift_ir(uint32_t instr);
uint32_t jtag_shift_dr(unsigned nbits);
void     jtag_write_dr(uint32_t value, unsigned nbits);
void     jtag_run_test(uint32_t n);
void     jtag_enter_shift_dr(void);
void     jtag_leave_shift_dr(void);
void     jtag_shift_last_bit(uint32_t tdi);
void     jtag_shift_bytes_out(const uint8_t *buf, uint32_t nbytes);
uint32_t jtag_read_reg32(uint32_t instr);
extern uint32_t g_sysclk_hz;

/* REFRESH restarts configuration and needs real settling time before the TAP
 * is talked to again -- the guide calls for tens of ms. No timer is free here
 * (timer0 belongs to nobody on this MCU, but busy_wait does not exist either),
 * so spin on the cycle counter, which clock_init() already set up. */
static void delay_ms(uint32_t ms)
{
    uint32_t per_ms = (g_sysclk_hz ? g_sysclk_hz : 72000000u) / 1000u;
    while (ms--) {
        uint32_t t0 = DWT_CYCCNT;
        while ((DWT_CYCCNT - t0) < per_ms) { }
    }
}

static int idcode_plausible(uint32_t id)
{
    /* Every ECP5 IDCODE is 0x?111?043; 0 and all-ones mean TDO is stuck. */
    return id != 0u && id != 0xFFFFFFFFu;
}

ecp5_err_t ecp5_config_begin(uint32_t *idcode, uint32_t *status_after_erase)
{
    jtag_tap_reset();

    /* REFRESH: drop any running configuration and start clean. */
    jtag_shift_ir(ECP5_LSC_REFRESH);
    jtag_run_test(2);
    delay_ms(50);

    jtag_tap_reset();
    uint32_t id = jtag_read_reg32(ECP5_READ_ID);
    if (idcode) *idcode = id;
    if (!idcode_plausible(id)) return ECP5_ERR_IDCODE;

    /* Preload: shift ones through the 510-bit boundary-scan chain so no pin is
     * left driving while the fabric is reconfigured. */
    jtag_shift_ir(ECP5_LSC_PRELOAD);
    jtag_enter_shift_dr();
    {
        static const uint8_t ones[8] = {
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
        };
        /* 510 bits = 63 whole bytes + 6 bits; the last bit exits the scan. */
        for (int i = 0; i < 7; i++) jtag_shift_bytes_out(ones, 8);   /* 448 */
        jtag_shift_bytes_out(ones, 7);                               /* 504 */
        for (int i = 0; i < 5; i++) jtag_shift_last_bit(1);          /* mid-scan */
    }
    jtag_leave_shift_dr();

    jtag_shift_ir(ECP5_ISC_ENABLE);
    jtag_write_dr(0x00, 8);
    jtag_run_test(2);

    jtag_shift_ir(ECP5_ISC_ERASE);
    jtag_write_dr(0x01, 8);
    jtag_run_test(2);
    delay_ms(10);

    /* After the erase DONE must have dropped. Reported rather than enforced:
     * it is the cheapest evidence the sequence is landing, and a part that
     * refuses to clear DONE is worth seeing rather than silently aborting. */
    if (status_after_erase) *status_after_erase = jtag_read_reg32(ECP5_LSC_READ_STATUS);

    jtag_shift_ir(ECP5_LSC_SET_WORKING_ADDR);
    jtag_write_dr(0x01, 8);
    jtag_run_test(2);

    /* Park in Shift-DR under BITSTREAM_BURST; every byte from here is payload. */
    jtag_shift_ir(ECP5_LSC_BITSTREAM_BURST);
    jtag_enter_shift_dr();
    return ECP5_OK;
}

void ecp5_config_data(const uint8_t *buf, uint32_t n)
{
    jtag_shift_bytes_out(buf, n);
}

ecp5_err_t ecp5_config_end(uint32_t *final_status)
{
    /* Leaving Shift-DR inherently clocks one more bit. That is fine: the
     * bitstream carries its own termination and trailing bits are ignored. */
    jtag_shift_last_bit(0);
    jtag_leave_shift_dr();
    jtag_run_test(100);

    uint32_t st = jtag_read_reg32(ECP5_LSC_READ_STATUS);

    jtag_shift_ir(ECP5_ISC_DISABLE);
    jtag_run_test(2);
    jtag_shift_ir(ECP5_ISC_NOOP);
    jtag_run_test(2);

    st = jtag_read_reg32(ECP5_LSC_READ_STATUS);
    if (final_status) *final_status = st;

    return (st & ECP5_STATUS_DONE) ? ECP5_OK : ECP5_ERR_NOT_DONE;
}
