/* ATWINC1500 BSP for the IcePi Zero (LiteX/VexRiscv).
 *
 * Sidebands wired in icepi_zero_winc.py:
 *   winc_reset : GPIOOut -> RESET_N (active low)  winc_reset_out_write(1 = release)
 *   winc_en    : GPIOOut -> CHIP_EN               winc_en_out_write(1 = enable)
 *   winc_irq   : GPIOIn  <- IRQN  (active low)    winc_irq_in_read() (0 = asserted)
 *
 * IRQ handling is POLLED for bring-up: nm_bsp_register_isr() stashes the HIF
 * handler and the main loop calls winc_service_irq() before
 * m2m_wifi_handle_events(). The GPIOIn EventManager is already in the gateware,
 * so switching to a true edge-triggered ISR later needs no RTL change.
 */
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"

#include <generated/csr.h>
#include <system.h>          /* busy_wait */

/* Polled IRQ service, called from the main loop (also declared in main.c). */
void winc_service_irq(void);

static tpfNmBspIsr gpfIsr;
static uint8       gu8IrqEnabled;

sint8 nm_bsp_init(void)
{
    gpfIsr        = NULL;
    gu8IrqEnabled = 0;
    /* Hold the chip off until nm_bsp_reset(): CHIP_EN low + RESET_N asserted. */
    winc_en_out_write(0);
    winc_reset_out_write(0);
    return M2M_SUCCESS;
}

sint8 nm_bsp_deinit(void)
{
    winc_en_out_write(0);
    winc_reset_out_write(0);
    return M2M_SUCCESS;
}

/* CHIP_EN low, RESET_N low -> CHIP_EN high -> RESET_N high, per the WINC
 * power-up timing (same sequence as the reference SAM ports). */
void nm_bsp_reset(void)
{
    winc_en_out_write(0);
    winc_reset_out_write(0);
    nm_bsp_sleep(1);
    winc_en_out_write(1);
    nm_bsp_sleep(10);
    winc_reset_out_write(1);
}

void nm_bsp_sleep(uint32 u32TimeMsec)
{
    busy_wait((unsigned int)u32TimeMsec);
}

void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
    gpfIsr = pfIsr;
    /* Reference ports (SAMD21 etc.) arm the IRQ as part of registering the
     * ISR -- hif_init() relies on that and never calls interrupt_ctrl(1)
     * itself; only the re-enable paths in hif_isr/hif_set_rx_done do. */
    gu8IrqEnabled = 1;
}

void nm_bsp_interrupt_ctrl(uint8 u8Enable)
{
    gu8IrqEnabled = u8Enable;
}

/* Polled IRQ service: call from the main loop. IRQN is active low, so a 0 on
 * the GPIOIn input means the WINC has a pending message. */
void winc_service_irq(void)
{
    if (gu8IrqEnabled && (gpfIsr != NULL) && (winc_irq_in_read() == 0))
        gpfIsr();
}
