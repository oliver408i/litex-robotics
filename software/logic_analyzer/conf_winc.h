#ifndef CONF_WINC_H_
#define CONF_WINC_H_

/* ATWINC1500 host-driver configuration for the IcePi Zero (LiteX/VexRiscv).
 *
 * Force-included into every translation unit by the Makefile (-include
 * conf_winc.h) so the driver headers see CONF_WINC_USE_SPI before they gate on
 * it (e.g. bus_wrapper/include/nm_bus_wrapper.h). */

/* The WINC rides the shared sensor/aux SPI bus (see icepi_zero_winc.py). */
#define CONF_WINC_USE_SPI       (1)

/* HIF interrupt model (exactly one). We service the WINC IRQ by polling its
 * level (IRQN low = pending) in winc_service_irq(), so LEVEL is the match. */
#define NM_LEVEL_INTERRUPT

/* Driver logging over UART (nm_debug.h M2M_ERR/INFO/REQ at M2M_LOG_REQ level).
 * Turned ON for bring-up to surface chip/firmware version info -- nmdrv.c
 * prints "Firmware ver / Min driver ver / Curr driver ver" during init, which
 * is how we diagnose M2M_ERR_FW_VER_MISMATCH (-13). printf prototype comes
 * from winc_prelude.h's <stdio.h>. Set back to 0 once bring-up is done. */
#define CONF_WINC_DEBUG         (1)
#define CONF_WINC_PRINTF        printf

#endif /* CONF_WINC_H_ */
