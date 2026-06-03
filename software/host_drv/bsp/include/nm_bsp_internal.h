#ifndef _NM_BSP_INTERNAL_H_
#define _NM_BSP_INTERNAL_H_

/* IcePi Zero port: this header normally selects an Arduino platform variant
 * (nm_bsp_avr.h / nm_bsp_samd21.h) that defines CONF_WINC_USE_SPI and the
 * interrupt model. Here those come from conf_winc.h, force-included into every
 * TU by the winc_test Makefile (CONF_WINC_USE_SPI + NM_LEVEL_INTERRUPT), so
 * the WiFi101 variant headers are intentionally not used. */

#endif /* _NM_BSP_INTERNAL_H_ */
