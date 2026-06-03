#ifndef WINC_COMPILER_SHIM_H_
#define WINC_COMPILER_SHIM_H_

/* Minimal stand-in for Atmel ASF's <compiler.h>, which m2m_wifi.h includes
 * unconditionally. Off-ASF (LiteX / picolibc) the WINC driver only needs the
 * standard fixed-width integer + bool types from it; the driver's own
 * uint8/uint16/uint32/sint8 typedefs come from bsp/include/nm_bsp.h. Found on
 * the include path via -I. (software/winc_test) in the Makefile. */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#endif /* WINC_COMPILER_SHIM_H_ */
