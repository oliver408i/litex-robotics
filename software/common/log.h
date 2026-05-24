#pragma once

#include <stdint.h>

/* Thin wrappers over libbase/uart.h. CRLF on '\n', hex helpers, decimal
 * uint. Use freely from any firmware target; relies only on uart_init()
 * being called once at boot. */

void log_char(char c);
void log_puts(const char *s);
void log_nl(void);

void log_hex4(uint8_t v);
void log_hex8(uint8_t v);
void log_hex16(uint16_t v);
void log_hex32(uint32_t v);

void log_uint(uint32_t v);
