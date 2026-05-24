#include "log.h"

#include <libbase/uart.h>

void log_char(char c)
{
	if(c == '\n')
		uart_write('\r');
	uart_write(c);
}

void log_puts(const char *s)
{
	while(*s)
		log_char(*s++);
}

void log_nl(void)
{
	log_char('\n');
	uart_sync();
}

void log_hex4(uint8_t v)
{
	v &= 0x0f;
	log_char(v < 10 ? '0' + v : 'a' + v - 10);
}

void log_hex8(uint8_t v)   { log_hex4(v >> 4); log_hex4(v); }
void log_hex16(uint16_t v) { log_hex8(v >> 8); log_hex8(v); }
void log_hex32(uint32_t v) { log_hex16(v >> 16); log_hex16(v); }

void log_uint(uint32_t v)
{
	char buf[10];
	unsigned int i = 0;

	if(v == 0) {
		log_char('0');
		return;
	}
	while(v && i < sizeof(buf)) {
		buf[i++] = '0' + v % 10;
		v /= 10;
	}
	while(i)
		log_char(buf[--i]);
}
