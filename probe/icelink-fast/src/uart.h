/* USART2 <-> USB CDC bridge: the target's serial console.
 *
 * The i9 has no FTDI, so this is the only way to see a LiteX BIOS prompt on
 * the new board -- the role /dev/ttyUSB0 played on the icepi zero.
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

/* Configure PA2/PA3 and USART2 at `baud`. Safe to call again to re-baud. */
void uart_init(uint32_t baud);

/* Recompute the divisor only. Called from the CDC line-coding callback, so the
 * host's requested baud (litex_term's 1 Mbaud) is what ends up on the wire. */
void uart_set_baud(uint32_t baud);
uint32_t uart_get_baud(void);

/* UART -> ring. Returns bytes copied out, up to `max`. */
uint32_t uart_read(uint8_t *dst, uint32_t max);

/* ring -> UART. Returns bytes accepted (may be < len if the ring is full). */
uint32_t uart_write(const uint8_t *src, uint32_t len);

/* Free space in the TX ring. Callers clamp their reads to this so they never
 * have to spin waiting for the wire. */
uint32_t uart_tx_space(void);

/* Bytes dropped because the RX ring was full, plus hardware overruns. Non-zero
 * means the USB side is not draining fast enough to keep up with the target. */
uint32_t uart_rx_dropped(void);

#endif /* UART_H */
