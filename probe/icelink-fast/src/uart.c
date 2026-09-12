/* USART2 on PA2 (TX) / PA3 (RX) -- the target's serial console, bridged to the
 * second USB CDC.
 *
 * Both directions are interrupt-driven through SPSC rings. Polling instead
 * would be simpler, but a 64-byte USB packet at 1 Mbaud is ~640 us of spinning,
 * and tud_task() has to keep running or the JTAG console stalls and the USB
 * stack starts NAKing. So the ISR owns the wire and the main loop only moves
 * bytes between rings and USB FIFOs.
 *
 * Ring discipline: each index has exactly one writer (rx.head in the ISR,
 * rx.tail in the main loop, and the reverse for tx), so no critical sections
 * are needed on a single-core M3 -- aligned 32-bit loads and stores are atomic.
 * Sizes must stay powers of two for the mask to work.
 */
#include "apm32f103.h"
#include "uart.h"

#define PIN_TX 2u   /* PA2 -- USART2_TX */
#define PIN_RX 3u   /* PA3 -- USART2_RX */

/* RX is the deep one: the target talks in bursts (a BIOS banner) and the USB
 * side only drains once per main-loop pass. TX is host-paced and shallow. */
#define RX_SIZE 1024u
#define TX_SIZE 512u
#define RX_MASK (RX_SIZE - 1u)
#define TX_MASK (TX_SIZE - 1u)

static struct { volatile uint32_t head, tail; uint8_t buf[RX_SIZE]; } rx;
static struct { volatile uint32_t head, tail; uint8_t buf[TX_SIZE]; } tx;

static volatile uint32_t rx_dropped;
static uint32_t cur_baud;

extern uint32_t g_sysclk_hz;   /* set by clock_init() */

/* APB1 is sysclk/2 on every path clock_init() can take (all three set
 * PPRE1_DIV2), so this holds for the HSE, HSI and no-PLL cases alike. */
static uint32_t apb1_hz(void)
{
    uint32_t sys = g_sysclk_hz ? g_sysclk_hz : 72000000u;
    return sys / 2u;
}

void uart_set_baud(uint32_t baud)
{
    if (baud == 0u) return;
    cur_baud = baud;

    /* BRR packs mantissa:fraction in 1/16ths, so fck/baud lands in the register
     * directly. At 36 MHz APB1 this is exact for 1 Mbaud (BRR=36 -> 2 + 4/16). */
    uint32_t brr = (apb1_hz() + baud / 2u) / baud;
    if (brr < 16u) brr = 16u;          /* below this the divisor underflows */

    uint32_t cr1 = USART2->CR1;
    USART2->CR1 = cr1 & ~USART_CR1_UE; /* quiesce before re-dividing */
    USART2->BRR = brr;
    USART2->CR1 = cr1;
}

uint32_t uart_get_baud(void) { return cur_baud; }

void uart_init(uint32_t baud)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    gpio_cfg(GPIOA, PIN_TX, GPIO_MODE_AF_PP_50);
    gpio_cfg(GPIOA, PIN_RX, GPIO_MODE_IN_FLOAT);

    rx.head = rx.tail = 0;
    tx.head = tx.tail = 0;
    rx_dropped = 0;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    uart_set_baud(baud);

    NVIC_ISER(USART2_IRQn / 32u) = 1u << (USART2_IRQn % 32u);
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    /* Reading SR then DR is also what clears ORE. Dropping the overrun flag on
     * the floor without that read wedges the receiver permanently. */
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t c = (uint8_t)USART2->DR;
        if (sr & USART_SR_ORE) rx_dropped++;

        uint32_t h = rx.head;
        uint32_t n = (h + 1u) & RX_MASK;
        if (n != rx.tail) {
            rx.buf[h] = c;
            rx.head = n;
        } else {
            rx_dropped++;      /* ring full: the USB side is behind */
        }
    }

    if (sr & USART_SR_TXE) {
        uint32_t t = tx.tail;
        if (t == tx.head) {
            /* Nothing left: stop asking. Re-armed by uart_write(). */
            USART2->CR1 &= ~USART_CR1_TXEIE;
        } else {
            USART2->DR = tx.buf[t];
            tx.tail = (t + 1u) & TX_MASK;
        }
    }
}

uint32_t uart_read(uint8_t *dst, uint32_t max)
{
    uint32_t n = 0;
    uint32_t t = rx.tail;
    while (n < max && t != rx.head) {
        dst[n++] = rx.buf[t];
        t = (t + 1u) & RX_MASK;
    }
    rx.tail = t;
    return n;
}

uint32_t uart_write(const uint8_t *src, uint32_t len)
{
    uint32_t n = 0;
    uint32_t h = tx.head;
    while (n < len) {
        uint32_t nx = (h + 1u) & TX_MASK;
        if (nx == tx.tail) break;      /* full -- caller keeps the rest */
        tx.buf[h] = src[n++];
        h = nx;
    }
    tx.head = h;

    /* Arm TXE only after head is visible, so the ISR cannot see an empty ring
     * and disable itself while bytes are already queued. */
    if (n) USART2->CR1 |= USART_CR1_TXEIE;
    return n;
}

uint32_t uart_tx_space(void)
{
    uint32_t h = tx.head, t = tx.tail;
    /* One slot is always left empty to distinguish full from empty. */
    return (t + TX_SIZE - h - 1u) & TX_MASK;
}

uint32_t uart_rx_dropped(void) { return rx_dropped; }
