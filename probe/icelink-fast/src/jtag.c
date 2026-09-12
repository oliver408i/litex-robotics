/* JTAG bitbang engine for the ECP5.
 *
 * Pins (all on GPIOB, so a whole bus state is one BSRR write):
 *   TCK = PB13   output   (also TIM1_CH1N, if we ever want a hardware clock)
 *   TMS = PB14   output
 *   TDI = PB8    output
 *   TDO = PB9    INPUT -- driven by the ECP5. Never make this an output.
 *
 * Verified against real silicon: IDCODE 0x41112043 (LFE5U-45F).
 *
 * Speed notes. The first version ran at 43.3 cycles/bit (1.67 MHz TCK), far off
 * the ~8 cycles/bit this should manage. Three causes, all fixed here:
 *   - the hot loop was fetched from flash, which runs at 2 wait states at
 *     72 MHz, so it stalled constantly -> the shift loops now live in .ramfunc
 *   - per-bit branches computed the BSRR word -> precomputed now
 *   - unconditional nops padded the half-period -> now a tunable g_jtag_delay
 * USB full-speed caps the host link near 8 Mbit/s, so there is no reason to
 * push TCK far past ~8 MHz.
 */
#include "apm32f103.h"

#define PIN_TCK 13u
#define PIN_TMS 14u
#define PIN_TDI 8u
#define PIN_TDO 9u

#define SET(pin) (1u << (pin))
#define CLR(pin) (1u << ((pin) + 16))

/* Precomputed bus states: TCK low, TMS/TDI as named. */
#define W_TMS0_TDI0 (CLR(PIN_TMS) | CLR(PIN_TDI))
#define W_TMS0_TDI1 (CLR(PIN_TMS) | SET(PIN_TDI))
#define W_TMS1_TDI0 (SET(PIN_TMS) | CLR(PIN_TDI))
#define W_TMS1_TDI1 (SET(PIN_TMS) | SET(PIN_TDI))

uint32_t g_idcode;
uint32_t g_idcode_tries;

/* Extra half-period padding, in loop iterations. 0 = as fast as the CPU goes.
 * Raise it if longer wiring or a different module needs more setup time. */
uint32_t g_jtag_delay;

void jtag_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* TMS high is the safe idle: five such clocks always reach
     * Test-Logic-Reset from any state. */
    GPIOB->BSRR = CLR(PIN_TCK) | SET(PIN_TMS) | CLR(PIN_TDI);

    gpio_cfg(GPIOB, PIN_TCK, GPIO_MODE_OUT_PP_50);
    gpio_cfg(GPIOB, PIN_TMS, GPIO_MODE_OUT_PP_50);
    gpio_cfg(GPIOB, PIN_TDI, GPIO_MODE_OUT_PP_50);
    gpio_cfg(GPIOB, PIN_TDO, GPIO_MODE_IN_FLOAT);
}

/* Release the JTAG bus so an external programmer (e.g. a CH347 clipped onto
 * the same nets) can drive it. TDI/TCK/TMS become floating inputs; TDO was
 * already an input. Nothing in this file re-acquires the bus implicitly --
 * only jtag_bus_acquire() does, so a stray read cannot cause contention.
 *
 * Clip points, LQFP48:  TCK=pin 26 (PB13)  TMS=pin 27 (PB14)
 *                       TDI=pin 45 (PB8)   TDO=pin 46 (PB9)
 */
uint32_t g_bus_released;

void jtag_bus_release(void)
{
    gpio_cfg(GPIOB, PIN_TCK, GPIO_MODE_IN_FLOAT);
    gpio_cfg(GPIOB, PIN_TMS, GPIO_MODE_IN_FLOAT);
    gpio_cfg(GPIOB, PIN_TDI, GPIO_MODE_IN_FLOAT);
    gpio_cfg(GPIOB, PIN_TDO, GPIO_MODE_IN_FLOAT);
    g_bus_released = 1u;
}

void jtag_bus_acquire(void)
{
    GPIOB->BSRR = CLR(PIN_TCK) | SET(PIN_TMS) | CLR(PIN_TDI);
    gpio_cfg(GPIOB, PIN_TCK, GPIO_MODE_OUT_PP_50);
    gpio_cfg(GPIOB, PIN_TMS, GPIO_MODE_OUT_PP_50);
    gpio_cfg(GPIOB, PIN_TDI, GPIO_MODE_OUT_PP_50);
    gpio_cfg(GPIOB, PIN_TDO, GPIO_MODE_IN_FLOAT);
    g_bus_released = 0u;
}

#define JDELAY() do { for (uint32_t d = g_jtag_delay; d; d--) __asm volatile("nop"); } while (0)

/* One TCK period. TMS/TDI are set up while TCK is low; the target samples them
 * on the rising edge. TDO is driven on the falling edge, so the value for this
 * cycle is read just before we raise TCK. */
__attribute__((section(".ramfunc"), noinline, used))
uint32_t jtag_clock(uint32_t tms, uint32_t tdi)
{
    GPIOB->BSRR = tms ? (tdi ? W_TMS1_TDI1 : W_TMS1_TDI0)
                      : (tdi ? W_TMS0_TDI1 : W_TMS0_TDI0);
    JDELAY();

    uint32_t tdo = (GPIOB->IDR >> PIN_TDO) & 1u;

    GPIOB->BSRR = SET(PIN_TCK);
    JDELAY();
    GPIOB->BSRR = CLR(PIN_TCK);

    return tdo;
}

/* The hot path. Design notes, after a first attempt that was slower than the
 * naive version it replaced:
 *   - no per-bit branch: a 2-entry table gives the BSRR word for TDI 0/1
 *   - no per-bit "is this the last bit" test: the final bit (which must raise
 *     TMS to exit) is done once, outside the loop
 *   - no delay-loop overhead in the common case: delay==0 gets its own loop
 *   - TDO accumulated by shifting down, so there is no variable shift per bit
 *   - unrolled x4 to amortise loop control
 * The floor here is the bus, not the CPU: three APB stores plus one APB load
 * per bit, and the load cannot be buffered because we need the value.
 */
__attribute__((section(".ramfunc"), noinline, used))
static uint32_t shift_bits_fast(uint32_t tdi_word, uint32_t nbits)
{
    GPIO_TypeDef *const gpio = GPIOB;
    const uint32_t tck_set = SET(PIN_TCK), tck_clr = CLR(PIN_TCK);
    const uint32_t lut0 = W_TMS0_TDI0, lut1 = W_TMS0_TDI1;
    const uint32_t delay = g_jtag_delay;
    uint32_t out = 0;
    uint32_t n = nbits - 1u;          /* last bit handled separately */

    if (delay == 0u) {
        while (n >= 4u) {
            #define STEP()                                                   \
                gpio->BSRR = (tdi_word & 1u) ? lut1 : lut0;                  \
                tdi_word >>= 1;                                              \
                out = (out >> 1) | (((gpio->IDR >> PIN_TDO) & 1u) << 31);    \
                gpio->BSRR = tck_set;                                        \
                gpio->BSRR = tck_clr;
            STEP(); STEP(); STEP(); STEP();
            n -= 4u;
        }
        while (n--) { STEP(); }
        #undef STEP
    } else {
        while (n--) {
            gpio->BSRR = (tdi_word & 1u) ? lut1 : lut0;
            tdi_word >>= 1;
            for (uint32_t d = delay; d; d--) __asm volatile("nop");
            out = (out >> 1) | (((gpio->IDR >> PIN_TDO) & 1u) << 31);
            gpio->BSRR = tck_set;
            for (uint32_t d = delay; d; d--) __asm volatile("nop");
            gpio->BSRR = tck_clr;
        }
    }

    /* Final bit: TMS high to leave Shift-xR via Exit1-xR. */
    gpio->BSRR = (tdi_word & 1u) ? W_TMS1_TDI1 : W_TMS1_TDI0;
    for (uint32_t d = delay; d; d--) __asm volatile("nop");
    out = (out >> 1) | (((gpio->IDR >> PIN_TDO) & 1u) << 31);
    gpio->BSRR = tck_set;
    for (uint32_t d = delay; d; d--) __asm volatile("nop");
    gpio->BSRR = tck_clr;

    /* Bits arrived at the top of `out`; right-align them. */
    return out >> (32u - nbits);
}

/* Test-Logic-Reset from any state, then Run-Test/Idle. */
void jtag_tap_reset(void)
{
    for (int i = 0; i < 8; i++) jtag_clock(1, 0);
    jtag_clock(0, 0);
}

/* Enters and leaves at Run-Test/Idle. */
uint32_t jtag_shift_dr(unsigned nbits)
{
    jtag_clock(1, 0);        /* Select-DR-Scan */
    jtag_clock(0, 0);        /* Capture-DR     */
    jtag_clock(0, 0);        /* Shift-DR       */

    uint32_t out = shift_bits_fast(0, nbits);   /* leaves via Exit1-DR */

    jtag_clock(1, 0);        /* Update-DR      */
    jtag_clock(0, 0);        /* Run-Test/Idle  */
    return out;
}

#define ECP5_IR_BITS      8u
#define ECP5_READ_ID      0xE0u
#define ECP5_USERCODE     0xC0u
#define ECP5_READ_STATUS  0x3Cu

void jtag_shift_ir(uint32_t instr)
{
    jtag_clock(1, 0);        /* Select-DR-Scan */
    jtag_clock(1, 0);        /* Select-IR-Scan */
    jtag_clock(0, 0);        /* Capture-IR     */
    jtag_clock(0, 0);        /* Shift-IR       */

    shift_bits_fast(instr, ECP5_IR_BITS);       /* leaves via Exit1-IR */

    jtag_clock(1, 0);        /* Update-IR      */
    jtag_clock(0, 0);        /* Run-Test/Idle  */
}

uint32_t jtag_read_reg32(uint32_t instr)
{
    jtag_shift_ir(instr);
    return jtag_shift_dr(32);
}

uint32_t jtag_read_usercode(void) { return jtag_read_reg32(ECP5_USERCODE); }
uint32_t jtag_read_status(void)   { return jtag_read_reg32(ECP5_READ_STATUS); }

uint32_t jtag_read_idcode(void)
{
    jtag_tap_reset();
    return jtag_shift_dr(32);
}

uint32_t jtag_read_id_via_ir(void)
{
    jtag_tap_reset();
    return jtag_read_reg32(ECP5_READ_ID);
}

/* Measured cycles per TCK period, x256 for fractional resolution. Uses the
 * same RAM-resident loop the real shifts use, so the number is honest. */
uint32_t jtag_bench_cycles_per_bit_x256(void)
{
    const uint32_t N = 4096u;
    uint32_t t0 = DWT_CYCCNT;
    for (uint32_t i = 0; i < N / 32u; i++) shift_bits_fast(0, 32);
    uint32_t elapsed = DWT_CYCCNT - t0;
    return (uint32_t)(((uint64_t)elapsed * 256u) / N);
}

/* Sample all four JTAG nets while somebody else drives them. Counts edges so
 * we can tell "this wire is dead" from "this wire is alive but the data is
 * wrong" -- the difference between a wiring fault and a protocol fault.
 * Requires the bus to be released (hi-Z) or we would be measuring ourselves.
 */
__attribute__((section(".ramfunc"), noinline, used))
void jtag_monitor(uint32_t iters, uint32_t *edges_tck, uint32_t *edges_tms,
                  uint32_t *edges_tdi, uint32_t *edges_tdo, uint32_t *seen_high,
                  uint32_t *seen_low)
{
    GPIO_TypeDef *const gpio = GPIOB;
    const uint32_t mask = (1u << PIN_TCK) | (1u << PIN_TMS) |
                          (1u << PIN_TDI) | (1u << PIN_TDO);
    uint32_t prev = gpio->IDR & mask;
    uint32_t hi = 0, lo = 0xFFFFFFFFu;
    uint32_t etck = 0, etms = 0, etdi = 0, etdo = 0;

    for (uint32_t i = 0; i < iters; i++) {
        uint32_t now = gpio->IDR & mask;
        uint32_t ch = now ^ prev;
        if (ch) {
            if (ch & (1u << PIN_TCK)) etck++;
            if (ch & (1u << PIN_TMS)) etms++;
            if (ch & (1u << PIN_TDI)) etdi++;
            if (ch & (1u << PIN_TDO)) etdo++;
            prev = now;
        }
        hi |= now;
        lo &= now;
    }

    *edges_tck = etck; *edges_tms = etms;
    *edges_tdi = etdi; *edges_tdo = etdo;
    *seen_high = hi;   *seen_low = lo;
}

/* Capture the real transaction: sample TMS/TDI/TDO on each TCK rising edge.
 * This is the only way to see what an external adapter actually puts on the
 * wire, as opposed to what its host software believes it asked for.
 * Returns the number of samples captured. Bit 0 = TMS, 1 = TDI, 2 = TDO.
 */
__attribute__((section(".ramfunc"), noinline, used))
uint32_t jtag_capture(uint8_t *buf, uint32_t n, uint32_t timeout)
{
    GPIO_TypeDef *const gpio = GPIOB;
    uint32_t count = 0;
    uint32_t prev = (gpio->IDR >> PIN_TCK) & 1u;

    while (count < n && timeout--) {
        uint32_t idr = gpio->IDR;
        uint32_t tck = (idr >> PIN_TCK) & 1u;
        if (tck && !prev) {                    /* rising edge */
            buf[count++] = (uint8_t)(((idr >> PIN_TMS) & 1u) |
                                     (((idr >> PIN_TDI) & 1u) << 1) |
                                     (((idr >> PIN_TDO) & 1u) << 2));
        }
        prev = tck;
    }
    return count;
}

/* Is the link still correct at the current delay? Reading IDCODE many times
 * catches marginal timing that a single sample would miss. */
int jtag_verify(unsigned rounds)
{
    for (unsigned i = 0; i < rounds; i++)
        if (jtag_read_idcode() != 0x41112043u) return 0;
    return 1;
}
