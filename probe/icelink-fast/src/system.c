/* Clock setup that does not gamble on a crystal being present.
 *
 * The first hardware run hung at the HSERDY spin: the bootloader launched our
 * image, but PC sat in clock_init and GPIO was never configured. Whether this
 * probe populates an 8 MHz crystal is not something we should have to know, so
 * try HSE briefly and fall back to the internal oscillator.
 *
 *   HSE path: 8 MHz * 9  = 72 MHz, USB = 72 / 1.5 = 48 MHz
 *   HSI path: 4 MHz * 12 = 48 MHz, USB = 48 / 1   = 48 MHz
 *
 * Both land USB on exactly 48 MHz, which is the only hard requirement. The HSI
 * path costs SYSCLK headroom (48 vs 72) and HSI's +-1% accuracy is marginal for
 * USB, so it is a fallback, not a target.
 */
#include "apm32f103.h"

uint32_t g_sysclk_hz;      /* read these over SWD to see which path ran */
uint32_t g_clock_source;   /* 0 = HSI fallback, 1 = HSE */

#define HSE_TIMEOUT 200000u

void clock_init(void)
{
    RCC->CR |= RCC_CR_HSEON;
    uint32_t spin = 0;
    while (!(RCC->CR & RCC_CR_HSERDY) && ++spin < HSE_TIMEOUT) { }

    const int have_hse = (RCC->CR & RCC_CR_HSERDY) != 0;

    if (have_hse) {
        FLASH_ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
        RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLSRC_HSE |
                    RCC_CFGR_PLLMULL9 | RCC_CFGR_USBPRE_DIV15;
        g_sysclk_hz = 72000000u;
        g_clock_source = 1;
    } else {
        RCC->CR &= ~RCC_CR_HSEON;
        FLASH_ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;
        /* PLLSRC = 0 selects HSI/2 = 4 MHz. */
        RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLMULL12 |
                    RCC_CFGR_USBPRE_DIV1;
        g_sysclk_hz = 48000000u;
        g_clock_source = 0;
    }

    RCC->CR |= RCC_CR_PLLON;
    spin = 0;
    while (!(RCC->CR & RCC_CR_PLLRDY) && ++spin < HSE_TIMEOUT) { }

    /* If even the PLL refuses, stay on HSI at 8 MHz rather than hanging --
     * a slow blink is diagnosable, a dead board is not. */
    if (RCC->CR & RCC_CR_PLLRDY) {
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        spin = 0;
        while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL &&
               ++spin < HSE_TIMEOUT) { }
    } else {
        g_sysclk_hz = 8000000u;
        g_clock_source = 2;
    }

    DEM_CR |= DEM_CR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}
