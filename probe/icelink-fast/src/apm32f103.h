/* Minimal register definitions -- APM32F103CB (STM32F103CB-compatible).
 * Only what we actually touch; no vendor HAL. */
#ifndef APM32F103_H
#define APM32F103_H

#include <stdint.h>

#define __IO volatile

/* ---- Cortex-M3 core ---- */
#define SCB_VTOR   (*(__IO uint32_t *)0xE000ED08u)
#define SCB_AIRCR  (*(__IO uint32_t *)0xE000ED0Cu)
#define DWT_CTRL   (*(__IO uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(__IO uint32_t *)0xE0001004u)
#define DEM_CR     (*(__IO uint32_t *)0xE000EDFCu)
#define NVIC_ISER(n) (*(__IO uint32_t *)(0xE000E100u + 4u * (n)))
#define DEM_CR_TRCENA (1u << 24)
#define DWT_CTRL_CYCCNTENA (1u << 0)

/* ---- RCC ---- */
typedef struct {
  __IO uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR, APB1ENR, BDCR, CSR;
} RCC_TypeDef;
#define RCC ((RCC_TypeDef *)0x40021000u)

#define RCC_CR_HSEON     (1u << 16)
#define RCC_CR_HSERDY    (1u << 17)
#define RCC_CR_PLLON     (1u << 24)
#define RCC_CR_PLLRDY    (1u << 25)

#define RCC_CFGR_SW_PLL      (2u << 0)
#define RCC_CFGR_SWS_MASK    (3u << 2)
#define RCC_CFGR_SWS_PLL     (2u << 2)
#define RCC_CFGR_PPRE1_DIV2  (4u << 8)
#define RCC_CFGR_PLLSRC_HSE  (1u << 16)
#define RCC_CFGR_PLLMULL9    (7u << 18)
#define RCC_CFGR_PLLMULL12   (10u << 18)
#define RCC_CFGR_USBPRE_DIV15 (0u << 22)   /* 0 = /1.5 -> 48MHz from 72MHz */
#define RCC_CFGR_USBPRE_DIV1  (1u << 22)   /* 1 = /1   -> 48MHz from 48MHz */

#define RCC_APB2ENR_IOPAEN  (1u << 2)
#define RCC_APB2ENR_IOPBEN  (1u << 3)
#define RCC_APB2ENR_IOPCEN  (1u << 4)
#define RCC_APB2ENR_AFIOEN  (1u << 0)
#define RCC_APB2ENR_TIM1EN  (1u << 11)
#define RCC_APB1ENR_USBEN   (1u << 23)
#define RCC_APB1ENR_USART2EN (1u << 17)

/* ---- embedded flash controller ---- */
#define FLASH_ACR   (*(__IO uint32_t *)0x40022000u)
#define FLASH_KEYR  (*(__IO uint32_t *)0x40022004u)
#define FLASH_SR    (*(__IO uint32_t *)0x4002200Cu)
#define FLASH_CR    (*(__IO uint32_t *)0x40022010u)
#define FLASH_AR    (*(__IO uint32_t *)0x40022014u)

#define FLASH_KEY1  0x45670123u
#define FLASH_KEY2  0xCDEF89ABu

#define FLASH_SR_BSY   (1u << 0)
#define FLASH_CR_PER   (1u << 1)
#define FLASH_CR_STRT  (1u << 6)
#define FLASH_CR_LOCK  (1u << 7)

#define AIRCR_SYSRESETREQ 0x05FA0004u
#define FLASH_ACR_LATENCY_1 (1u << 0)
#define FLASH_ACR_LATENCY_2 (2u << 0)
#define FLASH_ACR_PRFTBE    (1u << 4)

/* ---- USART2 (PA2 = TX, PA3 = RX) ----
 * The DAPLink VCP pinout on this probe. APB1 is sysclk/2 on every path
 * clock_init() can take, so the divisor is derived from g_sysclk_hz. */
typedef struct {
  __IO uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;
#define USART2 ((USART_TypeDef *)0x40004400u)

#define USART_SR_RXNE   (1u << 5)
#define USART_SR_TXE    (1u << 7)
#define USART_SR_ORE    (1u << 3)

#define USART_CR1_RE     (1u << 2)
#define USART_CR1_TE     (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_TXEIE  (1u << 7)
#define USART_CR1_UE     (1u << 13)

#define USART2_IRQn 38u

/* ---- GPIO ---- */
typedef struct {
  __IO uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)0x40010800u)
#define GPIOB ((GPIO_TypeDef *)0x40010C00u)
#define GPIOC ((GPIO_TypeDef *)0x40011000u)

/* CNF[1:0]MODE[1:0] nibbles */
#define GPIO_MODE_IN_FLOAT   0x4u
#define GPIO_MODE_IN_PULL    0x8u
#define GPIO_MODE_OUT_PP_50  0x3u
#define GPIO_MODE_AF_PP_50   0xBu

static inline void gpio_cfg(GPIO_TypeDef *p, unsigned pin, uint32_t mode4)
{
  __IO uint32_t *reg = (pin < 8) ? &p->CRL : &p->CRH;
  unsigned sh = (pin & 7u) * 4u;
  *reg = (*reg & ~(0xFu << sh)) | (mode4 << sh);
}

#endif /* APM32F103_H */
