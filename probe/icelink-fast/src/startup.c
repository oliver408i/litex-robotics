/* Startup for the interface slot. The bootloader jumps here with its own
 * clocks/peripherals still live, so we re-point VTOR and reset what we use. */
#include "apm32f103.h"
#include <stdint.h>
#include "tusb.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern uint32_t _sramfunc_load, _sramfunc, _eramfunc;
void boot_check(void);
extern int main(void);
void Reset_Handler(void);
void Default_Handler(void);

void USB_LP_CAN1_RX0_IRQHandler(void) { tud_int_handler(0); }
void USB_HP_CAN1_TX_IRQHandler(void)  { tud_int_handler(0); }
void USART2_IRQHandler(void);

void Reset_Handler(void)
{
    /* We are NOT at 0x08000000 -- the DFU bootloader's table is. Claim ours. */
    SCB_VTOR = 0x0800C000u;

    /* .ramfunc first: enter_bootloader() must be resident in SRAM before
     * boot_check() can possibly call it. */
    for (uint32_t *s = &_sramfunc_load, *d = &_sramfunc; d < &_eramfunc; )
        *d++ = *s++;
    for (uint32_t *s = &_sidata, *d = &_sdata; d < &_edata; ) *d++ = *s++;

    /* Before .bss is cleared -- the magic lives outside it, but check early so
     * a wedged app still honours a bootloader request. */
    boot_check();

    for (uint32_t *b = &_sbss; b < &_ebss; ) *b++ = 0;

    main();
    for (;;) { }
}

void Default_Handler(void) { for (;;) { } }

#define VEC __attribute__((section(".isr_vector"), used))
VEC void (* const g_vectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    Default_Handler,  /* NMI        */
    Default_Handler,  /* HardFault  */
    Default_Handler,  /* MemManage  */
    Default_Handler,  /* BusFault   */
    Default_Handler,  /* UsageFault */
    0,
    /* The DAPLink bootloader hides `daplink_info` in the reserved vector
     * slots and refuses to launch an image without it -- verified on hardware:
     * without these three words it stayed in MAINTENANCE at PC 0x080046b8.
     * Values read out of the stock interface image at 0x0800c020. */
    (void (*)(void))0x9B939E8Fu,   /* +0x20  DAPLINK_BUILD_KEY_IF */
    (void (*)(void))0x97969908u,   /* +0x24  HIC ID, stm32f103xb   */
    (void (*)(void))0x000000FEu,   /* +0x28  version               */
    Default_Handler,  /* SVC     */
    Default_Handler,  /* DebugMon*/
    0,
    Default_Handler,  /* PendSV  */
    Default_Handler,  /* SysTick */
    /* IRQ0.. : only the two USB lines are named; rest default. */
    [16 + 19] = USB_HP_CAN1_TX_IRQHandler,
    [16 + 20] = USB_LP_CAN1_RX0_IRQHandler,
    [16 + 38] = USART2_IRQHandler,     /* target UART bridge */
    [16 + 42] = Default_Handler,   /* pad table to full F103 length */
};
