/* Escape hatch: get back into the DAPLink bootloader without a reset pin.
 *
 * The DAPLink bootloader launches whatever is at 0x0800C000 if its vector table
 * looks valid, and presents its MAINTENANCE mass-storage drive when it does
 * not. So "reboot to bootloader" is simply: destroy our own vector table, then
 * reset. No reverse-engineering of DAPLink's shared-RAM magic required, and no
 * NRST -- the reset comes from AIRCR.SYSRESETREQ.
 *
 * The catch: this whole image is well under 1 KB, so it lives *inside* the page
 * being erased. Executing from flash while that flash is erasing faults the
 * core on the next instruction fetch. enter_bootloader() therefore runs from
 * SRAM (.ramfunc, copied by startup.c) and never returns.
 *
 * Cost of using it: the app is gone afterwards. That is fine -- the only reason
 * to call it is to flash a new one.
 */
#include "apm32f103.h"

#define APP_BASE   0x0800C000u

/* Survives SYSRESETREQ (RAM is not cleared) but not a power cycle. Lives above
 * _estack in a linker-reserved hole so nothing else can land on it. */
extern uint32_t _boot_magic;
#define BOOT_MAGIC 0xB007B007u

__attribute__((section(".ramfunc"), noinline, used))
void enter_bootloader(void)
{
    __asm volatile ("cpsid i");

    while (FLASH_SR & FLASH_SR_BSY) { }

    if (FLASH_CR & FLASH_CR_LOCK) {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }

    /* Page erase of the app's first page -- takes out our vector table. */
    FLASH_CR |= FLASH_CR_PER;
    FLASH_AR  = APP_BASE;
    FLASH_CR |= FLASH_CR_STRT;
    while (FLASH_SR & FLASH_SR_BSY) { }
    FLASH_CR &= ~FLASH_CR_PER;

    /* From here the vector table is 0xffffffff, so we must not take an
     * exception before the reset lands. Interrupts are already off. */
    SCB_AIRCR = AIRCR_SYSRESETREQ;
    __asm volatile ("dsb");
    for (;;) { }
}

/* Called early from Reset_Handler, before anything else touches RAM.
 * Lets a host (or a debugger) request the bootloader by setting the magic and
 * resetting, which is how this gets tested before the USB stack exists. */
void boot_check(void)
{
    if (_boot_magic == BOOT_MAGIC) {
        _boot_magic = 0;
        enter_bootloader();
    }
}

/* Request the bootloader on the next reset. Kept separate so a USB command
 * handler can call it from anywhere without worrying about running from RAM. */
void request_bootloader(void)
{
    _boot_magic = BOOT_MAGIC;
    SCB_AIRCR = AIRCR_SYSRESETREQ;
    __asm volatile ("dsb");
    for (;;) { }
}
