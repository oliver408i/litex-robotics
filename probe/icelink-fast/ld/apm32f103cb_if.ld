/* APM32F103CB -- DAPLink interface slot.
 * The stock DAPLink bootloader (48K at 0x08000000) is back in place: it is the
 * only bootloader that knows to assert USB_ENUM (PB15), so it is the only one
 * that enumerates on this board. Flash by copying the .bin onto the
 * MAINTENANCE drive it presents.
 * RAM kept at the datasheet-documented 20K; the dump's 36K read-extent is
 * unverified, so do not rely on it yet.
 */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x0800C000, LENGTH = 80K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Top 16 bytes are reserved: _boot_magic must survive SYSRESETREQ and must
 * not collide with the stack or with .data/.bss. */
_boot_magic = ORIGIN(RAM) + LENGTH(RAM) - 16;
_estack     = ORIGIN(RAM) + LENGTH(RAM) - 16;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

  .text : {
    *(.text*) *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.exidx : { *(.ARM.exidx*) } > FLASH

  /* Code that must not execute from flash (it erases flash). */
  _sramfunc_load = LOADADDR(.ramfunc);
  .ramfunc : {
    . = ALIGN(4); _sramfunc = .;
    *(.ramfunc*)
    . = ALIGN(4); _eramfunc = .;
  } > RAM AT > FLASH

  _sidata = LOADADDR(.data);
  .data : {
    . = ALIGN(4); _sdata = .;
    *(.data*)
    . = ALIGN(4); _edata = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) : {
    . = ALIGN(4); _sbss = .;
    *(.bss*) *(COMMON)
    . = ALIGN(4); _ebss = .;
  } > RAM

  /* Fail the build rather than silently overflowing into the bootloader. */
  ASSERT(ORIGIN(FLASH) == 0x0800C000, "interface must link at 0x0800C000")
}
