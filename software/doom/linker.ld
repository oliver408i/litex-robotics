/* DOOM linker script for the IcePi Zero SoC.
 *
 * UNLIKE the default LiteX app layout (which parks .data/.bss/heap/stack in the
 * 8 KB on-chip sram), DOOM puts EVERYTHING mutable in the 32 MB SDRAM: its .bss
 * alone (screens, colormaps, zone bookkeeping) blows past 8 KB, and it wants a
 * multi-MB heap and a deep (BSP-recursion) stack. So:
 *   .text/.rodata/.data/.bss -> main_ram
 *   heap  -> from _end up to (top - 1 MB)
 *   stack -> top 1 MB of main_ram (grows down)
 * The 8 KB sram is left free (candidate fast-scratch for a future accelerator). */
INCLUDE generated/output_format.ld
ENTRY(_start)

__DYNAMIC = 0;

INCLUDE generated/regions.ld

/* 1 MB stack reserved at the very top of SDRAM. */
STACK_SIZE = 0x100000;

SECTIONS
{
	.text :
	{
		_ftext = .;
		*crt0*(.text)
		KEEP(*crt0*(.text))
		KEEP(*(.text.isr))
		*(.text .stub .text.* .gnu.linkonce.t.*)
		_etext = .;
	} > main_ram

	.rodata :
	{
		. = ALIGN(8);
		_frodata = .;
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		*(.rodata1)
		*(.got .got.*)
		*(.toc .toc.*)
		. = ALIGN(8);
		_erodata = .;
	} > main_ram

	.data :
	{
		. = ALIGN(8);
		_fdata = .;
		*(.data .data.* .gnu.linkonce.d.*)
		*(.data1)
		_gp = ALIGN(16);
		*(.sdata .sdata.* .gnu.linkonce.s.*)
		. = ALIGN(8);
		_edata = .;
	} > main_ram

	.bss :
	{
		. = ALIGN(8);
		_fbss = .;
		*(.dynsbss)
		*(.sbss .sbss.* .gnu.linkonce.sb.*)
		*(.scommon)
		*(.dynbss)
		*(.bss .bss.* .gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(8);
		_ebss = .;
		_end = .;
	} > main_ram

	/* Heap: everything between end-of-bss and the reserved stack region.
	 * picolibc's sbrk (picosbrk.c) is bounded by __heap_start/__heap_end. */
	. = ALIGN(16);
	__heap_start = .;
	__heap_end = ORIGIN(main_ram) + LENGTH(main_ram) - STACK_SIZE;
}

PROVIDE(_fstack = ORIGIN(main_ram) + LENGTH(main_ram) - 8);

/* .data has identical load/run address (both in main_ram), so crt0's copy is a
 * no-op, but keep the symbols it references defined. */
PROVIDE(_fdata_rom = LOADADDR(.data));
PROVIDE(_edata_rom = LOADADDR(.data) + SIZEOF(.data));
