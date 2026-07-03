/* main_ram-resident link for the C3 loader.
 *
 * icepi_zero_c3loader.py builds the SoC with NO SDRAM (with_sdram=False); main_ram
 * is a 32 KB on-chip BRAM at 0x40000000. Everything -- text, rodata, data, bss,
 * stack -- lives there (it is real, working memory, unlike the dead SDRAM). The
 * BIOS loads us to MAIN_RAM_BASE on serialboot and keeps its own stack at the top
 * of main_ram during the load, exactly as in any normal LiteX boot:
 *   litex_term ... --kernel c3_flash.bin        (default load addr = main_ram)
 */
INCLUDE generated/output_format.ld
ENTRY(_start)

__DYNAMIC = 0;

INCLUDE generated/regions.ld

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
}

PROVIDE(_fstack = ORIGIN(main_ram) + LENGTH(main_ram) - 8);

PROVIDE(_fdata_rom = LOADADDR(.data));
PROVIDE(_edata_rom = LOADADDR(.data) + SIZEOF(.data));
