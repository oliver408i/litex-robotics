/* Chain-boot copy stub.
 *
 * The BIOS flash-boots THIS loader to MAIN_RAM_BASE (0x40000000) -- the same
 * address every app links at. So the final "copy app over main_ram and jump"
 * cannot run from main_ram. This stub is placed in .data, which the linker
 * puts in SRAM (0x10000000, see linker.ld) along with the stack: it survives
 * its own copy clobbering all of the loader's .text/.rodata.
 *
 * Caller contract (try_chain_boot in main.c): interrupts disabled, caches
 * flushed, a0..a3 set per the C prototype:
 *   void chain_stub(const void *src, void *dst, uint32_t len_bytes, uint32_t entry)
 * len is rounded up to a multiple of 4 by the caller and is > 0.
 */
	.section .data.chain_stub, "aw", @progbits
	.align 4
	.global chain_stub
chain_stub:
1:	lw   t0, 0(a0)
	sw   t0, 0(a1)
	addi a0, a0, 4
	addi a1, a1, 4
	addi a2, a2, -4
	bgtz a2, 1b
	/* VexRiscv cache flushes (opcodes from the generated system.h):
	 * dcache may hold stale lines for the just-overwritten region, icache
	 * holds the loader's old code. 0x500F = dcache flush, 0x100F = fence.i. */
	.word 0x500F
	.word 0x100F
	jr   a3
