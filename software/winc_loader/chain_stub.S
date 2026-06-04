/* Chain-boot copy stub: copies the app over the loader's own main_ram and
 * jumps. Lives in .data -> SRAM so it survives the copy (loader and app both
 * execute at 0x40000000 -- docs/boot_chain.md). Caller (try_chain_boot) has
 * irqs off and caches flushed; args per the C prototype:
 *   void chain_stub(const void *src, void *dst, uint32_t len, uint32_t entry)
 * with len > 0 and word-aligned. */
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
	.word 0x500F      /* VexRiscv dcache flush (encoding from system.h) */
	.word 0x100F      /* fence.i */
	jr   a3
