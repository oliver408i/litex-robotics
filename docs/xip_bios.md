# XIP BIOS from SPI flash — notes

Notes on why the SPI-flash path in `icepi_zero_base.py` looks the way it
does. The point of all this: stop baking the BIOS into EBR (frees ~16 EBR blocks
on the LFE5U-25F) by executing it in place from flash instead.

## Two ROM modes

- **`with_spi_flash=True` → XIP.** `integrated_rom_size=0` (no EBR ROM) and the CPU
  reset vector points into the memory-mapped flash region at `bios_flash_offset`.
  The BIOS must be flashed there (`--flash-bios`).
- **`with_spi_flash=False` → integrated EBR ROM** (current/old behaviour). `--load`
  alone gives a self-contained, runnable CPU. This is also the golden recovery
  image if a flashed BIOS goes bad. (`main` branch is all-integrated-ROM.)

## Gotchas (the non-obvious bits)

- **Flash base = `0x20000000`, must stay < `0x80000000`.** VexRiscv reserves
  `0x80000000+` as an *uncached* IO region. XIP instruction fetch needs the flash
  region cached, so it has to live below that. (First attempt at `0x80000000`
  errored: "Region in IO region, it can't be cached".)
- **Force `integrated_rom_size`/`cpu_reset_address`, don't `setdefault`.** The LiteX
  parser always supplies both via `soc_argdict` (rom size default `0x20000`, reset
  default `None`), so `setdefault` is a no-op — it would leave a stale EBR ROM at
  `0x0` that collides with the XIP `rom` linker region.
- **`with_master=False` is required for XIP.** The BIOS runs from this flash, so it
  must not drive raw SPI commands. The `CSR_SPIFLASH_MASTER_CS_ADDR` block in
  `spiflash_init()` reads the ID and re-enables quad mode, which knocks the flash
  out of the continuous read the CPU is fetching through → instant crash. Dropping
  the master also removes unused logic. Quad then relies on the flash's QE bit
  already being set (it is, non-volatile).
- **`SPIFLASH_SKIP_FREQ_INIT` constant.** Without it the BIOS auto-calibrates the
  flash SCLK by cranking the divisor up — while fetching its own code from that
  flash — and a too-fast step corrupts the instruction stream and hangs. We stay at
  the safe default divisor; it's a boot-only path (app runs from SDRAM).
- **CSR map shifts when the master is dropped.** Rebuild *and* reflash the BIOS and
  firmware together; a stale firmware will poke the wrong CSR addresses.

## Flash layout

| Offset | Contents |
| --- | --- |
| `0x000000` | bitstream (ECP5 self-config) |
| `0x100000` | BIOS (`--bios-flash-offset`, reset vector → `0x20100000`) |
| `0x200000` | firmware `.fbi` (`FLASH_BOOT_ADDRESS`) |

`--spiflash-1x` gives the same XIP path in single-lane mode (no QE dependency) as a
cold-boot recovery fallback.
