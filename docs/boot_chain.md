# IcePi Zero boot chain

How the board gets from power-on to application code, and how every stage is
updated over WiFi. This absorbs the old `xip_bios.md` notes plus the
boot-manager / FTDI / flash.py design.

```
power-on / FTDI reset / ctrl_reset
  └─ ECP5 self-config from flash @0x000000          (power-on only)
       └─ XIP BIOS @0x100000 (executes in place from flash)
            └─ flashboot: winc_loader .fbi @0x200000 -> SDRAM 0x40000000
                 ├─ stay requested?  -> WiFi loader mode (flash.py protocol)
                 └─ otherwise        -> chain-boot app .fbi @0x280000
```

## Flash layout

| Offset | Contents | Updated by |
| --- | --- | --- |
| `0x000000` | bitstream (ECP5 self-config) | `./flash.py --bitstream` (+ power-cycle) |
| `0x100000` | BIOS, XIP (`--bios-flash-offset`; reset vector → `0x20100000`) | `./flash.py --bios` |
| `0x200000` | winc_loader `.fbi` (`FLASH_BOOT_ADDRESS` — boots first on every reset) | `./flash.py --loader` |
| `0x280000` | application `.fbi` (`FLASH_APP_OFFSET`) | `./flash.py --app FILE` |

`.fbi` = LiteX flashboot image: u32le length + u32le crc32 + payload. The
slot offsets are single-sourced in `icepi_zero_base.py`
(`bios_flash_offset`, `flash_boot_offset`, `app_flash_offset`) and mirrored
in `flash.py`'s `SLOTS` table. Tops without the boot-manager (mnist_lcd)
still point `FLASH_BOOT` directly at their firmware.

## Stage 1 — bitstream

The ECP5 self-configures from flash offset 0 at power-on. A soft reset
(`ctrl_reset`, FTDI reset, WFLR) restarts the SoC but does NOT reload the
fabric — new bitstreams need a power-cycle (a host-driven JTAG
`LSC_REFRESH` via openFPGALoader is a possible future replacement). Quad
(4x) reads everywhere rely on the W25Q128's QE bit, which is non-volatile
and already set; `--spiflash-1x` builds a single-lane variant as the
no-QE-dependency recovery fallback.

## Stage 2 — XIP BIOS

The BIOS executes in place from the flash mmap (`0x20000000` + offset)
instead of an EBR ROM — frees ~16 EBR blocks on the LFE5U-25F, which the
SNN builds need. `with_spi_flash=False` builds keep the integrated EBR ROM
(self-contained `--load`, golden recovery image).

The non-obvious constraints, all encoded in `icepi_zero_base.py`:

- **Flash base `0x20000000`, must stay < `0x80000000`.** VexRiscv treats
  `0x80000000+` as uncached IO; XIP instruction fetch needs the region
  cached.
- **Force `integrated_rom_size`/`cpu_reset_address`, don't `setdefault`.**
  The LiteX parser always supplies both via `soc_argdict`; `setdefault`
  would leave a stale EBR ROM at `0x0` colliding with the XIP `rom` region.
- **The XIP BIOS must never drive the SPI master.** `spiflash_init()`'s
  master block (read-ID + quad-enable) knocks the flash out of the
  continuous read the CPU is fetching through → instant crash. Plain builds
  use `with_master=False`; `flash_master=True` builds (winc/all tops) guard
  the block out with the `SPIFLASH_SKIP_MASTER_INIT` constant — a **local
  litex patch**, mirrored at `patches/litex-spiflash-skip-master-init.patch`;
  re-apply after a litex update. Master ops are safe only from code running
  entirely in SDRAM while nothing touches the mmap (the loader's situation).
- **`SPIFLASH_SKIP_FREQ_INIT`.** The BIOS SCLK auto-calibration cranks the
  divisor while fetching its own code — same crash. The build-time divisor
  is the live speed instead: default **25 MHz** (sys/2, the 1:1 PHY ceiling;
  hardware-validated 2026-06-03 by a runtime divisor sweep from the
  SDRAM-resident loader). `--spiflash-clk-freq` overrides. Measured at
  25 MHz quad: ~8 MiB/s sequential, ~2.8 MiB/s random (40 SCLK cycles of
  command/address/dummy setup per discontinuous access). Faster needs
  LiteSPI's DDR `rate="2:1"` PHY.
- **CSR map shifts when peripherals change.** Bitstream, BIOS, loader and
  app must be rebuilt and flashed together after gateware changes; stale
  images poke wrong CSR addresses.
- **`--load` must come after flash steps** in JTAG flows: `--load` releases
  the CPU, which immediately XIP-fetches the BIOS; flashing afterwards means
  the CPU booted stale flash. `run_build` orders this correctly.

## Stage 3 — boot-manager loader (`software/winc_loader/`)

The BIOS flash-boots the loader on every reset; the loader decides whether
to stay in WiFi-loader mode or hand off to the app. Triage order:

| # | Check | Set by |
| --- | --- | --- |
| 1 | sticky `boot_ctl` flag == `LOADER_BOOT_MAGIC` | a running app's `loader_hook` (UDP `"WFLE"` :5558), cable-free |
| 2 | FTDI "stay" level (DTR# asserted, RTS# deasserted) | `flash.py` around an FTDI reset |
| 3 | `'l'`/`'L'` within a 500 ms UART window | a human holding `l` while resetting; `flash.py` spam fallback |
| 4 | otherwise: chain-boot the app | — |

- The `boot_ctl` flag is a `reset_less` CSR: survives `ctrl_reset`/FTDI
  reset, reads 0 after power-on/reconfigure (ECP5 FFs initialize from the
  bitstream). The loader clears it on consumption.
- **Chain-boot**: validate the app .fbi header, CRC the payload in place
  via the mmap, then copy to `0x40000000` and jump. Both loader and app
  execute at `0x40000000`, so the final copy cannot run from main_ram — it
  runs from `chain_stub` (`chain_stub.S`), ~10 instructions placed in
  `.data` → SRAM `0x10000000` (where the stack also lives), ending with the
  VexRiscv cache flushes (`0x500F` dcache, `0x100F` fence.i). The caller
  mirrors the BIOS `boot()` sequence (irq off, caches flushed, uart drained).
- **Brick safety**: an absent/invalid/corrupt app image fails the CRC and
  the loader stays up — a botched app flash can never lose the board.
  `'b'` on the console reboots out of loader mode into the app.
- In loader mode: joins WiFi (`wifi_secrets.h`), announces `icepi.local`
  (mDNS), serves the WFL protocol on UDP :5557. Image buffer in SDRAM at
  `0x40800000`, chunk bitmap below it; chunks are position-indexed so UDP
  loss/reorder/duplication is harmless. `WFLP` CRCs the SDRAM image
  *before* erasing anything, erases (64 KB blocks + 4 KB tail sectors),
  page-programs via the LiteSPI master, then CRC-verifies through the mmap.
  Erase/program runs with the radio unserviced — the WINC's on-module stack
  keeps the link; the host retries against cached, idempotent replies.

### WFL protocol (UDP :5557, little-endian)

| Pkt | Payload | Reply |
| --- | --- | --- |
| `WFLB` begin | u32 offset, u32 length, u32 crc32, u16 chunk | `WFLA` status + flash size (identical re-begin = resume) |
| `WFLD` data | u32 chunk_idx, ≤1408 B payload | none (bitmap, idempotent) |
| `WFLS` stat | — | `WFLT` got/total + missing indices (≤300) |
| `WFLP` program | — | `WFLZ` status, ms, flash crc (cached, re-sent) |
| `WFLX` exec | — | `WFLX` status; CRC the staged image, then chain-boot it **from SDRAM** — nothing flashed, no reset |
| `WFLR` reboot | — | `WFLR` ack, then `ctrl_reset` |

`WFLX` (host: `./flash.py --run FILE`) is the develop-fast path: the image is
staged exactly like a flash session (raw binary, no `.fbi` — length/CRC travel
in `WFLB`), but instead of erase/program the loader acks, drains the WINC, and
jumps through the same `chain_stub` exit as a flash chain-boot, with `IMG_BUF`
as the source. Capped at 8 MB (the app must end below `IMG_BUF` at
`0x40800000` or the copy would eat its own source). The flashed app slot is
untouched: any reset boots back through the loader into the *flashed* app, so
a RAM-run image is gone on reset by design — `--run` a broken build and the
board recovers with the reset button. After the ack drains, the loader parks
the WINC in true power-down (CHIP_EN + RESET_N low; both reach the chip via
the 74HC595 expander, Qc/Qh) — the same state every SoC reset produces, since
both sidebands are plain GPIOOuts that reset to 0 and the expander driver
re-shifts them out right after reset (`gateware/sr595.py`). The app sees the module exactly as after a
cold boot; WiFi apps power it back up in their own `nm_bsp_reset()`.

## FTDI sidebands (host-driven reset)

The FT231X's modem-control outputs are wired to plain FPGA IO: DTR# → L15,
RTS# → L16 (active low; `ser.dtr = True` drives the pin LOW). Because the
OS asserts BOTH lines on every port open, single-line or level triggers are
unusable — the gateware (`BootCtl` in `gateware/soc_features.py`) uses
esptool-style cross-conditions:

| DTR# | RTS# | Meaning |
| --- | --- | --- |
| asserted | asserted | idle (port-open default) — nothing |
| deasserted | deasserted | idle (port closed) — nothing |
| deasserted | asserted (≥50 ms) | **SoC reset pulse** (one-shot until released) |
| asserted | deasserted | **"stay in loader"** level (sampled at boot triage via `ftdi_sense`) |

The 50 ms filter rejects the brief mixed states of a non-atomic port
open/close; the detector state is `reset_less` so the reset it causes can't
retrigger or truncate it. The pulse drives `crg.user_rst` into the same
`pll.reset` path as `ctrl_reset`. Caveat: any tool holding RTS-without-DTR
for >50 ms is now a reset button; `litex_term`/normal terminals (both
asserted) are safe. `ftdi_sense`: bit0 = dtr_n, bit1 = rts_n (wiring
verification and the triage level).

## Host tooling — `./flash.py`

Slot presets (`--bitstream/--bios/--loader` default to the standard build
artifacts; `--app FILE` explicit), combinable in one session; legacy
`FILE --offset X [--fbi]` form; `--run FILE` = SDRAM-load + execute without
flashing (combinable with slots — the exec runs last and replaces the final
reboot); `--reset` = plain FTDI reboot to app. Entry ladder:

1. loader already answering :5557 → go
2. app `loader_hook` on :5558 (`WFLE`) → reboots itself into the loader,
   cable-free
3. `--port` given → FTDI reset + stay level (button-free); falls back to
   `'l'` spam + a "press reset" prompt on bitstreams without `BootCtl`

Apply rules: app/loader/BIOS changes take effect via the automatic `WFLR`
(a soft reset re-XIPs the BIOS and re-chains); a flashed **bitstream
suppresses the soft reset** — resetting there would run the old fabric
against new flash contents (CSR mismatch) — power-cycle once instead.

**Zero-JTAG full update**: from a running loader, flash bitstream + BIOS +
loader + app in one session, then one power-cycle. Safe because nothing
reads flash while the loader runs; every image is CRC-checked in SDRAM
before its slot is erased. The only JTAG-recovery scenario left is power
loss between erasing and rewriting the bitstream slot (or a new bitstream
that doesn't come up at all — test risky gateware via JTAG SRAM `--load`
first if worried).

## Recovery matrix

| Broken | Recovery |
| --- | --- |
| app image | automatic: loader catches the CRC, stays up; reflash `--app` |
| loader image | BIOS flashboot fails → falls through to serialboot: `litex_term --kernel winc_loader.bin` |
| BIOS / bitstream | JTAG (`--flash-bios` / `--flash`); EBR-ROM build (`with_spi_flash=False`) as golden image |
| wrong/mixed CSR maps | reflash all four slots together (one `flash.py` session) |
