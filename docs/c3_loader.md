# ESP32-C3 SPI flash-loader

Replaces the dead ATWINC1500 WiFi OTA path (the chip was destroyed; see the
`winc-archive` branch / `winc-final` tag for the old stack). The C3 gives a fast,
JTAG-free way to reflash the board for quick iteration. **Status: in progress** —
the FPGA SPI-slave gateware is built and sim-verified; firmware and C3 side are
next.

> **2026-07-03 update: everything below this point describes the original
> SPISlave design and is superseded.** The link stack that actually shipped
> pivoted to LiteX's **SPIBone** (C3 as a Wishbone master) + a plain mailbox
> RAM instead of the custom `SPISlave`/command-protocol design documented
> here — see `gateware/soc_features.py`'s `add_c3_spibone`/`add_c3_mailbox`,
> `icepi_zero_c3flash.py`, `software/c3_flash` (FPGA side) and
> `software/c3_flash_esp` (C3 side, incl. `flash_c3.py`). It's hardware-
> verified end to end: real bitstream/BIOS slots flashed and cold-booted with
> no JTAG, a C3-driven reset line (`'R'` command, FPGA `ext_reset`/G3 <->
> C3 GPIO7), and the loader itself is now flash-resident at the loader slot
> (0x200000, `.fbi`-wrapped) — BIOS auto-boots straight into it on every
> reset/power-cycle, no `litex_term --kernel` step needed. This doc's design
> section is kept for historical context; don't follow its pin/protocol
> tables for new work.
>
> **Boot-manager / chain-boot (added once SDRAM was fixed, [[halfrate-sdram]]):**
> the loader is resident by default (opposite of the old winc_loader, which
> defaulted to chain-booting). A sticky one-shot flag (`gateware/soc_features.py`'s
> `add_boot_flag`/`BootFlag` — the CSR half of `BootCtl`, deliberately with
> none of its FTDI DTR/RTS coupling, so GPIO7 stays the *only* reset path)
> gates `try_chain_boot()` in `software/c3_flash/main.c`: the host sets it via
> a new mailbox opcode (`CMD_BOOT_APP`, `flash.py --boot-app`), the loader
> copies the app slot (0x280000) to `main_ram` and jumps, clearing the flag
> first — so the *next* reset (from the app, GPIO7, or a power-cycle) lands
> back in the resident loader automatically, no separate "return to loader"
> step. Hardware-verified: `--boot-app` chain-booted `software/diag` (the
> generic SoC-agnostic test console) successfully.

## Architecture — hybrid (C3 brain, FPGA thin programmer)

The valuable, validated logic that is *transport-agnostic* (flash erase/program,
the `.fbi` slot map, boot-manager chain-boot) stays; only the WINC-specific radio
is replaced.

- **ESP32-C3 = the brain.** Holds and stages the whole image in its own RAM/flash,
  owns chunking, CRC and retries, and the host link (USB-CDC). It CRCs the full
  image *before* telling the FPGA to touch flash, so brick-safety is preserved
  with **no FPGA-side image buffer** — which means **no SDRAM dependency** (SDRAM
  is currently down; the old loader staged 8 MB in SDRAM and could not run).
- **FPGA = a thin SPI-slave flash-programmer.** A tiny SRAM-resident firmware
  drains the SPI-slave FIFO, parses a small command protocol, and drives the
  existing **LiteSPI master** to erase/program/verify. Reuses
  `software/winc_loader/flash_w25q.c` verbatim. The whole Atmel `host_drv` driver
  is gone, so the firmware is a fraction of the old 35 KB loader.
- **Host tool.** `flash.py` adapted to a USB-serial transport to the C3 (keeps the
  `.fbi` wrapping + slot map). The WFL protocol effectively moves into the
  host↔C3 link.

```
host (PC) --USB-CDC--> ESP32-C3 (brain: stage, CRC, retry) --SPI--> FPGA (SPI slave
                                                                     -> LiteSPI -> NOR flash)
                                                          <--IO4 BUSY/READY--
```

### Why hybrid (not pure gateware)
`flash_w25q.c` already does WREN / page-program / WIP-poll / erase and is
validated; reimplementing it as an RTL FSM is from-scratch and risks the XIP-bus
hazard (nothing may poke flash while the CPU XIP-fetches — see `boot_chain.md`).
Firmware also iterates far faster (recompile vs re-PnR) on a brand-new protocol.
Pure-gateware would buy a CPU-less recovery appliance — a polish goal, not an
iterate-fast one.

## Throughput
USB-CDC on the C3's Full-Speed USB ≈ 0.7–1.0 MB/s. W25Q128 page-program ≈
0.4–0.5 MB/s sustained + erase overhead. **Flash is the bottleneck, not the
link** — the C3 buffers ahead and the IO4 BUSY line throttles to flash speed. A
~560 KB bitstream ≈ ~3 s. (Beating ~0.5 MB/s would need faster flash / DDR
LiteSPI, not a faster link.)

## Wiring — dedicated SPI bus, FPGA is the SLAVE

All pins freed by the WINC removal; fully separate from the aux bus (IMU/MCP on
IO2/IO8/IO25), so no contention.

| Signal | IO | ECP5 ball | Direction | Note |
| --- | --- | --- | --- | --- |
| SCLK | IO27 | P3 | C3 → FPGA | master clock |
| MOSI | IO24 | L1 | C3 → FPGA | (was WINC IRQ) |
| MISO | IO23 | M2 | FPGA → C3 | (was WINC CS) |
| CS#  | IO12 | J3 | C3 → FPGA | active low |
| BUSY/READY | IO4 | R1 | FPGA → C3 | flow-control (C3 "GPIO10") |

SPI **mode 0** (CPOL=0, CPHA=0), MSB-first, 8 bits/byte.

## FPGA SPI-slave gateware — `gateware/spi_slave.py` (`SPISlave`)

The dumb transport half (mirrors `aux_spi.py`'s role on the master side). Lives
entirely in the `sys` domain: SCLK/MOSI/CS# pass through `MultiReg` synchronizers
and SCLK edges are oversampled — valid while **SCLK ≤ ~sys/4 (~12 MHz)**, far
above the flash-bound ceiling, so no extra clock domain. RX and TX byte FIFOs
(512 B) sit between the shift register and firmware.

CSR map (instance name `c3`):

| CSR | Access | Meaning |
| --- | --- | --- |
| `rxtx` | read | pop one byte the master clocked in (RX FIFO) |
| `rxtx` | write | push one byte to clock back out on MISO (TX FIFO) |
| `rxempty` | read | bit0: RX FIFO empty |
| `txfull` | read | bit0: TX FIFO full |
| `ready` | write | drive the BUSY/READY pad (1 = ready to accept; 0 = busy). Resets 0 so the C3 waits until firmware is up |
| `status` | read | bit0: CS# currently asserted |

Verified in `sim/test_spi_slave.py` (run `.venv/bin/python sim/test_spi_slave.py`):
master→RX ordering, TX→MISO, full-duplex in one CS frame, and the READY pad.

## Command protocol (C3 ↔ FPGA) — v1, implemented in `software/c3_loader/main.c`

Firmware drains `rxtx` and interprets this command stream (all multibyte
little-endian). Each command pushes a reply (status [+ payload]) to the TX FIFO;
the C3 reads it after READY goes high again.

| Op | Bytes | Reply (MISO) |
| --- | --- | --- |
| `0x01` PING | — | `0xA5`, jedec[4] |
| `0x02` ERASE | off:u32, len:u32 | status:u8 |
| `0x03` PROGRAM | off:u32, n:u16, data[n] (n ≤ 256) | status:u8 |
| `0x04` CRC | off:u32, len:u32 | status:u8, crc32[4] |
| `0x05` REBOOT | — | — (asserts `ctrl_reset`) |

Status: `0x00` OK, `0x01` bad command, `0x02` bad arg. CRC is computed by the
firmware over the flash mmap (`SPIFLASH_BASE + off`) *after* programming, so it
is a true read-back verify.

BUSY/READY (IO4): the firmware drives READY low (`c3_ready_write(0)`) while an
erase/program/CRC is in flight; the C3 must not clock the next command (or read
the reply) until READY is high again. The whole-image CRC guard lives on the C3
(it has the full image), so the FPGA can stream-program page by page.

Still TODO on the protocol: a `STAY` opcode + boot-triage integration (below).

### Boot "stay in loader" handshake
The old FTDI DTR/RTS sense + `'l'` UART window belonged to the FT231X console
path, not the C3. Replacement: at boot, the loader's triage briefly listens for a
`STAY` command from the C3 over SPI (no extra pin needed); the `boot_ctl` sticky
flag set by an app's `loader_hook` still works unchanged. Chain-boot itself is
reused as-is, but its app copy targets SDRAM (`0x40000000`) so that step is
dormant until SDRAM returns — flashing does not need it.

## Build order / status
1. ✅ FPGA SPI-slave gateware + sim (`gateware/spi_slave.py`, `sim/test_spi_slave.py`)
2. ✅ `add_c3_loader()` in `gateware/soc_features.py` + bring-up top `icepi_zero_c3loader.py`; PnR clean (sys 61 MHz, fits)
3. ✅ FPGA firmware `software/c3_loader/` (SRAM-resident, reuses `flash_w25q.c`); 32 KB `integrated_sram`; builds, 8.5 KB used
4. ◐ C3 firmware `software/c3_bridge/` (PlatformIO/Arduino) — SPI master + READY handshake + protocol mirror + boot PING self-test; **compiles**. Blocked on the C3-side GPIO pin numbers (SCLK/MOSI/MISO/CS); READY=GPIO10 confirmed. Host↔C3 transfer protocol still TODO.
5. ☐ Host tool (adapt `flash.py` to the USB-serial transport)
6. ☐ First hardware bring-up: PING → ERASE/PROGRAM/CRC the bitstream slot

## Devices (this bench)
- `/dev/ttyACM0` — the ESP32-C3 (Espressif USB-JTAG/serial `303a:1001`): `pio run -t upload` + `pio device monitor` (USB-CDC).
- `/dev/ttyUSB0` — the FPGA's FTDI (`0403:6015`): JTAG load + `litex_term` serial-boot.

## Bring-up sequence (MVP: prove the link with PING)
1. JTAG-load the `icepi_zero_c3loader.py` bitstream **and a matching BIOS** (the CSR
   map changed, so a stale flashed BIOS mismatches — flash both). This bitstream has
   **no SDRAM** (`with_sdram=False`): `main_ram` is a 32 KB on-chip BRAM, so the BIOS
   runs from working memory even with the SDRAM chip dead. (A *with-SDRAM* loader
   bitstream bricks serialboot: the BIOS keeps its stack/data in `main_ram`=dead
   SDRAM and garbles the upload with a `\xff` — observed 2026-06-28.)
2. Serial-boot the FPGA loader (default load addr = `main_ram` 0x40000000):
   `litex_term /dev/ttyUSB0 --speed 1000000 --kernel software/c3_loader/c3_loader.bin`
   It prints the flash JEDEC id and "waiting for ESP32-C3 commands".
3. Set the C3 pins in `software/c3_bridge/src/main.cpp`, then `pio run -d software/c3_bridge -t upload`.
4. `pio device monitor` → expect `PING ok -- flash JEDEC 0xEF4018 (W25Q128)` once a second. That validates the whole chain + the READY handshake.

MVP target: host → C3 (USB) → SPI → FPGA programs the bitstream slot @0 +
read-back CRC. Then full slot map, then WiFi transport (same SPI back-end) and
`--run` once SDRAM is back.
