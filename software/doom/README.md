# DOOM for icepi_zero

A software-rendered port of **DOOM** to the icepi_zero FPGA SoC (VexRiscv),
based on [doomgeneric](https://github.com/ozkl/doomgeneric) — a minimal DOOM
port that isolates the platform layer behind a small set of `DG_*` callbacks.

## What's in here

Most of `src/` is the upstream doomgeneric / Chocolate Doom source, vendored so
the SoC toolchain can build it directly. The port-specific files are:

- `src/doomgeneric_icepi.c` — the platform layer (framebuffer flush to the LCD,
  input, timing) implementing the doomgeneric `DG_*` callbacks
- `src/w_file_icepi.c` — WAD access backed by flash XIP
- `src/syscalls.c` — bare-metal libc syscall stubs
- `Makefile`, `linker.ld` — build + memory layout for the SoC

## WAD data

The IWAD (`doom1.wad`, shareware) is **not** committed to this repo. It is
flashed to XIP flash separately (`0x20400000`) and read in place by
`w_file_icepi.c`. Drop your own WAD in `wad/` locally to build/flash.

## License

DOOM's source and doomgeneric are licensed under the **GNU GPL v2**; see
[`COPYING`](./COPYING). DOOM is a trademark of id Software LLC. The WAD data
files are not covered by the GPL and are not distributed here.

Upstream:
- doomgeneric: https://github.com/ozkl/doomgeneric
- Chocolate Doom: https://github.com/chocolate-doom/chocolate-doom
