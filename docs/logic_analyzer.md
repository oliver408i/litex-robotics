# IcePi Zero Logic Analyzer

A 3.3 V logic analyzer variant: an SDRAM-streaming capture core
(`gateware/logic_analyzer.py`) on the GPIO-bank pins that free up when the
LCD/touch module is unplugged, alongside the mandatory WiFi-OTA baseline so the
board still flashes over WiFi. Top: `icepi_zero_la.py`.

This is a **first cut** — the gateware constructs/finalizes but has not been
simulated or hardware-verified. See "Status & next steps" at the end.

---

## 1. Channels (18)

The probe bus is the set of GPIO-bank pins that are electrically independent of
the aux SPI bus (WINC/IMU/MCP), so capturing can never disturb WiFi flashing.
Ten of them belong to the LCD/touch group and are only safe **with the LCD
module physically unplugged** (see the guard in §3 and
`docs/icepi_zero_pin_mapping.md`).

| ch | Board IO | FPGA | Normal function (when LCD present) |
| -- | -------- | ---- | ---------------------------------- |
| 0  | IO1  | K3 | RGB LED / NeoPixel data (on-board load — see note) |
| 1  | IO4  | R1 | free |
| 2  | IO5  | E1 | free |
| 3  | IO7  | G1 | LCD_RS |
| 4  | IO9  | J1 | free |
| 5  | IO10 | L2 | LCD_RST + CTP_RST |
| 6  | IO11 | G2 | free |
| 7  | IO12 | J3 | free |
| 8  | IO13 | E3 | LCD_MISO |
| 9  | IO14 | P1 | LCD backlight |
| 10 | IO15 | N1 | **CTP_SCL** (guard line) |
| 11 | IO16 | H3 | LCD_CS |
| 12 | IO17 | R3 | free |
| 13 | IO18 | N4 | **CTP_SDA** (guard line) |
| 14 | IO19 | E4 | LCD_SCK |
| 15 | IO21 | F2 | CTP_INT |
| 16 | IO26 | D4 | LCD_MOSI |
| 17 | IO27 | P3 | free |

All channels carry an internal **pull-down**, so unconnected probes idle low.
`ch0`/IO1 is the on-board NeoPixel data line; reading it works but the NeoPixel
is a load on that net, so treat it as the least-clean channel.

**Off-limits (not probe channels):** the aux SPI bus + WINC sidebands
(IO2/IO8/IO25/IO23/IO20/IO22/IO24) stay live for flashing, and IO6/IO3 (IMU/MCP
chip-selects) stay parked-deasserted so those chips don't fight the bus.

---

## 2. Capture core (`gateware/logic_analyzer.py`)

```
probe pins --2FF sync--> decimator --> elastic FIFO --> LiteDRAMDMAWriter --> SDRAM ring
```

- **2FF sync** (`MultiReg`): the probes are async; synchronizing avoids
  metastability. Fixed 2-cycle latency, uniform across channels.
- **Decimator**: sample every `(sample_div+1)` sys clocks → rate =
  `50 MHz / (sample_div+1)`. `sample_div=0` is full rate (50 MS/s).
- **Elastic FIFO** (512 deep, BRAM): absorbs SDRAM refresh/arbitration gaps. If
  a sample arrives while it's full, `overrun` latches — the rate outran DRAM.
- **DMA writer**: one 32-bit word per sample (18 channels zero-extended),
  written to a ring `[ring_base, ring_base + 4*ring_size)` in SDRAM. The port is
  a dedicated native SDRAM write port (like the SNN's read port), so it rides
  DRAM bandwidth independently of the CPU.

### Capture model

Benchtop-LA style: **sample fast into the deep SDRAM ring, offload slowly over
WiFi.** The ring continuously keeps the last `ring_size` samples. On trigger it
captures `post_trig` more samples, then stops — so the buffer holds a pre/post
window around the trigger event.

### Bandwidth & depth

- **SDRAM write side** (capture): half-rate SDRAM is ~100 MB/s raw. One sample =
  4 bytes, so the FIFO sustains roughly **~20 MS/s continuous** before
  `overrun`; short bursts up to full 50 MS/s fit within the 512-sample FIFO.
- **Capture depth**: reserve, say, 16 MB of the 32 MB SDRAM for the ring →
  `ring_size = 4 Mi samples`. At 20 MS/s that's ~200 ms; at 1 MS/s, ~4 s.
- **Offload side** (WiFi): WINC TCP delivers on the order of **~1 MB/s** (≈10×
  the 1 Mbaud UART — measure yours with `software/winc_test/winc_throughput.py`).
  So a full 16 MB ring takes ~16 s to download. Offload is the bottleneck, which
  is why we capture to deep memory first and drain afterward.

### CSR map (instance `la`, accessors `la_*`)

| CSR | dir | meaning |
| --- | --- | --- |
| `arm`        | W-pulse | reset state + start a capture |
| `abort`      | W-pulse | stop now (→ done) |
| `sample_div` | W | sample period = `sample_div+1` sys clocks |
| `trig_mask`  | W | care-mask; only set channels matter. `0` ⇒ trigger immediately |
| `trig_value` | W | match value under the mask |
| `trig_edge`  | W | `1` = fire on transition into match (one-shot); `0` = level |
| `ring_base`  | W | ring base, **byte** offset into SDRAM from `MAIN_RAM_BASE` (4-byte aligned) |
| `ring_size`  | W | ring length in **samples** (32-bit words) |
| `post_trig`  | W | samples to capture after the trigger fires |
| `status`     | R | fields: `running triggered done overrun wrapped lcd_present` |
| `wr_count`   | R | total samples written this run |
| `wptr`       | R | current/final write pointer (sample index within ring) |
| `trig_addr`  | R | sample index within ring where the trigger fired |
| `probe_live` | R | live synchronized probe value (debug + guard read) |

### Readback ordering (host)

After `done`:
- If `wrapped == 0`: valid samples are ring slots `0 .. wptr-1`, in order. The
  trigger sample is at `trig_addr`.
- If `wrapped == 1`: the whole ring is valid; the **oldest** sample is at slot
  `wptr` (the next-write position) and the newest at `wptr-1`. Read
  `size` slots starting at `wptr`, wrapping, to get chronological order. The
  trigger sample sits at `(trig_addr - wptr) mod size` in that linearized stream.

---

## 3. LCD-present guard ("refuse if the module is still attached")

Driving 18 probe pins — ten of which are wired to a powered LCD/touch module —
risks contention and garbage captures. So the build refuses to capture unless
the module is unplugged.

**Mechanism:** the FT6336U touch I²C lines, `ch10`/IO15 (CTP_SCL) and
`ch13`/IO18 (CTP_SDA), carry pull-up resistors **on the touch module**. Every
probe pin has an internal pull-**down**. So:

- Module unplugged → only the internal pull-down acts → both lines read **0**.
- Module attached → the on-module pull-ups win → both lines read **1**.

The core computes `status.lcd_present = (probe_sync & 0x2400) != 0`
(`guard_mask = (1<<10)|(1<<13)`). Firmware reads it at boot **before arming**;
if set, it lights the red status LED, logs the condition over UART + WiFi, and
refuses to arm. (`probe_live` exposes the raw bits for diagnostics.)

> Assumption: the touch pull-ups are on the **module**, not the carrier board.
> This is the usual case for a wired module, but confirm with a continuity check
> on first bring-up — if they're on the carrier, the guard would false-trip.

---

## 4. Data path to the consumer app

```
probe -> capture core -> SDRAM ring        (gateware, fast)
SDRAM ring -> CPU -> WINC TCP -> host app   (firmware, ~1 MB/s)
host app -> VCD / sigrok / CSV              (la_host.py)
```

Firmware mediates: the host never touches CSRs directly, it speaks a tiny TCP
control protocol and the firmware pokes the `la_*` CSRs and reads SDRAM. The
firmware reuses the WINC networking scaffold from `software/winc_test`
(DHCP + mDNS `icepi.local` + a TCP server); only the per-command handlers are
new.

### TCP control protocol (port 5559)

4-byte command tag + little-endian payload. The board replies per command.

| Cmd | Payload (host→board) | Reply (board→host) |
| --- | --- | --- |
| `LNFO` | — | `"LA01", u32 n_channels, u32 ring_capacity_samples, u32 sys_clk_hz, u8 lcd_present` (17 B) |
| `LCFG` | `u32 sample_div, u32 trig_mask, u32 trig_value, u8 trig_edge, u32 ring_size, u32 post_trig` | `"LOK\0"` |
| `LARM` | — | `"LOK\0"` (or `"LBSY"` if `lcd_present`) |
| `LSTA` | — | `u8 flags, u32 wr_count, u32 wptr, u32 trig_addr` (flags bit0..5 = status fields) |
| `LDMP` | `u32 start_sample, u32 n_samples` | raw `n_samples*4` bytes (ring slots, host reorders) |
| `LABT` | — | `"LOK\0"` |

The board **owns** the ring buffer, so the host does not send a base address —
it queries capacity via `LNFO` and the firmware clamps `ring_size`. `flags`
packs the `status` fields (bit0 running … bit5 lcd_present). The host polls
`LSTA` until `done`, then pulls the ring with `LDMP` reads and reorders per §2.

### Firmware (`software/logic_analyzer/main.c`)

Forked from `software/winc_test` (DHCP + mDNS + loader_hook scaffold); the LA
TCP server replaces the echo/sink servers. Key points as built:

- **Ring placement:** `.bss`/stack live in the 8 KiB on-chip `sram`, so the ring
  cannot be a C array. It's a fixed high SDRAM offset instead —
  `LA_RING_OFFSET = 0x800000` (8 MiB in), `LA_RING_SAMPLES = 4 Mi` (16 MiB),
  which clears the ~30 KB of code at the bottom of `main_ram` and fits the
  32 MiB part. `la_ring_base_write(LA_RING_OFFSET)` (the core takes a byte
  offset from `MAIN_RAM_BASE`); the CPU reads at `MAIN_RAM_BASE + LA_RING_OFFSET`.
- **Boot guard:** `main()` logs `lcd_present` at startup; `LARM` returns `LBSY`
  (refuses to arm) while the guard bit is set.
- **L2 flush:** the DMA writes straight to SDRAM, bypassing the CPU caches. On
  `LDMP` start the firmware calls `flush_cpu_dcache()` + `flush_l2_cache()` so
  the bytes it ships are the captured data, not stale cache lines.
- **Streaming dump:** `LDMP` sends the ring in 1400 B (350-sample) segments,
  one per `SOCKET_MSG_SEND` completion, straight out of SDRAM (no staging copy).

Build: `make -C software/logic_analyzer` → `la_firmware.bin`.

### Host consumer app (`software/logic_analyzer/la_host.py`)

Connects to `icepi.local:5559`, sends `LCFG`+`LARM`, polls `LSTA` to completion,
downloads via `LDMP`, reorders, and exports. A runnable skeleton is committed;
flesh out the firmware to talk to it. VCD export lets you open captures in
GTKWave / PulseView; `--csv` for quick inspection.

---

## 5. Build / flash / package

```bash
# build (note the half-rate SDRAM build flags from the SDRAM memory)
.venv/bin/python icepi_zero_la.py --build --flash-master \
    --yosys-abc9 --nextpnr-seed 2
grep -i Fmax build/icepi_zero/gateware/*.rpt    # nextpnr --timing-allow-fail: always check

# snapshot the matched variant right after building (variants share build/icepi_zero)
./syspkg.py pack la --app software/logic_analyzer/firmware.bin

# flash over WiFi
./flash.py --syspkg dist/la.syspkg
```

The LA is a deployable, so it composes `add_flashing_baseline` (WINC + boot
manager) — flashing works exactly like the other WINC variants.

---

## 6. Status & next steps

- [x] Capture core, SoC integration, top — construct + finalize cleanly.
- [x] **Migen-sim bench** `sim/test_logic_analyzer.py` (run:
      `.venv/bin/python sim/test_logic_analyzer.py`). Uses migen's simulator +
      a behavioral DRAM write-port model (like litedram/test/test_dma.py), NOT
      the iverilog/cocotb flow. Verifies: decimation + immediate trigger + exact
      ring contents (no wrap), pattern/edge trigger placement with ring wrap,
      overrun latch at full rate, and the LCD-present guard. All pass.
- [x] Firmware (`software/logic_analyzer/`): forked `winc_test`; protocol
      handlers + L2 flush on `LDMP` + boot guard + streaming dump. **Compiles
      and links** to `la_firmware.bin` (~30 KB). Not yet hardware-run.
- [x] Host (`la_host.py`): `LNFO`/`LCFG`/`LARM`/`LSTA`/`LDMP`/`LABT` client,
      ring reorder, VCD + CSV export. Reorder/export self-tested; protocol
      matches the firmware.
- [ ] Confirm the SDRAM write port `data_width=32` arbitrates cleanly against
      the CPU/L2 port under sustained writes; tune `fifo_depth` / FIFO sizing.
- [ ] Hardware bring-up: continuity-check the touch pull-ups (guard assumption),
      then verify capture against a known stimulus and a real `la_host.py`
      download (VCD opened in PulseView/GTKWave).
