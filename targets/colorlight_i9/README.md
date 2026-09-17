# Colorlight i9 SoC targets

Thin `BaseSoC` subclasses, same shape as `targets/icepi_zero/`. Run them from
the repo root:

```bash
.venv/bin/python targets/colorlight_i9/bringup.py --build --load
```

`base.py` is library-only — `BaseSoC`, the CRG, the shared argparse/build
helpers, and the `sys.path` bootstrap for the pinned `litex-setup/` checkouts.

Everything builds into **`build/colorlight_i5/`** (the platform's name, not the
board's), so building one top clobbers the previous one's matched
bitstream/BIOS/csr.csv.

**The probe is the only link to this board.** No on-board FTDI, and
openFPGALoader's CH347 backend is a proven dead end — so `--load` goes through
`probe/icelink-fast/` via `tools/icelink.py`, and `--flash` deliberately raises
rather than silently doing nothing. The console is the probe's second CDC.
Script anything that drives it with `tools/socboot.py`, not `litex_term`, which
needs a tty.

| Top | What it is | Status |
|---|---|---|
| `bringup.py` | CPU + SDRAM + flash + UART, nothing else | **Deployable.** Build this when changing the clock or the SDRAM PHY |
| `psc18sr.py` | `bringup.py` + PSC18SR as a **non-boot side block** | Works on hardware — all firmware checks pass. See `docs/psc18sr_prototype.md` |
| `eth.py` | Ethernet PHY bring-up (MDIO only); `--with-ethernet` adds the MAC, `--with-etherbone` a Wishbone master over UDP | **Datapath verified in PHY loopback** (12/12 pings, no magjacks). See below |
| `lcd.py` | ST7796S LCD engine + FT6336U touch, on P4 | **Builds; untested on hardware.** All 12 pins place, `cd_spi` passes at 209 MHz; `sys` fails STA by 4% and ships anyway (see Timing) |

## Ethernet PHY — what is known, measured 2026-09-12

The board has two RGMII PHYs and **no RJ45 magjacks fitted** (they are on
order), so nothing can link yet. MDIO needs no magnetics, though, so the PHYs
themselves were brought up and identified with `eth.py`:

| | |
|---|---|
| MDIO addresses responding | **0 and 1** (2+ read back `0xffff`, i.e. nobody home) |
| `PHYID1`/`PHYID2` (reg 2/3) | `0x0362` / `0x5E62` — model `0x26`, rev `2` |
| `BMCR` (reg 0) | `0x1140` — autoneg on, 1000 Mbps, full duplex |
| `BMSR` (reg 1) | `0x7949` — autoneg-capable; **link bit 2 = 0**, as expected with no jacks |
| MDIO writes | verified: `BMCR` ← `0x4140` (internal loopback) read back exactly, then restored |

That confirms the whole management path — `rst_n` (P4), `mdio` (P5), `mdc` (N5)
— and that both PHYs are powered and out of reset. The ID points to the
Broadcom part these boards are documented to carry (B50612D); confirm against
the silkscreen before relying on any strap-dependent behaviour, because
`tx_delay=0` is exactly such a dependency.

Two structural notes for whoever wires the MAC up:

- **Only one PHY can be instantiated at a time.** `eth 0` and `eth 1` share
  `rst_n`, `mdio` and `mdc`, so requesting both double-drives those pads.
  `--eth-phy` picks which gets the RGMII data pins; MDIO reaches both either
  way, being an addressed bus. The platform's PHY numbering is swapped relative
  to the silkscreen (it follows the PHYA[0] straps).
- **Ethernet costs zero PLL outputs.** `cd_eth_rx` is driven straight from the
  PHY's RX clock pin and `cd_eth_tx` defaults to that same clock. The corollary
  is that **no link means no eth clocks**, so nothing downstream of the PHY can
  be exercised by a passive board — PHY internal loopback (BMCR bit 14, which
  the part accepts) is the way to get clocks before the jacks arrive.

### The datapath works, tested without magjacks

`--with-ethernet` adds the MAC with `dynamic_ip=True`, which is what puts
`ping`, `eth_local_ip` and friends in the BIOS (`cmd_liteeth.c` gates them on
`ETH_DYNAMIC_IP`). That unlocks the cheapest possible end-to-end test on a
board with no jacks: **put the PHY in internal loopback and let the board ping
itself.**

    mdio_write 0 0 0x4140        # BMCR: loopback + forced 1000 + full duplex
    eth_local_ip 192.168.1.50
    ping 192.168.1.50            # -> 32 bytes from 192.168.1.50: icmp_seq=1

Measured 2026-09-12, with a control rather than a single hopeful result:

| | |
|---|---|
| loopback ON (`BMCR=0x4140`) | **12 pings, 12 replies** |
| loopback OFF (`BMCR=0x1140`) | **8 pings, 0 replies** |

The control is the part that makes it evidence: with loopback disabled the
frames have nowhere to go and nothing comes back, so the replies are genuinely
traversing the PHY rather than short-circuiting inside the MAC. A reply also
means the frame passed FCS — LiteEth drops bad-CRC frames silently — so this is
not marginal sampling that happened to survive.

What that covers: MAC TX → RGMII TX pads → PHY → back → RGMII RX pads → MAC RX
→ CPU, plus ARP and ICMP. Which strongly suggests the pin-level RGMII timing,
`tx_delay=0` included, is right. What it does **not** cover: the magnetics,
autonegotiation, a real link partner, and sustained throughput. Those wait for
the jacks.

### Live link, 2026-09-13 — it works

Magjack fitted, cable to a switch. Everything below is a real link, not
loopback.

**The cable was on PHY 1, not PHY 0**, and that was diagnosable without
touching the build: MDIO is a shared bus, so `mdio_read 1 1` answered while the
gateware was still wired to PHY 0. Read `BMSR` (reg 1) **twice** — bit 2 is
latched-low, so the first read returns the stale state and the second the
current one (`0x7969` then `0x796d`). Rebuild with `--eth-phy 1` and the
datapath follows the cable.

It negotiated **gigabit**, which is the case that works — see the 10/100 caveat
below. Evidence, from the PHY rather than from hope: `reg 9 = 0x0200` (we
advertise 1000FD), `reg 10 = 0x3800` (partner 1000FD-capable, both receivers
OK), and Broadcom's aux status `reg 0x19 = 0xff3c`, bits [10:8] = `111` =
1000BASE-T full duplex. `EthRxClockProbe` independently reads 125.1 MHz, which
is the same fact measured at the RGMII pins.

| test | result |
|---|---|
| board → host `ping` | reply, first try |
| host → board `ping`, 10 pkts | **0% loss**, 0.145 ms avg |
| host → board, 100 pkts each at 64 / 512 / 1400 B | **0% loss at every size** |
| RTT vs payload | 0.16 / 0.43 / 0.90 ms — linear in size, so the 48 MHz CPU copying bytes, not the wire (1400 B is 11 µs of gigabit) |

**Etherbone works, which retires the probe as the only host link.**
`--with-etherbone` plus `litex_server --udp --udp-ip <board>`:

    ident            : 'LiteX SoC on Colorlight i9 ...'
    eth_rx clock     : 125.08 MHz   (measured from the host, no probe involved)
    CSR read latency : 346 us  (2889 reads/s)
    ctrl_scratch     : wrote 0xdeadbeef, read 0xdeadbeef -> OK

The write/read round trip is the part that matters: it is a real Wishbone
master in both directions, not a read-only window. Note the Etherbone build
answers ICMP in **gateware** — ping RTT drops from 0.145 ms to 0.092 ms because
no CPU is involved.

The 346 µs per CSR read is `litex_server`'s Python round trip, not the link;
batch reads amortise it.

### Etherbone `buffer_depth` — the one that will bite you

**LiteX's `add_etherbone` defaults to `buffer_depth=16`, and a burst longer
than that does not fail gracefully: it deadlocks the Etherbone core, and the
whole LiteEth stack behind it.** The board then stops answering even ICMP and
needs a bitstream reload. Nothing in the tooling warns you; the host just sees
timeouts, or worse, silently wrong data.

Bisected on hardware 2026-09-13, from a clean reload each time:

| burst (32-bit words) | `buffer_depth=16` (LiteX default) | `buffer_depth=256` |
|---|---|---|
| 1 (single word) | OK | OK |
| 2, 8 | OK | OK |
| 32 | **MISMATCH, board off the network** | OK |
| 64, 128, 255 | — | OK |

`eth.py` therefore builds with **`buffer_depth=256`**, the maximum LiteX
asserts, which covers the 255-word maximum burst the Etherbone protocol itself
allows — so the host cannot construct a burst that overflows it. Do not lower
it.

Two diagnostic notes that cost time here. The CPU stays **completely alive**
when this happens — serial console responsive, `sdram_init` passes memtest —
because only the gateware Ethernet path is wedged; do not read a working
console as a working link. And `litex_server` must be restarted after a board
reload: a stale server returns all-zero reads that look exactly like a bus
routing failure.

### Throughput, measured 2026-09-13

| | |
|---|---|
| Etherbone write | **~11 MB/s** (fire-and-forget, no per-burst ack) |
| Etherbone read | ~64 KB/s (round-trip bound: ~346 µs per burst through `litex_server`) |
| verify at 128 KB | OK |

So a 1.15 MB bitstream uploads to SDRAM in **~0.1 s**. Reads are 170× slower
because each one is a request/response through Python.

**Do not verify by reading back — checksum on the device instead.** The BIOS's
`crc <addr> <len>` computes CRC32 in place and returns eight hex digits, which
turns an 18-second read-back into a 0.4-second command. Measured on the real
1,159,816-byte bitstream:

| verify method | time | bytes over the wire |
|---|---|---|
| Etherbone read-back | ~18 s | 1.16 MB |
| on-device `crc` | **0.40 s** | 8 |

(0.2 s of that 0.40 is console round trip — a 4 KB `crc` also takes 0.20 s — so
the CRC itself costs ~0.2 s for 1.16 MB.)

Two details make it trustworthy rather than merely fast. It is **zlib's
CRC32**, so the host side is a plain `zlib.crc32(data)` with no convention
mismatch — verified byte-identical on hardware (`28635b9f` both sides). And the
handler calls `flush_cpu_dcache_range` + `flush_l2_cache` before computing,
which matters enormously here: **Etherbone writes land behind the CPU's
caches**, so a naive checksum would happily verify stale cache contents.

For verifying flash specifically there is something better still:
`mem_cmp <flash_window> <sdram> <count>` compares the two regions on-device, so
no checksum convention is involved at all.

### The flash ships WRITE-PROTECTED. Read this before anything else.

**The i9's W25Q64 arrives with its whole array locked**: status register 1
reads `0x1C`, i.e. `BP2:BP0 = 111` with `TB=0`, `CMP=0`. Measured 2026-09-13
(JEDEC `ef 40 17`).

The failure mode is vicious out of all proportion to the cause, which is why it
gets top billing here. The chip still accepts `WREN` and the sector-erase
opcode -- it simply declines to modify a protected block. `WIP` therefore never
sets, the BIOS's "wait while busy" loop falls straight through, and
`spiflash_erase_range` then verifies the sector and prints

    Error: location 0x00012a7b not erased (22x)

**once per byte** -- 4096 lines per sector, ~1.16 million lines for a bitstream
-sized region. At 115200 baud that floods the console for the better part of an
hour, burying the reply to every command you run next. What you see is not "the
erase failed"; it is the *next* command hanging, then the one after that, with
a console that looks possessed. Two hours of this session went into it.

Fix, once, non-volatile:

    .venv/bin/python tools/spiflash_protect.py              # read the state
    .venv/bin/python tools/spiflash_protect.py --unprotect  # BP -> 000

`--protect` puts it back. `SRP0=0` on this board, so the /WP pin does not gate
status-register writes; `QE=1` and is preserved (both status bytes are written
in one transaction, because writing only SR1 clears SR2 on many Winbond parts).

Diagnosing this from the console alone is close to impossible -- the evidence
is buried under its own error output. **Read the flash over Etherbone instead**,
which bypasses the CPU and its caches entirely: if `0x00800000` still holds the
old contents after an erase, the erase did not happen, and the status register
is the next thing to look at.

### Flashing the gateware over Ethernet — done, 2026-09-13

Everything needed was already in the SoC and the BIOS; no new gateware. It is
wrapped in one script that verifies before it erases:

    .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &
    tools/i9_flash_bitstream.sh build/colorlight_i5/gateware/colorlight_i5.bit
    .venv/bin/python tools/icelink.py --refresh      # boot it, no power cycle

Verified end to end: flash CRC `28635b9f` identical to the bitstream, then
REFRESH, then the board answered on Ethernet and over Etherbone (reads *and*
writes) running from flash. **`ecppack`'s `.bit` boots from SPI flash as-is** --
no separate framing from the SRAM configuration image, which was the one real
unknown going in.

Supporting tools, all of which stand alone:

| tool | what it does |
|---|---|
| `tools/etherbone_upload.py` | file -> memory over Etherbone, prints CRC32; refuses to write `0x10000000` |
| `tools/etherbone_backup.py` | memory -> file, for backing the flash up first |
| `tools/soc_console.py` | one BIOS command, drains first, returns only that command's reply |
| `tools/spiflash_protect.py` | read/clear/set the flash block-protection bits |
| `tools/i9_flash_bitstream.sh` | the whole cycle, with the safety ordering |
| `tools/icelink.py --refresh` | JTAG REFRESH: reconfigure from flash |

Order matters and the script enforces it: **upload and verify the staged copy
by CRC before anything is erased.** A bad upload then costs nothing, because
the flash is still intact. Only then erase, program, and verify what landed.

Two more things the script handles that cost time to learn:

* **Pace the upload.** At full rate (~12 MB/s) a 1.15 MB write into SDRAM
  intermittently **wedges the Wishbone bus**: the CPU goes silent -- console
  dead, so the next `crc` never answers -- while gateware ICMP keeps replying,
  so the board still pings and looks healthy. Ping proves nothing here. 2 ms
  between bursts costs ~3 s and has been reliable; a bitstream reload clears a
  wedge.
* **Verify by checksum, never by reading back.** `crc` is 0.4 s against ~18 s,
  and eight hex digits instead of 1.16 MB.

**None of this is a brick risk.** If the flashed image is bad the ECP5 just
fails to configure at power-up, and the probe still loads a bitstream into SRAM
over JTAG exactly as before. Flash is the boot path, not the only path -- which
is also why the whole experiment was safe to run.

Stage in SDRAM at **0x40100000**, not 0x40000000, and never touch 0x10000000:
that is the CPU's 8 KB SRAM, where the running BIOS keeps its stack and data.
Writing test patterns there over Etherbone corrupts the live BIOS -- every
console command starts answering "Command not found", which looks like a dead
SoC and is only a reload away from fixed. Learned the obvious way.

`CSR_SPIFLASH_MASTER_CS_ADDR` is present in these builds, which is what gates
the BIOS `flash_write` / `flash_erase_range` commands on top of
`spiflash_write_stream`. This is the `--flash` that `run_build()` refuses to
implement over JTAG — it just arrives by a different road.

**Back up the existing flash first.** The board ships with a factory Colorlight
image and overwriting it is not reversible. It is readable over Etherbone
through the memory-mapped flash window at `0x00800000`.

**It is not a bricking risk**, which is worth stating plainly: if a bad image
is written, the ECP5 simply fails to configure at power-up, and the probe still
SRAM-configures over JTAG exactly as it does today. Flash is the boot path, not
the only path.

### Timing: STA said no, three times, and was wrong three times

| build | `eth_rx` post-route | verdict | reality |
|---|---|---|---|
| PHY only | 323 MHz | PASS at 125 | — |
| PHY + MAC (phy 0) | **105 MHz** | **FAIL** | 12/12 pings in loopback |
| PHY + MAC (phy 1) | **99 MHz** | **FAIL** | 300 packets on a live link, 0% loss |
| PHY + Etherbone | **70 MHz** | **FAIL** | CSR reads and writes from the host |
| LCD + Etherbone, `sys` | **45.9 MHz** | **FAIL at 47.7** | not yet tested on hardware |

That last row is a different kind of failure from the four above it and is
recorded here so nobody re-derives it: it is the **`sys`** domain, not a
source-synchronous RGMII one, and it is caused by Etherbone widening the
Wishbone address decode that sits in front of every slave. Measured both ways
on the same design, 2026-09-16:

| `targets/colorlight_i9/lcd.py` | `sys` Fmax | LUT |
|---|---|---|
| Etherbone on (default) | 45.9 MHz — FAIL at 47.7 | 28% |
| `--no-etherbone` | 62.3 MHz — PASS | 17% |

So Etherbone costs ~16 MHz of reported `sys` headroom, and the critical path
runs `etherbone_record_fifo -> bus adr decode -> spiflash mmap comparator`.
**The build ships at 48 MHz with Etherbone anyway.** nextpnr times the
worst-case corner -- slowest speed grade, 85 C, minimum voltage -- and a 4%
shortfall against that corner is not evidence of anything on a bench at room
temperature. This board's history is the argument: the 50 MHz core has been run
up to 90 MHz without trouble. `--no-etherbone` remains there if a build ever
needs to be judged on its STA number instead.



The Etherbone row is the sharpest: STA says 70 MHz on a domain that is
provably clocking at 125.08 MHz and carrying correct traffic — a **1.79×** gap,
and every frame is FCS-checked, so it is not marginal sampling getting lucky. That is the
same ~70% pessimism measured on `sys` (see the sweep above), and the failing
path is internal MAC logic in `eth_rx`, not the external interface to the PHY.

Do not file that as "fine". It is fine *here, today*, and the flow runs
`--timing-allow-fail`, so nothing will warn you when it stops being fine. If
the MAC is going to be relied on, the failing path is worth actually reading —
and `data_width=32` on `add_ethernet` is the standard lever, since it runs the
MAC's internals at a quarter rate.

Resource cost of the MAC: 61/108 EBR (from 51) and 18% COMB (from 14%).

The PHY-only build closes timing with room: `eth_clocks0_rx` at 323 MHz against
the 125 MHz constraint, `sys` at 58.1 MHz, 14% COMB, 51/108 EBR.

## LCD + touch wiring — P4, as built 2026-09-16

The ST7796S + FT6336U module moved over from the IcePi Zero. Same panel, same
`LCDEngine`, same firmware; only the pin map changes, and that lives in
`gateware/soc_features.py` as `COLORLIGHT_I9_LCD_PINS`. Wiring as physically
connected:

| Wire | Module pin | FPGA ball | Header row | Note |
|---|---|---|---|---|
| brown | VCC | 3V3 | bottom | |
| red | GND | GND | bottom | |
| orange | LCD_CS | E1 | bottom | `lcd_spi.cs_n` |
| yellow | LCD_RST | E4 | bottom | `lcd_ctrl.reset_n` |
| green | LCD_RS (D/C) | F2 | top | `lcd_ctrl.dc` |
| blue | MOSI | F1 | top | |
| purple | SCK | F3 | bottom | |
| grey | LED (backlight) | H3 | bottom | `lcd_ctrl.backlight` |
| white | MISO | H5 | bottom | |
| black | SCL | G3 | top | `ctp_i2c.scl` |
| brown2 | CTP_RST | H4 | top | `lcd_ctrl.ctp_reset_n` |
| red2 | SDA | J4 | top | `ctp_i2c.sda` |
| orange2 | INT | K3 | top | `ctp_int` |
| yellow2 | SD_CS | J5 | bottom | tied high in gateware, see below |

`E1 E4 F3 H3` and `F2 F1 G3 H4` are the two halves of `pmodg`. `J4 K3 H5 J5`
are **not** in litex-boards' pmod tables for this board but are real IO on the
LFE5U-45F CABGA381 — verified by feeding them to nextpnr, not by assumption —
so the map uses raw ball names throughout. None of the fourteen collide with
the SDRAM, the Ethernet PHY, `clk25`, `cpu_reset_n` or the serial port.

Two things differ from the IcePi Zero wiring:

**LCD_RST and CTP_RST are separate pins here** (E4 and H4) where the IcePi ties
them to one (L2). `add_lcd_touch()` drives `ctp_reset_n` from the same
`reset_n` CSR bit, which reproduces the tied behaviour exactly — so
`software/common/lcd.c` needs no change. Splitting them later is a one-line
gateware change plus a new CSR field, if independent touch reset is ever
wanted.

**SD_CS is wired and is held high in gateware.** The module's microSD slot
shares MOSI/SCK/MISO with the panel. Floating, it can latch low, and then the
card drives MISO and corrupts every LCD register read — which would look like a
flaky panel, not a card problem. Nothing here talks to the card, so the pin is
tied deasserted for good.

## Etherbone is on by default — and why it is a feature, not a base

`BaseSoC` stays plain. Ethernet is a composable adder, `add_eth()` in
`gateware/soc_features.py`, the same shape as `add_lcd_touch()` — **not**
something the tops inherit by subclassing `eth.py`, which is a bring-up top
carrying diagnostics (`EthRxClockProbe`, the PHY selector) that no product
build wants.

What it defaults to, per top:

| top | Etherbone default | why |
|---|---|---|
| `lcd.py` | **on** | meant to end up in flash |
| `psc18sr.py` | **on** | meant to end up in flash |
| `bringup.py` | off | its job is to be the minimal substrate and give an STA report you can trust |
| `eth.py` | off | answers "is the PHY alive?" on a board with no magjacks; must not depend on a datapath |

`--no-etherbone` / `--with-etherbone` overrides either way on every top.

**Why default on.** `tools/i9_flash_bitstream.sh` drives a *running* Etherbone
image. Flash a build without one and REFRESH, and the board comes up with no
network path — the way back in is the probe. Not a brick, just an avoidable
trip to the bench.

**What it costs, and it is not fabric.** The `eth_rx`/`eth_tx` domains make
nextpnr report a post-route failure on every build that has them, and that
report has been wrong every time it was checked against hardware (the table
under "Timing" below). The danger is habituation: if every build fails STA,
nobody reads STA, and the *real* timing failure goes out the door. That is what
`--no-etherbone` is for — build with it when the timing number is the thing you
need to trust.

**MAC and Etherbone are mutually exclusive** on one PHY in this LiteX version:
`add_ethernet()` here has no `with_etherbone` parameter. `add_eth()` asserts it,
and `eth.py` catches the pair in `main()` so you get a sentence instead of a
traceback.

## Clock headroom — measured 2026-09-12, not estimated

`nextpnr` reports Fmax at the **-6 worst-case corner** (slowest process, 85 °C,
1.14 V). On a bench board at room temperature the real ceiling is far above it,
so the flow runs `--timing-allow-fail` and the reported number is a floor.
Measured on the `psc18sr.py` build (CPU + SDRAM + flash + the core), each point
a fresh build loaded onto hardware, judged by the BIOS memtest plus the
`psc18sr_host` firmware:

| `--sys-clk-freq` | SDRAM (half-rate = 2×sys) | STA said | Hardware | Memspeed W/R |
|---|---|---|---|---|
| 50 MHz | 100 MHz | 48.0 — FAIL | **pass** | 19.4 / 27.9 MiB/s |
| 60 MHz | 120 MHz | 51.7 — FAIL | **pass** | 23.3 / 33.5 |
| 70 MHz | 140 MHz | 52.8 — FAIL | **pass** | 27.2 / 39.0 |
| 80 MHz | 160 MHz | 55.1 — FAIL | **pass** | 31.0 / 44.9 |
| 87.5 MHz | 175 MHz | 57.7 — FAIL | **pass** | 33.9 / 48.9 |
| 93.75 MHz | 187.5 MHz | 54.9 — FAIL | **pass** | 36.4 / 52.6 |
| 96.875 MHz | 193.75 MHz | 52.6 — FAIL | **Memtest KO** | — |
| 100 MHz | 200 MHz | 57.8 — FAIL | **Memtest KO** | — |
| 100 MHz | 100 MHz (full rate) | 57.5 — FAIL | **Memtest KO** | — |

Three things worth keeping from that:

- **STA is ~70% pessimistic here.** 93.75 MHz runs clean while nextpnr reports
  54.9 MHz. The critical path it names is real, not a false path — VexRiscv's
  cache tag RAM, 5.61 ns clk-to-q out of an EBR — it is just quoted at a corner
  this bench is nowhere near.
- **The ceiling is the sys domain, not the memory.** The last row is the proof:
  100 MHz fails even with the SDRAM dropped to 100 MHz, while 187.5 MHz SDRAM
  was fine at sys = 93.75. So the half-rate PHY is not what caps this design,
  and there is no exactly-achievable PLL step between 93.75 and 96.875 to
  narrow the cliff further.
- **93.75 MHz is a measurement, not an operating point.** It runs the SDRAM at
  187.5 MHz against a part rated 143–166 MHz, on one board, at one temperature,
  today. Margin bought from an unmodelled corner disappears on a hot day or a
  different die. The default stays **48 MHz**; 80 MHz (160 MHz SDRAM, 17 MHz
  below the observed cliff) is the highest point with real margin on both axes.

Only exactly-achievable PLL frequencies were used, so requested == actual:
`VCO = 25 MHz × k` must land in 400–800 MHz, then `sys = VCO / 2d`. This
matters because LiteX derives the UART divisor from the **requested** frequency
— the committed 48 MHz default is really 47.727 MHz (525/11), a 0.6% error that
is harmless, but a badly-chosen target can be far enough off to break the
console.
