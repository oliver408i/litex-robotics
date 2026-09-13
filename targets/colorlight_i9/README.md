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
