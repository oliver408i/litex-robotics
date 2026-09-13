#!/usr/bin/env python3
"""LiteX wrapper for the PSC18SR prototype core (verilog/psc18sr.v).

PROTOTYPE against a DRAFT ISA -- see docs/psc18sr_isa_draft.md. Unlike
gateware/pisc.py this has no golden model behind it yet, so nothing here is a
contract.

Differences from the v1 wrapper that matter for integration:

- **It is a Wishbone master.** The `BUS` instruction reaches a 64K window at a
  synthesis-time base (default the SoC CSR base). Call `add_psc18sr(soc, ...)`
  rather than instantiating directly, so the master gets registered on the SoC
  bus.
- **imem is 18 bits wide and split.** Words [0, ro_words) come from the
  bitstream (`init_file`, a $readmemh hex) and reject host writes; the rest
  behave like v1's imem.
- **`start_pc`**, so the CPU can run a post-boot program out of the writable
  region without disturbing the stage-0 program at pc=0.
- **`autostart`** runs the core at pc=0 out of reset with no host involvement.
  Leave it OFF for bring-up: an autostarting core that hangs holds whatever
  its output ports hold, and on a real stage-0 build that includes the CPU
  reset line.

Firmware flow (autostart=0), mirroring gateware/snn_mlp.py and v1:

    psc18sr_control_abort_write(1)
    for addr, word in enumerate(prog, start=RO_WORDS):
        psc18sr_imem_addr_write(addr)
        psc18sr_imem_data_write(word)        # 18 bits
        psc18sr_imem_ctl_we_write(1)
    psc18sr_start_pc_write(RO_WORDS)
    psc18sr_control_run_write(1)
    while not (psc18sr_status_read() & 2):   # halted
        pass
    result = psc18sr_result_read()
"""
from __future__ import annotations

import os

from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import AutoCSR, CSRField, CSRStatus, CSRStorage


def _log2_words(n: int) -> int:
    bits = 1
    while (1 << bits) < n:
        bits += 1
    return bits


class PSC18SR(LiteXModule, AutoCSR):
    """PISC v2 sequencer core: CSR-mapped peripheral + Wishbone master."""

    def __init__(self, platform, imem_words: int = 1024, ro_words: int = 256,
                 num_out: int = 4, num_in: int = 4, delay_prescale: int = 1024,
                 autostart: bool = False, bus_base: int = 0xf0000000,
                 wdt_cycles: int = 0, cpu_rst_port: int = 0, cpu_rst_bit: int = 0,
                 init_file: str = ""):
        assert (imem_words & (imem_words - 1)) == 0, "imem_words must be a power of 2"
        assert 0 <= ro_words <= imem_words, "ro_words must fit in imem_words"
        assert bus_base % 0x10000 == 0, "bus_base must be 64K-aligned"

        repo_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
        platform.add_source(os.path.join(repo_root, "verilog", "psc18sr.v"))

        self.imem_words = imem_words
        self.ro_words   = ro_words
        self.num_out    = num_out
        self.num_in     = num_in
        addr_bits = _log2_words(imem_words)

        # Wishbone master for the BUS instruction.
        self.bus = wishbone.Interface()

        # --- CSRs ---------------------------------------------------------
        self.control = CSRStorage(fields=[
            CSRField("run", size=1, pulse=True,
                     description="Pulse to start execution at start_pc."),
            CSRField("abort", size=1, pulse=True,
                     description="Pulse to stop now and go idle. The only way "
                                 "to recover a program stuck in WAIT or on an "
                                 "unacked BUS access."),
        ])
        self.status = CSRStatus(fields=[
            CSRField("running", size=1, description="Core is executing."),
            CSRField("halted",  size=1, description="Core stopped (result valid)."),
            CSRField("aborted", size=1, description="Last stop was an abort, not HLT."),
            CSRField("wdt",     size=1, description="Watchdog fired: the CPU-reset "
                                                    "port bit was force-released."),
        ])
        self.result   = CSRStatus(16, description="r7 latched at HLT (0xFFFF = watchdog).")
        self.start_pc = CSRStorage(addr_bits, reset=ro_words if ro_words < imem_words else 0,
                                   description="Entry PC for the next run pulse.")

        self.imem_ctl = CSRStorage(fields=[
            CSRField("we", size=1, pulse=True,
                     description="Pulse to commit imem_addr -> imem[addr] = imem_data. "
                                 "Ignored while running, and for addr < ro_words."),
        ])
        self.imem_addr = CSRStorage(16)
        self.imem_data = CSRStorage(18)

        self.gpio_out = CSRStatus(16 * num_out,
            description="Packed output ports driven by PORT/BITOP.")
        self.gpio_in = CSRStorage(16 * num_in,
            description="Packed input ports read by PORT/WAIT.")
        self.dbg_pc = CSRStatus(addr_bits, description="Current/next program counter.")

        # --- core signals ---------------------------------------------------
        running, halted, aborted, wdt_fired = Signal(), Signal(), Signal(), Signal()
        result = Signal(16)
        io_out = Signal(16 * num_out)
        dbg_pc = Signal(addr_bits)
        retire = Signal()

        self.specials += Instance(
            "psc18sr",
            p_IMEM_WORDS     = imem_words,
            p_RO_WORDS       = ro_words,
            p_NUM_OUT        = num_out,
            p_NUM_IN         = num_in,
            p_DELAY_PRESCALE = delay_prescale,
            p_AUTOSTART      = 1 if autostart else 0,
            p_BUS_BASE       = bus_base,
            p_WDT_CYCLES     = wdt_cycles,
            p_CPU_RST_PORT   = cpu_rst_port,
            p_CPU_RST_BIT    = cpu_rst_bit,
            p_INIT_FILE      = init_file,

            i_clk = ClockSignal(),
            i_rst = ResetSignal(),

            i_run      = self.control.fields.run,
            i_abort    = self.control.fields.abort,
            i_start_pc = self.start_pc.storage,
            o_running  = running,
            o_halted   = halted,
            o_aborted  = aborted,
            o_result   = result,

            i_imem_we   = self.imem_ctl.fields.we,
            i_imem_addr = self.imem_addr.storage,
            i_imem_data = self.imem_data.storage,

            o_io_out = io_out,
            i_io_in  = self.gpio_in.storage,

            o_wb_cyc   = self.bus.cyc,
            o_wb_stb   = self.bus.stb,
            o_wb_we    = self.bus.we,
            o_wb_adr   = self.bus.adr,
            o_wb_dat_w = self.bus.dat_w,
            o_wb_sel   = self.bus.sel,
            i_wb_dat_r = self.bus.dat_r,
            i_wb_ack   = self.bus.ack,

            o_wdt_fired = wdt_fired,
            o_retire    = retire,
            o_dbg_pc    = dbg_pc,
        )

        self.comb += [
            self.status.fields.running.eq(running),
            self.status.fields.halted.eq(halted),
            self.status.fields.aborted.eq(aborted),
            self.status.fields.wdt.eq(wdt_fired),
            self.result.status.eq(result),
            self.gpio_out.status.eq(io_out),
            self.dbg_pc.status.eq(dbg_pc),
        ]


def add_psc18sr(soc, imem_words: int = 1024, ro_words: int = 256,
               num_out: int = 4, num_in: int = 4, delay_prescale: int = 1024,
               autostart: bool = False, wdt_cycles: int = 0,
               cpu_rst_port: int = 0, cpu_rst_bit: int = 0,
               init_file: str = "", name: str = "psc18sr"):
    """Attach a PSC18SR to `soc`, registering its Wishbone master.

    bus_base defaults to the SoC's own CSR base, which is the whole point of
    the window: the core can reach every CSR and structurally nothing else.
    """
    bus_base = soc.mem_map["csr"]
    core = PSC18SR(soc.platform,
                  imem_words = imem_words,
                  ro_words   = ro_words,
                  num_out    = num_out,
                  num_in     = num_in,
                  delay_prescale = delay_prescale,
                  autostart  = autostart,
                  bus_base   = bus_base,
                  wdt_cycles = wdt_cycles,
                  cpu_rst_port = cpu_rst_port,
                  cpu_rst_bit  = cpu_rst_bit,
                  init_file  = init_file)
    setattr(soc, name, core)
    soc.add_csr(name)
    soc.bus.add_master(name=name, master=core.bus)

    # Synthesis-time facts firmware cannot otherwise know, exported as
    # generated constants: where the writable imem region starts (a program
    # loaded below it is silently discarded) and how long a DELAY tick is.
    # Hard-coding either in firmware means it drifts the first time a build
    # overrides them.
    up = name.upper()
    soc.add_constant(f"{up}_IMEM_WORDS", imem_words)
    soc.add_constant(f"{up}_RO_WORDS", ro_words)
    soc.add_constant(f"{up}_NUM_OUT", num_out)
    soc.add_constant(f"{up}_NUM_IN", num_in)
    soc.add_constant(f"{up}_DELAY_PRESCALE", delay_prescale)
    return core
