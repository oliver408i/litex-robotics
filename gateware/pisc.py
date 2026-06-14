#!/usr/bin/env python3
"""LiteX wrapper for the PISC sequencer core (verilog/pisc.v).

Exposes the core as a host-loaded peripheral, mirroring the load/start/poll flow
of gateware/snn_mlp.py. The host (VexRiscv) writes a program into the core's
instruction memory over CSRs, pulses `run`, polls `halted`, then reads `result`
(= r7 at HLT).

Per-program flow from firmware:
    pisc_control_clr_write(1)            # idle, clear arch state
    for addr, word in enumerate(prog):   # load program
        pisc_imem_addr_write(addr)
        pisc_imem_data_write(word)
        pisc_imem_ctl_we_write(1)
    pisc_control_run_write(1)            # start at pc=0
    while not (pisc_status_read() & 2):  # poll halted
        pass
    result = pisc_result_read()

I/O ports are exposed as flat CSRs for bring-up: `gpio_out` mirrors the core's
output ports (OUT/SETB/CLRB), `gpio_in` feeds its input ports (IN/WAIT). In a
later phase these get sliced out and wired to real pads / a dedicated SPI
shifter instead (see docs/pisc_isa.md, docs/reset_sidebands.md).

CSR map (accessors shown for instance name "pisc"):
    control   : .control_run_write(1) / .control_clr_write(1)   (write-pulses)
    status    : bit0 = running, bit1 = halted
    result    : r7 latched at HLT
    imem_ctl  : .imem_ctl_we_write(1)   commits imem_addr -> imem[addr]=imem_data
    imem_addr / imem_data
    gpio_out  : packed output ports (port p = bits [16*p +: 16]), read-only
    gpio_in   : packed input ports (host-driven for bring-up)
    dbg_pc    : current/next PC (debug)
"""
from __future__ import annotations

import os

from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect.csr import AutoCSR, CSRField, CSRStatus, CSRStorage


def _log2_words(n: int) -> int:
    bits = 1
    while (1 << bits) < n:
        bits += 1
    return bits


class PISC(LiteXModule, AutoCSR):
    """Programmable I/O Sequencer Core as a CSR-mapped peripheral."""

    def __init__(self, platform, imem_words: int = 256,
                 num_out: int = 2, num_in: int = 2, delay_prescale: int = 1):
        assert (imem_words & (imem_words - 1)) == 0, "imem_words must be a power of 2"
        repo_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
        platform.add_source(os.path.join(repo_root, "verilog", "pisc.v"))

        self.imem_words = imem_words
        self.num_out = num_out
        self.num_in = num_in
        addr_bits = _log2_words(imem_words)

        # --- CSRs ---------------------------------------------------------
        self.control = CSRStorage(fields=[
            CSRField("run", size=1, pulse=True,
                     description="Pulse to start execution at pc=0."),
            CSRField("clr", size=1, pulse=True,
                     description="Pulse to clear arch state (regs/pc/io/halt) and idle."),
        ])
        self.status = CSRStatus(fields=[
            CSRField("running", size=1, description="Core is executing."),
            CSRField("halted",  size=1, description="Core hit HLT (result valid)."),
        ])
        self.result = CSRStatus(16, description="r7 latched at HLT.")

        # Instruction-memory write port (honored only while !running).
        self.imem_ctl = CSRStorage(fields=[
            CSRField("we", size=1, pulse=True,
                     description="Pulse to commit imem_addr -> imem[addr] = imem_data."),
        ])
        self.imem_addr = CSRStorage(16)
        self.imem_data = CSRStorage(16)

        # I/O ports (flat, bring-up). Packed: port p occupies bits [16*p +: 16].
        self.gpio_out = CSRStatus(16 * num_out,
            description="Packed output ports driven by OUT/SETB/CLRB.")
        self.gpio_in = CSRStorage(16 * num_in,
            description="Packed input ports read by IN / waited on by WAIT.")
        self.dbg_pc = CSRStatus(addr_bits, description="Current/next program counter.")

        # --- core signals -------------------------------------------------
        running = Signal()
        halted  = Signal()
        result  = Signal(16)
        io_out  = Signal(16 * num_out)
        dbg_pc  = Signal(addr_bits)
        retire  = Signal()   # unused here; a future debug CSR could expose it

        # --- instantiate verilog/pisc.v -----------------------------------
        self.specials += Instance(
            "pisc",
            p_IMEM_WORDS     = imem_words,
            p_NUM_OUT        = num_out,
            p_NUM_IN         = num_in,
            p_DELAY_PRESCALE = delay_prescale,

            i_clk = ClockSignal(),
            i_rst = ResetSignal(),

            i_run = self.control.fields.run,
            i_clr = self.control.fields.clr,
            o_running = running,
            o_halted  = halted,
            o_result  = result,

            i_imem_we   = self.imem_ctl.fields.we,
            i_imem_addr = self.imem_addr.storage,
            i_imem_data = self.imem_data.storage,

            o_io_out = io_out,
            i_io_in  = self.gpio_in.storage,

            o_retire = retire,
            o_dbg_pc = dbg_pc,
        )

        # --- status routing ----------------------------------------------
        self.comb += [
            self.status.fields.running.eq(running),
            self.status.fields.halted.eq(halted),
            self.result.status.eq(result),
            self.gpio_out.status.eq(io_out),
            self.dbg_pc.status.eq(dbg_pc),
        ]
