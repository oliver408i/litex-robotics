#!/usr/bin/env python3
"""LiteX wrapper for the RREF demo engine (verilog/rref_engine.v).

DEMO. The engine does elementary row operations on an up-to-8x8 matrix of
exact rationals; a PSC18SR program (software/psc18sr_rref/rref.s) decides
which ones. The core drives it over its IO ports:

    PSC18SR out port 4  -> command word
    PSC18SR out port 5  -> bit 0 = go
    PSC18SR in  port 4  <- status (busy, zero, ovf, err, cols, rows)

The host side is CSRs: set rows/cols, write the matrix element by element
(`addr` = r*8+c, then `num`, `den`, pulse `ctl.we`), run the PSC18SR program,
read the matrix back the same way. See tools/rref_demo.py.
"""
from __future__ import annotations

import os

from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect.csr import AutoCSR, CSRField, CSRStatus, CSRStorage


class RREFEngine(LiteXModule, AutoCSR):
    def __init__(self, platform, cmd, go, W=32):
        repo_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
        for f in ("rref_engine.v", "rref_rat.v"):
            platform.add_source(os.path.join(repo_root, "verilog", f))

        self.sts = Signal(16)     # -> a PSC18SR input port

        self.dims = CSRStorage(fields=[
            CSRField("rows", size=4, description="Matrix rows (1-8)."),
            CSRField("cols", size=4, description="Matrix columns (1-8)."),
        ])
        self.ctl = CSRStorage(fields=[
            CSRField("we",        size=1, pulse=True, description="Write num/den to addr."),
            CSRField("clr_flags", size=1, pulse=True, description="Clear ovf/err."),
        ])
        self.addr = CSRStorage(6, description="Element address, r*8 + c.")
        self.num  = CSRStorage(W, description="Numerator to write (two's complement).")
        self.den  = CSRStorage(W, description="Denominator to write (> 0).")
        self.rnum = CSRStatus(W, description="Numerator at addr.")
        self.rden = CSRStatus(W, description="Denominator at addr.")
        self.status   = CSRStatus(16, description="Same word the PSC18SR sees on its port.")
        self.ops_done = CSRStatus(32, description="Row operations completed since reset.")

        self.specials += Instance("rref_engine",
            p_W = W,
            i_clk = ClockSignal(), i_rst = ResetSignal(),
            i_cmd = cmd, i_go = go, o_sts = self.sts,
            i_rows = self.dims.fields.rows, i_cols = self.dims.fields.cols,
            i_clr_flags = self.ctl.fields.clr_flags,
            i_h_addr = self.addr.storage, i_h_we = self.ctl.fields.we,
            i_h_wn = self.num.storage, i_h_wd = self.den.storage,
            o_h_rn = self.rnum.status, o_h_rd = self.rden.status,
            o_ops_done = self.ops_done.status,
        )
        self.comb += self.status.status.eq(self.sts)
