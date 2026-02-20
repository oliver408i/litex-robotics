#!/usr/bin/env python3
from pathlib import Path
import sys

from migen.fhdl import verilog
from migen import Module, Signal, ClockDomain


def collect_ios(top):
    ios = []
    # Clock/reset.
    ios += [top.sys_clk, top.sys_rst]
    # Wishbone master.
    ios += [
        top.mlp.bus.cyc,
        top.mlp.bus.stb,
        top.mlp.bus.we,
        top.mlp.bus.adr,
        top.mlp.bus.dat_w,
        top.mlp.bus.dat_r,
        top.mlp.bus.sel,
        top.mlp.bus.ack,
    ]
    # CSR storages (inputs).
    ios += [
        top.mlp._start.storage,
        top.mlp._start.re,
        top.mlp._relu_en.storage,
        top.mlp._shift.storage,
        top.mlp._dim_in.storage,
        top.mlp._dim_hid.storage,
        top.mlp._dim_out.storage,
        top.mlp._base_in.storage,
        top.mlp._base_w1.storage,
        top.mlp._base_b1.storage,
        top.mlp._base_w2.storage,
        top.mlp._base_b2.storage,
        top.mlp._base_hid.storage,
        top.mlp._base_out.storage,
    ]
    # CSR statuses (outputs).
    ios += [
        top.mlp._busy.status,
        top.mlp._done.status,
        top.mlp._error.status,
        top.mlp._dbg_acc_lo.status,
        top.mlp._dbg_acc_hi.status,
        top.mlp._dbg_w.status,
        top.mlp._dbg_x.status,
    ]
    return set(ios)


repo_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(repo_root))

from gateware.mlp_stream import MLPStream


class MLPStreamTop(Module):
    def __init__(self):
        self.sys_clk = Signal()
        self.sys_rst = Signal()
        self.clock_domains.cd_sys = ClockDomain()
        self.comb += [
            self.cd_sys.clk.eq(self.sys_clk),
            self.cd_sys.rst.eq(self.sys_rst),
        ]
        self.submodules.mlp = MLPStream(max_in=784, max_hid=128, max_out=10, data_bits=16)


def main():

    out_dir = Path(__file__).resolve().parent / "build"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "mlp_stream.v"

    dut = MLPStreamTop()
    v = verilog.convert(dut, ios=collect_ios(dut), name="mlp_stream")
    out_path.write_text(str(v))
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
