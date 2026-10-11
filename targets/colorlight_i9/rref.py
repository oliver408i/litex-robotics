#!/usr/bin/env python3
"""Colorlight i9: PSC18SR + the RREF row-operation engine. DEMO.

The psc18sr.py side-block top plus one block on the core's ports: the PSC18SR
program (software/psc18sr_rref/rref.s) runs Gauss-Jordan elimination, and
verilog/rref_engine.v does each elementary row operation in exact rationals.
The host drives everything over Etherbone -- no firmware to build:

    .venv/bin/python targets/colorlight_i9/rref.py --build --load
    .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &
    .venv/bin/python tools/rref_demo.py '[[2,4,-2],[4,9,-3],[-2,-3,7]]'

Simulation of the same thing, no board: python3 sim/test_rref.py
"""
from base import BaseSoC, make_parser, run_build, add_eth_arguments, eth_kwargs

from gateware.psc18sr import add_psc18sr
from gateware.rref_engine import RREFEngine
from gateware.soc_features import add_eth


class RREFSoC(BaseSoC):
    def __init__(self, sys_clk_freq=87.5e6, sdram_rate="1:2", eth=None, **kwargs):
        super().__init__(sys_clk_freq=sys_clk_freq, sdram_rate=sdram_rate, **kwargs)

        # Ports 0-3 stay the CSR gpio ports; the engine takes out 4-5 and in 4.
        psc = add_psc18sr(self, num_out=6, num_in=4, ext_in=1,
                          autostart=False, wdt_cycles=0, init_file="")
        self.rref = RREFEngine(self.platform,
                               cmd = psc.io_out[16*4:16*5],
                               go  = psc.io_out[16*5])
        self.comb += psc.ext_in.eq(self.rref.sts)
        self.add_csr("rref")

        if eth is not None and eth["with_etherbone"]:
            add_eth(self, **eth)


def main():
    parser = make_parser(description="Colorlight i9 SoC + PSC18SR + RREF engine (demo).")
    add_eth_arguments(parser)
    args = parser.parse_args()
    soc = RREFSoC(
        sys_clk_freq = args.sys_clk_freq,
        sdram_rate   = "1:2" if args.sdram_half_rate else "1:1",
        eth          = eth_kwargs(args),
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
