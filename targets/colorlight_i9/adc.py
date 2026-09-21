#!/usr/bin/env python3
"""Colorlight i9 SoC + MCP3008 ADC on P3/`pmodh`. First step of the radar chain.

`bringup.py` plus one peripheral: a free-running 8-channel MCP3008 sampler
(gateware/mcp3008.py). Nothing else changes, so if this does not boot the ADC
is not why.

Wiring, MCP3008 DIP-16 pin -> FPGA ball (`pmodh` is "- E19 B3 K5 - B2 K4 A2",
so all four land on the one header):

    | MCP3008    | signal        | i9 ball |
    |------------|---------------|---------|
    | 13 CLK     | SCLK   (out)  | K5      |
    | 10 CS/SHDN | CS_n   (out)  | A2      |
    | 11 D_IN    | MOSI   (out)  | B3      |
    | 12 D_OUT   | MISO   (in)   | K4      |
    | 16 VDD     | 3V3           | header  |
    | 15 VREF    | 3V3           | header  |
    | 14 AGND    | GND           | header  |
    |  9 DGND    | GND           | header  |

VREF is what the counts are measured against, so tying it to the same 3V3 the
FPGA runs on makes the LSB 3.22 mV and makes every reading only as clean as
that rail. That is fine for bring-up and is *not* fine for the radar -- the
front end will want its own reference.

**MISO is pulled up in the FPGA** (gateware/mcp3008.py explains why): with no
chip on the header every channel reads **1023**, so "all 1023" means nothing
is answering, not that the input is at the rail. All channels reading 0 means
the part is answering with a stuck-low D_OUT, which is a different fault.

Build, load, and test:

    .venv/bin/python targets/colorlight_i9/adc.py --build --load

    cd software/mcp3008_test && make && cd ../..
    .venv/bin/python tools/socboot.py \
        --kernel software/mcp3008_test/mcp3008_test.bin \
        --expect "MCP3008 test complete" --timeout 90

SCLK defaults to 1 MHz (`--adc-sclk-freq`). Do not raise it past ~1.8 MHz on a
3.3 V part: the SAR stops settling and the numbers stay plausible while being
wrong, which is the one failure this rig cannot catch by looking at the
console.

This top is the sampler only. Hardware-timed capture straight out a UDP
socket -- the thing the radar actually needs -- is a later step; see the
project notes. Etherbone is on by default as on every other i9 top, so CSRs
are reachable from the host without the probe (`litex_server` + a script
beats reflashing firmware to try a different channel mask).
"""
from base import BaseSoC, make_parser, run_build, add_eth_arguments, eth_kwargs

from gateware.mcp3008 import add_mcp3008
from gateware.soc_features import add_eth


class AdcSoC(BaseSoC):
    def __init__(self, sys_clk_freq=87.5e6, sdram_rate="1:2",
                 adc_sclk_freq=1e6, adc_interval_ms=10, eth=None, **kwargs):
        super().__init__(sys_clk_freq=sys_clk_freq, sdram_rate=sdram_rate, **kwargs)

        add_mcp3008(self,
                    sclk_freq           = adc_sclk_freq,
                    default_interval_ms = adc_interval_ms)

        if eth is not None and eth["with_etherbone"]:
            add_eth(self, **eth)


def main():
    parser = make_parser(description="Colorlight i9 SoC + MCP3008 ADC on P3/pmodh.")
    add_eth_arguments(parser)
    parser.add_target_argument("--adc-sclk-freq", default=1e6, type=float,
                               help="MCP3008 SCLK (Hz). The divisor is an integer, so "
                                    "the achieved rate is the highest one at or below "
                                    "this; it is exported as MCP3008_SCLK_FREQ. Keep it "
                                    "under ~1.8 MHz -- that is the part's 3.3 V limit, "
                                    "and past it the readings are wrong rather than "
                                    "obviously broken.")
    parser.add_target_argument("--adc-interval-ms", default=10, type=float,
                               help="Reset value of the sample-interval timer, in ms "
                                    "between conversions. Firmware can write 0 to free "
                                    "run (~41 ksps aggregate at 1 MHz SCLK).")
    args = parser.parse_args()

    soc = AdcSoC(
        sys_clk_freq    = args.sys_clk_freq,
        sdram_rate      = "1:2" if args.sdram_half_rate else "1:1",
        adc_sclk_freq   = args.adc_sclk_freq,
        adc_interval_ms = args.adc_interval_ms,
        eth             = eth_kwargs(args),
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
