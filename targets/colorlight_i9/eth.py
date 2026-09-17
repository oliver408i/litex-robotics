#!/usr/bin/env python3
"""Colorlight i9 PHY bring-up: the Ethernet PHY and nothing else.

Deliberately has **no MAC and no Etherbone**. The point is the question that
comes first and that can be answered with no magjacks fitted: is the PHY alive,
which PHY is it, and is the reset/MDIO wiring right?

MDIO is a management bus to the PHY chip -- two pins, no magnetics, no link, no
cable. So this build is useful on a board whose RJ45s are not populated yet,
which is the state of the bench as of 2026-09-12. The BIOS gates its
`mdio_read`/`mdio_write`/`mdio_dump` commands on `CSR_ETHPHY_MDIO_W_ADDR`
alone, which the PHY module provides on its own -- hence no MAC here.

    .venv/bin/python targets/colorlight_i9/eth.py --build --load
    .venv/bin/python tools/socboot.py --send 'mdio_dump 0 8\\n' --monitor 10

Registers worth reading (IEEE 802.3 clause 22, same on every PHY):

    reg 0   BMCR    control: reset (b15), loopback (b14), speed, autoneg
    reg 1   BMSR    status: link up (b2), autoneg complete (b5)
    reg 2/3 PHYID1/2  the chip's OUI + model -- this is what identifies it

reg 2/3 is the payoff: Colorlight boards ship with more than one PHY (Broadcom
B50612D and Realtek RTL8211 both appear), and they strap differently. Whether
`tx_delay=0` below is right depends on which one is down there, so read the ID
before trusting any RGMII timing.

**Only one PHY can be instantiated at a time.** `eth 0` and `eth 1` share
`rst_n` (P4), `mdio` (P5) and `mdc` (N5) on this board, so requesting both
would drive those pads twice. `--eth-phy` picks which one gets the RGMII data
pins; either choice reaches BOTH PHYs over MDIO, since MDIO is a shared bus
addressed by PHY address. Note also that the platform's PHY numbering is
swapped relative to the silkscreen (it follows the PHYA[0] straps).

`tx_delay=0` follows litex-boards' own colorlight target: it means the PHY is
expected to add the RGMII TX clock delay internally, from strapping, rather
than the FPGA adding it in DELAYG taps. Untested here -- it does not matter for
MDIO, and it is the first thing to question when a link comes up but corrupts
frames.
"""
from base import BaseSoC, make_parser, run_build, add_eth_arguments, eth_kwargs

from gateware.soc_features import add_eth


class EthPhySoC(BaseSoC):
    """PHY-only by default; --with-ethernet / --with-etherbone add a datapath.

    The gateware all lives in gateware/soc_features.py: add_eth. What is
    specific to this top is only the default (no MAC, no Etherbone, so PHY
    bring-up never depends on the datapath being right) and the RX clock probe.
    """
    def __init__(self, sys_clk_freq=48e6, sdram_rate="1:1", eth=None, **kwargs):
        super().__init__(sys_clk_freq=sys_clk_freq, sdram_rate=sdram_rate, **kwargs)
        add_eth(self, with_rx_probe=True, **eth)


def main():
    parser = make_parser(description="Colorlight i9 Ethernet PHY bring-up "
                                     "(MDIO only by default -- no MAC, no Etherbone).")
    # default_etherbone=False: unlike the feature tops, this one exists to
    # answer "is the PHY alive?" on a board with no magjacks fitted, and that
    # question must not depend on a datapath.
    add_eth_arguments(parser, default_etherbone=False)
    parser.add_target_argument("--with-ethernet", action="store_true",
                               help="Add the MAC (LiteEthMAC + BIOS netboot/ping). "
                                    "Mutually exclusive with Etherbone on one PHY.")
    args = parser.parse_args()

    # add_eth asserts this too, but a traceback is a poor way to learn it.
    if args.with_ethernet and args.with_etherbone:
        raise SystemExit("--with-ethernet and --with-etherbone cannot share one PHY "
                         "in this LiteX; pick one.")

    soc = EthPhySoC(
        sys_clk_freq  = args.sys_clk_freq,
        sdram_rate    = "1:2" if args.sdram_half_rate else "1:1",
        eth           = dict(eth_kwargs(args), with_ethernet=args.with_ethernet),
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
