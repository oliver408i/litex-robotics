#!/usr/bin/env python3
"""Colorlight i9 SoC that streams one ADC channel to a host over raw UDP.

The capture half of the radar chain. Gateware samples one MCP3008 channel on a
hardware timer and pushes the samples out a UDP socket in ~1 KB packets; the
host runs `tools/adc_stream_rx.py` and gets a raw int16 file. No DSP on the
board, no CPU in the sample path -- see gateware/adc_stream.py for why both of
those are deliberate.

**It streams from power-on.** `enable` resets to 1, so a loaded bitstream is
already sending; no firmware has to run and nothing has to be poked first. If
no host is listening the packets are simply dropped on the floor, which costs
nothing. That is what makes this "plug a waveform in and look at it" rather
than a procedure.

Wiring is unchanged from targets/colorlight_i9/adc.py -- SCLK K5, CS A2,
MOSI B3, MISO K4 on P3/`pmodh` -- so the same ADC that passed the bring-up test
works here. Feed the waveform to **CH0** unless you pass `--adc-channel`.

    # build, load, and listen
    .venv/bin/python targets/colorlight_i9/adc_stream.py --build --load
    .venv/bin/python tools/adc_stream_rx.py --seconds 5 --out /tmp/capture.raw

Defaults: CH0, 20 ksps, 512 samples/packet (a 1056-byte datagram, ~39/s,
40 KB/s). 20 ksps is 5x Nyquist on the 5 kHz top of the Doppler band, and the
whole stream is a rounding error on a gigabit link.

**Destination defaults to broadcast, not one computer.** Every host on the
local link gets the stream, and `tools/adc_stream_rx.py` et al. already bind
`0.0.0.0` so this needs no setup -- plug the board into any network and listen.
`--adc-dest-ip`/`tools/adc_stream_ctl.py --dest-ip` re-point it at one host if
you specifically want to stop broadcasting; see gateware/adc_stream.py for why
a unicast default pointed at a since-moved host is actively dangerous (it can
wedge Etherbone itself, not just the stream).

Rate and channel are CSRs, so they can be changed at runtime over Etherbone
without rebuilding -- `tools/adc_stream_ctl.py`. Etherbone stays up alongside
the stream: they are two UDP ports on the same core.

**The sample rate the host reports is the truth.** `--adc-sample-rate` picks a
divisor off the sys clock, so the achieved rate is `sys_clk_freq // period`, not
necessarily the number asked for. (At the 87.5 MHz default, 20 ksps is exact:
period 4375.) Every packet carries both `period` and
`sys_clk_freq` and the receiver prints the quotient; do not assume the CLI
value when the frequency axis matters.

Timing: this is `bringup.py` plus a small SPI shifter, a couple of FIFOs and a
second UDP port. The eth domains report their usual post-route failure (see
the README) and the design runs regardless, as every i9 build with Ethernet
has so far.
"""
from base import BaseSoC, make_parser, run_build, add_eth_arguments, eth_kwargs

from gateware.adc_stream import add_adc_stream
from gateware.soc_features import add_eth


class AdcStreamSoC(BaseSoC):
    def __init__(self, sys_clk_freq=87.5e6, sdram_rate="1:2",
                 adc_sclk_freq=1e6, sample_rate=20e3, samples_per_packet=512,
                 dest_ip="255.255.255.255", udp_port=5005, eth=None, **kwargs):
        super().__init__(sys_clk_freq=sys_clk_freq, sdram_rate=sdram_rate, **kwargs)

        # Ethernet first: add_adc_stream hangs a second UDP port off the core
        # that add_eth creates, so the core has to exist before it is asked for
        # a port.
        if eth is None or not eth["with_etherbone"]:
            raise SystemExit(
                "adc_stream.py needs the Ethernet core: the stream is a UDP port on it.\n"
                "  Drop --no-etherbone. (Etherbone itself stays available on 1234 --\n"
                "  the two share one core and coexist.)"
            )
        add_eth(self, **eth)

        add_adc_stream(self,
                       sclk_freq          = adc_sclk_freq,
                       sample_rate        = sample_rate,
                       samples_per_packet = samples_per_packet,
                       dest_ip            = dest_ip,
                       udp_port           = udp_port)


def main():
    parser = make_parser(description="Colorlight i9 SoC that streams one MCP3008 "
                                     "channel to a host over UDP.")
    add_eth_arguments(parser)
    parser.add_target_argument("--adc-sclk-freq", default=1e6, type=float,
                               help="MCP3008 SCLK (Hz). Keep under ~1.8 MHz on a 3.3 V "
                                    "part. A conversion is 24 SCLK, so this also caps "
                                    "the sample rate: 1 MHz SCLK -> ~41 ksps.")
    parser.add_target_argument("--adc-sample-rate", default=20e3, type=float,
                               help="Reset value of the sample rate (Hz). Realised as "
                                    "an integer divisor of sys, so the achieved rate is "
                                    "sys/round(sys/rate); the receiver prints the true "
                                    "one from the packet header. Runtime-changeable.")
    parser.add_target_argument("--adc-samples-per-packet", default=512,
                               type=lambda x: int(x, 0),
                               help="Samples per datagram; must be even. 512 gives a "
                                    "1056-byte packet. The payload must stay under "
                                    "1472 bytes -- LiteEth does not fragment, so a "
                                    "bigger packet is a broken one, and the build "
                                    "asserts rather than shipping it.")
    parser.add_target_argument("--adc-dest-ip", default="255.255.255.255",
                               help="Host the samples are sent to. Defaults to the "
                                    "broadcast address so a stock build needs no "
                                    "network-specific config and can never wedge "
                                    "ARP chasing an unreachable host (see the long "
                                    "comment in gateware/adc_stream.py). Also a CSR, "
                                    "so it can be re-pointed at one host at runtime "
                                    "with tools/adc_stream_ctl.py --dest-ip/--auto.")
    parser.add_target_argument("--adc-udp-port", default=5005,
                               type=lambda x: int(x, 0),
                               help="Destination UDP port. Not 1234, which is Etherbone's.")
    args = parser.parse_args()

    soc = AdcStreamSoC(
        sys_clk_freq       = args.sys_clk_freq,
        sdram_rate         = "1:2" if args.sdram_half_rate else "1:1",
        adc_sclk_freq      = args.adc_sclk_freq,
        sample_rate        = args.adc_sample_rate,
        samples_per_packet = args.adc_samples_per_packet,
        dest_ip            = args.adc_dest_ip,
        udp_port           = args.adc_udp_port,
        eth                = eth_kwargs(args),
        **parser.soc_argdict,
    )
    run_build(soc, args, parser)


if __name__ == "__main__":
    main()
