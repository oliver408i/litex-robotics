"""74HC595 output-expander driver (3 pins -> 8 slow outputs).

The '595 carries the project's slow reset/enable sidebands (LCD_RST,
CTP_RST, WINC_EN, WINC_RST) so they stop costing one FPGA pin each.
This driver is transparent to firmware: features comb-drive bits of
`value` from their existing CSRs, and the core re-shifts the register
(MSB first, so value[7] lands on Qh / value[0] on Qa) and pulses RCLK
whenever `value` changes. SRCLR is assumed tied high and OE tied low.

One forced shift runs right after (soft) reset because the '595 powers
up with random latch contents -- this preserves the old direct-GPIOOut
guarantee that the resets/enables sit at their CSR reset values from
configuration onward (a few us of random levels at power-up aside).

An update takes 18 SRCLK half-periods (~4.5 us at the default 2 MHz);
back-to-back CSR writes landing mid-shift simply trigger another shift,
so the latched outputs always converge on the last written state. That
is orders of magnitude faster than the ms-scale reset sequencing in
firmware (nm_bsp_reset, lcd_hw_reset), so no handshake is needed.
"""
from math import ceil

from migen import If, Signal, Cat, FSM, NextState, NextValue

from litex.gen import LiteXModule


class SR595(LiteXModule):
    def __init__(self, pads, sys_clk_freq, sclk_freq=2e6):
        self.value = Signal(8)   # in: desired Qa..Qh levels (bit0 = Qa)

        # Half-period tick. 2 MHz SRCLK is conservative for an HC part at
        # 3.3 V on jumper wiring (the part itself is good for >20 MHz).
        div  = max(1, ceil(sys_clk_freq / (2 * sclk_freq)))
        cnt  = Signal(max=div)
        tick = Signal()
        self.comb += tick.eq(cnt == div - 1)
        self.sync += If(tick, cnt.eq(0)).Else(cnt.eq(cnt + 1))

        shadow = Signal(8)  # bits in flight, MSB next on the wire
        last   = Signal(8)  # value most recently latched (or being shifted)
        dirty  = Signal(reset=1)  # force one shift after (soft) reset
        idx    = Signal(3)

        # SER/SRCLK/RCLK are decoded combinationally from the (binary-encoded)
        # FSM state. A bare comb decode to the pins can glitch while the state
        # bits settle with skew on a transition -- a spurious SRCLK edge shifts
        # every bit one position (Qa/Qb diverge even though firmware drives them
        # identically), and a spurious RCLK latches a half-shifted register.
        # Those hazards show up as random "half-works / didn't work this boot"
        # behaviour across power cycles. Drive the comb decode into output flops
        # so the pins only change on a clock edge, after the decode has settled:
        # the glitches never reach the wire. The one extra sys-clk of latency is
        # applied equally to all three, so their relative timing is preserved
        # (and div >> 1, so it is negligible against the SRCLK half-period).
        ser_c, srclk_c, rclk_c = Signal(), Signal(), Signal()
        self.sync += [
            pads.ser.eq(ser_c),
            pads.srclk.eq(srclk_c),
            pads.rclk.eq(rclk_c),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(tick & (dirty | (self.value != last)),
                NextValue(shadow, self.value),
                NextValue(last,   self.value),
                NextValue(dirty,  0),
                NextValue(idx,    0),
                NextState("LOW"),
            ),
        )
        fsm.act("LOW",   # SRCLK low half: present the data bit
            ser_c.eq(shadow[7]),
            If(tick, NextState("HIGH")),
        )
        fsm.act("HIGH",  # SRCLK high half: bit was captured on the rising edge
            ser_c.eq(shadow[7]),
            srclk_c.eq(1),
            If(tick,
                NextValue(shadow, Cat(Signal(), shadow[:7])),
                If(idx == 7,
                    NextState("LATCH"),
                ).Else(
                    NextValue(idx, idx + 1),
                    NextState("LOW"),
                ),
            ),
        )
        fsm.act("LATCH",  # RCLK rising edge moves the shift reg to the outputs
            rclk_c.eq(1),
            If(tick, NextState("IDLE")),
        )
