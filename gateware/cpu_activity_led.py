from migen import *
from litex.gen import *


class CpuActivityLED(LiteXModule):
    def __init__(self, pad, activity, sys_clk_freq, hold_time=0.02, invert=False, pwm=None):
        hold_cycles = max(1, int(sys_clk_freq * hold_time))
        counter     = Signal(max=hold_cycles + 1)
        led         = Signal()
        gated       = Signal()

        # Stretch activity pulses to a visible LED blink.
        self.sync += [
            If(activity,
                counter.eq(hold_cycles)
            ).Elif(counter != 0,
                counter.eq(counter - 1)
            )
        ]
        self.comb += led.eq(counter != 0)

        if pwm is None:
            self.comb += gated.eq(led)
        else:
            self.comb += gated.eq(led & pwm)

        self.comb += pad.eq(~gated if invert else gated)
