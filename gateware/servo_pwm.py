from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage


class ServoPWM(LiteXModule, AutoCSR):
    def __init__(self, pads, sys_clk_freq, pwm_freq=50, channels=5):
        if pwm_freq <= 0:
            raise ValueError("pwm_freq must be > 0")

        if channels <= 0:
            raise ValueError("channels must be > 0")

        duties = []
        for i in range(channels):
            csr = CSRStorage(32, reset=0, name=f"duty{i}", description=f"Servo {i} duty in ticks.")
            setattr(self, f"_duty{i}", csr)
            duties.append(csr.storage)

        period_ticks = int(sys_clk_freq // pwm_freq)
        if period_ticks <= 0:
            raise ValueError("pwm period too small")

        counter = Signal(max=period_ticks)
        self.sync += If(counter == (period_ticks - 1), counter.eq(0)).Else(counter.eq(counter + 1))

        outputs = []
        for duty in duties:
            outputs.append(counter < duty)

        if isinstance(pads, Signal):
            self.comb += pads.eq(Cat(*outputs))
        else:
            for i, pad in enumerate(pads):
                self.comb += pad.eq(outputs[i])
