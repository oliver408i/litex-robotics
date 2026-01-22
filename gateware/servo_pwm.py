from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage


class ServoPWM(LiteXModule, AutoCSR):
    def __init__(self, pads, sys_clk_freq, pwm_freq=50):
        if pwm_freq <= 0:
            raise ValueError("pwm_freq must be > 0")

        self._duty0 = CSRStorage(32, reset=0, description="Servo 0 duty in ticks.")
        self._duty1 = CSRStorage(32, reset=0, description="Servo 1 duty in ticks.")
        self._duty2 = CSRStorage(32, reset=0, description="Servo 2 duty in ticks.")
        self._duty3 = CSRStorage(32, reset=0, description="Servo 3 duty in ticks.")
        self._duty4 = CSRStorage(32, reset=0, description="Servo 4 duty in ticks.")

        period_ticks = int(sys_clk_freq // pwm_freq)
        if period_ticks <= 0:
            raise ValueError("pwm period too small")

        counter = Signal(max=period_ticks)
        self.sync += If(counter == (period_ticks - 1), counter.eq(0)).Else(counter.eq(counter + 1))

        duties = [self._duty0.storage, self._duty1.storage, self._duty2.storage,
                  self._duty3.storage, self._duty4.storage]
        outputs = []
        for duty in duties:
            outputs.append(counter < duty)

        if isinstance(pads, Signal):
            self.comb += pads.eq(Cat(*outputs))
        else:
            for i, pad in enumerate(pads):
                self.comb += pad.eq(outputs[i])
