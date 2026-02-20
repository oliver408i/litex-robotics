from migen import *
from litex.gen import *


class LEDPwm(LiteXModule):
    def __init__(self, width=8, duty=64):
        if duty < 0 or duty >= (1 << width):
            raise ValueError("duty must be in range [0, 2**width)")

        self.pwm = Signal()

        counter = Signal(width)
        self.sync += counter.eq(counter + 1)
        self.comb += self.pwm.eq(counter < duty)
