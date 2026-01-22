from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage


class MotorPWM(LiteXModule, AutoCSR):
    def __init__(self, pads):
        self._duty0 = CSRStorage(8, reset=0, description="Motor 0 PWM duty (0-255).")
        self._duty1 = CSRStorage(8, reset=0, description="Motor 1 PWM duty (0-255).")

        counter = Signal(8)
        self.sync += counter.eq(counter + 1)

        pwm0 = Signal()
        pwm1 = Signal()

        self.comb += [
            pwm0.eq(counter < self._duty0.storage),
            pwm1.eq(counter < self._duty1.storage),
        ]

        if hasattr(pads, "pwm0"):
            self.comb += [
                pads.pwm0.eq(pwm0),
                pads.pwm1.eq(pwm1),
            ]
        else:
            self.comb += pads.eq(Cat(pwm0, pwm1))
