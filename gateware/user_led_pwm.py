from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage


class UserLEDPwm(LiteXModule, AutoCSR):
    def __init__(self, pads, count, width=8, default_duty=26):
        if count <= 0:
            raise ValueError("count must be > 0")
        if default_duty < 0 or default_duty >= (1 << width):
            raise ValueError("default_duty must be in range [0, 2**width)")

        self.raw = Signal(count)

        duties = []
        for i in range(count):
            csr = CSRStorage(width, reset=default_duty, name=f"duty{i}",
                             description=f"User LED {i} PWM duty (0-{(1 << width) - 1}).")
            duties.append(csr)

        counter = Signal(width)
        self.sync += counter.eq(counter + 1)

        pwm_mask = Signal(count)
        self.comb += pwm_mask.eq(Cat(*[counter < duty.storage for duty in duties]))
        self.comb += pads.eq(self.raw & pwm_mask)
