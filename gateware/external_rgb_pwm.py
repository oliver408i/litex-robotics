from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage


class ExternalRGBPWM(LiteXModule, AutoCSR):
    def __init__(self, pads=None, active_low=False):
        self._r = CSRStorage(8, reset=0, description="External RGB red PWM duty (0-255).")
        self._g = CSRStorage(8, reset=0, description="External RGB green PWM duty (0-255).")
        self._b = CSRStorage(8, reset=0, description="External RGB blue PWM duty (0-255).")

        counter = Signal(8)
        self.sync += counter.eq(counter + 1)

        r_out = Signal()
        g_out = Signal()
        b_out = Signal()

        self.comb += [
            r_out.eq(counter < self._r.storage),
            g_out.eq(counter < self._g.storage),
            b_out.eq(counter < self._b.storage),
        ]

        r_drive = Signal()
        g_drive = Signal()
        b_drive = Signal()

        if active_low:
            self.comb += [
                r_drive.eq(~r_out),
                g_drive.eq(~g_out),
                b_drive.eq(~b_out),
            ]
        else:
            self.comb += [
                r_drive.eq(r_out),
                g_drive.eq(g_out),
                b_drive.eq(b_out),
            ]

        self.r = r_drive
        self.g = g_drive
        self.b = b_drive

        if pads is not None:
            if hasattr(pads, "r"):
                self.comb += [
                    pads.r.eq(r_drive),
                    pads.g.eq(g_drive),
                    pads.b.eq(b_drive),
                ]
            else:
                self.comb += pads.eq(Cat(r_drive, g_drive, b_drive))
