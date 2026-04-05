from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage


class GPIOOutReg(LiteXModule, AutoCSR):
    def __init__(self, reset=0, description="GPIO output register."):
        self._out = CSRStorage(reset=reset, description=description)
        self._status = CSRStatus(description="Current GPIO output value.")

        self.out = self._out.storage
        self.comb += self._status.status.eq(self._out.storage)
