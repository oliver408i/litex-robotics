from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage


class LaserControl(LiteXModule, AutoCSR):
    def __init__(self, reset=0):
        self._enable = CSRStorage(reset=reset, description="Laser enable (1=on).")
        self._status = CSRStatus(description="Laser enable status.")

        self.enable = self._enable.storage
        self.comb += self._status.status.eq(self._enable.storage)
