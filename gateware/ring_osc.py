import os

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus


class RingMonitor(LiteXModule, AutoCSR):
    """Tiny on-die frequency / PVT monitor, exposed over CSR.

    Wraps the self-contained ring_monitor.v core. The ring's frequency tracks
    voltage/temperature/process, so software gets a cheap thermometer/voltage
    canary by reading one register. Reconstruct the frequency off the hot path:

        f_ring = count * prescale * sys_clk_freq / gate_cycles

    Cost is tiny (~5 LUTs + a few dozen FFs) and the CPU does nothing but read
    `_count` whenever it likes -- the measurement runs continuously in hardware.
    `_seq` bumps on every new window, so software can tell a fresh sample apart
    from a re-read without any timer or interrupt.

    Two clock domains only (sys + the internal ring `osc`); the prescaler is a
    synchronous counter, so PnR doesn't see a clock-per-divider-bit. The ring is
    a combinational loop: build with `--nextpnr-ignoreloops`.

    `_enable` resets to 0: a free-running ring left on during BIOS boot / SDRAM
    init can corrupt firmware running from SDRAM. Software turns it on once up.

    Optional `num_heaters` adds a gated bank of extra ring oscillators (see
    ring_heater.v) purely to self-heat the die for a thermal-sensitivity demo.
    Leave it at 0 for a real drop-in monitor.
    """
    def __init__(self, platform, sys_clk_freq, n_stages=5, gate_ms=10.0, prescale_bits=8,
                 num_heaters=0):
        assert n_stages % 2 == 1, "ring oscillator needs an odd number of stages"
        self.gate_cycles = int(sys_clk_freq * gate_ms / 1000)
        self.prescale    = 1 << prescale_bits   # firmware multiplies count by this

        self._enable = CSRStorage(reset=0, description="Run the monitor (off at boot).")
        self._count  = CSRStatus(32, description="Prescaled ring edges in the last window.")
        self._valid  = CSRStatus(description="Set once the first window completes.")
        self._seq    = CSRStatus(8, description="Increments on every new measurement.")

        verilog_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "verilog"))
        platform.add_source(os.path.join(verilog_root, "ring_monitor.v"))

        count = Signal(32)
        valid = Signal()
        seq   = Signal(8)

        self.specials += Instance(
            "ring_monitor",
            p_N_STAGES      = int(n_stages),
            p_PRESCALE_BITS = int(prescale_bits),
            p_GATE_CYCLES   = int(self.gate_cycles),
            i_clk    = ClockSignal("sys"),
            i_rstn   = ~ResetSignal("sys"),
            i_enable = self._enable.storage,
            o_count  = count,
            o_valid  = valid,
            o_seq    = seq,
        )
        # The core already latches a complete window, so just expose it.
        self.comb += [
            self._count.status.eq(count),
            self._valid.status.eq(valid),
            self._seq.status.eq(seq),
        ]

        # Optional on-die heater bank for the self-heating demo (not part of the
        # monitor proper). `_heater` resets off => zero power.
        if num_heaters > 0:
            self._heater       = CSRStorage(reset=0, description="Enable on-die heater rings.")
            self._heater_alive = CSRStatus(description="Live tap from the heater bank (also a noise source).")

            platform.add_source(os.path.join(verilog_root, "ring_heater.v"))

            heat_alive = Signal()
            self.specials += Instance(
                "ring_heater",
                p_N_RINGS  = int(num_heaters),
                p_N_STAGES = int(n_stages),
                i_enable   = self._heater.storage,
                o_alive    = heat_alive,
            )
            alive_sync = Signal(2)   # 2-FF sync (heat_alive toggles asynchronously)
            self.sync += alive_sync.eq(Cat(heat_alive, alive_sync[0]))
            self.comb += self._heater_alive.status.eq(alive_sync[1])
