from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage


class MLP2Accel(LiteXModule, AutoCSR):
    def __init__(self, in_size=16, hidden_size=8, out_size=4, macs_per_cycle=2):
        assert in_size > 0 and (in_size % macs_per_cycle == 0)
        assert hidden_size > 0 and (hidden_size % macs_per_cycle == 0)
        assert out_size > 0
        assert macs_per_cycle == 2  # current implementation assumes 2 MACs/cycle

        # Control/status.
        self._ctrl = CSRStorage(2, description="bit0=start, bit1=clear_done")
        self._status = CSRStatus(2, description="bit0=busy, bit1=done")

        # Input vector write.
        self._in_addr = CSRStorage(bits_for(in_size - 1), description="Input index.")
        self._in_data = CSRStorage(8, description="Input value (int8).")
        self._in_we = CSRStorage(description="Write strobe for input.")

        # Layer1 weight matrix write: addr[15:8]=hidden, addr[7:0]=in.
        self._w1_addr = CSRStorage(16, description="L1 weight address: [15:8]=hidden, [7:0]=in.")
        self._w1_data = CSRStorage(8, description="L1 weight value (int8).")
        self._w1_we = CSRStorage(description="Write strobe for L1 weight.")

        # Layer1 bias write.
        self._b1_addr = CSRStorage(bits_for(hidden_size - 1), description="L1 bias index.")
        self._b1_data = CSRStorage(16, description="L1 bias value (int16).")
        self._b1_we = CSRStorage(description="Write strobe for L1 bias.")

        # Layer2 weight matrix write: addr[15:8]=out, addr[7:0]=hidden.
        self._w2_addr = CSRStorage(16, description="L2 weight address: [15:8]=out, [7:0]=hidden.")
        self._w2_data = CSRStorage(8, description="L2 weight value (int8).")
        self._w2_we = CSRStorage(description="Write strobe for L2 weight.")

        # Layer2 bias write.
        self._b2_addr = CSRStorage(bits_for(out_size - 1), description="L2 bias index.")
        self._b2_data = CSRStorage(16, description="L2 bias value (int16).")
        self._b2_we = CSRStorage(description="Write strobe for L2 bias.")

        # Output read.
        self._out_addr = CSRStorage(bits_for(out_size - 1), description="Output index.")
        self._out_data = CSRStatus(32, description="Output value (int32, ReLU applied).")

        # Memories.
        in_mem = Array(Signal((8, True), reset=0) for _ in range(in_size))
        w1_mem = Array(
            Array(Signal((8, True), reset=0) for _ in range(in_size))
            for _ in range(hidden_size)
        )
        b1_mem = Array(Signal((16, True), reset=0) for _ in range(hidden_size))
        h_mem = Array(Signal((16, True), reset=0) for _ in range(hidden_size))

        w2_mem = Array(
            Array(Signal((8, True), reset=0) for _ in range(hidden_size))
            for _ in range(out_size)
        )
        b2_mem = Array(Signal((16, True), reset=0) for _ in range(out_size))
        out_mem = Array(Signal((32, True), reset=0) for _ in range(out_size))

        # Write paths.
        self.sync += [
            If(self._in_we.re,
                in_mem[self._in_addr.storage].eq(self._in_data.storage)
            ),
            If(self._w1_we.re,
                w1_mem[self._w1_addr.storage[8:16]][self._w1_addr.storage[0:8]].eq(self._w1_data.storage)
            ),
            If(self._b1_we.re,
                b1_mem[self._b1_addr.storage].eq(self._b1_data.storage)
            ),
            If(self._w2_we.re,
                w2_mem[self._w2_addr.storage[8:16]][self._w2_addr.storage[0:8]].eq(self._w2_data.storage)
            ),
            If(self._b2_we.re,
                b2_mem[self._b2_addr.storage].eq(self._b2_data.storage)
            ),
        ]

        # Output read (combinational).
        self.comb += self._out_data.status.eq(out_mem[self._out_addr.storage])

        # Compute engine (2 MACs per cycle).
        in_idx = Signal(bits_for(in_size - 1))
        h_idx = Signal(bits_for(hidden_size - 1))
        out_idx = Signal(bits_for(out_size - 1))
        acc = Signal((32, True))
        busy = Signal(reset=0)
        done = Signal(reset=0)

        in0 = Signal((8, True))
        in1 = Signal((8, True))
        w10 = Signal((8, True))
        w11 = Signal((8, True))
        h0 = Signal((16, True))
        h1 = Signal((16, True))
        w20 = Signal((8, True))
        w21 = Signal((8, True))

        self.comb += [
            in0.eq(in_mem[in_idx]),
            in1.eq(in_mem[in_idx + 1]),
            w10.eq(w1_mem[h_idx][in_idx]),
            w11.eq(w1_mem[h_idx][in_idx + 1]),
            h0.eq(h_mem[in_idx]),
            h1.eq(h_mem[in_idx + 1]),
            w20.eq(w2_mem[out_idx][in_idx]),
            w21.eq(w2_mem[out_idx][in_idx + 1]),
        ]

        relu32 = Signal((32, True))
        relu16 = Signal((16, True))
        self.comb += [
            relu32.eq(Mux(acc < 0, 0, acc)),
            relu16.eq(Mux(acc < 0, 0, acc[0:16])),
        ]

        self.comb += [
            self._status.status[0].eq(busy),
            self._status.status[1].eq(done),
        ]

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        self.sync += [
            If(self._ctrl.storage[1], done.eq(0)),
        ]

        fsm.act("IDLE",
            If(self._ctrl.storage[0] & ~busy,
                NextValue(busy, 1),
                NextValue(done, 0),
                NextValue(h_idx, 0),
                NextValue(in_idx, 0),
                NextValue(acc, b1_mem[0]),
                NextState("L1_MAC"),
            ),
        )

        fsm.act("L1_MAC",
            NextValue(acc, acc + (in0 * w10) + (in1 * w11)),
            If(in_idx == (in_size - 2),
                NextState("L1_STORE"),
            ).Else(
                NextValue(in_idx, in_idx + 2),
            ),
        )

        fsm.act("L1_STORE",
            NextValue(h_mem[h_idx], relu16),
            If(h_idx == (hidden_size - 1),
                NextValue(out_idx, 0),
                NextValue(in_idx, 0),
                NextValue(acc, b2_mem[0]),
                NextState("L2_MAC"),
            ).Else(
                NextValue(h_idx, h_idx + 1),
                NextValue(in_idx, 0),
                NextValue(acc, b1_mem[h_idx + 1]),
                NextState("L1_MAC"),
            ),
        )

        fsm.act("L2_MAC",
            NextValue(acc, acc + (h0 * w20) + (h1 * w21)),
            If(in_idx == (hidden_size - 2),
                NextState("L2_STORE"),
            ).Else(
                NextValue(in_idx, in_idx + 2),
            ),
        )

        fsm.act("L2_STORE",
            NextValue(out_mem[out_idx], relu32),
            If(out_idx == (out_size - 1),
                NextValue(busy, 0),
                NextValue(done, 1),
                NextState("IDLE"),
            ).Else(
                NextValue(out_idx, out_idx + 1),
                NextValue(in_idx, 0),
                NextValue(acc, b2_mem[out_idx + 1]),
                NextState("L2_MAC"),
            ),
        )
