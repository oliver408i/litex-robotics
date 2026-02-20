from migen import *

from litex.gen import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage


class MLP2Accel(LiteXModule, AutoCSR):
    def __init__(self, in_size_max=784, hidden_size_max=128, out_size_max=16, tile_in_max=64, macs_per_cycle=2):
        assert in_size_max > 0
        assert hidden_size_max > 0
        assert out_size_max > 0
        assert tile_in_max > 0 and (tile_in_max % macs_per_cycle == 0)
        assert macs_per_cycle == 2

        # Control/status.
        self._ctrl = CSRStorage(3, description="bit0=start, bit1=clear_done, bit2=load_from_mem")
        self._status = CSRStatus(2, description="bit0=busy, bit1=done")

        # Runtime sizes (must be <= max).
        self._in_size = CSRStorage(16, reset=min(in_size_max, 16), description="Runtime input size (even).")
        self._hidden_size = CSRStorage(16, reset=min(hidden_size_max, 8), description="Runtime hidden size (even).")
        self._out_size = CSRStorage(16, reset=min(out_size_max, 4), description="Runtime output size.")
        self._tile_in = CSRStorage(16, reset=min(tile_in_max, 16), description="Input tile size (even, <= max, divides in_size).")

        # SDRAM base pointers (byte addresses).
        self._in_ptr = CSRStorage(32, description="SDRAM base for inputs (bytes).")
        self._w1_ptr = CSRStorage(32, description="SDRAM base for L1 weights (bytes).")
        self._b1_ptr = CSRStorage(32, description="SDRAM base for L1 bias (bytes).")
        self._w2_ptr = CSRStorage(32, description="SDRAM base for L2 weights (bytes).")
        self._b2_ptr = CSRStorage(32, description="SDRAM base for L2 bias (bytes).")

        # Input vector write (CSR mode).
        self._in_addr = CSRStorage(bits_for(tile_in_max - 1), description="Input index.")
        self._in_data = CSRStorage(8, description="Input value (int8).")
        self._in_we = CSRStorage(description="Write strobe for input.")

        # Layer1 weight matrix write (CSR mode): addr[15:8]=hidden, addr[7:0]=in.
        self._w1_addr = CSRStorage(16, description="L1 weight address: [15:8]=hidden, [7:0]=in.")
        self._w1_data = CSRStorage(8, description="L1 weight value (int8).")
        self._w1_we = CSRStorage(description="Write strobe for L1 weight.")

        # Layer1 bias write (CSR mode).
        self._b1_addr = CSRStorage(bits_for(hidden_size_max - 1), description="L1 bias index.")
        self._b1_data = CSRStorage(16, description="L1 bias value (int16).")
        self._b1_we = CSRStorage(description="Write strobe for L1 bias.")

        # Layer2 weight matrix write (CSR mode): addr[15:8]=out, addr[7:0]=hidden.
        self._w2_addr = CSRStorage(16, description="L2 weight address: [15:8]=out, [7:0]=hidden.")
        self._w2_data = CSRStorage(8, description="L2 weight value (int8).")
        self._w2_we = CSRStorage(description="Write strobe for L2 weight.")

        # Layer2 bias write (CSR mode).
        self._b2_addr = CSRStorage(bits_for(out_size_max - 1), description="L2 bias index.")
        self._b2_data = CSRStorage(16, description="L2 bias value (int16).")
        self._b2_we = CSRStorage(description="Write strobe for L2 bias.")

        # Output read.
        self._out_addr = CSRStorage(bits_for(out_size_max - 1), description="Output index.")
        self._out_data = CSRStatus(32, description="Output value (int32, ReLU applied).")

        # Wishbone master for SDRAM reads.
        self.wb = wishbone.Interface()

        # Memories/buffers.
        in_mem = Array(Signal((8, True), reset=0) for _ in range(tile_in_max))
        w1_mem = Array(
            Array(Signal((8, True), reset=0) for _ in range(tile_in_max))
            for _ in range(hidden_size_max)
        )
        b1_mem = Array(Signal((16, True), reset=0) for _ in range(hidden_size_max))
        h_accum = Array(Signal((32, True), reset=0) for _ in range(hidden_size_max))
        h_mem = Array(Signal((16, True), reset=0) for _ in range(hidden_size_max))

        w2_mem = Array(
            Array(Signal((8, True), reset=0) for _ in range(hidden_size_max))
            for _ in range(out_size_max)
        )
        b2_mem = Array(Signal((16, True), reset=0) for _ in range(out_size_max))
        out_mem = Array(Signal((32, True), reset=0) for _ in range(out_size_max))

        # CSR write paths (small models).
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

        # Runtime sizes.
        in_size_r = Signal(16)
        hidden_size_r = Signal(16)
        out_size_r = Signal(16)
        tile_in_r = Signal(16)

        # Compute indices.
        in_idx = Signal(bits_for(tile_in_max - 1))
        h_idx = Signal(bits_for(hidden_size_max - 1))
        out_idx = Signal(bits_for(out_size_max - 1))
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

        # Loader state.
        rd_word = Signal(32)
        byte_phase = Signal(2)
        byte_count = Signal(32)
        wb_addr = Signal(32)
        byte_val = Signal(8)
        self.comb += byte_val.eq(
            Mux(byte_phase == 0, rd_word[0:8],
                Mux(byte_phase == 1, rd_word[8:16],
                    Mux(byte_phase == 2, rd_word[16:24], rd_word[24:32]))))

        # Tile control.
        tile_base = Signal(32)   # byte offset into input
        tile_done = Signal()

        # W1 row control.
        w1_h = Signal(bits_for(hidden_size_max - 1))
        w1_i = Signal(bits_for(tile_in_max - 1))
        w1_row_base = Signal(32)

        # W2 load control.
        w2_o = Signal(bits_for(out_size_max - 1))
        w2_h = Signal(bits_for(hidden_size_max - 1))

        # Bias load control.
        b1_idx = Signal(bits_for(hidden_size_max - 1))
        b2_idx = Signal(bits_for(out_size_max - 1))
        b1_half = Signal()
        b2_half = Signal()
        b1_lo = Signal(8)
        b2_lo = Signal(8)

        # Wishbone defaults.
        self.comb += [
            self.wb.cyc.eq(0),
            self.wb.stb.eq(0),
            self.wb.we.eq(0),
            self.wb.adr.eq(wb_addr),
            self.wb.sel.eq(0xF),
            self.wb.dat_w.eq(0),
        ]

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        self.sync += [
            If(self._ctrl.storage[1], done.eq(0)),
        ]

        # --- Control ---
        fsm.act("IDLE",
            If(self._ctrl.storage[0] & ~busy,
                NextValue(busy, 1),
                NextValue(done, 0),
                NextValue(in_size_r, self._in_size.storage),
                NextValue(hidden_size_r, self._hidden_size.storage),
                NextValue(out_size_r, self._out_size.storage),
                NextValue(tile_in_r, self._tile_in.storage),
                If(self._ctrl.storage[2],
                    NextValue(byte_count, 0),
                    NextValue(byte_phase, 0),
                    NextValue(b1_idx, 0),
                    NextValue(b1_half, 0),
                    NextValue(wb_addr, self._b1_ptr.storage[2:]),
                    NextState("B1_RD"),
                ).Else(
                    NextValue(h_idx, 0),
                    NextState("H_INIT"),
                ),
            ),
        )

        # --- Initialize h_accum from b1 (CSR mode) ---
        fsm.act("H_INIT",
            NextValue(h_accum[h_idx], b1_mem[h_idx]),
            If(h_idx == (hidden_size_r - 1),
                NextValue(tile_base, 0),
                NextState("TILE_SETUP"),
            ).Else(
                NextValue(h_idx, h_idx + 1),
            ),
        )

        # --- Load B1 from SDRAM ---
        fsm.act("B1_RD",
            self.wb.cyc.eq(1),
            self.wb.stb.eq(1),
            If(self.wb.ack,
                NextValue(rd_word, self.wb.dat_r),
                NextValue(byte_phase, 0),
                NextState("B1_BYTES"),
            ),
        )

        fsm.act("B1_BYTES",
            If(byte_count == ((hidden_size_r * 2) - 1),
                If(b1_half,
                    NextValue(b1_mem[b1_idx], Cat(b1_lo, byte_val)),
                    NextValue(b1_half, 0),
                ).Else(
                    NextValue(b1_lo, byte_val),
                    NextValue(b1_half, 1),
                ),
                NextValue(byte_count, 0),
                NextValue(byte_phase, 0),
                NextValue(w2_o, 0),
                NextValue(w2_h, 0),
                NextValue(wb_addr, self._w2_ptr.storage[2:]),
                NextState("W2_RD"),
            ).Else(
                If(b1_half,
                    NextValue(b1_mem[b1_idx], Cat(b1_lo, byte_val)),
                    NextValue(b1_half, 0),
                    NextValue(b1_idx, b1_idx + 1),
                ).Else(
                    NextValue(b1_lo, byte_val),
                    NextValue(b1_half, 1),
                ),
                NextValue(byte_count, byte_count + 1),
                If(byte_phase == 3,
                    NextValue(byte_phase, 0),
                    NextValue(wb_addr, wb_addr + 1),
                    NextState("B1_RD"),
                ).Else(
                    NextValue(byte_phase, byte_phase + 1),
                ),
            ),
        )

        # --- Load W2 (full) from SDRAM ---
        fsm.act("W2_RD",
            self.wb.cyc.eq(1),
            self.wb.stb.eq(1),
            If(self.wb.ack,
                NextValue(rd_word, self.wb.dat_r),
                NextValue(byte_phase, 0),
                NextState("W2_BYTES"),
            ),
        )

        fsm.act("W2_BYTES",
            If(byte_count == ((out_size_r * hidden_size_r) - 1),
                NextValue(w2_mem[w2_o][w2_h], byte_val),
                NextValue(byte_count, 0),
                NextValue(byte_phase, 0),
                NextValue(b2_idx, 0),
                NextValue(b2_half, 0),
                NextValue(wb_addr, self._b2_ptr.storage[2:]),
                NextState("B2_RD"),
            ).Else(
                NextValue(w2_mem[w2_o][w2_h], byte_val),
                NextValue(byte_count, byte_count + 1),
                If(w2_h == (hidden_size_r - 1),
                    NextValue(w2_h, 0),
                    NextValue(w2_o, w2_o + 1),
                ).Else(
                    NextValue(w2_h, w2_h + 1),
                ),
                If(byte_phase == 3,
                    NextValue(byte_phase, 0),
                    NextValue(wb_addr, wb_addr + 1),
                    NextState("W2_RD"),
                ).Else(
                    NextValue(byte_phase, byte_phase + 1),
                ),
            ),
        )

        # --- Load B2 from SDRAM ---
        fsm.act("B2_RD",
            self.wb.cyc.eq(1),
            self.wb.stb.eq(1),
            If(self.wb.ack,
                NextValue(rd_word, self.wb.dat_r),
                NextValue(byte_phase, 0),
                NextState("B2_BYTES"),
            ),
        )

        fsm.act("B2_BYTES",
            If(byte_count == ((out_size_r * 2) - 1),
                If(b2_half,
                    NextValue(b2_mem[b2_idx], Cat(b2_lo, byte_val)),
                    NextValue(b2_half, 0),
                ).Else(
                    NextValue(b2_lo, byte_val),
                    NextValue(b2_half, 1),
                ),
                NextValue(byte_count, 0),
                NextValue(byte_phase, 0),
                NextValue(h_idx, 0),
                NextState("H_INIT"),
            ).Else(
                If(b2_half,
                    NextValue(b2_mem[b2_idx], Cat(b2_lo, byte_val)),
                    NextValue(b2_half, 0),
                    NextValue(b2_idx, b2_idx + 1),
                ).Else(
                    NextValue(b2_lo, byte_val),
                    NextValue(b2_half, 1),
                ),
                NextValue(byte_count, byte_count + 1),
                If(byte_phase == 3,
                    NextValue(byte_phase, 0),
                    NextValue(wb_addr, wb_addr + 1),
                    NextState("B2_RD"),
                ).Else(
                    NextValue(byte_phase, byte_phase + 1),
                ),
            ),
        )

        # --- Tile setup ---
        fsm.act("TILE_SETUP",
            NextValue(byte_count, 0),
            NextValue(byte_phase, 0),
            NextValue(wb_addr, (self._in_ptr.storage + tile_base)[2:]),
            NextState("IN_RD"),
        )

        # --- Load input tile ---
        fsm.act("IN_RD",
            self.wb.cyc.eq(1),
            self.wb.stb.eq(1),
            If(self.wb.ack,
                NextValue(rd_word, self.wb.dat_r),
                NextValue(byte_phase, 0),
                NextState("IN_BYTES"),
            ),
        )

        fsm.act("IN_BYTES",
            If(byte_count == (tile_in_r - 1),
                NextValue(in_mem[byte_count], byte_val),
                NextValue(byte_count, 0),
                NextValue(byte_phase, 0),
                NextValue(w1_h, 0),
                NextValue(w1_i, 0),
                NextValue(w1_row_base, self._w1_ptr.storage + (w1_h * in_size_r) + tile_base),
                NextValue(wb_addr, (self._w1_ptr.storage + (w1_h * in_size_r) + tile_base)[2:]),
                NextState("W1_RD"),
            ).Else(
                NextValue(in_mem[byte_count], byte_val),
                NextValue(byte_count, byte_count + 1),
                If(byte_phase == 3,
                    NextValue(byte_phase, 0),
                    NextValue(wb_addr, wb_addr + 1),
                    NextState("IN_RD"),
                ).Else(
                    NextValue(byte_phase, byte_phase + 1),
                ),
            ),
        )

        # --- Load W1 tile (row by row) ---
        fsm.act("W1_RD",
            self.wb.cyc.eq(1),
            self.wb.stb.eq(1),
            If(self.wb.ack,
                NextValue(rd_word, self.wb.dat_r),
                NextValue(byte_phase, 0),
                NextState("W1_BYTES"),
            ),
        )

        fsm.act("W1_BYTES",
            NextValue(w1_mem[w1_h][w1_i], byte_val),
            If(w1_i == (tile_in_r - 1),
                If(w1_h == (hidden_size_r - 1),
                    NextValue(h_idx, 0),
                    NextValue(in_idx, 0),
                    NextValue(acc, h_accum[0]),
                    NextState("L1_MAC"),
                ).Else(
                    NextValue(w1_h, w1_h + 1),
                    NextValue(w1_i, 0),
                    NextValue(byte_phase, 0),
                    NextValue(wb_addr, (self._w1_ptr.storage + ((w1_h + 1) * in_size_r) + tile_base)[2:]),
                    NextState("W1_RD"),
                ),
            ).Else(
                NextValue(w1_i, w1_i + 1),
                If(byte_phase == 3,
                    NextValue(byte_phase, 0),
                    NextValue(wb_addr, wb_addr + 1),
                    NextState("W1_RD"),
                ).Else(
                    NextValue(byte_phase, byte_phase + 1),
                ),
            ),
        )

        # --- L1 compute for current tile ---
        fsm.act("L1_MAC",
            NextValue(acc, acc + (in0 * w10) + (in1 * w11)),
            If(in_idx == (tile_in_r - 2),
                NextState("L1_STORE"),
            ).Else(
                NextValue(in_idx, in_idx + 2),
            ),
        )

        fsm.act("L1_STORE",
            NextValue(h_accum[h_idx], acc),
            If(h_idx == (hidden_size_r - 1),
                NextState("TILE_NEXT"),
            ).Else(
                NextValue(h_idx, h_idx + 1),
                NextValue(in_idx, 0),
                NextValue(acc, h_accum[h_idx + 1]),
                NextState("L1_MAC"),
            ),
        )

        # --- Tile iteration ---
        self.comb += tile_done.eq((tile_base + tile_in_r) >= in_size_r)
        fsm.act("TILE_NEXT",
            If(tile_done,
                NextValue(h_idx, 0),
                NextState("H_ACT"),
            ).Else(
                NextValue(tile_base, tile_base + tile_in_r),
                NextState("TILE_SETUP"),
            ),
        )

        # --- Activation for hidden ---
        fsm.act("H_ACT",
            NextValue(h_mem[h_idx], Mux(h_accum[h_idx] < 0, 0, h_accum[h_idx][0:16])),
            If(h_idx == (hidden_size_r - 1),
                NextValue(out_idx, 0),
                NextValue(in_idx, 0),
                NextValue(acc, b2_mem[0]),
                NextState("L2_MAC"),
            ).Else(
                NextValue(h_idx, h_idx + 1),
            ),
        )

        # --- L2 compute (full, no tiling) ---
        fsm.act("L2_MAC",
            NextValue(acc, acc + (h0 * w20) + (h1 * w21)),
            If(in_idx == (hidden_size_r - 2),
                NextState("L2_STORE"),
            ).Else(
                NextValue(in_idx, in_idx + 2),
            ),
        )

        fsm.act("L2_STORE",
            NextValue(out_mem[out_idx], relu32),
            If(out_idx == (out_size_r - 1),
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
