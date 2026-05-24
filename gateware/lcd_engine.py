import os

from migen import *
from migen.genlib.fsm import FSM, NextState, NextValue
from migen.genlib.fifo import AsyncFIFO
from migen.genlib.cdc import PulseSynchronizer

from litex.gen import LiteXModule
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourcePulse


# Op kinds (op.kind field).
KIND_IDLE          = 0
KIND_CMD           = 1   # 1 byte, DC=0, CS framed
KIND_CMD_DATA_DMA  = 2   # cmd byte (DC=0) + DMA payload (DC=1), CS framed
KIND_CMD_DATA_FILL = 3   # cmd byte (DC=0) + fill payload (DC=1), CS framed
KIND_FILL_RECT     = 4   # CASET + RASET + RAMWR(fill), 3 CS frames
KIND_DMA_RECT      = 5   # CASET + RASET + RAMWR(dma),  3 CS frames

# Internal payload-source tag.
PT_NONE   = 0
PT_DMA    = 1
PT_FILL   = 2
PT_INLINE = 3

# ST7796S opcodes baked into the RECT sequencer.
ST7796_CASET = 0x2A
ST7796_RASET = 0x2B
ST7796_RAMWR = 0x2C


class LCDEngine(LiteXModule, AutoCSR):
    """ST7796S LCD engine with HW-driven CS_N / DC framing.

    The FSM owns CS_N and DC. Software issues high-level ops; the engine
    drives the per-byte DC value and frames each transfer with CS_N.
    RECT ops emit the full CASET / RASET / RAMWR sub-frame sequence
    internally, so a `lcd_fill_rect` / `lcd_dma_rect` from firmware is
    just a handful of CSR writes plus one op start.

    SPI shifter still lives in the `spi` clock domain so SCK can outrun
    sys. CDC at the boundary:
      - sys -> spi: AsyncFIFO (depth 2) carries TX bytes.
      - spi -> sys: PulseSynchronizer brings rx_dv to the FSM.

    Rect coord packing:
      rect_x = (x1 << 16) | x0
      rect_y = (y1 << 16) | y0
    """

    def __init__(self, pads, ctrl_pads, platform, sclk_div=1):
        # Slow / non-per-transfer pads, still firmware-driven.
        self.pads_ctrl = CSRStorage(fields=[
            CSRField("reset_n",   size=1, offset=0, reset=1, description="LCD reset#, active low."),
            CSRField("backlight", size=1, offset=1, reset=0, description="Backlight enable."),
        ])
        self.cmd_byte   = CSRStorage(8,  reset_less=True, description="Command byte (sent with DC=0).")
        self.dma_src    = CSRStorage(32, description="DMA byte-address source.")
        self.dma_len    = CSRStorage(24, description="DMA byte length.")
        self.fill_color = CSRStorage(16, description="Fill color, MSB first on wire.")
        self.fill_count = CSRStorage(24, description="Fill pixel count (2 bytes each).")
        self.rect_x     = CSRStorage(32, description="Rect cols: (x1<<16) | x0.")
        self.rect_y     = CSRStorage(32, description="Rect rows: (y1<<16) | y0.")
        self.op = CSRStorage(fields=[
            CSRField("kind",  size=3, offset=0, description=
                     "0=idle, 1=CMD, 2=CMD_DATA_DMA, 3=CMD_DATA_FILL, "
                     "4=FILL_RECT, 5=DMA_RECT."),
            CSRField("start", size=1, offset=8, pulse=True, description="Pulse to launch op."),
        ])
        self.status = CSRStatus(fields=[
            CSRField("busy",       size=1, offset=0,
                     description="Engine has unfinished work (running or queued)."),
            CSRField("can_accept", size=1, offset=1,
                     description="Queue slot free; safe to write CSRs and fire op.start."),
        ])

        # Op-completion event. One pulse per completed op (after the last
        # sub-frame of a RECT op, or after a CMD/CMD_DATA op finishes its
        # only sub-frame). Lets firmware run an IRQ-driven flush_ready
        # path (e.g. for LVGL) instead of polling.
        self.ev = EventManager()
        self.ev.done = EventSourcePulse(description="Op completed.")
        self.ev.finalize()

        self.comb += [
            ctrl_pads.reset_n.eq(self.pads_ctrl.fields.reset_n),
            ctrl_pads.backlight.eq(self.pads_ctrl.fields.backlight),
        ]

        verilog_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "verilog"))
        platform.add_source(os.path.join(verilog_root, "spi", "SPI_host.v"))

        # ---- SPI domain: SPI_Master + FIFO consumer ---------------------
        spi_tx_byte  = Signal(8)
        spi_tx_dv    = Signal()
        spi_tx_ready = Signal()
        spi_rx_dv    = Signal()
        spi_rx_byte  = Signal(8)
        spi_clk_pad  = Signal()
        spi_mosi_pad = Signal()
        self.specials += Instance(
            "SPI_Master",
            p_SPI_MODE          = 0,
            p_CLKS_PER_HALF_BIT = int(sclk_div),
            i_i_Rst_L    = ~ResetSignal("spi"),
            i_i_Clk      = ClockSignal("spi"),
            i_i_TX_Byte  = spi_tx_byte,
            i_i_TX_DV    = spi_tx_dv,
            o_o_TX_Ready = spi_tx_ready,
            o_o_RX_DV    = spi_rx_dv,
            o_o_RX_Byte  = spi_rx_byte,
            o_o_SPI_Clk  = spi_clk_pad,
            i_i_SPI_MISO = pads.miso,
            o_o_SPI_MOSI = spi_mosi_pad,
        )
        self.comb += [
            pads.clk.eq(spi_clk_pad),
            pads.mosi.eq(spi_mosi_pad),
        ]

        # ---- TX byte CDC: AsyncFIFO sys -> spi --------------------------
        # Depth=4 gives the sys-side push loop headroom to keep the SPI
        # shifter fed across DMA word-boundary fetches.
        tx_byte = Signal(8)
        tx_we   = Signal()
        tx_fifo = ClockDomainsRenamer({"write": "sys", "read": "spi"})(
            AsyncFIFO(width=8, depth=4))
        self.submodules.tx_fifo = tx_fifo
        self.comb += [
            tx_fifo.din.eq(tx_byte),
            tx_fifo.we.eq(tx_we),
        ]
        self.comb += [
            spi_tx_byte.eq(tx_fifo.dout),
            spi_tx_dv.eq(tx_fifo.readable & spi_tx_ready),
            tx_fifo.re.eq(tx_fifo.readable & spi_tx_ready),
        ]

        # ---- RX CDC: rx_dv pulse spi -> sys -----------------------------
        rx_dv = Signal()
        ps_rx = PulseSynchronizer("spi", "sys")
        self.submodules.ps_rx = ps_rx
        self.comb += [
            ps_rx.i.eq(spi_rx_dv),
            rx_dv.eq(ps_rx.o),
        ]

        # ---- Wishbone master (sys domain) -------------------------------
        self.bus = wb = wishbone.Interface(data_width=32, adr_width=30, addressing="word")

        # ---- HW-driven framing pads -------------------------------------
        cs_n_reg = Signal(reset=1)
        dc_reg   = Signal(reset=0)
        self.comb += [
            pads.cs_n[0].eq(cs_n_reg),
            ctrl_pads.dc.eq(dc_reg),
        ]

        # ---- Op state (active = running, queue = next op waiting) -------
        # FSM reads only the active (a_*) registers. Live CSR storage is
        # touched exclusively by the q_load capture below, so software is
        # free to overwrite CSRs the moment it has fired op.start.
        a_kind       = Signal(3)
        a_cmd_byte   = Signal(8)
        a_dma_src    = Signal(32)
        a_dma_len    = Signal(24)
        a_fill_color = Signal(16)
        a_fill_count = Signal(24)
        a_rect_x     = Signal(32)
        a_rect_y     = Signal(32)

        q_kind       = Signal(3)
        q_cmd_byte   = Signal(8)
        q_dma_src    = Signal(32)
        q_dma_len    = Signal(24)
        q_fill_color = Signal(16)
        q_fill_count = Signal(24)
        q_rect_x     = Signal(32)
        q_rect_y     = Signal(32)
        q_valid      = Signal()

        q_load    = Signal()   # this cycle: latch CSRs into queue
        q_consume = Signal()   # this cycle: FSM is promoting queue -> active

        # Capture CSRs into the queue slot when op.start fires and the slot
        # is (or is becoming) free. If start lands on the same cycle the
        # FSM consumes the queue, the new op atomically refills the slot.
        self.comb += q_load.eq(
            self.op.fields.start
          & (self.op.fields.kind != KIND_IDLE)
          & (~q_valid | q_consume)
        )
        self.sync += [
            If(q_load,
                q_valid.eq(1),
                q_kind.eq(self.op.fields.kind),
                q_cmd_byte.eq(self.cmd_byte.storage),
                q_dma_src.eq(self.dma_src.storage),
                q_dma_len.eq(self.dma_len.storage),
                q_fill_color.eq(self.fill_color.storage),
                q_fill_count.eq(self.fill_count.storage),
                q_rect_x.eq(self.rect_x.storage),
                q_rect_y.eq(self.rect_y.storage),
            ).Elif(q_consume,
                q_valid.eq(0),
            )
        ]

        # Op-completion pulse: 1 sys cycle when the engine finishes the
        # last sub-frame of an op. Drives the EventManager source.
        op_done = Signal()
        self.comb += self.ev.done.trigger.eq(op_done)

        # ---- Per-op transient state -------------------------------------
        frame_phase  = Signal(2)   # 0=CASET, 1=RASET, 2=RAMWR (RECT only)
        cur_cmd      = Signal(8)
        cur_ptype    = Signal(2)
        tx_remaining = Signal(25)  # payload bytes still to push into the FIFO

        addr_byte        = Signal(32)
        cached_word      = Signal(32)
        word_byte_idx    = Signal(2)
        fill_msb_pending = Signal(reset=1)
        coord_byte_idx   = Signal(2)
        gap_count        = Signal(3)

        is_rect_op = Signal()
        self.comb += is_rect_op.eq((a_kind == KIND_FILL_RECT) | (a_kind == KIND_DMA_RECT))

        # ---- In-flight byte counter -------------------------------------
        # rx_pending = bytes pushed into the FIFO that have not yet
        # completed shifting on SPI. Increments on push (tx_we), decrements
        # on rx_dv. Used by CMD_WAIT (wait for cmd byte to land before
        # toggling DC) and DRAIN (wait for the payload tail to clock out
        # before deasserting CS).
        # Max value: FIFO depth (4) + SPI shifter (1) + a slot of CDC
        # slack = 6. Fits in 3 bits.
        rx_pending = Signal(3)
        self.sync += [
            If(tx_we & ~rx_dv,
                rx_pending.eq(rx_pending + 1),
            ).Elif(~tx_we & rx_dv,
                rx_pending.eq(rx_pending - 1),
            )
        ]

        # ---- Combinational byte-source muxes ----------------------------
        cur_word_byte = Signal(8)
        self.comb += Case(word_byte_idx, {
            0: cur_word_byte.eq(cached_word[ 0: 8]),
            1: cur_word_byte.eq(cached_word[ 8:16]),
            2: cur_word_byte.eq(cached_word[16:24]),
            3: cur_word_byte.eq(cached_word[24:32]),
        })

        # Inline (rect coord) byte source. Wire order: w0 MSB, w0 LSB,
        # w1 MSB, w1 LSB. coord_src picks rect_x in CASET phase, rect_y
        # in RASET phase.
        inline_byte = Signal(8)
        coord_src   = Signal(32)
        self.comb += coord_src.eq(Mux(frame_phase == 0, a_rect_x, a_rect_y))
        self.comb += Case(coord_byte_idx, {
            0: inline_byte.eq(coord_src[ 8:16]),
            1: inline_byte.eq(coord_src[ 0: 8]),
            2: inline_byte.eq(coord_src[24:32]),
            3: inline_byte.eq(coord_src[16:24]),
        })

        # ---- FSM --------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")

        # Promote the queued op into the active slot and start it.
        # Used from both IDLE (first op) and FRAME_END (op done with a
        # queued follow-up). q_consume is comb-asserted so the queue's
        # sync block clears q_valid this cycle, freeing the slot for a
        # concurrent q_load.
        def promote_queue():
            return [
                NextValue(a_kind,       q_kind),
                NextValue(a_cmd_byte,   q_cmd_byte),
                NextValue(a_dma_src,    q_dma_src),
                NextValue(a_dma_len,    q_dma_len),
                NextValue(a_fill_color, q_fill_color),
                NextValue(a_fill_count, q_fill_count),
                NextValue(a_rect_x,     q_rect_x),
                NextValue(a_rect_y,     q_rect_y),
                NextValue(frame_phase,  0),
                NextValue(gap_count,    0),
                q_consume.eq(1),
                NextState("FRAME_SETUP"),
            ]

        fsm.act("IDLE",
            If(q_valid, *promote_queue()),
        )

        # Configure cur_cmd / payload state for the current sub-frame and
        # drive CS_N=0, DC=0 for the upcoming cmd byte. All inputs come
        # from the active (latched) op state; CSR storage is not read
        # from here on, so software may already be staging the next op.
        fsm.act("FRAME_SETUP",
            NextValue(cs_n_reg, 0),
            NextValue(dc_reg,   0),
            If(a_kind == KIND_CMD,
                NextValue(cur_cmd,   a_cmd_byte),
                NextValue(cur_ptype, PT_NONE),
            ).Elif(a_kind == KIND_CMD_DATA_DMA,
                NextValue(cur_cmd,        a_cmd_byte),
                NextValue(cur_ptype,      PT_DMA),
                NextValue(tx_remaining,   a_dma_len),
                NextValue(addr_byte,      a_dma_src),
                NextValue(word_byte_idx,  a_dma_src[0:2]),
            ).Elif(a_kind == KIND_CMD_DATA_FILL,
                NextValue(cur_cmd,           a_cmd_byte),
                NextValue(cur_ptype,         PT_FILL),
                NextValue(tx_remaining,      a_fill_count << 1),
                NextValue(fill_msb_pending,  1),
            ).Else(  # FILL_RECT / DMA_RECT
                If(frame_phase == 0,  # CASET
                    NextValue(cur_cmd,         ST7796_CASET),
                    NextValue(cur_ptype,       PT_INLINE),
                    NextValue(tx_remaining,    4),
                    NextValue(coord_byte_idx,  0),
                ).Elif(frame_phase == 1,  # RASET
                    NextValue(cur_cmd,         ST7796_RASET),
                    NextValue(cur_ptype,       PT_INLINE),
                    NextValue(tx_remaining,    4),
                    NextValue(coord_byte_idx,  0),
                ).Else(  # RAMWR + payload
                    NextValue(cur_cmd, ST7796_RAMWR),
                    If(a_kind == KIND_FILL_RECT,
                        NextValue(cur_ptype,         PT_FILL),
                        NextValue(tx_remaining,      a_fill_count << 1),
                        NextValue(fill_msb_pending,  1),
                    ).Else(  # KIND_DMA_RECT
                        NextValue(cur_ptype,      PT_DMA),
                        NextValue(tx_remaining,   a_dma_len),
                        NextValue(addr_byte,      a_dma_src),
                        NextValue(word_byte_idx,  a_dma_src[0:2]),
                    ),
                ),
            ),
            NextState("CMD_PUSH"),
        )

        # Push the cmd byte (DC stays 0).
        fsm.act("CMD_PUSH",
            tx_byte.eq(cur_cmd),
            If(tx_fifo.writable,
                tx_we.eq(1),
                NextState("CMD_WAIT"),
            )
        )

        # Wait for the cmd byte to drain through the SPI shifter (using
        # the in-flight counter, not rx_dv directly, so we never miss a
        # pulse that arrives before this state actually runs). Then jump
        # to the payload phase (DC=1) or end the frame.
        fsm.act("CMD_WAIT",
            If(rx_pending == 0,
                If(cur_ptype == PT_NONE,
                    NextState("FRAME_END"),
                ).Else(
                    NextValue(dc_reg, 1),
                    If(cur_ptype == PT_DMA,
                        NextState("DMA_FETCH"),
                    ).Elif(cur_ptype == PT_FILL,
                        NextState("PAYLOAD_FILL"),
                    ).Else(  # PT_INLINE
                        NextState("PAYLOAD_INLINE"),
                    ),
                ),
            )
        )

        fsm.act("DMA_FETCH",
            wb.cyc.eq(1),
            wb.stb.eq(1),
            wb.adr.eq(addr_byte[2:]),
            wb.sel.eq(0b1111),
            wb.we.eq(0),
            If(wb.ack,
                NextValue(cached_word, wb.dat_r),
                NextState("PAYLOAD_DMA"),
            )
        )

        # Pipelined DMA push: drives one byte per cycle when the FIFO has
        # room. On a word boundary with bytes still to go, jump out to
        # DMA_FETCH (the FIFO covers the fetch latency). When the last
        # byte has been pushed, drop into DRAIN.
        fsm.act("PAYLOAD_DMA",
            tx_byte.eq(cur_word_byte),
            If(tx_remaining == 0,
                NextState("DRAIN"),
            ).Elif(tx_fifo.writable,
                tx_we.eq(1),
                NextValue(tx_remaining, tx_remaining - 1),
                NextValue(addr_byte,    addr_byte + 1),
                If(word_byte_idx == 3,
                    NextValue(word_byte_idx, 0),
                    # Only fetch a new word if there is a byte beyond the
                    # one we just pushed (tx_remaining == 1 means this
                    # push was the last).
                    If(tx_remaining != 1,
                        NextState("DMA_FETCH"),
                    ),
                ).Else(
                    NextValue(word_byte_idx, word_byte_idx + 1),
                ),
            )
        )

        # Pipelined fill push. fill_msb_pending flips per push so the
        # color streams out MSB-first across the wire.
        fsm.act("PAYLOAD_FILL",
            tx_byte.eq(Mux(fill_msb_pending,
                           a_fill_color[8:16],
                           a_fill_color[0:8])),
            If(tx_remaining == 0,
                NextState("DRAIN"),
            ).Elif(tx_fifo.writable,
                tx_we.eq(1),
                NextValue(tx_remaining,     tx_remaining - 1),
                NextValue(fill_msb_pending, ~fill_msb_pending),
            )
        )

        # Pipelined inline (rect coord) push.
        fsm.act("PAYLOAD_INLINE",
            tx_byte.eq(inline_byte),
            If(tx_remaining == 0,
                NextState("DRAIN"),
            ).Elif(tx_fifo.writable,
                tx_we.eq(1),
                NextValue(tx_remaining,    tx_remaining - 1),
                NextValue(coord_byte_idx,  coord_byte_idx + 1),
            )
        )

        # Wait for the last queued byte to finish shifting before we
        # deassert CS. rx_pending counts bytes pushed but not yet rx_dv'd.
        fsm.act("DRAIN",
            If(rx_pending == 0,
                NextState("FRAME_END"),
            )
        )

        # Deassert CS for a short gap. For RECT ops, advance to the next
        # sub-frame; otherwise either promote a queued op straight into
        # the active slot (no IDLE round-trip) or settle back to IDLE.
        # Fires op_done exactly once per completed op.
        fsm.act("FRAME_END",
            NextValue(cs_n_reg, 1),
            NextValue(dc_reg,   0),
            NextValue(gap_count, gap_count + 1),
            If(gap_count == 7,
                NextValue(gap_count, 0),
                If(is_rect_op & (frame_phase != 2),
                    NextValue(frame_phase, frame_phase + 1),
                    NextState("FRAME_SETUP"),
                ).Else(
                    op_done.eq(1),
                    If(q_valid,
                        *promote_queue(),
                    ).Else(
                        NextValue(frame_phase, 0),
                        NextState("IDLE"),
                    ),
                ),
            )
        )

        # busy = engine is running an op OR has one waiting in the queue.
        # can_accept = queue slot is free (safe to stage and start a new op).
        self.comb += [
            self.status.fields.busy.eq(~fsm.ongoing("IDLE") | q_valid),
            self.status.fields.can_accept.eq(~q_valid),
        ]
