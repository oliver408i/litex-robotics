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

    def __init__(self, pads, ctrl_pads, platform, sclk_div=1,
                 with_boot_splash=False, sys_clk_freq=50e6,
                 splash_init_addr=0, splash_image_addr=0):
        # Slow / non-per-transfer pads, still firmware-driven.
        self.pads_ctrl = CSRStorage(fields=[
            CSRField("reset_n",   size=1, offset=0, reset=1, description="LCD reset#, active low."),
            CSRField("backlight", size=1, offset=1, reset=0, description="Backlight enable."),
        ])
        self.cmd_byte      = CSRStorage(8,  reset_less=True, description="Command byte (sent with DC=0).")
        self.dma_src       = CSRStorage(32, description="DMA byte-address source.")
        self.dma_row_bytes = CSRStorage(24, description=
                             "Bytes per row of useful payload data.")
        self.dma_row_count = CSRStorage(16, reset=1, description=
                             "Number of rows. 1 = contiguous run (back-compat with old dma_len).")
        self.dma_stride    = CSRStorage(24, description=
                             "Bytes from start-of-row to start-of-next-row in source memory. "
                             "Ignored when dma_row_count == 1.")
        self.fill_color    = CSRStorage(16, description="Fill color, MSB first on wire.")
        self.fill_count    = CSRStorage(24, description="Fill pixel count (2 bytes each).")
        self.rect_x        = CSRStorage(32, description="Rect cols: (x1<<16) | x0.")
        self.rect_y        = CSRStorage(32, description="Rect rows: (y1<<16) | y0.")
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
            CSRField("boot_done",  size=1, offset=2,
                     description="HW boot-splash sequencer has finished (panel up, "
                                 "splash painted). Always 0 when the boot-splash "
                                 "feature is not built in."),
        ])

        # Op-completion event. One pulse per completed op (after the last
        # sub-frame of a RECT op, or after a CMD/CMD_DATA op finishes its
        # only sub-frame). Lets firmware run an IRQ-driven flush_ready
        # path (e.g. for LVGL) instead of polling.
        self.ev = EventManager()
        self.ev.done = EventSourcePulse(description="Op completed.")
        self.ev.finalize()

        # ---- HW boot-splash ownership ----------------------------------
        # When built in (with_boot_splash), the sequencer at the end of this
        # module brings the panel up and paints a splash straight from SPI
        # flash at reset, before the CPU runs. While it owns the engine
        # (boot_active) it drives reset_n / backlight and the op interface;
        # firmware hands control back by writing boot_ctl.release.
        #
        # With the feature disabled boot_active is a constant 0, so every mux
        # below collapses to the plain CSR path -- the engine is byte-for-byte
        # the original.
        boot_active   = Signal()
        seq_reset_n   = Signal(reset=1)
        seq_backlight = Signal(reset=0)
        boot_done     = Signal()
        engine_busy   = Signal()
        if with_boot_splash:
            self.boot_ctl = CSRStorage(fields=[
                CSRField("release", size=1, offset=0, description=
                         "Firmware writes 1 to hand the engine back from the boot "
                         "sequencer (panel stays lit; CPU ops take over)."),
            ])
            self.comb += boot_active.eq(~self.boot_ctl.fields.release)
        else:
            self.comb += boot_active.eq(0)
        self.comb += self.status.fields.boot_done.eq(boot_done)

        self.comb += [
            ctrl_pads.reset_n.eq(Mux(boot_active, seq_reset_n,
                                     self.pads_ctrl.fields.reset_n)),
            ctrl_pads.backlight.eq(Mux(boot_active, seq_backlight,
                                       self.pads_ctrl.fields.backlight)),
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
        a_kind          = Signal(3)
        a_cmd_byte      = Signal(8)
        a_dma_src       = Signal(32)
        a_dma_row_bytes = Signal(24)
        a_dma_row_count = Signal(16)
        a_dma_stride    = Signal(24)
        a_fill_color    = Signal(16)
        a_fill_count    = Signal(24)
        a_rect_x        = Signal(32)
        a_rect_y        = Signal(32)

        q_kind          = Signal(3)
        q_cmd_byte      = Signal(8)
        q_dma_src       = Signal(32)
        q_dma_row_bytes = Signal(24)
        q_dma_row_count = Signal(16)
        q_dma_stride    = Signal(24)
        q_fill_color    = Signal(16)
        q_fill_count    = Signal(24)
        q_rect_x        = Signal(32)
        q_rect_y        = Signal(32)
        q_valid         = Signal()

        q_load    = Signal()   # this cycle: latch op inputs into queue
        q_consume = Signal()   # this cycle: FSM is promoting queue -> active

        # Sequencer-driven op inputs (only meaningful while boot_active; left
        # at 0 otherwise). The boot sequencer at the end of this module drives
        # these to replay panel init + paint the splash.
        seq_start         = Signal()
        seq_kind          = Signal(3)
        seq_cmd_byte      = Signal(8)
        seq_dma_src       = Signal(32)
        seq_dma_row_bytes = Signal(24)
        seq_dma_row_count = Signal(16)
        seq_dma_stride    = Signal(24)
        seq_rect_x        = Signal(32)
        seq_rect_y        = Signal(32)

        # Effective op inputs: the boot sequencer while it owns the engine,
        # otherwise the CSRs (the normal CPU path). boot_active is a constant
        # 0 when the splash is not built in, so these collapse to the CSRs.
        eff_start         = Signal()
        eff_kind          = Signal(3)
        eff_cmd_byte      = Signal(8)
        eff_dma_src       = Signal(32)
        eff_dma_row_bytes = Signal(24)
        eff_dma_row_count = Signal(16)
        eff_dma_stride    = Signal(24)
        eff_rect_x        = Signal(32)
        eff_rect_y        = Signal(32)
        self.comb += [
            eff_start.eq        (Mux(boot_active, seq_start,         self.op.fields.start)),
            eff_kind.eq         (Mux(boot_active, seq_kind,          self.op.fields.kind)),
            eff_cmd_byte.eq     (Mux(boot_active, seq_cmd_byte,      self.cmd_byte.storage)),
            eff_dma_src.eq      (Mux(boot_active, seq_dma_src,       self.dma_src.storage)),
            eff_dma_row_bytes.eq(Mux(boot_active, seq_dma_row_bytes, self.dma_row_bytes.storage)),
            eff_dma_row_count.eq(Mux(boot_active, seq_dma_row_count, self.dma_row_count.storage)),
            eff_dma_stride.eq   (Mux(boot_active, seq_dma_stride,    self.dma_stride.storage)),
            eff_rect_x.eq       (Mux(boot_active, seq_rect_x,        self.rect_x.storage)),
            eff_rect_y.eq       (Mux(boot_active, seq_rect_y,        self.rect_y.storage)),
        ]

        # Capture op inputs into the queue slot when start fires and the slot
        # is (or is becoming) free. If start lands on the same cycle the FSM
        # consumes the queue, the new op atomically refills the slot. The boot
        # sequencer never issues FILL ops, so fill_* always come from the CSRs.
        self.comb += q_load.eq(
            eff_start
          & (eff_kind != KIND_IDLE)
          & (~q_valid | q_consume)
        )
        self.sync += [
            If(q_load,
                q_valid.eq(1),
                q_kind.eq(eff_kind),
                q_cmd_byte.eq(eff_cmd_byte),
                q_dma_src.eq(eff_dma_src),
                q_dma_row_bytes.eq(eff_dma_row_bytes),
                q_dma_row_count.eq(eff_dma_row_count),
                q_dma_stride.eq(eff_dma_stride),
                q_fill_color.eq(self.fill_color.storage),
                q_fill_count.eq(self.fill_count.storage),
                q_rect_x.eq(eff_rect_x),
                q_rect_y.eq(eff_rect_y),
            ).Elif(q_consume,
                q_valid.eq(0),
            )
        ]

        # Op-completion pulse: 1 sys cycle when the engine finishes the
        # last sub-frame of an op. Drives the EventManager source.
        op_done = Signal()
        self.comb += self.ev.done.trigger.eq(op_done)

        # ---- Per-op transient state -------------------------------------
        frame_phase       = Signal(2)   # 0=CASET, 1=RASET, 2=RAMWR (RECT only)
        cur_cmd           = Signal(8)
        cur_ptype         = Signal(2)
        # Two-axis "bytes remaining" so the same FSM serves FILL/INLINE
        # (single contiguous run) and strided DMA (row_count rows of
        # row_bytes, with stride bytes between row starts in source mem).
        # Done = (tx_row_remaining == 0) and (tx_rows_remaining == 0).
        tx_row_remaining  = Signal(25)
        tx_rows_remaining = Signal(16)

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
                NextValue(a_kind,          q_kind),
                NextValue(a_cmd_byte,      q_cmd_byte),
                NextValue(a_dma_src,       q_dma_src),
                NextValue(a_dma_row_bytes, q_dma_row_bytes),
                NextValue(a_dma_row_count, q_dma_row_count),
                NextValue(a_dma_stride,    q_dma_stride),
                NextValue(a_fill_color,    q_fill_color),
                NextValue(a_fill_count,    q_fill_count),
                NextValue(a_rect_x,        q_rect_x),
                NextValue(a_rect_y,        q_rect_y),
                NextValue(frame_phase,     0),
                NextValue(gap_count,       0),
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
        # For DMA: tx_row_remaining starts at row_bytes (one row); the
        # remaining rows (row_count - 1) live in tx_rows_remaining and get
        # consumed each time we cross a row boundary in PAYLOAD_DMA.
        # For FILL/INLINE: tx_rows_remaining is 0; tx_row_remaining is the
        # single contiguous byte count.
        fsm.act("FRAME_SETUP",
            NextValue(cs_n_reg, 0),
            NextValue(dc_reg,   0),
            If(a_kind == KIND_CMD,
                NextValue(cur_cmd,   a_cmd_byte),
                NextValue(cur_ptype, PT_NONE),
            ).Elif(a_kind == KIND_CMD_DATA_DMA,
                NextValue(cur_cmd,            a_cmd_byte),
                NextValue(cur_ptype,          PT_DMA),
                NextValue(tx_row_remaining,   a_dma_row_bytes),
                NextValue(tx_rows_remaining,  a_dma_row_count - 1),
                NextValue(addr_byte,          a_dma_src),
                NextValue(word_byte_idx,      a_dma_src[0:2]),
            ).Elif(a_kind == KIND_CMD_DATA_FILL,
                NextValue(cur_cmd,           a_cmd_byte),
                NextValue(cur_ptype,         PT_FILL),
                NextValue(tx_row_remaining,  a_fill_count << 1),
                NextValue(tx_rows_remaining, 0),
                NextValue(fill_msb_pending,  1),
            ).Else(  # FILL_RECT / DMA_RECT
                If(frame_phase == 0,  # CASET
                    NextValue(cur_cmd,            ST7796_CASET),
                    NextValue(cur_ptype,          PT_INLINE),
                    NextValue(tx_row_remaining,   4),
                    NextValue(tx_rows_remaining,  0),
                    NextValue(coord_byte_idx,     0),
                ).Elif(frame_phase == 1,  # RASET
                    NextValue(cur_cmd,            ST7796_RASET),
                    NextValue(cur_ptype,          PT_INLINE),
                    NextValue(tx_row_remaining,   4),
                    NextValue(tx_rows_remaining,  0),
                    NextValue(coord_byte_idx,     0),
                ).Else(  # RAMWR + payload
                    NextValue(cur_cmd, ST7796_RAMWR),
                    If(a_kind == KIND_FILL_RECT,
                        NextValue(cur_ptype,           PT_FILL),
                        NextValue(tx_row_remaining,    a_fill_count << 1),
                        NextValue(tx_rows_remaining,   0),
                        NextValue(fill_msb_pending,    1),
                    ).Else(  # KIND_DMA_RECT
                        NextValue(cur_ptype,          PT_DMA),
                        NextValue(tx_row_remaining,   a_dma_row_bytes),
                        NextValue(tx_rows_remaining,  a_dma_row_count - 1),
                        NextValue(addr_byte,          a_dma_src),
                        NextValue(word_byte_idx,      a_dma_src[0:2]),
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
        # room. On a word boundary mid-row, jump out to DMA_FETCH (the
        # FIFO covers the fetch latency). On a row boundary with more
        # rows queued, advance addr_byte to the next row's start using
        # the stride and refresh the word cache.
        fsm.act("PAYLOAD_DMA",
            tx_byte.eq(cur_word_byte),
            If(tx_row_remaining == 0,
                # All rows exhausted (drained the last byte of the last
                # row on the previous cycle).
                NextState("DRAIN"),
            ).Elif(tx_fifo.writable,
                tx_we.eq(1),
                NextValue(tx_row_remaining, tx_row_remaining - 1),
                If(tx_row_remaining == 1,
                    # This push completes the current row.
                    If(tx_rows_remaining != 0,
                        # More rows to come: jump addr to next row start.
                        # addr_byte currently points at the last byte of
                        # the current row; advancing it by 1 puts us at
                        # row_start + row_bytes, and we want to land at
                        # row_start + stride - so add (stride - row_bytes + 1).
                        NextValue(addr_byte,
                                  addr_byte + a_dma_stride - a_dma_row_bytes + 1),
                        NextValue(word_byte_idx,
                                  (addr_byte + a_dma_stride - a_dma_row_bytes + 1)[0:2]),
                        NextValue(tx_row_remaining,  a_dma_row_bytes),
                        NextValue(tx_rows_remaining, tx_rows_remaining - 1),
                        NextState("DMA_FETCH"),
                    ),
                    # else: last byte of last row, no addr advance needed;
                    # next cycle hits tx_row_remaining == 0 -> DRAIN.
                ).Else(
                    # Mid-row push: normal byte-walk + word-boundary fetch.
                    NextValue(addr_byte, addr_byte + 1),
                    If(word_byte_idx == 3,
                        NextValue(word_byte_idx, 0),
                        NextState("DMA_FETCH"),
                    ).Else(
                        NextValue(word_byte_idx, word_byte_idx + 1),
                    ),
                ),
            )
        )

        # Pipelined fill push. fill_msb_pending flips per push so the
        # color streams out MSB-first across the wire.
        fsm.act("PAYLOAD_FILL",
            tx_byte.eq(Mux(fill_msb_pending,
                           a_fill_color[8:16],
                           a_fill_color[0:8])),
            If(tx_row_remaining == 0,
                NextState("DRAIN"),
            ).Elif(tx_fifo.writable,
                tx_we.eq(1),
                NextValue(tx_row_remaining, tx_row_remaining - 1),
                NextValue(fill_msb_pending, ~fill_msb_pending),
            )
        )

        # Pipelined inline (rect coord) push.
        fsm.act("PAYLOAD_INLINE",
            tx_byte.eq(inline_byte),
            If(tx_row_remaining == 0,
                NextState("DRAIN"),
            ).Elif(tx_fifo.writable,
                tx_we.eq(1),
                NextValue(tx_row_remaining, tx_row_remaining - 1),
                NextValue(coord_byte_idx,   coord_byte_idx + 1),
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
        self.comb += engine_busy.eq(~fsm.ongoing("IDLE") | q_valid)
        self.comb += [
            self.status.fields.busy.eq(engine_busy),
            self.status.fields.can_accept.eq(~q_valid),
        ]

        # ---- HW boot-splash sequencer -----------------------------------
        # Runs once out of reset, no CPU involved: pulse the panel reset,
        # replay the ST7796S init recipe (per-command payload bytes DMA'd
        # from the init region of the flash splash blob), DMA the full-frame
        # splash image from flash, light the backlight, then hold the panel
        # lit in DONE until firmware releases the engine (boot_ctl.release).
        #
        # It drives the same seq_*/queue path the CPU uses via the eff_*
        # muxes above, so the core op FSM is untouched -- nothing here changes
        # the engine's behaviour once firmware has taken over.
        if with_boot_splash:
            from gateware.st7796_boot import (LCD_WIDTH as _W, LCD_HEIGHT as _H,
                                              BYTES_PER_PIXEL as _BPP, init_table)
            cyc_per_ms = int(sys_clk_freq // 1000)
            table   = init_table()
            n_steps = len(table)

            tbl_cmd    = Array(Constant(c, 8)  for (c, l, o, d) in table)
            tbl_len    = Array(Constant(l, 8)  for (c, l, o, d) in table)
            tbl_off    = Array(Constant(o, 16) for (c, l, o, d) in table)
            tbl_dlycyc = Array(Constant(d * cyc_per_ms, 32) for (c, l, o, d) in table)

            step     = Signal(max=n_steps + 1)
            dly      = Signal(32)
            has_data = Signal()
            self.comb += has_data.eq(tbl_len[step] != 0)

            img_row_bytes = _W * _BPP

            self.boot_fsm = bfsm = FSM(reset_state="RST_H0")

            # Panel reset pulse: high (20 ms) -> low (20 ms) -> high (120 ms),
            # mirroring lcd_hw_reset() in software/common/lcd.c.
            bfsm.act("RST_H0",
                NextValue(seq_reset_n, 1),
                NextValue(dly, 20 * cyc_per_ms),
                NextState("RST_H0_W"),
            )
            bfsm.act("RST_H0_W",
                NextValue(dly, dly - 1),
                If(dly == 0, NextState("RST_LO")),
            )
            bfsm.act("RST_LO",
                NextValue(seq_reset_n, 0),
                NextValue(dly, 20 * cyc_per_ms),
                NextState("RST_LO_W"),
            )
            bfsm.act("RST_LO_W",
                NextValue(dly, dly - 1),
                If(dly == 0, NextState("RST_H1")),
            )
            bfsm.act("RST_H1",
                NextValue(seq_reset_n, 1),
                NextValue(dly, 120 * cyc_per_ms),
                NextState("RST_H1_W"),
            )
            bfsm.act("RST_H1_W",
                NextValue(dly, dly - 1),
                If(dly == 0,
                    NextValue(step, 0),
                    NextState("INIT_ISSUE"),
                ),
            )

            # Issue one init command (with optional flash-sourced payload),
            # wait for it to complete, then honour the recipe's post-delay.
            bfsm.act("INIT_ISSUE",
                If(step == n_steps,
                    NextState("IMG_ISSUE"),
                ).Else(
                    seq_cmd_byte.eq(tbl_cmd[step]),
                    If(has_data,
                        seq_kind.eq(KIND_CMD_DATA_DMA),
                        seq_dma_src.eq(splash_init_addr + tbl_off[step]),
                        seq_dma_row_bytes.eq(tbl_len[step]),
                        seq_dma_row_count.eq(1),
                    ).Else(
                        seq_kind.eq(KIND_CMD),
                    ),
                    seq_start.eq(1),
                    NextState("INIT_GO"),
                ),
            )
            bfsm.act("INIT_GO", If(engine_busy, NextState("INIT_DONE")))
            bfsm.act("INIT_DONE",
                If(~engine_busy,
                    NextValue(dly, tbl_dlycyc[step]),
                    NextState("INIT_DLY"),
                ),
            )
            bfsm.act("INIT_DLY",
                If(dly == 0,
                    NextValue(step, step + 1),
                    NextState("INIT_ISSUE"),
                ).Else(
                    NextValue(dly, dly - 1),
                ),
            )

            # Full-frame splash image, DMA'd straight from flash.
            bfsm.act("IMG_ISSUE",
                seq_kind.eq(KIND_DMA_RECT),
                seq_rect_x.eq(((_W - 1) << 16) | 0),
                seq_rect_y.eq(((_H - 1) << 16) | 0),
                seq_dma_src.eq(splash_image_addr),
                seq_dma_row_bytes.eq(img_row_bytes),
                seq_dma_row_count.eq(_H),
                seq_dma_stride.eq(img_row_bytes),
                seq_start.eq(1),
                NextState("IMG_GO"),
            )
            bfsm.act("IMG_GO", If(engine_busy, NextState("IMG_DONE")))
            bfsm.act("IMG_DONE",
                If(~engine_busy,
                    NextValue(dly, 20 * cyc_per_ms),
                    NextState("BL_DLY"),
                ),
            )
            # Backlight on only after the image has landed, so the reveal is
            # the splash -- not the panel's power-on RAM garbage.
            bfsm.act("BL_DLY",
                NextValue(dly, dly - 1),
                If(dly == 0, NextState("LIT")),
            )
            bfsm.act("LIT",
                NextValue(seq_backlight, 1),
                NextState("DONE"),
            )
            # Terminal: hold panel lit + out of reset, flag boot_done. Stays
            # here (still boot_active) until firmware releases the engine.
            bfsm.act("DONE",
                NextValue(seq_backlight, 1),
                NextValue(seq_reset_n, 1),
                boot_done.eq(1),
            )
