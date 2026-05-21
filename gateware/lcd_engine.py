import os

from migen import *
from migen.genlib.fsm import FSM, NextState, NextValue
from migen.genlib.fifo import AsyncFIFO
from migen.genlib.cdc import PulseSynchronizer

from litex.gen import LiteXModule
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField


class LCDEngine(LiteXModule, AutoCSR):
    """ST7796S LCD engine: SPI master + CS/DC/RESET/backlight + DMA/fill.

    The SPI shifter lives in its own clock domain (`spi`) so SCK can run
    faster than the SoC sys clock. CDC at the boundary:
      - sys -> spi:  AsyncFIFO (depth 2) carries TX bytes.
      - spi -> sys:  PulseSynchronizer brings rx_dv ("byte done") to the FSM.

    Three byte-stream ops, all driven by the sys-domain FSM:
      kind=1 SINGLE - send one byte from the `byte` CSR
      kind=2 DMA    - stream `dma_len` bytes from `dma_src` (any byte addr)
      kind=3 FILL   - send `fill_color` (high byte first) `fill_count` times

    Software owns CS via pads_ctrl so multi-byte commands stay framed by a
    single CS pulse.
    """

    def __init__(self, pads, ctrl_pads, platform, sclk_div=1):
        self.pads_ctrl = CSRStorage(fields=[
            CSRField("cs_n",      size=1, offset=0, reset=1, description="CS#, active low."),
            CSRField("dc",        size=1, offset=1, reset=0, description="DC: 0=cmd, 1=data."),
            CSRField("reset_n",   size=1, offset=2, reset=1, description="LCD reset#, active low."),
            CSRField("backlight", size=1, offset=3, reset=0, description="Backlight enable."),
        ])
        self.byte       = CSRStorage(8,  reset_less=True, description="SINGLE-op byte.")
        self.dma_src    = CSRStorage(32, description="DMA byte-address source.")
        self.dma_len    = CSRStorage(24, description="DMA byte length.")
        self.fill_color = CSRStorage(16, description="Fill color, MSB first.")
        self.fill_count = CSRStorage(24, description="Fill pixel count (2 bytes each).")
        self.op = CSRStorage(fields=[
            CSRField("kind",  size=2, offset=0, description="0=idle, 1=SINGLE, 2=DMA, 3=FILL."),
            CSRField("start", size=1, offset=8, pulse=True, description="Pulse to launch op."),
        ])
        self.status = CSRStatus(fields=[
            CSRField("busy", size=1, offset=0, description="Engine running."),
        ])

        self.comb += [
            pads.cs_n[0].eq(self.pads_ctrl.fields.cs_n),
            ctrl_pads.dc.eq(self.pads_ctrl.fields.dc),
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
        tx_byte = Signal(8)
        tx_we   = Signal()
        tx_fifo = ClockDomainsRenamer({"write": "sys", "read": "spi"})(
            AsyncFIFO(width=8, depth=2))
        self.submodules.tx_fifo = tx_fifo
        self.comb += [
            tx_fifo.din.eq(tx_byte),
            tx_fifo.we.eq(tx_we),
        ]
        # Spi-side consumer: pop FIFO when SPI shifter is ready
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

        # ---- FSM state (sys domain) -------------------------------------
        kind             = Signal(2)
        next_byte        = Signal(8)
        bytes_left       = Signal(25)
        addr_byte        = Signal(32)
        cached_word      = Signal(32)
        word_byte_idx    = Signal(2)
        fill_msb_pending = Signal(reset=1)

        cur_word_byte = Signal(8)
        self.comb += Case(word_byte_idx, {
            0: cur_word_byte.eq(cached_word[ 0: 8]),
            1: cur_word_byte.eq(cached_word[ 8:16]),
            2: cur_word_byte.eq(cached_word[16:24]),
            3: cur_word_byte.eq(cached_word[24:32]),
        })

        self.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(self.op.fields.start,
                NextValue(kind, self.op.fields.kind),
                If(self.op.fields.kind == 1,
                    NextValue(next_byte,   self.byte.storage),
                    NextValue(bytes_left,  1),
                    NextState("XFER"),
                ).Elif(self.op.fields.kind == 2,
                    NextValue(addr_byte,      self.dma_src.storage),
                    NextValue(bytes_left,     self.dma_len.storage),
                    NextValue(word_byte_idx,  self.dma_src.storage[0:2]),
                    NextState("DMA_FETCH"),
                ).Elif(self.op.fields.kind == 3,
                    NextValue(bytes_left,        self.fill_count.storage << 1),
                    NextValue(fill_msb_pending,  1),
                    NextState("FILL_FEED"),
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
                NextState("DMA_FEED"),
            )
        )

        fsm.act("DMA_FEED",
            NextValue(next_byte, cur_word_byte),
            NextState("XFER"),
        )

        fsm.act("FILL_FEED",
            If(fill_msb_pending,
                NextValue(next_byte, self.fill_color.storage[8:16]),
            ).Else(
                NextValue(next_byte, self.fill_color.storage[0:8]),
            ),
            NextState("XFER"),
        )

        # Push next_byte into the async FIFO. Depth=2 with strictly serial
        # use (one push per rx_dv) means writable is always asserted here.
        fsm.act("XFER",
            tx_byte.eq(next_byte),
            If(tx_fifo.writable,
                tx_we.eq(1),
                NextState("XFER_WAIT"),
            )
        )

        fsm.act("XFER_WAIT",
            If(rx_dv,
                If(bytes_left == 1,
                    NextState("IDLE"),
                ).Else(
                    NextValue(bytes_left, bytes_left - 1),
                    If(kind == 2,
                        NextValue(addr_byte, addr_byte + 1),
                        If(word_byte_idx == 3,
                            NextValue(word_byte_idx, 0),
                            NextState("DMA_FETCH"),
                        ).Else(
                            NextValue(word_byte_idx, word_byte_idx + 1),
                            NextState("DMA_FEED"),
                        ),
                    ).Elif(kind == 3,
                        NextValue(fill_msb_pending, ~fill_msb_pending),
                        NextState("FILL_FEED"),
                    ),
                ),
            )
        )

        self.comb += self.status.fields.busy.eq(~fsm.ongoing("IDLE"))
