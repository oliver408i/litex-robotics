"""SPI-slave FIFO peripheral for the ESP32-C3 flash-loader link.

The FPGA is the SPI *slave*; the ESP32-C3 (master) clocks command + data frames
in and reads status/CRC back. This is the dumb transport half of the hybrid C3
loader (see docs/c3_loader.md): it shifts bytes in/out on CS framing into
firmware-drained FIFOs, mirroring the role gateware/aux_spi.py plays on the
master side. All protocol/flash logic lives in firmware.

Lives entirely in the `sys` clock domain. The SPI inputs (SCLK/MOSI/CS#) are
asynchronous, so they pass through MultiReg synchronizers and the SCLK edges are
oversampled -- valid as long as SCLK <= ~sys/4 (~12 MHz at 50 MHz sys), which is
far above the flash-program-bound throughput ceiling. No separate clock domain.

SPI mode 0 (CPOL=0, CPHA=0), MSB-first, 8 bits/byte, CS# active-low.

CSR map (accessors shown for instance name "c3"):
  rxtx    : c3_rxtx_read()    pop one byte the master clocked in (RX FIFO).
            c3_rxtx_write(b)   push one byte to clock back out on MISO (TX FIFO).
  rxempty : c3_rxempty_read() & 1   1 = no byte waiting in the RX FIFO.
  txfull  : c3_txfull_read()  & 1   1 = TX FIFO full.
  ready   : c3_ready_write(1)  drive the BUSY/READY pad high (= ready to accept).
            Firmware lowers it (busy) across a flash erase/program so the C3
            pauses; the dedicated IO4 line is the flow-control handshake. Resets
            to 0 (busy) so the master waits until the loader firmware is up.
  status  : c3_status_read()  & 1   bit0 = CS# currently asserted.
"""
from migen import *
from migen.genlib.fifo import SyncFIFO
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSR


class SPISlave(LiteXModule, AutoCSR):
    def __init__(self, pads, fifo_depth=512):
        self.pads = pads

        self._rxtx    = CSR(8)
        self._rxempty = CSRStatus(1, description="1 = RX FIFO empty.")
        self._txfull  = CSRStatus(1, description="1 = TX FIFO full.")
        self._ready   = CSRStorage(1, reset=0, description=
            "Drives the BUSY/READY handshake pad. 1 = ready to accept; 0 = busy "
            "(master must wait). Resets 0 (busy) until firmware is up.")
        self._status  = CSRStatus(1, description="bit0 = CS# asserted.")

        self.submodules.rx_fifo = rx_fifo = SyncFIFO(8, fifo_depth)
        # TX FIFO is deliberately shallow (8 bytes).  Protocol max reply is 5
        # bytes (PING / CRC).  More importantly, depth-8 stays below the EBR
        # threshold so Yosys synthesises it as LUT RAM -- which has a truly
        # async read port.  A 512-deep EBR with async_read=True can still be
        # synthesised with REGMODE=OUTREG by Yosys; the output register then
        # only updates when re fires, so dout=0 during the inter-frame preload
        # window and tx_shift never gets the first byte.  LUT RAM avoids this.
        self.submodules.tx_fifo = tx_fifo = SyncFIFO(8, 8)

        # ---- async SPI inputs -> sys domain ------------------------------
        sclk = Signal(); mosi = Signal(); cs_n = Signal(reset=1)
        self.specials += MultiReg(pads.sclk, sclk)
        self.specials += MultiReg(pads.mosi, mosi)
        self.specials += MultiReg(pads.cs_n, cs_n, reset=1)

        cs_active = Signal()
        self.comb += cs_active.eq(~cs_n)

        sclk_d = Signal(); cs_d = Signal()
        self.sync += [sclk_d.eq(sclk), cs_d.eq(cs_active)]
        sclk_rise = Signal(); sclk_fall = Signal(); cs_rise = Signal()
        self.comb += [
            sclk_rise.eq(sclk & ~sclk_d),
            sclk_fall.eq(~sclk & sclk_d),
            cs_rise.eq(cs_active & ~cs_d),
        ]

        bitcnt   = Signal(3)
        rx_shift = Signal(8)
        tx_shift = Signal(8)
        byte_stb = Signal()
        self.comb += byte_stb.eq(cs_active & sclk_rise & (bitcnt == 7))

        # ---- RX: sample MOSI on the rising edge, MSB first ---------------
        # rx_shift left-shifts the new bit into bit0, so after 8 shifts bit7 is
        # the first (MSB) bit. The completed byte is the current shift reg with
        # the 8th bit appended -- captured combinationally at byte_stb.
        self.sync += [
            If(~cs_active,
                bitcnt.eq(0),
            ).Elif(sclk_rise,
                rx_shift.eq(Cat(mosi, rx_shift[:7])),
                If(bitcnt == 7, bitcnt.eq(0)).Else(bitcnt.eq(bitcnt + 1)),
            ),
        ]

        # ---- TX: drive MISO from the shift register --------------------------
        # TX FIFO is LUT RAM (depth=8), so dout = memory[rd_ptr] immediately
        # (true async read, no REGMODE=OUTREG latency).
        # Preload: while CS# is deasserted, continuously copy tx_fifo.dout into
        # tx_shift so the first byte is ready before cs_rise.  The first pop
        # happens at cs_rise; subsequent pops at each byte boundary (sclk_fall,
        # bitcnt==0).  In-byte falling edges just shift MSB-first.
        tx_load = Signal()
        self.comb += tx_load.eq(cs_rise | (cs_active & sclk_fall & (bitcnt == 0)))
        self.sync += [
            If(~cs_active & tx_fifo.readable,
                tx_shift.eq(tx_fifo.dout),               # preload while idle
            ).Elif(cs_active & sclk_fall & (bitcnt == 0),
                tx_shift.eq(tx_fifo.dout),               # byte boundary: next byte
            ).Elif(cs_active & sclk_fall & (bitcnt != 0),
                tx_shift.eq(Cat(0, tx_shift[:7])),        # in-byte shift
            ),
        ]

        self.comb += [
            # RX FIFO <- shift register; CSR read pops it.
            rx_fifo.din.eq(Cat(mosi, rx_shift[:7])),
            rx_fifo.we.eq(byte_stb & rx_fifo.writable),
            self._rxtx.w.eq(rx_fifo.dout),
            rx_fifo.re.eq(self._rxtx.re),
            self._rxempty.status.eq(~rx_fifo.readable),
            # TX FIFO <- CSR write; shift register pops it at tx_load.
            tx_fifo.din.eq(self._rxtx.r),
            tx_fifo.we.eq(self._rxtx.we),
            tx_fifo.re.eq(tx_load & tx_fifo.readable),
            self._txfull.status.eq(~tx_fifo.writable),
            # Pads + status.
            pads.miso.eq(Mux(cs_active, tx_shift[7], 0)),
            pads.ready.eq(self._ready.storage),
            self._status.status.eq(cs_active),
        ]
