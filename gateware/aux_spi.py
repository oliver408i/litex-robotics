import os

from migen import *

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSR


class AuxSPIMaster(LiteXModule, AutoCSR):
    """Firmware-driven SPI master for the shared sensor/aux bus.

    Wraps verilog/spi/SPI_Master_Var.v -- a runtime-divider variant of the
    nandland SPI_Master the LCD uses -- and adds the things that core lacks: a
    CSR interface, software-held active-low chip-selects, and a runtime SCLK
    divider so each device on the shared bus (WINC1500, LSM6DS3, MCP3008) can
    run at its own clock.

    Lives entirely in the `sys` clock domain (no CDC). Max SCLK is sys/2
    (25 MHz at 50 MHz sys), which covers every device on this bus.

    One byte per transfer, full duplex. CS is held by firmware across a
    multi-byte transaction (set `cs`, run the bytes, clear `cs`).

      SCLK = sys_clk / (2 * clk_divider),  clk_divider >= 1.

    CSR map (accessors shown for instance name "aux_spi"):
      clk_divider : aux_spi_clk_divider_write(n)  SCLK half-bit period, sys cycles (>=1).
      cs          : aux_spi_cs_write(mask)         bit i asserts cs_n[i] LOW; 0 => all idle.
      mosi        : aux_spi_mosi_write(b)          byte to clock out on next start.
      start       : aux_spi_start_write(1)         launch one 8-bit transfer (write-pulse).
      status      : aux_spi_status_read() & 1      1 = ready (idle / transfer complete).
      miso        : aux_spi_miso_read()            byte clocked in on the last transfer.
    """
    def __init__(self, pads, platform, sys_clk_freq,
                 default_spi_clk_freq=1e6, div_width=8):
        self.pads = pads
        ncs = len(pads.cs_n)

        default_div = int(round(sys_clk_freq / (2 * default_spi_clk_freq)))
        default_div = max(1, min(default_div, (1 << div_width) - 1))

        self._clk_divider = CSRStorage(div_width, reset=default_div, description=
            "SCLK half-bit period in sys-clock cycles (>=1). "
            "SCLK = sys_clk/(2*clk_divider).")
        self._cs     = CSRStorage(ncs, description=
            "One bit per device; bit i drives cs_n[i] LOW. 0 => all deasserted.")
        self._mosi   = CSRStorage(8, description="Byte to transmit on the next start.")
        self._start  = CSR()  # write-pulse: launch one 8-bit full-duplex transfer
        self._status = CSRStatus(1, description="bit0 = ready (idle / transfer complete).")
        self._miso   = CSRStatus(8, description="Byte received on the last transfer.")

        # Convenience: high while a transfer is in flight (for a status LED).
        self.busy = Signal()

        # ---- SPI engine -------------------------------------------------
        tx_ready = Signal()
        rx_dv    = Signal()   # unused: we poll `ready` rather than rx_dv
        rx_byte  = Signal(8)
        spi_clk  = Signal()
        spi_mosi = Signal()

        verilog_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "verilog"))
        platform.add_source(os.path.join(verilog_root, "spi", "SPI_Master_Var.v"))

        self.specials += Instance("SPI_Master_Var",
            p_SPI_MODE  = 0,
            p_DIV_W     = div_width,
            i_i_Rst_L   = ~ResetSignal("sys"),
            i_i_Clk     = ClockSignal("sys"),
            i_i_clks_per_half_bit = self._clk_divider.storage,
            i_i_TX_Byte = self._mosi.storage,
            i_i_TX_DV   = self._start.re,
            o_o_TX_Ready = tx_ready,
            o_o_RX_DV    = rx_dv,
            o_o_RX_Byte  = rx_byte,
            o_o_SPI_Clk  = spi_clk,
            i_i_SPI_MISO = pads.miso,
            o_o_SPI_MOSI = spi_mosi,
        )

        self.comb += [
            self._status.status.eq(tx_ready),
            self._miso.status.eq(rx_byte),
            self.busy.eq(~tx_ready),
            # Shared bus output lines.
            pads.clk.eq(spi_clk),
            pads.mosi.eq(spi_mosi),
            # Active-low chip-selects: cs bit set => that cs_n line LOW.
            pads.cs_n.eq(~self._cs.storage),
        ]
