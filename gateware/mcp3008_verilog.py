import os

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus


class MCP3008ReaderVerilog(LiteXModule, AutoCSR):
    def __init__(self, pads, sys_clk_freq, platform, sclk_div=16, default_interval_ms=10):
        self._enable = CSRStorage(reset=1, description="Enable MCP3008 sampling.")
        self._channel_mask = CSRStorage(8, reset=0xFF, description="Channel mask (bit0=CH0..bit7=CH7).")
        self._sample_interval = CSRStorage(32, reset=int(sys_clk_freq * default_interval_ms / 1000),
                                           description="Sample interval ticks (0=continuous).")
        self._clear_update = CSRStorage(8, description="Write mask to clear update bits.")

        self._busy = CSRStatus(description="Sampler busy flag.")
        self._update_mask = CSRStatus(8, description="Channel update flags.")
        self._last_channel = CSRStatus(3, description="Last sampled channel.")
        self._sample_count = CSRStatus(32, description="Total samples captured.")
        self._sample_ch0 = CSRStatus(16, description="Channel 0 raw sample.")
        self._sample_ch1 = CSRStatus(16, description="Channel 1 raw sample.")
        self._sample_ch2 = CSRStatus(16, description="Channel 2 raw sample.")
        self._sample_ch3 = CSRStatus(16, description="Channel 3 raw sample.")
        self._sample_ch4 = CSRStatus(16, description="Channel 4 raw sample.")
        self._sample_ch5 = CSRStatus(16, description="Channel 5 raw sample.")
        self._sample_ch6 = CSRStatus(16, description="Channel 6 raw sample.")
        self._sample_ch7 = CSRStatus(16, description="Channel 7 raw sample.")

        clear_mask = Signal(8)
        self.comb += clear_mask.eq(Mux(self._clear_update.re, self._clear_update.storage, 0))

        verilog_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "verilog"))
        platform.add_source(os.path.join(verilog_root, "spi", "SPI_host.v"))
        platform.add_source(os.path.join(verilog_root, "spi", "mcp3008_reader.v"))

        busy = Signal()
        update_mask = Signal(8)
        last_channel = Signal(3)
        sample_count = Signal(32)
        sample_ch = [Signal(10) for _ in range(8)]

        self.specials += Instance(
            "mcp3008_reader",
            p_SCLK_DIV=int(sclk_div),
            i_clk=ClockSignal("sys"),
            i_rstn=~ResetSignal("sys"),
            i_enable=self._enable.storage,
            i_channel_mask=self._channel_mask.storage,
            i_sample_interval_ticks=self._sample_interval.storage,
            i_clear_update_mask=clear_mask,
            o_busy=busy,
            o_update_mask=update_mask,
            o_last_channel=last_channel,
            o_sample_count=sample_count,
            o_adc_sclk=pads.sclk,
            o_adc_cs_n=pads.cs_n,
            o_adc_mosi=pads.mosi,
            i_adc_miso=pads.miso,
            o_sample_ch0=sample_ch[0],
            o_sample_ch1=sample_ch[1],
            o_sample_ch2=sample_ch[2],
            o_sample_ch3=sample_ch[3],
            o_sample_ch4=sample_ch[4],
            o_sample_ch5=sample_ch[5],
            o_sample_ch6=sample_ch[6],
            o_sample_ch7=sample_ch[7],
        )

        self.comb += [
            self._busy.status.eq(busy),
            self._update_mask.status.eq(update_mask),
            self._last_channel.status.eq(last_channel),
            self._sample_count.status.eq(sample_count),
            self._sample_ch0.status.eq(sample_ch[0]),
            self._sample_ch1.status.eq(sample_ch[1]),
            self._sample_ch2.status.eq(sample_ch[2]),
            self._sample_ch3.status.eq(sample_ch[3]),
            self._sample_ch4.status.eq(sample_ch[4]),
            self._sample_ch5.status.eq(sample_ch[5]),
            self._sample_ch6.status.eq(sample_ch[6]),
            self._sample_ch7.status.eq(sample_ch[7]),
        ]
