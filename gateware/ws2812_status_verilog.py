import os

from migen import Cat, ClockSignal, Instance, Signal

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import AutoCSR, CSRStorage


class WS2812StatusVerilog(LiteXModule, AutoCSR):
    def __init__(self, pad, sys_clk_freq, platform, led_count=300, status_led=False):
        self._neo_en = CSRStorage(reset=1)
        self._neo_brightness = CSRStorage(8, reset=255)
        self._neo_color_hi = CSRStorage(16, reset=0x00FF)
        self._neo_color_lo = CSRStorage(8, reset=0x00)
        self._neo_activity = CSRStorage()
        self._strip_index = CSRStorage(16, reset=0)
        self._strip_color_hi = CSRStorage(16, reset=0)
        self._strip_color_lo = CSRStorage(8, reset=0)
        self._strip_write = CSRStorage()

        override_color = Signal(24)
        strip_color = Signal(24)

        self.comb += [
            override_color.eq(Cat(self._neo_color_lo.storage, self._neo_color_hi.storage)),
            strip_color.eq(Cat(self._strip_color_lo.storage, self._strip_color_hi.storage)),
        ]

        verilog_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "verilog", "ws2812.v")
        )
        platform.add_source(verilog_path)

        self.specials += Instance(
            "status_ws2812_strip",
            p_CLK_HZ=int(sys_clk_freq),
            p_LED_COUNT=int(led_count),
            p_STATUS_LED=int(1 if status_led else 0),
            i_clk_g=ClockSignal("sys"),
            i_activity_pulse=self._neo_activity.re,
            i_override_en=self._neo_en.storage,
            i_override_color_grb=override_color,
            i_override_brightness=self._neo_brightness.storage,
            i_strip_write=self._strip_write.re,
            i_strip_index=self._strip_index.storage,
            i_strip_color_grb=strip_color,
            o_ws2812_din=pad,
        )
