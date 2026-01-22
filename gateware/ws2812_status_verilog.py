import os

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage


class WS2812StatusVerilog(LiteXModule, AutoCSR):
    def __init__(self, pad, sys_clk_freq, platform, led_count=150):
        self._neo_en = CSRStorage(reset=1, description="Enable NeoPixel manual override.")
        self._neo_brightness = CSRStorage(8, reset=255, description="NeoPixel brightness (0-255).")
        self._neo_color_hi = CSRStorage(16, reset=0x00ff, description="NeoPixel GRB high bytes (G,R).")
        self._neo_color_lo = CSRStorage(8, reset=0x00, description="NeoPixel B byte.")
        self._neo_activity = CSRStorage(description="Write to pulse activity.")
        self._strip_index = CSRStorage(16, reset=0, description="NeoPixel strip LED index (0=LED1).")
        self._strip_color_hi = CSRStorage(16, reset=0x0000, description="Strip LED GRB high bytes (G,R).")
        self._strip_color_lo = CSRStorage(8, reset=0x00, description="Strip LED B byte.")
        self._strip_write = CSRStorage(description="Write to update strip LED at current index.")

        activity_pulse = Signal()
        override_en = Signal()
        override_color = Signal(24)
        override_brightness = Signal(8)
        strip_index = Signal(16)
        strip_color = Signal(24)
        strip_write = Signal()

        boot_override = Signal(reset=1)
        blink = Signal(reset=0)
        blink_ctr_max = max(2, int(sys_clk_freq // 4))
        blink_ctr = Signal(max=blink_ctr_max)

        self.sync += [
            If(blink_ctr == (blink_ctr_max - 1),
                blink_ctr.eq(0),
                blink.eq(~blink)
            ).Else(
                blink_ctr.eq(blink_ctr + 1)
            ),
            If(self._neo_en.re | self._neo_brightness.re | self._neo_color_hi.re |
               self._neo_color_lo.re | self._neo_activity.re,
                boot_override.eq(0)
            )
        ]

        self.comb += [
            activity_pulse.eq(self._neo_activity.re),
            If(boot_override,
                override_en.eq(1),
                override_brightness.eq(Mux(blink, 64, 0)),
                override_color.eq(0x00ff00),
            ).Else(
                override_en.eq(self._neo_en.storage),
                override_brightness.eq(self._neo_brightness.storage),
                override_color.eq(Cat(self._neo_color_lo.storage, self._neo_color_hi.storage)),
            )
        ]
        self.comb += [
            strip_index.eq(self._strip_index.storage),
            strip_color.eq(Cat(self._strip_color_lo.storage, self._strip_color_hi.storage)),
            strip_write.eq(self._strip_write.re),
        ]

        verilog_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "verilog", "ws2812.v"))
        platform.add_source(verilog_path)

        self.specials += Instance(
            "status_ws2812_strip",
            p_CLK_HZ=int(sys_clk_freq),
            p_LED_COUNT=int(led_count),
            i_clk_g=ClockSignal("sys"),
            i_activity_pulse=activity_pulse,
            i_override_en=override_en,
            i_override_color_grb=override_color,
            i_override_brightness=override_brightness,
            i_strip_write=strip_write,
            i_strip_index=strip_index,
            i_strip_color_grb=strip_color,
            o_ws2812_din=pad,
        )
