import math

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStorage


def _ceil_cycles(sys_clk_freq, seconds):
    return int(math.ceil(sys_clk_freq * seconds))


class WS2812OneLED(LiteXModule):
    def __init__(self, sys_clk_freq):
        self.color_grb = Signal(24)
        self.start = Signal()
        self.busy = Signal(reset=0)
        self.dout = Signal(reset=0)

        t0h = max(2, _ceil_cycles(sys_clk_freq, 0.35e-6))
        t1h = max(3, _ceil_cycles(sys_clk_freq, 0.70e-6))
        tbit = max(6, _ceil_cycles(sys_clk_freq, 1.25e-6))
        tres = max(1, _ceil_cycles(sys_clk_freq, 300e-6))

        sending = Signal(reset=0)
        shreg = Signal(24)
        bit_idx = Signal(6)
        cyc = Signal(bits_for(tbit))
        res_cnt = Signal(bits_for(tres + 1))
        cur_bit = Signal()
        hi_lim = Signal(bits_for(max(t0h, t1h) + 1))

        self.comb += [
            cur_bit.eq(shreg[23]),
            hi_lim.eq(Mux(cur_bit, t1h, t0h)),
        ]

        self.sync += [
            If(~sending,
                self.dout.eq(0),
                If(~self.busy & self.start,
                    shreg.eq(self.color_grb),
                    bit_idx.eq(0),
                    cyc.eq(0),
                    sending.eq(1),
                    self.busy.eq(1),
                ).Elif(self.busy,
                    If(res_cnt != 0,
                        res_cnt.eq(res_cnt - 1)
                    ).Else(
                        self.busy.eq(0)
                    )
                )
            ).Else(
                self.dout.eq(cyc < hi_lim),
                If(cyc == (tbit - 1),
                    cyc.eq(0),
                    If(bit_idx == 23,
                        sending.eq(0),
                        self.dout.eq(0),
                        res_cnt.eq(tres),
                    ).Else(
                        shreg.eq(Cat(0, shreg[0:23])),
                        bit_idx.eq(bit_idx + 1)
                    )
                ).Else(
                    cyc.eq(cyc + 1)
                )
            )
        ]


class WS2812Status(LiteXModule, AutoCSR):
    def __init__(self, pad, sys_clk_freq, flash_ms=80, idle_ms=1000, blink_hz=0.5):
        self._neo_en = CSRStorage(description="Enable NeoPixel manual override.")
        self._neo_brightness = CSRStorage(8, reset=0, description="NeoPixel brightness (0-255).")
        self._neo_color_hi = CSRStorage(16, reset=0, description="NeoPixel GRB high bytes (G,R).")
        self._neo_color_lo = CSRStorage(8, reset=0, description="NeoPixel B byte.")
        self._neo_activity = CSRStorage(description="Write to pulse activity.")

        flash_cycles = max(1, int(sys_clk_freq * flash_ms / 1000))
        idle_cycles = max(1, int(sys_clk_freq * idle_ms / 1000))
        blink_cycles = max(1, int(sys_clk_freq / (2.0 * blink_hz)))

        flash_cnt = Signal(32)
        idle_cnt = Signal(32)
        blink_cnt = Signal(32)
        blink = Signal(reset=0)

        activity_pulse = Signal()
        self.comb += activity_pulse.eq(self._neo_activity.re)

        self.sync += [
            If(activity_pulse,
                flash_cnt.eq(flash_cycles),
                idle_cnt.eq(0),
            ).Else(
                If(flash_cnt != 0,
                    flash_cnt.eq(flash_cnt - 1)
                ),
                If(idle_cnt < idle_cycles,
                    idle_cnt.eq(idle_cnt + 1)
                )
            ),
            If(blink_cnt >= (blink_cycles - 1),
                blink_cnt.eq(0),
                blink.eq(~blink)
            ).Else(
                blink_cnt.eq(blink_cnt + 1)
            )
        ]

        override_en = Signal()
        override_color = Signal(24)
        policy_color = Signal(24)
        chosen_color = Signal(24)
        brightness = Signal(8)

        g = Signal(8)
        r = Signal(8)
        b = Signal(8)

        scaled_g = Signal(8)
        scaled_r = Signal(8)
        scaled_b = Signal(8)

        g_mul = Signal(16)
        r_mul = Signal(16)
        b_mul = Signal(16)

        self.comb += [
            override_en.eq(self._neo_en.storage),
            override_color.eq(Cat(self._neo_color_lo.storage, self._neo_color_hi.storage)),
            policy_color.eq(Mux(flash_cnt != 0, 0x0A0000, Mux((idle_cnt >= idle_cycles) & blink, 0x060800, 0x000000))),
            chosen_color.eq(Mux(override_en, override_color, policy_color)),
            brightness.eq(Mux(override_en, self._neo_brightness.storage, 0xFF)),
            g.eq(chosen_color[16:24]),
            r.eq(chosen_color[8:16]),
            b.eq(chosen_color[0:8]),
            g_mul.eq(g * brightness),
            r_mul.eq(r * brightness),
            b_mul.eq(b * brightness),
            scaled_g.eq(g_mul[8:16]),
            scaled_r.eq(r_mul[8:16]),
            scaled_b.eq(b_mul[8:16])
        ]

        color_scaled = Signal(24)
        self.comb += color_scaled.eq(Cat(scaled_b, scaled_r, scaled_g))

        self.submodules.driver = WS2812OneLED(sys_clk_freq)
        self.comb += [
            self.driver.color_grb.eq(color_scaled),
            self.driver.start.eq(~self.driver.busy),
            pad.eq(self.driver.dout),
        ]
