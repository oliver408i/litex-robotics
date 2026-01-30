from migen import *

from litex.gen import *


class ShiftRegister595(LiteXModule):
    def __init__(self, pads, data):

        shift_word = Signal(8, reset=0)
        bit_index = Signal(3, reset=0)
        phase = Signal(reset=0)
        ser = Signal(reset=0)
        srclk = Signal(reset=0)
        rclk = Signal(reset=0)

        self.sync += [
            rclk.eq(0),
            If(phase == 0,
                srclk.eq(0),
                ser.eq(shift_word[7]),
                phase.eq(1),
            ).Else(
                srclk.eq(1),
                phase.eq(0),
                shift_word.eq(shift_word << 1),
                If(bit_index == 7,
                    bit_index.eq(0),
                    rclk.eq(1),
                    shift_word.eq(data),
                ).Else(
                    bit_index.eq(bit_index + 1),
                ),
            ),
        ]

        self.comb += [
            pads.ser.eq(ser),
            pads.srclk.eq(srclk),
            pads.rclk.eq(rclk),
        ]
