#!/usr/bin/env python3
"""LiteX wrapper for the 8-neuron feedforward Verilog LIF debug core."""

from __future__ import annotations

import os

from migen import *

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import AutoCSR, CSRField, CSRStatus, CSRStorage


class SNNTrackingEstimator(LiteXModule, AutoCSR):
    data_width = 16

    def __init__(self, platform):
        verilog_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "verilog",
            "lif_bank_debug.v",
        )
        platform.add_source(verilog_path)

        self.control = CSRStorage(fields=[
            CSRField("start", size=1, description="Pulse high for one sample update."),
            CSRField("clear_state", size=1, description="Pulse high to clear state."),
        ])
        self.weight_control = CSRStorage(fields=[
            CSRField("write", size=1, description="Load weight_data at weight_addr into the local model cache."),
            CSRField("commit", size=1, description="Mark the local model cache ready for inference."),
            CSRField("clear", size=1, description="Clear the local model cache and model-ready flag."),
        ])
        self.weight_addr = CSRStorage(7, reset=0, description="Linear model weight address, 0..117.")
        self.weight_data = CSRStorage(self.data_width, reset=0, description="Signed Q4.12 model weight value.")
        self.measurement = CSRStorage(self.data_width, reset=0, description="Signed Q4.12 input sample.")
        self.input_override = CSRStorage(4, reset=0, description="Externally supplied encoder spike bits.")
        self.status = CSRStatus(fields=[
            CSRField("busy", size=1, description="Core is processing a sample."),
            CSRField("done", size=1, description="A sample has completed."),
            CSRField("model_ready", size=1, description="CPU has committed a complete local model cache."),
            CSRField("phase", size=2, description="0=idle 1=update."),
            CSRField("index", size=6, description="Unused in single-neuron bring-up core."),
        ])
        self.position = CSRStatus(self.data_width, description="Current membrane state (Q4.12).")
        self.velocity = CSRStatus(self.data_width, description="Latched delta input (Q4.12).")
        self.cycles = CSRStatus(16, description="Cycles spent on the last sample.")
        self.debug = CSRStatus(32, description="Packed debug state.")

        self.debug_measurement_raw = CSRStatus(self.data_width)
        self.debug_delta_raw = CSRStatus(self.data_width)
        self.debug_measurement_feature = CSRStatus(self.data_width)
        self.debug_delta_feature = CSRStatus(self.data_width)
        self.debug_membrane0 = CSRStatus(self.data_width)
        self.debug_membrane1 = CSRStatus(self.data_width)
        self.debug_membrane2 = CSRStatus(self.data_width)
        self.debug_membrane3 = CSRStatus(self.data_width)
        self.debug_membrane4 = CSRStatus(self.data_width)
        self.debug_input_spikes = CSRStatus(4)
        self.debug_beta_product = CSRStatus(self.data_width)
        self.debug_input_sum = CSRStatus(self.data_width)
        self.debug_recurrent_sum = CSRStatus(self.data_width)
        self.debug_membrane_clip = CSRStatus(self.data_width)

        busy = Signal()
        done = Signal()
        model_ready = Signal()
        phase = Signal(2)
        cycles = Signal(16)
        position = Signal((self.data_width, True))
        velocity = Signal((self.data_width, True))
        debug_measurement_raw = Signal((self.data_width, True))
        debug_delta_raw = Signal((self.data_width, True))
        debug_measurement_feature = Signal((self.data_width, True))
        debug_delta_feature = Signal((self.data_width, True))
        debug_membrane0 = Signal((self.data_width, True))
        debug_membrane1 = Signal((self.data_width, True))
        debug_membrane2 = Signal((self.data_width, True))
        debug_membrane3 = Signal((self.data_width, True))
        debug_membrane4 = Signal((self.data_width, True))
        debug_input_spikes = Signal(4)
        debug_beta_product = Signal((self.data_width, True))
        debug_input_sum = Signal((self.data_width, True))
        debug_recurrent_sum = Signal((self.data_width, True))
        debug_membrane_clip = Signal((self.data_width, True))
        debug_packed = Signal(32)

        self.specials += Instance(
            "lif_bank_debug",
            i_clk=ClockSignal(),
            i_rst=ResetSignal(),
            i_start=self.control.fields.start,
            i_clear_state=self.control.fields.clear_state,
            i_weight_write=self.weight_control.fields.write,
            i_weight_commit=self.weight_control.fields.commit,
            i_weight_clear=self.weight_control.fields.clear,
            i_weight_addr=self.weight_addr.storage,
            i_weight_data=self.weight_data.storage,
            i_measurement_in=self.measurement.storage,
            i_input_override=self.input_override.storage,
            o_busy=busy,
            o_done=done,
            o_model_ready=model_ready,
            o_phase=phase,
            o_cycles=cycles,
            o_position_out=position,
            o_velocity_out=velocity,
            o_debug_measurement_raw=debug_measurement_raw,
            o_debug_delta_raw=debug_delta_raw,
            o_debug_measurement_feature=debug_measurement_feature,
            o_debug_delta_feature=debug_delta_feature,
            o_debug_membrane0=debug_membrane0,
            o_debug_membrane1=debug_membrane1,
            o_debug_membrane2=debug_membrane2,
            o_debug_membrane3=debug_membrane3,
            o_debug_membrane4=debug_membrane4,
            o_debug_input_spikes=debug_input_spikes,
            o_debug_beta_product=debug_beta_product,
            o_debug_input_sum=debug_input_sum,
            o_debug_recurrent_sum=debug_recurrent_sum,
            o_debug_membrane_clip=debug_membrane_clip,
            o_debug_packed=debug_packed,
        )

        self.comb += [
            self.status.fields.busy.eq(busy),
            self.status.fields.done.eq(done),
            self.status.fields.model_ready.eq(model_ready),
            self.status.fields.phase.eq(phase),
            self.status.fields.index.eq(0),
            self.position.status.eq(position),
            self.velocity.status.eq(velocity),
            self.cycles.status.eq(cycles),
            self.debug.status.eq(debug_packed),
            self.debug_measurement_raw.status.eq(debug_measurement_raw),
            self.debug_delta_raw.status.eq(debug_delta_raw),
            self.debug_measurement_feature.status.eq(debug_measurement_feature),
            self.debug_delta_feature.status.eq(debug_delta_feature),
            self.debug_membrane0.status.eq(debug_membrane0),
            self.debug_membrane1.status.eq(debug_membrane1),
            self.debug_membrane2.status.eq(debug_membrane2),
            self.debug_membrane3.status.eq(debug_membrane3),
            self.debug_membrane4.status.eq(debug_membrane4),
            self.debug_input_spikes.status.eq(debug_input_spikes),
            self.debug_beta_product.status.eq(debug_beta_product),
            self.debug_input_sum.status.eq(debug_input_sum),
            self.debug_recurrent_sum.status.eq(debug_recurrent_sum),
            self.debug_membrane_clip.status.eq(debug_membrane_clip),
        ]
