#!/usr/bin/env python3
"""LiteX wrapper for the SNN-MLP MNIST inference core.

Instantiates verilog/snn_mlp_core.v + verilog/snn_weight_loader.v together
and exposes them as a LiteX peripheral with:

- control / status CSRs
- pixel write port (host writes 784 Q4.12 pixel values into the core's BRAM)
- bias write port (host writes HIDDEN + OUT_SIZE Q4.12 bias values once)
- weight base pointer + per-cycle/cycle-count CSRs (configured once)
- spike-count readback (per-output 8-bit counters, packed)
- Wishbone master to SDRAM for streaming the weight blob

Per-image flow from the firmware:
    write 784 pixels via pixel_addr / pixel_data / pixel_ctl.we
    pulse control.clear_state (resets membrane between images)
    pulse control.start (kicks core + loader together)
    poll status.done
    read status.classification and spike_counts
"""
from __future__ import annotations

import os

from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import AutoCSR, CSRField, CSRStatus, CSRStorage


class SNNMLP(LiteXModule, AutoCSR):
    """Trainable-weight LIF SNN-MLP, sized for 784->64->10 MNIST by default."""

    def __init__(
        self,
        platform,
        in_size: int = 784,
        hidden: int = 64,
        out_size: int = 10,
        timesteps: int = 25,
        n_mac: int = 1,
        data_width: int = 16,
        frac_bits: int = 12,
        beta_shift: int = 3,
        spk_width: int = 8,
    ):
        # Verilog sources --------------------------------------------------
        repo_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
        for vfile in ("verilog/snn_mlp_core.v", "verilog/snn_weight_loader.v"):
            platform.add_source(os.path.join(repo_root, vfile))

        self.n_mac = n_mac
        self.out_size = out_size
        self.spk_width = spk_width

        # --- CSRs ---------------------------------------------------------
        self.control = CSRStorage(fields=[
            CSRField("start", size=1, pulse=True,
                     description="Pulse to begin inference. Triggers core + loader together."),
            CSRField("clear_state", size=1, pulse=True,
                     description="Pulse to clear membrane / spike state (between images)."),
        ])

        self.status = CSRStatus(fields=[
            CSRField("busy", size=1, description="snn_mlp_core busy"),
            CSRField("done", size=1, description="snn_mlp_core inference complete"),
            CSRField("loader_busy", size=1, description="snn_weight_loader busy"),
            CSRField("loader_done", size=1, description="snn_weight_loader completed"),
            CSRField("classification_valid", size=1),
            CSRField("classification", size=4, description="argmax of output spike counts"),
        ])

        # Weight stream layout — configured once at boot.
        self.weight_base = CSRStorage(
            32, description="SDRAM byte address of the packed weight blob (one timestep)."
        )
        self.weight_beats_per_cycle = CSRStorage(
            32, description="Beats consumed per timestep (host fills from pack tool)."
        )
        self.weight_num_cycles = CSRStorage(
            32, description="Number of timesteps to replay the weight blob (T)."
        )

        # Pixel and bias write ports — host writes addr then data then pulses we.
        self.pixel_ctl = CSRStorage(fields=[
            CSRField("we", size=1, pulse=True,
                     description="Pulse to commit pixel_addr -> pixel_mem[pixel_addr] = pixel_data."),
        ])
        self.pixel_addr = CSRStorage(16)
        self.pixel_data = CSRStorage(data_width)

        self.bias_ctl = CSRStorage(fields=[
            CSRField("we", size=1, pulse=True,
                     description="Pulse to commit bias_addr -> bias_mem[bias_addr] = bias_data."),
        ])
        self.bias_addr = CSRStorage(16)
        self.bias_data = CSRStorage(data_width)

        # Spike count readback: one CSRStatus per output neuron. Keeps each
        # accessor a clean single-word read from the firmware side and
        # sidesteps multi-word CSR endianness on the packed alternative.
        for i in range(out_size):
            name = f"spike_count_{i}"
            setattr(self, name, CSRStatus(spk_width, name=name,
                description=f"Spike count for output neuron {i} after the last inference."))

        # --- Wishbone master to SDRAM ------------------------------------
        # LiteX wishbone is word-addressed; the loader speaks byte addresses
        # internally, so we drop the bottom 2 bits below.
        self.wb = wishbone.Interface(data_width=32, adr_width=30)

        # --- Shared signals between loader and core ----------------------
        w_valid_int = Signal()
        w_ready_int = Signal()
        w_data_int  = Signal(n_mac * data_width)

        loader_busy = Signal()
        loader_done = Signal()
        loader_wb_adr_byte = Signal(32)

        core_busy = Signal()
        core_done = Signal()
        cls = Signal(4)
        cls_valid = Signal()
        spk_packed = Signal(out_size * spk_width)

        # --- Instantiate the loader --------------------------------------
        self.specials += Instance(
            "snn_weight_loader",
            p_WB_DATA_WIDTH = 32,
            p_WB_ADDR_WIDTH = 32,
            p_N_MAC         = n_mac,
            p_DATA_WIDTH    = data_width,

            i_clk = ClockSignal(),
            i_rst = ResetSignal(),

            i_start            = self.control.fields.start,
            i_base_addr        = self.weight_base.storage,
            i_beats_per_cycle  = self.weight_beats_per_cycle.storage,
            i_num_cycles       = self.weight_num_cycles.storage,
            o_busy             = loader_busy,
            o_done             = loader_done,

            o_wb_cyc   = self.wb.cyc,
            o_wb_stb   = self.wb.stb,
            o_wb_we    = self.wb.we,
            o_wb_adr   = loader_wb_adr_byte,
            o_wb_dat_w = self.wb.dat_w,
            o_wb_sel   = self.wb.sel,
            i_wb_ack   = self.wb.ack,
            i_wb_dat_r = self.wb.dat_r,

            o_w_valid = w_valid_int,
            i_w_ready = w_ready_int,
            o_w_data  = w_data_int,
        )

        # Byte -> word address (drop the 2 LSBs for the 32-bit-data bus).
        self.comb += self.wb.adr.eq(loader_wb_adr_byte[2:])

        # --- Instantiate the core ----------------------------------------
        self.specials += Instance(
            "snn_mlp_core",
            p_IN_SIZE    = in_size,
            p_HIDDEN     = hidden,
            p_OUT_SIZE   = out_size,
            p_TIMESTEPS  = timesteps,
            p_N_MAC      = n_mac,
            p_DATA_WIDTH = data_width,
            p_FRAC_BITS  = frac_bits,
            p_BETA_SHIFT = beta_shift,
            p_SPK_WIDTH  = spk_width,

            i_clk = ClockSignal(),
            i_rst = ResetSignal(),

            i_start       = self.control.fields.start,
            i_clear_state = self.control.fields.clear_state,
            o_busy        = core_busy,
            o_done        = core_done,

            i_pixel_we   = self.pixel_ctl.fields.we,
            i_pixel_addr = self.pixel_addr.storage,
            i_pixel_data = self.pixel_data.storage,
            i_bias_we    = self.bias_ctl.fields.we,
            i_bias_addr  = self.bias_addr.storage,
            i_bias_data  = self.bias_data.storage,

            i_w_valid = w_valid_int,
            o_w_ready = w_ready_int,
            i_w_data  = w_data_int,

            o_classification        = cls,
            o_classification_valid  = cls_valid,
            o_spike_counts_packed   = spk_packed,
        )

        # --- Status routing ----------------------------------------------
        self.comb += [
            self.status.fields.busy.eq(core_busy),
            self.status.fields.done.eq(core_done),
            self.status.fields.loader_busy.eq(loader_busy),
            self.status.fields.loader_done.eq(loader_done),
            self.status.fields.classification_valid.eq(cls_valid),
            self.status.fields.classification.eq(cls),
        ]
        for i in range(out_size):
            csr = getattr(self, f"spike_count_{i}")
            self.comb += csr.status.eq(spk_packed[i*spk_width:(i+1)*spk_width])
