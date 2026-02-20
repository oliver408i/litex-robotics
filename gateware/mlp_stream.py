from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage
from litex.soc.interconnect import wishbone


def _sign_extend(sig, width, out_width):
    if out_width <= width:
        return sig
    return Cat(sig, Replicate(sig[width - 1], out_width - width))


class MLPStream(LiteXModule, AutoCSR):
    def __init__(self, max_in=784, max_hid=128, max_out=10, data_bits=16):
        if data_bits != 16:
            raise ValueError("data_bits must be 16 for this implementation")

        self.bus = wishbone.Interface(data_width=32, adr_width=32)

        # Control/Status CSRs.
        self._start    = CSRStorage(description="Write to start MLP inference.")
        self._relu_en  = CSRStorage(1, reset=1, description="Enable ReLU on hidden layer.")
        self._shift    = CSRStorage(5, reset=8, description="Right shift applied to accumulator for output scaling.")
        self._dim_in   = CSRStorage(16, reset=4, description="Input dimension.")
        self._dim_hid  = CSRStorage(16, reset=8, description="Hidden dimension.")
        self._dim_out  = CSRStorage(16, reset=2, description="Output dimension.")

        self._base_in  = CSRStorage(32, description="Base address of input vector (SDRAM).")
        self._base_w1  = CSRStorage(32, description="Base address of W1 (SDRAM).")
        self._base_b1  = CSRStorage(32, description="Base address of B1 (SDRAM).")
        self._base_w2  = CSRStorage(32, description="Base address of W2 (SDRAM).")
        self._base_b2  = CSRStorage(32, description="Base address of B2 (SDRAM).")
        self._base_hid = CSRStorage(32, description="Base address of hidden activations (SDRAM).")
        self._base_out = CSRStorage(32, description="Base address of output vector (SDRAM).")

        self._busy  = CSRStatus(description="MLP engine busy.")
        self._done  = CSRStatus(description="MLP engine done (sticky until next start).")
        self._error = CSRStatus(description="Dimension error (sticky until next start).")
        self._dbg_acc_lo = CSRStatus(32, description="Debug: accumulator low 32 bits.")
        self._dbg_acc_hi = CSRStatus(32, description="Debug: accumulator high 32 bits.")
        self._dbg_w = CSRStatus(16, description="Debug: last weight (signed 16).")
        self._dbg_x = CSRStatus(16, description="Debug: last input/hidden (signed 16).")
        self._dbg_sel = CSRStorage(16, description="Debug: select index for in/hid RAM read.")
        self._dbg_in = CSRStatus(16, description="Debug: in_mem at dbg_sel.")
        self._dbg_hid = CSRStatus(16, description="Debug: hid_mem at dbg_sel.")
        self._dbg_in_idx = CSRStatus(16, description="Debug: current in_idx.")
        self._dbg_hid_idx = CSRStatus(16, description="Debug: current hid_idx.")
        self._dbg_in_x = CSRStatus(16, description="Debug: in_x_reg.")
        self._dbg_hid_x = CSRStatus(16, description="Debug: hid_x_reg.")
        self._dbg_req_addr = CSRStatus(32, description="Debug: last WB request address.")

        # On-chip buffers for inputs and hidden activations.
        in_mem = Memory(data_bits, max_in)
        hid_mem = Memory(data_bits, max_hid)
        in_wr = in_mem.get_port(write_capable=True)
        in_rd = in_mem.get_port(has_re=True)
        hid_wr = hid_mem.get_port(write_capable=True)
        hid_rd = hid_mem.get_port(has_re=True)
        in_dbg = in_mem.get_port(has_re=True)
        hid_dbg = hid_mem.get_port(has_re=True)
        self.specials += in_mem, in_wr, in_rd, in_dbg, hid_mem, hid_wr, hid_rd, hid_dbg

        # Wishbone single-transaction controller.
        wb_cyc = Signal(reset=0)
        wb_stb = Signal(reset=0)
        wb_we  = Signal(reset=0)
        wb_adr = Signal(32)
        wb_dat_w = Signal(32)
        wb_sel = Signal(4, reset=0xF)

        req_valid = Signal()
        req_we    = Signal()
        req_addr  = Signal(32)
        req_wdata = Signal(32)
        req_wsel  = Signal(4)
        resp_valid = Signal()
        resp_rdata = Signal(32)

        bus_idle = Signal()
        self.comb += bus_idle.eq(~wb_cyc)

        self.comb += [
            self.bus.cyc.eq(wb_cyc),
            self.bus.stb.eq(wb_stb),
            self.bus.we.eq(wb_we),
            self.bus.adr.eq(wb_adr[2:]),
            self.bus.dat_w.eq(wb_dat_w),
            self.bus.sel.eq(wb_sel),
        ]

        self.sync += [
            resp_valid.eq(0),
            If(~wb_cyc,
                If(req_valid,
                    wb_cyc.eq(1),
                    wb_stb.eq(1),
                    wb_we.eq(req_we),
                    wb_adr.eq(req_addr),
                    wb_dat_w.eq(req_wdata),
                    wb_sel.eq(req_wsel),
                )
            ).Else(
                If(self.bus.ack,
                    wb_cyc.eq(0),
                    wb_stb.eq(0),
                    resp_valid.eq(1),
                    resp_rdata.eq(self.bus.dat_r),
                )
            )
        ]

        # Counters and registers.
        in_idx = Signal(16)
        hid_idx = Signal(16)
        out_idx = Signal(16)
        row_base_w1 = Signal(32)
        row_base_w2 = Signal(32)
        weight_addr = Signal(32)
        in_idx_rd = Signal(16)
        hid_idx_rd = Signal(16)

        acc = Signal((48, True))
        last_w = Signal((data_bits, True))
        last_x = Signal((data_bits, True))
        mac_w_ext = Signal((32, True))
        mac_x_ext = Signal((32, True))
        mac_x_raw = Signal(data_bits)
        in_x_reg = Signal(data_bits)
        hid_x_reg = Signal(data_bits)

        # Helper values.
        in_rd.adr  = in_idx_rd
        hid_rd.adr = hid_idx_rd

        max_pos = Constant((1 << (data_bits - 1)) - 1, 48)
        min_neg = Constant(-(1 << (data_bits - 1)), 48)

        out_val = Signal((data_bits, True))
        hid_write_val = Signal((data_bits, True))
        out_write_val = Signal((data_bits, True))
        shifted = Signal((48, True))
        self.comb += shifted.eq(acc >> self._shift.storage)

        self.comb += [
            If(self._relu_en.storage & (shifted < 0),
                hid_write_val.eq(0),
            ).Elif(shifted > max_pos,
                hid_write_val.eq(max_pos[0:data_bits]),
            ).Elif(shifted < min_neg,
                hid_write_val.eq(min_neg[0:data_bits]),
            ).Else(
                hid_write_val.eq(shifted[0:data_bits]),
            ),
            If(shifted > max_pos,
                out_write_val.eq(max_pos[0:data_bits]),
            ).Elif(shifted < min_neg,
                out_write_val.eq(min_neg[0:data_bits]),
            ).Else(
                out_write_val.eq(shifted[0:data_bits]),
            ),
        ]

        self.comb += [
            req_valid.eq(0),
            req_we.eq(0),
            req_addr.eq(0),
            req_wdata.eq(0),
            req_wsel.eq(0xF),
            in_wr.we.eq(0),
            in_wr.adr.eq(0),
            in_wr.dat_w.eq(0),
            hid_wr.we.eq(0),
            hid_wr.adr.eq(0),
            hid_wr.dat_w.eq(0),
        ]

        self.comb += [
            self._dbg_acc_lo.status.eq(acc[0:32]),
            self._dbg_acc_hi.status.eq(acc[32:48]),
            self._dbg_w.status.eq(last_w),
            self._dbg_x.status.eq(last_x),
            self._dbg_in.status.eq(in_dbg.dat_r),
            self._dbg_hid.status.eq(hid_dbg.dat_r),
            self._dbg_in_idx.status.eq(in_idx),
            self._dbg_hid_idx.status.eq(hid_idx),
            self._dbg_in_x.status.eq(in_x_reg),
            self._dbg_hid_x.status.eq(hid_x_reg),
            self._dbg_req_addr.status.eq(wb_adr),
        ]

        self.comb += [
            in_dbg.adr.eq(self._dbg_sel.storage),
            hid_dbg.adr.eq(self._dbg_sel.storage),
            in_rd.re.eq(1),
            hid_rd.re.eq(1),
            in_dbg.re.eq(1),
            hid_dbg.re.eq(1),
        ]

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        self.comb += [
            If(fsm.ongoing("HID_W_WAIT"),
                mac_x_raw.eq(in_x_reg),
            ).Elif(fsm.ongoing("OUT_W_WAIT"),
                mac_x_raw.eq(hid_x_reg),
            ).Else(
                mac_x_raw.eq(0),
            ),
            mac_w_ext.eq(_sign_extend(resp_rdata[0:data_bits], data_bits, 32)),
            mac_x_ext.eq(_sign_extend(mac_x_raw, data_bits, 32)),
        ]

        fsm.act(
            "IDLE",
            self._busy.status.eq(0),
            If(self._start.re,
                NextValue(self._done.status, 0),
                NextValue(self._error.status, 0),
                If((self._dim_in.storage > max_in) |
                   (self._dim_hid.storage > max_hid) |
                   (self._dim_out.storage > max_out),
                    NextValue(self._error.status, 1),
                    NextValue(self._done.status, 1),
                ).Else(
                    NextValue(in_idx, 0),
                    NextValue(hid_idx, 0),
                    NextValue(out_idx, 0),
                    NextValue(row_base_w1, self._base_w1.storage),
                    NextValue(row_base_w2, self._base_w2.storage),
                    NextState("HID_BIAS_REQ"),
                )
            )
        )

        fsm.act(
            "LOAD_IN_REQ",
            self._busy.status.eq(1),
            If(in_idx < self._dim_in.storage,
                If(bus_idle,
                    req_valid.eq(1),
                    req_we.eq(0),
                    req_addr.eq(self._base_in.storage + (in_idx << 2)),
                    NextState("LOAD_IN_WAIT"),
                )
            ).Else(
                NextValue(hid_idx, 0),
                NextState("HID_BIAS_REQ"),
            )
        )

        fsm.act(
            "LOAD_IN_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                in_wr.we.eq(1),
                in_wr.adr.eq(in_idx),
                in_wr.dat_w.eq(resp_rdata[0:data_bits]),
                NextValue(in_idx, in_idx + 1),
                NextState("LOAD_IN_REQ"),
            )
        )

        fsm.act(
            "HID_BIAS_REQ",
            self._busy.status.eq(1),
            If(hid_idx < self._dim_hid.storage,
                If(bus_idle,
                    req_valid.eq(1),
                    req_we.eq(0),
                    req_addr.eq(self._base_b1.storage + (hid_idx << 2)),
                    NextState("HID_BIAS_WAIT"),
                )
            ).Else(
                NextValue(out_idx, 0),
                NextState("OUT_BIAS_REQ"),
            )
        )

        fsm.act(
            "HID_BIAS_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                NextValue(acc, _sign_extend(resp_rdata[0:data_bits], data_bits, 48)),
                NextValue(in_idx, 0),
                NextValue(weight_addr, row_base_w1),
                NextState("HID_IN_REQ"),
            )
        )

        fsm.act(
            "HID_IN_REQ",
            self._busy.status.eq(1),
            If(in_idx < self._dim_in.storage,
                If(bus_idle,
                    req_valid.eq(1),
                    req_we.eq(0),
                    req_addr.eq(self._base_in.storage + (in_idx << 2)),
                    NextState("HID_IN_WAIT"),
                )
            ).Else(
                NextState("HID_WRITE"),
            )
        )

        fsm.act(
            "HID_IN_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                NextValue(in_x_reg, resp_rdata[0:data_bits]),
                NextState("HID_W_REQ"),
            )
        )

        fsm.act(
            "HID_W_REQ",
            self._busy.status.eq(1),
            If(bus_idle,
                req_valid.eq(1),
                req_we.eq(0),
                req_addr.eq(weight_addr),
                NextState("HID_W_WAIT"),
            )
        )

        fsm.act(
            "HID_W_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                NextValue(last_w, resp_rdata[0:data_bits]),
                NextValue(last_x, mac_x_raw),
                NextValue(
                    acc,
                    acc + mac_w_ext * mac_x_ext
                ),
                NextValue(in_idx, in_idx + 1),
                NextValue(weight_addr, weight_addr + 4),
                NextState("HID_IN_REQ"),
            )
        )

        fsm.act(
            "HID_WRITE",
            self._busy.status.eq(1),
            NextState("HID_STORE_REQ"),
        )

        fsm.act(
            "HID_STORE_REQ",
            self._busy.status.eq(1),
            If(bus_idle,
                req_valid.eq(1),
                req_we.eq(1),
                req_addr.eq(self._base_hid.storage + (hid_idx << 2)),
                req_wdata.eq(Cat(hid_write_val, Replicate(hid_write_val[data_bits - 1], 16))),
                NextState("HID_STORE_WAIT"),
            )
        )

        fsm.act(
            "HID_STORE_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                NextValue(hid_idx, hid_idx + 1),
                NextValue(row_base_w1, row_base_w1 + (self._dim_in.storage << 2)),
                NextState("HID_BIAS_REQ"),
            )
        )

        fsm.act(
            "OUT_BIAS_REQ",
            self._busy.status.eq(1),
            If(out_idx < self._dim_out.storage,
                If(bus_idle,
                    req_valid.eq(1),
                    req_we.eq(0),
                    req_addr.eq(self._base_b2.storage + (out_idx << 2)),
                    NextState("OUT_BIAS_WAIT"),
                )
            ).Else(
                NextState("DONE"),
            )
        )

        fsm.act(
            "OUT_BIAS_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                NextValue(acc, _sign_extend(resp_rdata[0:data_bits], data_bits, 48)),
                NextValue(hid_idx, 0),
                NextValue(weight_addr, row_base_w2),
                NextState("OUT_HID_REQ"),
            )
        )

        fsm.act(
            "OUT_HID_REQ",
            self._busy.status.eq(1),
            If(hid_idx < self._dim_hid.storage,
                If(bus_idle,
                    req_valid.eq(1),
                    req_we.eq(0),
                    req_addr.eq(self._base_hid.storage + (hid_idx << 2)),
                    NextState("OUT_HID_WAIT"),
                )
            ).Else(
                NextState("OUT_WRITE"),
            )
        )

        fsm.act(
            "OUT_HID_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                NextValue(hid_x_reg, resp_rdata[0:data_bits]),
                NextState("OUT_W_REQ"),
            )
        )

        fsm.act(
            "OUT_W_REQ",
            self._busy.status.eq(1),
            If(bus_idle,
                req_valid.eq(1),
                req_we.eq(0),
                req_addr.eq(weight_addr),
                NextState("OUT_W_WAIT"),
            )
        )

        fsm.act(
            "OUT_W_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                NextValue(last_w, resp_rdata[0:data_bits]),
                NextValue(last_x, mac_x_raw),
                NextValue(
                    acc,
                    acc + mac_w_ext * mac_x_ext
                ),
                NextValue(hid_idx, hid_idx + 1),
                NextValue(weight_addr, weight_addr + 4),
                NextState("OUT_HID_REQ"),
            )
        )

        fsm.act(
            "OUT_WRITE",
            self._busy.status.eq(1),
            NextValue(out_val, out_write_val),
            NextState("OUT_STORE_REQ"),
        )

        fsm.act(
            "OUT_STORE_REQ",
            self._busy.status.eq(1),
            If(bus_idle,
                req_valid.eq(1),
                req_we.eq(1),
                req_addr.eq(self._base_out.storage + (out_idx << 2)),
                req_wdata.eq(Cat(out_val, Replicate(out_val[data_bits - 1], 16))),
                NextState("OUT_STORE_WAIT"),
            )
        )

        fsm.act(
            "OUT_STORE_WAIT",
            self._busy.status.eq(1),
            If(resp_valid,
                NextValue(out_idx, out_idx + 1),
                NextValue(row_base_w2, row_base_w2 + (self._dim_hid.storage << 2)),
                NextState("OUT_BIAS_REQ"),
            )
        )

        fsm.act(
            "DONE",
            self._busy.status.eq(0),
            NextValue(self._done.status, 1),
            If(self._start.re,
                NextValue(self._done.status, 0),
                NextState("IDLE"),
            )
        )
