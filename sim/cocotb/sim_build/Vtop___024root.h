// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See Vtop.h for the primary calling header

#ifndef VERILATED_VTOP___024ROOT_H_
#define VERILATED_VTOP___024ROOT_H_  // guard

#include "verilated.h"


class Vtop__Syms;

class alignas(VL_CACHE_LINE_BYTES) Vtop___024root final {
  public:

    // DESIGN SPECIFIC STATE
    // Anonymous structures to workaround compiler member-count bugs
    struct {
        VL_IN8(sys_clk,0,0);
        VL_IN8(sys_rst,0,0);
        VL_OUT8(bus_sel,3,0);
        VL_OUT8(bus_cyc,0,0);
        VL_OUT8(bus_stb,0,0);
        VL_IN8(bus_ack,0,0);
        VL_OUT8(bus_we,0,0);
        VL_IN8(storage,0,0);
        VL_IN8(re,0,0);
        VL_IN8(storage_1,0,0);
        VL_IN8(storage_2,4,0);
        VL_OUT8(status,0,0);
        VL_OUT8(status_1,0,0);
        VL_OUT8(status_2,0,0);
        CData/*0:0*/ mlp_stream__DOT__sys_clk;
        CData/*0:0*/ mlp_stream__DOT__sys_rst;
        CData/*3:0*/ mlp_stream__DOT__bus_sel;
        CData/*0:0*/ mlp_stream__DOT__bus_cyc;
        CData/*0:0*/ mlp_stream__DOT__bus_stb;
        CData/*0:0*/ mlp_stream__DOT__bus_ack;
        CData/*0:0*/ mlp_stream__DOT__bus_we;
        CData/*0:0*/ mlp_stream__DOT__storage;
        CData/*0:0*/ mlp_stream__DOT__re;
        CData/*0:0*/ mlp_stream__DOT__storage_1;
        CData/*4:0*/ mlp_stream__DOT__storage_2;
        CData/*0:0*/ mlp_stream__DOT__status;
        CData/*0:0*/ mlp_stream__DOT__status_1;
        CData/*0:0*/ mlp_stream__DOT__status_2;
        CData/*0:0*/ mlp_stream__DOT__sys_clk_1;
        CData/*0:0*/ mlp_stream__DOT__sys_rst_1;
        CData/*0:0*/ mlp_stream__DOT__in_wr_we;
        CData/*6:0*/ mlp_stream__DOT__hid_wr_adr;
        CData/*0:0*/ mlp_stream__DOT__hid_wr_we;
        CData/*0:0*/ mlp_stream__DOT__wb_cyc;
        CData/*0:0*/ mlp_stream__DOT__wb_stb;
        CData/*0:0*/ mlp_stream__DOT__wb_we;
        CData/*3:0*/ mlp_stream__DOT__wb_sel;
        CData/*0:0*/ mlp_stream__DOT__req_valid;
        CData/*0:0*/ mlp_stream__DOT__req_we;
        CData/*3:0*/ mlp_stream__DOT__req_wsel;
        CData/*0:0*/ mlp_stream__DOT__resp_valid;
        CData/*0:0*/ mlp_stream__DOT__bus_idle;
        CData/*0:0*/ mlp_stream__DOT__is_ongoing0;
        CData/*0:0*/ mlp_stream__DOT__is_ongoing1;
        CData/*3:0*/ mlp_stream__DOT__state;
        CData/*3:0*/ mlp_stream__DOT__next_state;
        CData/*0:0*/ mlp_stream__DOT__last_w_next_value_ce0;
        CData/*0:0*/ mlp_stream__DOT__last_x_next_value_ce1;
        CData/*0:0*/ mlp_stream__DOT__acc_next_value_ce2;
        CData/*0:0*/ mlp_stream__DOT__in_idx_next_value_ce3;
        CData/*0:0*/ mlp_stream__DOT__weight_addr_next_value_ce4;
        CData/*0:0*/ mlp_stream__DOT__hid_idx_next_value_ce5;
        CData/*0:0*/ mlp_stream__DOT__done_status_next_value6;
        CData/*0:0*/ mlp_stream__DOT__done_status_next_value_ce6;
        CData/*0:0*/ mlp_stream__DOT__error_status_next_value7;
        CData/*0:0*/ mlp_stream__DOT__error_status_next_value_ce7;
        CData/*0:0*/ mlp_stream__DOT__out_idx_f_next_value_ce0;
        CData/*0:0*/ mlp_stream__DOT__row_base_w1_f_next_value_ce1;
        CData/*0:0*/ mlp_stream__DOT__row_base_w2_f_next_value_ce2;
        CData/*0:0*/ mlp_stream__DOT__in_x_reg_t_next_value_ce0;
        CData/*0:0*/ mlp_stream__DOT__hid_x_reg_t_next_value_ce1;
        CData/*0:0*/ mlp_stream__DOT__out_val_next_value_ce8;
        CData/*0:0*/ mlp_stream__DOT__dummy_s;
        CData/*0:0*/ mlp_stream__DOT__dummy_d;
    };
    struct {
        CData/*0:0*/ mlp_stream__DOT__dummy_d_1;
        CData/*0:0*/ mlp_stream__DOT__dummy_d_2;
        CData/*0:0*/ mlp_stream__DOT__dummy_d_3;
        CData/*6:0*/ mlp_stream__DOT__memadr_1;
        CData/*0:0*/ __VstlFirstIteration;
        CData/*0:0*/ __VstlPhaseResult;
        CData/*0:0*/ __VicoFirstIteration;
        CData/*0:0*/ __VicoPhaseResult;
        CData/*0:0*/ __Vtrigprevexpr___TOP__mlp_stream__DOT__sys_clk_1__0;
        CData/*0:0*/ __VactPhaseResult;
        CData/*0:0*/ __VnbaPhaseResult;
        VL_IN16(storage_3,15,0);
        VL_IN16(storage_4,15,0);
        VL_IN16(storage_5,15,0);
        VL_OUT16(status_5,15,0);
        VL_OUT16(status_6,15,0);
        SData/*15:0*/ mlp_stream__DOT____Vlvbound_hc2aa3eeb__0;
        SData/*15:0*/ mlp_stream__DOT__storage_3;
        SData/*15:0*/ mlp_stream__DOT__storage_4;
        SData/*15:0*/ mlp_stream__DOT__storage_5;
        SData/*15:0*/ mlp_stream__DOT__status_5;
        SData/*15:0*/ mlp_stream__DOT__status_6;
        SData/*9:0*/ mlp_stream__DOT__in_wr_adr;
        SData/*15:0*/ mlp_stream__DOT__in_wr_dat_r;
        SData/*15:0*/ mlp_stream__DOT__in_wr_dat_w;
        SData/*15:0*/ mlp_stream__DOT__in_rd_dat_r;
        SData/*15:0*/ mlp_stream__DOT__hid_wr_dat_r;
        SData/*15:0*/ mlp_stream__DOT__hid_wr_dat_w;
        SData/*15:0*/ mlp_stream__DOT__hid_rd_dat_r;
        SData/*15:0*/ mlp_stream__DOT__in_idx;
        SData/*15:0*/ mlp_stream__DOT__hid_idx;
        SData/*15:0*/ mlp_stream__DOT__out_idx;
        SData/*15:0*/ mlp_stream__DOT__last_w;
        SData/*15:0*/ mlp_stream__DOT__last_x;
        SData/*15:0*/ mlp_stream__DOT__mac_x_raw;
        SData/*15:0*/ mlp_stream__DOT__in_x_reg;
        SData/*15:0*/ mlp_stream__DOT__hid_x_reg;
        SData/*15:0*/ mlp_stream__DOT__out_val;
        SData/*15:0*/ mlp_stream__DOT__hid_write_val;
        SData/*15:0*/ mlp_stream__DOT__out_write_val;
        SData/*15:0*/ mlp_stream__DOT__last_w_next_value0;
        SData/*15:0*/ mlp_stream__DOT__last_x_next_value1;
        SData/*15:0*/ mlp_stream__DOT__in_idx_next_value3;
        SData/*15:0*/ mlp_stream__DOT__hid_idx_next_value5;
        SData/*15:0*/ mlp_stream__DOT__out_idx_f_next_value0;
        SData/*15:0*/ mlp_stream__DOT__in_x_reg_t_next_value0;
        SData/*15:0*/ mlp_stream__DOT__hid_x_reg_t_next_value1;
        SData/*15:0*/ mlp_stream__DOT__out_val_next_value8;
        SData/*15:0*/ mlp_stream__DOT__rhs_slice_proxy;
        SData/*15:0*/ mlp_stream__DOT__complexslicelowerer_slice_proxy0;
        SData/*15:0*/ mlp_stream__DOT__complexslicelowerer_slice_proxy1;
        SData/*9:0*/ mlp_stream__DOT__memadr;
        VL_OUT(bus_adr,31,0);
        VL_OUT(bus_dat_w,31,0);
        VL_IN(bus_dat_r,31,0);
        VL_IN(storage_6,31,0);
        VL_IN(storage_7,31,0);
        VL_IN(storage_8,31,0);
        VL_IN(storage_9,31,0);
        VL_IN(storage_10,31,0);
        VL_IN(storage_11,31,0);
        VL_OUT(status_3,31,0);
        VL_OUT(status_4,31,0);
        IData/*31:0*/ mlp_stream__DOT__bus_adr;
    };
    struct {
        IData/*31:0*/ mlp_stream__DOT__bus_dat_w;
        IData/*31:0*/ mlp_stream__DOT__bus_dat_r;
        IData/*31:0*/ mlp_stream__DOT__storage_6;
        IData/*31:0*/ mlp_stream__DOT__storage_7;
        IData/*31:0*/ mlp_stream__DOT__storage_8;
        IData/*31:0*/ mlp_stream__DOT__storage_9;
        IData/*31:0*/ mlp_stream__DOT__storage_10;
        IData/*31:0*/ mlp_stream__DOT__storage_11;
        IData/*31:0*/ mlp_stream__DOT__status_3;
        IData/*31:0*/ mlp_stream__DOT__status_4;
        IData/*31:0*/ mlp_stream__DOT__wb_adr;
        IData/*31:0*/ mlp_stream__DOT__wb_dat_w;
        IData/*31:0*/ mlp_stream__DOT__req_addr;
        IData/*31:0*/ mlp_stream__DOT__req_wdata;
        IData/*31:0*/ mlp_stream__DOT__resp_rdata;
        IData/*31:0*/ mlp_stream__DOT__row_base_w1;
        IData/*31:0*/ mlp_stream__DOT__row_base_w2;
        IData/*31:0*/ mlp_stream__DOT__weight_addr;
        IData/*31:0*/ mlp_stream__DOT__mac_w_ext;
        IData/*31:0*/ mlp_stream__DOT__mac_x_ext;
        IData/*31:0*/ mlp_stream__DOT__weight_addr_next_value4;
        IData/*31:0*/ mlp_stream__DOT__row_base_w1_f_next_value1;
        IData/*31:0*/ mlp_stream__DOT__row_base_w2_f_next_value2;
        IData/*31:0*/ __VactIterCount;
        QData/*47:0*/ mlp_stream__DOT__acc;
        QData/*47:0*/ mlp_stream__DOT__shifted;
        QData/*47:0*/ mlp_stream__DOT__acc_next_value2;
        QData/*47:0*/ mlp_stream__DOT__t_slice_proxy0;
        QData/*47:0*/ mlp_stream__DOT__f_slice_proxy;
        QData/*47:0*/ mlp_stream__DOT__t_slice_proxy1;
        QData/*47:0*/ mlp_stream__DOT__t_slice_proxy2;
        VlUnpacked<SData/*15:0*/, 784> mlp_stream__DOT__in_mem;
        VlUnpacked<SData/*15:0*/, 128> mlp_stream__DOT__hid_mem;
        VlUnpacked<QData/*63:0*/, 1> __VstlTriggered;
        VlUnpacked<QData/*63:0*/, 1> __VicoTriggered;
        VlUnpacked<QData/*63:0*/, 1> __VactTriggered;
        VlUnpacked<QData/*63:0*/, 1> __VnbaTriggered;
    };

    // INTERNAL VARIABLES
    Vtop__Syms* vlSymsp;
    const char* vlNamep;

    // CONSTRUCTORS
    Vtop___024root(Vtop__Syms* symsp, const char* namep);
    ~Vtop___024root();
    VL_UNCOPYABLE(Vtop___024root);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
};


#endif  // guard
