// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Symbol table implementation internals

#include "Vtop__pch.h"

Vtop__Syms::Vtop__Syms(VerilatedContext* contextp, const char* namep, Vtop* modelp)
    : VerilatedSyms{contextp}
    // Setup internal state of the Syms class
    , __Vm_modelp{modelp}
    // Setup top module instance
    , TOP{this, namep}
{
    // Check resources
    Verilated::stackCheck(280);
    // Setup sub module instances
    // Configure time unit / time precision
    _vm_contextp__->timeunit(-9);
    _vm_contextp__->timeprecision(-12);
    // Setup each module's pointers to their submodules
    // Setup each module's pointer back to symbol table (for public functions)
    TOP.__Vconfigure(true);
    // Setup scopes
    __Vscopep_TOP = new VerilatedScope{this, "TOP", "TOP", "<null>", 0, VerilatedScope::SCOPE_OTHER};
    __Vscopep_mlp_stream = new VerilatedScope{this, "mlp_stream", "mlp_stream", "mlp_stream", -9, VerilatedScope::SCOPE_MODULE};
    // Set up scope hierarchy
    __Vhier.add(0, __Vscopep_mlp_stream);
    // Setup export functions - final: 0
    // Setup export functions - final: 1
    // Setup public variables
    __Vscopep_TOP->varInsert("bus_ack", &(TOP.bus_ack), false, VLVT_UINT8, VLVD_IN|VLVF_PUB_RW, 0, 0);
    __Vscopep_TOP->varInsert("bus_adr", &(TOP.bus_adr), false, VLVT_UINT32, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("bus_cyc", &(TOP.bus_cyc), false, VLVT_UINT8, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_TOP->varInsert("bus_dat_r", &(TOP.bus_dat_r), false, VLVT_UINT32, VLVD_IN|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("bus_dat_w", &(TOP.bus_dat_w), false, VLVT_UINT32, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("bus_sel", &(TOP.bus_sel), false, VLVT_UINT8, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,3,0);
    __Vscopep_TOP->varInsert("bus_stb", &(TOP.bus_stb), false, VLVT_UINT8, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_TOP->varInsert("bus_we", &(TOP.bus_we), false, VLVT_UINT8, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_TOP->varInsert("re", &(TOP.re), false, VLVT_UINT8, VLVD_IN|VLVF_PUB_RW, 0, 0);
    __Vscopep_TOP->varInsert("status", &(TOP.status), false, VLVT_UINT8, VLVD_OUT|VLVF_PUB_RW, 0, 0);
    __Vscopep_TOP->varInsert("status_1", &(TOP.status_1), false, VLVT_UINT8, VLVD_OUT|VLVF_PUB_RW, 0, 0);
    __Vscopep_TOP->varInsert("status_2", &(TOP.status_2), false, VLVT_UINT8, VLVD_OUT|VLVF_PUB_RW, 0, 0);
    __Vscopep_TOP->varInsert("status_3", &(TOP.status_3), false, VLVT_UINT32, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("status_4", &(TOP.status_4), false, VLVT_UINT32, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("status_5", &(TOP.status_5), false, VLVT_UINT16, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_TOP->varInsert("status_6", &(TOP.status_6), false, VLVT_UINT16, VLVD_OUT|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_TOP->varInsert("storage", &(TOP.storage), false, VLVT_UINT8, VLVD_IN|VLVF_PUB_RW, 0, 0);
    __Vscopep_TOP->varInsert("storage_1", &(TOP.storage_1), false, VLVT_UINT8, VLVD_IN|VLVF_PUB_RW, 0, 0);
    __Vscopep_TOP->varInsert("storage_10", &(TOP.storage_10), false, VLVT_UINT32, VLVD_IN|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("storage_11", &(TOP.storage_11), false, VLVT_UINT32, VLVD_IN|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("storage_2", &(TOP.storage_2), false, VLVT_UINT8, VLVD_IN|VLVF_PUB_RW, 0, 1 ,4,0);
    __Vscopep_TOP->varInsert("storage_3", &(TOP.storage_3), false, VLVT_UINT16, VLVD_IN|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_TOP->varInsert("storage_4", &(TOP.storage_4), false, VLVT_UINT16, VLVD_IN|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_TOP->varInsert("storage_5", &(TOP.storage_5), false, VLVT_UINT16, VLVD_IN|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_TOP->varInsert("storage_6", &(TOP.storage_6), false, VLVT_UINT32, VLVD_IN|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("storage_7", &(TOP.storage_7), false, VLVT_UINT32, VLVD_IN|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("storage_8", &(TOP.storage_8), false, VLVT_UINT32, VLVD_IN|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("storage_9", &(TOP.storage_9), false, VLVT_UINT32, VLVD_IN|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_TOP->varInsert("sys_clk", &(TOP.sys_clk), false, VLVT_UINT8, VLVD_IN|VLVF_PUB_RW, 0, 0);
    __Vscopep_TOP->varInsert("sys_rst", &(TOP.sys_rst), false, VLVT_UINT8, VLVD_IN|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("acc", &(TOP.mlp_stream__DOT__acc), false, VLVT_UINT64, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,47,0);
    __Vscopep_mlp_stream->varInsert("acc_next_value2", &(TOP.mlp_stream__DOT__acc_next_value2), false, VLVT_UINT64, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,47,0);
    __Vscopep_mlp_stream->varInsert("acc_next_value_ce2", &(TOP.mlp_stream__DOT__acc_next_value_ce2), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("bus_ack", &(TOP.mlp_stream__DOT__bus_ack), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("bus_adr", &(TOP.mlp_stream__DOT__bus_adr), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("bus_cyc", &(TOP.mlp_stream__DOT__bus_cyc), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_mlp_stream->varInsert("bus_dat_r", &(TOP.mlp_stream__DOT__bus_dat_r), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("bus_dat_w", &(TOP.mlp_stream__DOT__bus_dat_w), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("bus_idle", &(TOP.mlp_stream__DOT__bus_idle), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_mlp_stream->varInsert("bus_sel", &(TOP.mlp_stream__DOT__bus_sel), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,3,0);
    __Vscopep_mlp_stream->varInsert("bus_stb", &(TOP.mlp_stream__DOT__bus_stb), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_mlp_stream->varInsert("bus_we", &(TOP.mlp_stream__DOT__bus_we), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_mlp_stream->varInsert("complexslicelowerer_slice_proxy0", &(TOP.mlp_stream__DOT__complexslicelowerer_slice_proxy0), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("complexslicelowerer_slice_proxy1", &(TOP.mlp_stream__DOT__complexslicelowerer_slice_proxy1), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("done_status_next_value6", &(TOP.mlp_stream__DOT__done_status_next_value6), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("done_status_next_value_ce6", &(TOP.mlp_stream__DOT__done_status_next_value_ce6), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("dummy_d", &(TOP.mlp_stream__DOT__dummy_d), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("dummy_d_1", &(TOP.mlp_stream__DOT__dummy_d_1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("dummy_d_2", &(TOP.mlp_stream__DOT__dummy_d_2), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("dummy_d_3", &(TOP.mlp_stream__DOT__dummy_d_3), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("dummy_s", &(TOP.mlp_stream__DOT__dummy_s), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("error_status_next_value7", &(TOP.mlp_stream__DOT__error_status_next_value7), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("error_status_next_value_ce7", &(TOP.mlp_stream__DOT__error_status_next_value_ce7), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("f_slice_proxy", &(TOP.mlp_stream__DOT__f_slice_proxy), false, VLVT_UINT64, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY|VLVF_SIGNED, 0, 1 ,47,0);
    __Vscopep_mlp_stream->varInsert("hid_idx", &(TOP.mlp_stream__DOT__hid_idx), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_idx_next_value5", &(TOP.mlp_stream__DOT__hid_idx_next_value5), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_idx_next_value_ce5", &(TOP.mlp_stream__DOT__hid_idx_next_value_ce5), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("hid_mem", &(TOP.mlp_stream__DOT__hid_mem), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 1, 1 ,0,127 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_rd_dat_r", &(TOP.mlp_stream__DOT__hid_rd_dat_r), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_wr_adr", &(TOP.mlp_stream__DOT__hid_wr_adr), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,6,0);
    __Vscopep_mlp_stream->varInsert("hid_wr_dat_r", &(TOP.mlp_stream__DOT__hid_wr_dat_r), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_wr_dat_w", &(TOP.mlp_stream__DOT__hid_wr_dat_w), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_wr_we", &(TOP.mlp_stream__DOT__hid_wr_we), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("hid_write_val", &(TOP.mlp_stream__DOT__hid_write_val), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_x_reg", &(TOP.mlp_stream__DOT__hid_x_reg), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_x_reg_t_next_value1", &(TOP.mlp_stream__DOT__hid_x_reg_t_next_value1), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("hid_x_reg_t_next_value_ce1", &(TOP.mlp_stream__DOT__hid_x_reg_t_next_value_ce1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("in_idx", &(TOP.mlp_stream__DOT__in_idx), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("in_idx_next_value3", &(TOP.mlp_stream__DOT__in_idx_next_value3), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("in_idx_next_value_ce3", &(TOP.mlp_stream__DOT__in_idx_next_value_ce3), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("in_mem", &(TOP.mlp_stream__DOT__in_mem), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 1, 1 ,0,783 ,15,0);
    __Vscopep_mlp_stream->varInsert("in_rd_dat_r", &(TOP.mlp_stream__DOT__in_rd_dat_r), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("in_wr_adr", &(TOP.mlp_stream__DOT__in_wr_adr), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,9,0);
    __Vscopep_mlp_stream->varInsert("in_wr_dat_r", &(TOP.mlp_stream__DOT__in_wr_dat_r), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("in_wr_dat_w", &(TOP.mlp_stream__DOT__in_wr_dat_w), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("in_wr_we", &(TOP.mlp_stream__DOT__in_wr_we), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("in_x_reg", &(TOP.mlp_stream__DOT__in_x_reg), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("in_x_reg_t_next_value0", &(TOP.mlp_stream__DOT__in_x_reg_t_next_value0), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("in_x_reg_t_next_value_ce0", &(TOP.mlp_stream__DOT__in_x_reg_t_next_value_ce0), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("is_ongoing0", &(TOP.mlp_stream__DOT__is_ongoing0), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("is_ongoing1", &(TOP.mlp_stream__DOT__is_ongoing1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("last_w", &(TOP.mlp_stream__DOT__last_w), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("last_w_next_value0", &(TOP.mlp_stream__DOT__last_w_next_value0), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("last_w_next_value_ce0", &(TOP.mlp_stream__DOT__last_w_next_value_ce0), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("last_x", &(TOP.mlp_stream__DOT__last_x), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("last_x_next_value1", &(TOP.mlp_stream__DOT__last_x_next_value1), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("last_x_next_value_ce1", &(TOP.mlp_stream__DOT__last_x_next_value_ce1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("mac_w_ext", &(TOP.mlp_stream__DOT__mac_w_ext), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY|VLVF_SIGNED, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("mac_x_ext", &(TOP.mlp_stream__DOT__mac_x_ext), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY|VLVF_SIGNED, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("mac_x_raw", &(TOP.mlp_stream__DOT__mac_x_raw), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("memadr", &(TOP.mlp_stream__DOT__memadr), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,9,0);
    __Vscopep_mlp_stream->varInsert("memadr_1", &(TOP.mlp_stream__DOT__memadr_1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,6,0);
    __Vscopep_mlp_stream->varInsert("next_state", &(TOP.mlp_stream__DOT__next_state), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,3,0);
    __Vscopep_mlp_stream->varInsert("out_idx", &(TOP.mlp_stream__DOT__out_idx), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("out_idx_f_next_value0", &(TOP.mlp_stream__DOT__out_idx_f_next_value0), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("out_idx_f_next_value_ce0", &(TOP.mlp_stream__DOT__out_idx_f_next_value_ce0), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("out_val", &(TOP.mlp_stream__DOT__out_val), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("out_val_next_value8", &(TOP.mlp_stream__DOT__out_val_next_value8), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("out_val_next_value_ce8", &(TOP.mlp_stream__DOT__out_val_next_value_ce8), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("out_write_val", &(TOP.mlp_stream__DOT__out_write_val), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_SIGNED, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("re", &(TOP.mlp_stream__DOT__re), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("req_addr", &(TOP.mlp_stream__DOT__req_addr), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("req_valid", &(TOP.mlp_stream__DOT__req_valid), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("req_wdata", &(TOP.mlp_stream__DOT__req_wdata), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("req_we", &(TOP.mlp_stream__DOT__req_we), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("req_wsel", &(TOP.mlp_stream__DOT__req_wsel), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,3,0);
    __Vscopep_mlp_stream->varInsert("resp_rdata", &(TOP.mlp_stream__DOT__resp_rdata), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("resp_valid", &(TOP.mlp_stream__DOT__resp_valid), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("rhs_slice_proxy", &(TOP.mlp_stream__DOT__rhs_slice_proxy), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("row_base_w1", &(TOP.mlp_stream__DOT__row_base_w1), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("row_base_w1_f_next_value1", &(TOP.mlp_stream__DOT__row_base_w1_f_next_value1), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("row_base_w1_f_next_value_ce1", &(TOP.mlp_stream__DOT__row_base_w1_f_next_value_ce1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("row_base_w2", &(TOP.mlp_stream__DOT__row_base_w2), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("row_base_w2_f_next_value2", &(TOP.mlp_stream__DOT__row_base_w2_f_next_value2), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("row_base_w2_f_next_value_ce2", &(TOP.mlp_stream__DOT__row_base_w2_f_next_value_ce2), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("shifted", &(TOP.mlp_stream__DOT__shifted), false, VLVT_UINT64, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY|VLVF_SIGNED, 0, 1 ,47,0);
    __Vscopep_mlp_stream->varInsert("state", &(TOP.mlp_stream__DOT__state), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,3,0);
    __Vscopep_mlp_stream->varInsert("status", &(TOP.mlp_stream__DOT__status), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("status_1", &(TOP.mlp_stream__DOT__status_1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("status_2", &(TOP.mlp_stream__DOT__status_2), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("status_3", &(TOP.mlp_stream__DOT__status_3), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("status_4", &(TOP.mlp_stream__DOT__status_4), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("status_5", &(TOP.mlp_stream__DOT__status_5), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("status_6", &(TOP.mlp_stream__DOT__status_6), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("storage", &(TOP.mlp_stream__DOT__storage), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("storage_1", &(TOP.mlp_stream__DOT__storage_1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("storage_10", &(TOP.mlp_stream__DOT__storage_10), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("storage_11", &(TOP.mlp_stream__DOT__storage_11), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("storage_2", &(TOP.mlp_stream__DOT__storage_2), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,4,0);
    __Vscopep_mlp_stream->varInsert("storage_3", &(TOP.mlp_stream__DOT__storage_3), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("storage_4", &(TOP.mlp_stream__DOT__storage_4), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("storage_5", &(TOP.mlp_stream__DOT__storage_5), false, VLVT_UINT16, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,15,0);
    __Vscopep_mlp_stream->varInsert("storage_6", &(TOP.mlp_stream__DOT__storage_6), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("storage_7", &(TOP.mlp_stream__DOT__storage_7), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("storage_8", &(TOP.mlp_stream__DOT__storage_8), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("storage_9", &(TOP.mlp_stream__DOT__storage_9), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("sys_clk", &(TOP.mlp_stream__DOT__sys_clk), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("sys_clk_1", &(TOP.mlp_stream__DOT__sys_clk_1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_mlp_stream->varInsert("sys_rst", &(TOP.mlp_stream__DOT__sys_rst), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("sys_rst_1", &(TOP.mlp_stream__DOT__sys_rst_1), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 0);
    __Vscopep_mlp_stream->varInsert("t_slice_proxy0", &(TOP.mlp_stream__DOT__t_slice_proxy0), false, VLVT_UINT64, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,47,0);
    __Vscopep_mlp_stream->varInsert("t_slice_proxy1", &(TOP.mlp_stream__DOT__t_slice_proxy1), false, VLVT_UINT64, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY, 0, 1 ,47,0);
    __Vscopep_mlp_stream->varInsert("t_slice_proxy2", &(TOP.mlp_stream__DOT__t_slice_proxy2), false, VLVT_UINT64, VLVD_NODIR|VLVF_PUB_RW|VLVF_CONTINUOUSLY|VLVF_SIGNED, 0, 1 ,47,0);
    __Vscopep_mlp_stream->varInsert("wb_adr", &(TOP.mlp_stream__DOT__wb_adr), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("wb_cyc", &(TOP.mlp_stream__DOT__wb_cyc), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("wb_dat_w", &(TOP.mlp_stream__DOT__wb_dat_w), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("wb_sel", &(TOP.mlp_stream__DOT__wb_sel), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,3,0);
    __Vscopep_mlp_stream->varInsert("wb_stb", &(TOP.mlp_stream__DOT__wb_stb), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("wb_we", &(TOP.mlp_stream__DOT__wb_we), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
    __Vscopep_mlp_stream->varInsert("weight_addr", &(TOP.mlp_stream__DOT__weight_addr), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("weight_addr_next_value4", &(TOP.mlp_stream__DOT__weight_addr_next_value4), false, VLVT_UINT32, VLVD_NODIR|VLVF_PUB_RW, 0, 1 ,31,0);
    __Vscopep_mlp_stream->varInsert("weight_addr_next_value_ce4", &(TOP.mlp_stream__DOT__weight_addr_next_value_ce4), false, VLVT_UINT8, VLVD_NODIR|VLVF_PUB_RW, 0, 0);
}

Vtop__Syms::~Vtop__Syms() {
    // Tear down scope hierarchy
    __Vhier.remove(0, __Vscopep_mlp_stream);
    // Clear keys from hierarchy map after values have been removed
    __Vhier.clear();
    // Tear down scopes
    VL_DO_CLEAR(delete __Vscopep_TOP, __Vscopep_TOP = nullptr);
    VL_DO_CLEAR(delete __Vscopep_mlp_stream, __Vscopep_mlp_stream = nullptr);
    // Tear down sub module instances
}
