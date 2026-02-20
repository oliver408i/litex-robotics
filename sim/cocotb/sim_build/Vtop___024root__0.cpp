// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vtop.h for the primary calling header

#include "Vtop__pch.h"

void Vtop___024root___eval_triggers_vec__ico(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_triggers_vec__ico\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.__VicoTriggered[0U] = ((0xfffffffffffffffeULL 
                                      & vlSelfRef.__VicoTriggered[0U]) 
                                     | (IData)((IData)(vlSelfRef.__VicoFirstIteration)));
}

bool Vtop___024root___trigger_anySet__ico(const VlUnpacked<QData/*63:0*/, 1> &in) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___trigger_anySet__ico\n"); );
    // Locals
    IData/*31:0*/ n;
    // Body
    n = 0U;
    do {
        if (in[n]) {
            return (1U);
        }
        n = ((IData)(1U) + n);
    } while ((1U > n));
    return (0U);
}

void Vtop___024root___ico_sequent__TOP__0(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___ico_sequent__TOP__0\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Locals
    SData/*15:0*/ __VdfgRegularize_h6e95ff9d_0_0;
    __VdfgRegularize_h6e95ff9d_0_0 = 0;
    // Body
    vlSelfRef.mlp_stream__DOT__out_val_next_value_ce8 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_we = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_we = 0U;
    vlSelfRef.mlp_stream__DOT__dummy_d = vlSelfRef.mlp_stream__DOT__dummy_s;
    vlSelfRef.mlp_stream__DOT__dummy_d_1 = vlSelfRef.mlp_stream__DOT__dummy_s;
    vlSelfRef.mlp_stream__DOT__dummy_d_2 = vlSelfRef.mlp_stream__DOT__dummy_s;
    vlSelfRef.mlp_stream__DOT__dummy_d_3 = vlSelfRef.mlp_stream__DOT__dummy_s;
    vlSelfRef.mlp_stream__DOT__bus_dat_r = vlSelfRef.bus_dat_r;
    vlSelfRef.mlp_stream__DOT__bus_ack = vlSelfRef.bus_ack;
    vlSelfRef.mlp_stream__DOT__storage = vlSelfRef.storage;
    vlSelfRef.status_1 = vlSelfRef.mlp_stream__DOT__status_1;
    vlSelfRef.status_2 = vlSelfRef.mlp_stream__DOT__status_2;
    vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 0U;
    vlSelfRef.mlp_stream__DOT__last_x_next_value_ce1 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_adr = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_adr = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_we = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_we = 0U;
    vlSelfRef.mlp_stream__DOT__last_w_next_value_ce0 = 0U;
    vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_dat_r = vlSelfRef.mlp_stream__DOT__hid_mem
        [vlSelfRef.mlp_stream__DOT__memadr_1];
    vlSelfRef.mlp_stream__DOT__in_wr_dat_r = ((0x030fU 
                                               >= (IData)(vlSelfRef.mlp_stream__DOT__memadr))
                                               ? vlSelfRef.mlp_stream__DOT__in_mem
                                              [vlSelfRef.mlp_stream__DOT__memadr]
                                               : 0U);
    vlSelfRef.mlp_stream__DOT__in_wr_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_adr = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_adr = 0U;
    vlSelfRef.mlp_stream__DOT__last_w_next_value0 = 0U;
    vlSelfRef.mlp_stream__DOT__sys_clk = vlSelfRef.sys_clk;
    vlSelfRef.mlp_stream__DOT__status = 0U;
    vlSelfRef.mlp_stream__DOT__status_3 = (IData)(vlSelfRef.mlp_stream__DOT__acc);
    vlSelfRef.mlp_stream__DOT__status_4 = (0x0000ffffU 
                                           & (IData)(
                                                     (vlSelfRef.mlp_stream__DOT__acc 
                                                      >> 0x00000020U)));
    vlSelfRef.mlp_stream__DOT__bus_cyc = vlSelfRef.mlp_stream__DOT__wb_cyc;
    vlSelfRef.mlp_stream__DOT__sys_rst = vlSelfRef.sys_rst;
    vlSelfRef.mlp_stream__DOT__bus_adr = VL_SHIFTR_III(32,32,32, vlSelfRef.mlp_stream__DOT__wb_adr, 2U);
    vlSelfRef.mlp_stream__DOT__bus_dat_w = vlSelfRef.mlp_stream__DOT__wb_dat_w;
    vlSelfRef.mlp_stream__DOT__bus_sel = vlSelfRef.mlp_stream__DOT__wb_sel;
    vlSelfRef.mlp_stream__DOT__bus_stb = vlSelfRef.mlp_stream__DOT__wb_stb;
    vlSelfRef.mlp_stream__DOT__bus_we = vlSelfRef.mlp_stream__DOT__wb_we;
    vlSelfRef.mlp_stream__DOT__status_5 = vlSelfRef.mlp_stream__DOT__last_w;
    vlSelfRef.mlp_stream__DOT__status_6 = vlSelfRef.mlp_stream__DOT__last_x;
    vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_rd_dat_r = vlSelfRef.mlp_stream__DOT__hid_mem
        [(0x0000007fU & (IData)(vlSelfRef.mlp_stream__DOT__hid_idx))];
    vlSelfRef.mlp_stream__DOT__in_rd_dat_r = ((0x030fU 
                                               >= (0x000003ffU 
                                                   & (IData)(vlSelfRef.mlp_stream__DOT__in_idx)))
                                               ? vlSelfRef.mlp_stream__DOT__in_mem
                                              [(0x000003ffU 
                                                & (IData)(vlSelfRef.mlp_stream__DOT__in_idx))]
                                               : 0U);
    vlSelfRef.mlp_stream__DOT__storage_1 = vlSelfRef.storage_1;
    vlSelfRef.mlp_stream__DOT__storage_7 = vlSelfRef.storage_7;
    vlSelfRef.mlp_stream__DOT__complexslicelowerer_slice_proxy0 
        = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
    vlSelfRef.mlp_stream__DOT__complexslicelowerer_slice_proxy1 
        = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
    vlSelfRef.mlp_stream__DOT__storage_9 = vlSelfRef.storage_9;
    vlSelfRef.mlp_stream__DOT__rhs_slice_proxy = (0x0000ffffU 
                                                  & vlSelfRef.mlp_stream__DOT__resp_rdata);
    vlSelfRef.mlp_stream__DOT__storage_10 = vlSelfRef.storage_10;
    vlSelfRef.mlp_stream__DOT__storage_11 = vlSelfRef.storage_11;
    vlSelfRef.mlp_stream__DOT__storage_6 = vlSelfRef.storage_6;
    vlSelfRef.mlp_stream__DOT__storage_8 = vlSelfRef.storage_8;
    vlSelfRef.mlp_stream__DOT__is_ongoing1 = 0U;
    vlSelfRef.mlp_stream__DOT__is_ongoing0 = 0U;
    vlSelfRef.mlp_stream__DOT__storage_2 = vlSelfRef.storage_2;
    vlSelfRef.mlp_stream__DOT__bus_idle = (1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__wb_cyc)));
    vlSelfRef.mlp_stream__DOT__re = vlSelfRef.re;
    vlSelfRef.mlp_stream__DOT__storage_5 = vlSelfRef.storage_5;
    vlSelfRef.mlp_stream__DOT__storage_4 = vlSelfRef.storage_4;
    vlSelfRef.mlp_stream__DOT__storage_3 = vlSelfRef.storage_3;
    vlSelfRef.mlp_stream__DOT__sys_clk_1 = vlSelfRef.mlp_stream__DOT__sys_clk;
    if ((8U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__out_val_next_value_ce8 = 1U;
                }
            }
        }
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__hid_wr_we = 1U;
                    vlSelfRef.mlp_stream__DOT__hid_wr_adr 
                        = (0x0000007fU & (IData)(vlSelfRef.mlp_stream__DOT__hid_idx));
                }
            }
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 1U;
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 1U;
                    }
                }
            }
        }
        vlSelfRef.mlp_stream__DOT__status = ((1U & 
                                              (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                                                  >> 2U))) 
                                             || ((1U 
                                                  & (~ 
                                                     ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                                                      >> 1U))) 
                                                 || (1U 
                                                     & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))));
    } else {
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 1U;
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 1U;
                    }
                }
            }
        } else if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                             >> 1U)))) {
            if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 1U;
                    vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 1U;
                }
            } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 1U;
                vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 1U;
            }
        }
        vlSelfRef.mlp_stream__DOT__status = ((1U & 
                                              ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                                               >> 2U)) 
                                             || ((1U 
                                                  & (~ 
                                                     ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                                                      >> 1U))) 
                                                 || (1U 
                                                     & (IData)(vlSelfRef.mlp_stream__DOT__state))));
    }
    vlSelfRef.status = vlSelfRef.mlp_stream__DOT__status;
    vlSelfRef.status_3 = vlSelfRef.mlp_stream__DOT__status_3;
    vlSelfRef.status_4 = vlSelfRef.mlp_stream__DOT__status_4;
    vlSelfRef.bus_cyc = vlSelfRef.mlp_stream__DOT__bus_cyc;
    vlSelfRef.mlp_stream__DOT__sys_rst_1 = vlSelfRef.mlp_stream__DOT__sys_rst;
    vlSelfRef.bus_adr = vlSelfRef.mlp_stream__DOT__bus_adr;
    vlSelfRef.bus_dat_w = vlSelfRef.mlp_stream__DOT__bus_dat_w;
    vlSelfRef.bus_sel = vlSelfRef.mlp_stream__DOT__bus_sel;
    vlSelfRef.bus_stb = vlSelfRef.mlp_stream__DOT__bus_stb;
    vlSelfRef.bus_we = vlSelfRef.mlp_stream__DOT__bus_we;
    vlSelfRef.status_5 = vlSelfRef.mlp_stream__DOT__status_5;
    vlSelfRef.status_6 = vlSelfRef.mlp_stream__DOT__status_6;
    vlSelfRef.mlp_stream__DOT__mac_w_ext = (((- (IData)(
                                                        (1U 
                                                         & ((IData)(vlSelfRef.mlp_stream__DOT__rhs_slice_proxy) 
                                                            >> 0x0000000fU)))) 
                                             << 0x00000010U) 
                                            | (0x0000ffffU 
                                               & vlSelfRef.mlp_stream__DOT__resp_rdata));
    if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                  >> 3U)))) {
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__last_x_next_value_ce1 = 1U;
                        vlSelfRef.mlp_stream__DOT__last_w_next_value_ce0 = 1U;
                        vlSelfRef.mlp_stream__DOT__last_w_next_value0 
                            = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
                    }
                    vlSelfRef.mlp_stream__DOT__is_ongoing1 = 1U;
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__last_x_next_value_ce1 = 1U;
                    vlSelfRef.mlp_stream__DOT__last_w_next_value_ce0 = 1U;
                    vlSelfRef.mlp_stream__DOT__last_w_next_value0 
                        = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
                }
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__is_ongoing0 = 1U;
                }
            }
        }
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__in_wr_we = 1U;
                        vlSelfRef.mlp_stream__DOT__in_wr_dat_w 
                            = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
                        vlSelfRef.mlp_stream__DOT__in_wr_adr 
                            = (0x000003ffU & (IData)(vlSelfRef.mlp_stream__DOT__in_idx));
                    }
                }
            }
        }
    }
    vlSelfRef.mlp_stream__DOT__mac_x_raw = ((IData)(vlSelfRef.mlp_stream__DOT__is_ongoing0)
                                             ? (IData)(vlSelfRef.mlp_stream__DOT__in_x_reg)
                                             : ((IData)(vlSelfRef.mlp_stream__DOT__is_ongoing1)
                                                 ? (IData)(vlSelfRef.mlp_stream__DOT__hid_x_reg)
                                                 : 0U));
    vlSelfRef.mlp_stream__DOT__shifted = (0x0000ffffffffffffULL 
                                          & VL_SHIFTRS_QQI(48,48,5, vlSelfRef.mlp_stream__DOT__acc, (IData)(vlSelfRef.mlp_stream__DOT__storage_2)));
    vlSelfRef.mlp_stream__DOT__req_wdata = 0U;
    vlSelfRef.mlp_stream__DOT__req_wdata = 0U;
    vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value_ce1 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value1 = 0U;
    vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value_ce0 = 0U;
    vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value0 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value_ce1 = 0U;
    vlSelfRef.mlp_stream__DOT__error_status_next_value_ce7 = 0U;
    vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 0U;
    vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 0U;
    vlSelfRef.mlp_stream__DOT__error_status_next_value7 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value_ce2 = 0U;
    vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value1 = 0U;
    vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0 = 0U;
    vlSelfRef.mlp_stream__DOT__in_idx_next_value3 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value2 = 0U;
    vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0 = 0U;
    vlSelfRef.mlp_stream__DOT__req_we = 0U;
    vlSelfRef.mlp_stream__DOT__req_we = 0U;
    vlSelfRef.mlp_stream__DOT__req_valid = 0U;
    vlSelfRef.mlp_stream__DOT__req_valid = 0U;
    vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 = 0U;
    vlSelfRef.mlp_stream__DOT__next_state = 0U;
    vlSelfRef.mlp_stream__DOT__next_state = vlSelfRef.mlp_stream__DOT__state;
    vlSelfRef.mlp_stream__DOT__req_addr = 0U;
    vlSelfRef.mlp_stream__DOT__req_addr = 0U;
    vlSelfRef.mlp_stream__DOT__last_x_next_value1 = 0U;
    if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                  >> 3U)))) {
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                         < (IData)(vlSelfRef.mlp_stream__DOT__storage_3))) {
                        if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                            vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value_ce0 = 1U;
                            vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value0 
                                = vlSelfRef.mlp_stream__DOT__in_rd_dat_r;
                        }
                    }
                }
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 1U;
                        vlSelfRef.mlp_stream__DOT__in_idx_next_value3 = 0U;
                    }
                }
            } else if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 1U;
                    vlSelfRef.mlp_stream__DOT__in_idx_next_value3 
                        = (0x0000ffffU & ((IData)(1U) 
                                          + (IData)(vlSelfRef.mlp_stream__DOT__in_idx)));
                }
            }
        } else if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                if (vlSelfRef.mlp_stream__DOT__re) {
                    if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                   | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                  | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                        vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 1U;
                        vlSelfRef.mlp_stream__DOT__in_idx_next_value3 = 0U;
                    }
                }
            }
        } else if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
            if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 1U;
                vlSelfRef.mlp_stream__DOT__in_idx_next_value3 
                    = (0x0000ffffU & ((IData)(1U) + (IData)(vlSelfRef.mlp_stream__DOT__in_idx)));
            }
        }
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__re) {
                        vlSelfRef.mlp_stream__DOT__error_status_next_value_ce7 = 1U;
                        vlSelfRef.mlp_stream__DOT__error_status_next_value7 = 0U;
                        if ((((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                              | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                             | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))) {
                            vlSelfRef.mlp_stream__DOT__error_status_next_value_ce7 = 1U;
                            vlSelfRef.mlp_stream__DOT__error_status_next_value7 = 1U;
                        }
                    }
                }
            }
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__last_x_next_value1 
                            = vlSelfRef.mlp_stream__DOT__mac_x_raw;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__last_x_next_value1 
                        = vlSelfRef.mlp_stream__DOT__mac_x_raw;
                }
            }
        }
    }
    vlSelfRef.mlp_stream__DOT__mac_x_ext = (((- (IData)(
                                                        (1U 
                                                         & ((IData)(vlSelfRef.mlp_stream__DOT__mac_x_raw) 
                                                            >> 0x0000000fU)))) 
                                             << 0x00000010U) 
                                            | (IData)(vlSelfRef.mlp_stream__DOT__mac_x_raw));
    __VdfgRegularize_h6e95ff9d_0_0 = (VL_LTS_IQQ(49, 0x0000000000007fffULL, 
                                                 (0x0001ffffffffffffULL 
                                                  & VL_EXTENDS_QQ(49,48, vlSelfRef.mlp_stream__DOT__shifted)))
                                       ? 0x00007fffU
                                       : (VL_GTS_IQQ(48, 0x0000ffffffff8000ULL, vlSelfRef.mlp_stream__DOT__shifted)
                                           ? 0x00008000U
                                           : (0x0000ffffU 
                                              & (IData)(vlSelfRef.mlp_stream__DOT__shifted))));
    vlSelfRef.mlp_stream__DOT__acc_next_value2 = 0ULL;
    vlSelfRef.mlp_stream__DOT__out_write_val = __VdfgRegularize_h6e95ff9d_0_0;
    vlSelfRef.mlp_stream__DOT__hid_write_val = ((0U 
                                                 != 
                                                 ((IData)(vlSelfRef.mlp_stream__DOT__storage_1) 
                                                  & VL_GTS_IQQ(48, 0ULL, vlSelfRef.mlp_stream__DOT__shifted)))
                                                 ? 0U
                                                 : (IData)(__VdfgRegularize_h6e95ff9d_0_0));
    vlSelfRef.mlp_stream__DOT__out_val_next_value8 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_dat_w = 0U;
    if ((8U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 
                            = vlSelfRef.mlp_stream__DOT__row_base_w2;
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 = 0U;
                        vlSelfRef.mlp_stream__DOT__acc_next_value2 
                            = (((QData)((IData)((- (IData)(
                                                           (1U 
                                                            & ((IData)(vlSelfRef.mlp_stream__DOT__complexslicelowerer_slice_proxy1) 
                                                               >> 0x0fU)))))) 
                                << 0x00000010U) | (QData)((IData)(
                                                                  (0x0000ffffU 
                                                                   & vlSelfRef.mlp_stream__DOT__resp_rdata))));
                    }
                }
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                         < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) {
                        if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                            vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value_ce1 = 1U;
                            vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value1 
                                = vlSelfRef.mlp_stream__DOT__hid_rd_dat_r;
                        }
                    }
                }
            } else if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 
                    = (0x0000ffffU & ((IData)(1U) + (IData)(vlSelfRef.mlp_stream__DOT__hid_idx)));
            }
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value_ce1 = 1U;
                    vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value1 
                        = (vlSelfRef.mlp_stream__DOT__row_base_w1 
                           + ((IData)(vlSelfRef.mlp_stream__DOT__storage_3) 
                              << 2U));
                    vlSelfRef.mlp_stream__DOT__hid_wr_dat_w 
                        = vlSelfRef.mlp_stream__DOT__hid_write_val;
                }
            }
        }
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                        vlSelfRef.mlp_stream__DOT__req_wdata 
                            = (((- (IData)((1U & ((IData)(vlSelfRef.mlp_stream__DOT__out_val) 
                                                  >> 0x0fU)))) 
                                << 0x00000010U) | (IData)(vlSelfRef.mlp_stream__DOT__out_val));
                        vlSelfRef.mlp_stream__DOT__req_we = 1U;
                        vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                        vlSelfRef.mlp_stream__DOT__req_addr 
                            = (vlSelfRef.mlp_stream__DOT__storage_11 
                               + ((IData)(vlSelfRef.mlp_stream__DOT__out_idx) 
                                  << 2U));
                    }
                }
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__out_val_next_value8 
                        = vlSelfRef.mlp_stream__DOT__out_write_val;
                }
            }
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 1U;
                    vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 1U;
                    if (vlSelfRef.mlp_stream__DOT__re) {
                        vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 0U;
                        vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 1U;
                        vlSelfRef.mlp_stream__DOT__next_state = 2U;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__next_state = 9U;
                }
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value_ce2 = 1U;
                        vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0 = 1U;
                        vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value2 
                            = (vlSelfRef.mlp_stream__DOT__row_base_w2 
                               + ((IData)(vlSelfRef.mlp_stream__DOT__storage_4) 
                                  << 2U));
                        vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0 
                            = (0x0000ffffU & ((IData)(1U) 
                                              + (IData)(vlSelfRef.mlp_stream__DOT__out_idx)));
                    }
                }
            } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                    vlSelfRef.mlp_stream__DOT__next_state = 0x0eU;
                }
            } else {
                vlSelfRef.mlp_stream__DOT__next_state = 0x0dU;
            }
        } else if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                     < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) {
                    if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                        vlSelfRef.mlp_stream__DOT__req_we = 0U;
                        vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                        vlSelfRef.mlp_stream__DOT__next_state = 1U;
                        vlSelfRef.mlp_stream__DOT__req_addr 
                            = vlSelfRef.mlp_stream__DOT__weight_addr;
                    }
                } else {
                    vlSelfRef.mlp_stream__DOT__next_state = 0x0cU;
                }
            } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__next_state = 0x0bU;
            }
        } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if (((IData)(vlSelfRef.mlp_stream__DOT__out_idx) 
                 < (IData)(vlSelfRef.mlp_stream__DOT__storage_5))) {
                if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                    vlSelfRef.mlp_stream__DOT__req_we = 0U;
                    vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                    vlSelfRef.mlp_stream__DOT__next_state = 0x0aU;
                    vlSelfRef.mlp_stream__DOT__req_addr 
                        = (vlSelfRef.mlp_stream__DOT__storage_10 
                           + ((IData)(vlSelfRef.mlp_stream__DOT__out_idx) 
                              << 2U));
                }
            } else {
                vlSelfRef.mlp_stream__DOT__next_state = 0x0fU;
            }
        } else {
            vlSelfRef.mlp_stream__DOT__next_state = 5U;
        }
    } else {
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 
                            = vlSelfRef.mlp_stream__DOT__row_base_w1;
                        vlSelfRef.mlp_stream__DOT__acc_next_value2 
                            = (((QData)((IData)((- (IData)(
                                                           (1U 
                                                            & ((IData)(vlSelfRef.mlp_stream__DOT__complexslicelowerer_slice_proxy0) 
                                                               >> 0x0fU)))))) 
                                << 0x00000010U) | (QData)((IData)(
                                                                  (0x0000ffffU 
                                                                   & vlSelfRef.mlp_stream__DOT__resp_rdata))));
                    }
                }
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                         < (IData)(vlSelfRef.mlp_stream__DOT__storage_3))) {
                        if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                            vlSelfRef.mlp_stream__DOT__req_we = 0U;
                            vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                            vlSelfRef.mlp_stream__DOT__next_state = 0U;
                            vlSelfRef.mlp_stream__DOT__req_addr 
                                = vlSelfRef.mlp_stream__DOT__weight_addr;
                        }
                    } else {
                        vlSelfRef.mlp_stream__DOT__next_state = 8U;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__next_state = 7U;
                }
            } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                     < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) {
                    if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                        vlSelfRef.mlp_stream__DOT__req_we = 0U;
                        vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                        vlSelfRef.mlp_stream__DOT__next_state = 6U;
                        vlSelfRef.mlp_stream__DOT__req_addr 
                            = (vlSelfRef.mlp_stream__DOT__storage_8 
                               + ((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                                  << 2U));
                    }
                } else {
                    vlSelfRef.mlp_stream__DOT__next_state = 9U;
                }
            } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__next_state = 3U;
            }
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                         >= (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) {
                        vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0 = 1U;
                        vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0 = 0U;
                    }
                }
            }
        } else {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 
                            = ((IData)(4U) + vlSelfRef.mlp_stream__DOT__weight_addr);
                        vlSelfRef.mlp_stream__DOT__acc_next_value2 
                            = (0x0000ffffffffffffULL 
                               & (vlSelfRef.mlp_stream__DOT__acc 
                                  + VL_MULS_QQQ(48, 
                                                (0x0000ffffffffffffULL 
                                                 & VL_EXTENDS_QI(48,32, vlSelfRef.mlp_stream__DOT__mac_w_ext)), 
                                                (0x0000ffffffffffffULL 
                                                 & VL_EXTENDS_QI(48,32, vlSelfRef.mlp_stream__DOT__mac_x_ext)))));
                    }
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 
                        = ((IData)(4U) + vlSelfRef.mlp_stream__DOT__weight_addr);
                    vlSelfRef.mlp_stream__DOT__acc_next_value2 
                        = (0x0000ffffffffffffULL & 
                           (vlSelfRef.mlp_stream__DOT__acc 
                            + VL_MULS_QQQ(48, (0x0000ffffffffffffULL 
                                               & VL_EXTENDS_QI(48,32, vlSelfRef.mlp_stream__DOT__mac_w_ext)), 
                                          (0x0000ffffffffffffULL 
                                           & VL_EXTENDS_QI(48,32, vlSelfRef.mlp_stream__DOT__mac_x_ext)))));
                }
            }
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__re) {
                        if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                       | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                      | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                            vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0 = 1U;
                            vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0 = 0U;
                        }
                    }
                }
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                         < (IData)(vlSelfRef.mlp_stream__DOT__storage_3))) {
                        if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                            vlSelfRef.mlp_stream__DOT__req_we = 0U;
                            vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                            vlSelfRef.mlp_stream__DOT__next_state = 4U;
                            vlSelfRef.mlp_stream__DOT__req_addr 
                                = (vlSelfRef.mlp_stream__DOT__storage_6 
                                   + ((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                                      << 2U));
                        }
                    } else {
                        vlSelfRef.mlp_stream__DOT__next_state = 5U;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__re) {
                    if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                   | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                  | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                        vlSelfRef.mlp_stream__DOT__next_state = 3U;
                    }
                }
            } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__next_state = 0x0bU;
                }
            } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__next_state = 7U;
            }
        }
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__re) {
                        if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                       | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                      | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                            vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value_ce1 = 1U;
                            vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value_ce2 = 1U;
                            vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value1 
                                = vlSelfRef.mlp_stream__DOT__storage_7;
                            vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value2 
                                = vlSelfRef.mlp_stream__DOT__storage_9;
                        }
                        vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 0U;
                        vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 1U;
                        if ((((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                              | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                             | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))) {
                            vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 1U;
                            vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 1U;
                        }
                    }
                }
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                         >= (IData)(vlSelfRef.mlp_stream__DOT__storage_3))) {
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 = 0U;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__re) {
                    if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                   | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                  | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 = 0U;
                    }
                }
            } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                    vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 
                        = (0x0000ffffU & ((IData)(1U) 
                                          + (IData)(vlSelfRef.mlp_stream__DOT__hid_idx)));
                }
            }
        }
    }
}

void Vtop___024root___eval_ico(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_ico\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1ULL & vlSelfRef.__VicoTriggered[0U])) {
        Vtop___024root___ico_sequent__TOP__0(vlSelf);
    }
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtop___024root___dump_triggers__ico(const VlUnpacked<QData/*63:0*/, 1> &triggers, const std::string &tag);
#endif  // VL_DEBUG

bool Vtop___024root___eval_phase__ico(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_phase__ico\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Locals
    CData/*0:0*/ __VicoExecute;
    // Body
    Vtop___024root___eval_triggers_vec__ico(vlSelf);
#ifdef VL_DEBUG
    if (VL_UNLIKELY(vlSymsp->_vm_contextp__->debug())) {
        Vtop___024root___dump_triggers__ico(vlSelfRef.__VicoTriggered, "ico"s);
    }
#endif
    __VicoExecute = Vtop___024root___trigger_anySet__ico(vlSelfRef.__VicoTriggered);
    if (__VicoExecute) {
        Vtop___024root___eval_ico(vlSelf);
    }
    return (__VicoExecute);
}

void Vtop___024root___eval_triggers_vec__act(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_triggers_vec__act\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.__VactTriggered[0U] = (QData)((IData)(
                                                    ((IData)(vlSelfRef.mlp_stream__DOT__sys_clk_1) 
                                                     & (~ (IData)(vlSelfRef.__Vtrigprevexpr___TOP__mlp_stream__DOT__sys_clk_1__0)))));
    vlSelfRef.__Vtrigprevexpr___TOP__mlp_stream__DOT__sys_clk_1__0 
        = vlSelfRef.mlp_stream__DOT__sys_clk_1;
}

bool Vtop___024root___trigger_anySet__act(const VlUnpacked<QData/*63:0*/, 1> &in) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___trigger_anySet__act\n"); );
    // Locals
    IData/*31:0*/ n;
    // Body
    n = 0U;
    do {
        if (in[n]) {
            return (1U);
        }
        n = ((IData)(1U) + n);
    } while ((1U > n));
    return (0U);
}

void Vtop___024root___nba_sequent__TOP__0(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___nba_sequent__TOP__0\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Locals
    SData/*15:0*/ __VdfgRegularize_h6e95ff9d_0_0;
    __VdfgRegularize_h6e95ff9d_0_0 = 0;
    SData/*15:0*/ __VdlyVal__mlp_stream__DOT__in_mem__v0;
    __VdlyVal__mlp_stream__DOT__in_mem__v0 = 0;
    SData/*9:0*/ __VdlyDim0__mlp_stream__DOT__in_mem__v0;
    __VdlyDim0__mlp_stream__DOT__in_mem__v0 = 0;
    CData/*0:0*/ __VdlySet__mlp_stream__DOT__in_mem__v0;
    __VdlySet__mlp_stream__DOT__in_mem__v0 = 0;
    SData/*15:0*/ __VdlyVal__mlp_stream__DOT__hid_mem__v0;
    __VdlyVal__mlp_stream__DOT__hid_mem__v0 = 0;
    CData/*6:0*/ __VdlyDim0__mlp_stream__DOT__hid_mem__v0;
    __VdlyDim0__mlp_stream__DOT__hid_mem__v0 = 0;
    CData/*0:0*/ __VdlySet__mlp_stream__DOT__hid_mem__v0;
    __VdlySet__mlp_stream__DOT__hid_mem__v0 = 0;
    // Body
    __VdlySet__mlp_stream__DOT__hid_mem__v0 = 0U;
    __VdlySet__mlp_stream__DOT__in_mem__v0 = 0U;
    if (vlSelfRef.mlp_stream__DOT__hid_wr_we) {
        __VdlyVal__mlp_stream__DOT__hid_mem__v0 = vlSelfRef.mlp_stream__DOT__hid_wr_dat_w;
        __VdlyDim0__mlp_stream__DOT__hid_mem__v0 = vlSelfRef.mlp_stream__DOT__hid_wr_adr;
        __VdlySet__mlp_stream__DOT__hid_mem__v0 = 1U;
    }
    if (vlSelfRef.mlp_stream__DOT__in_wr_we) {
        vlSelfRef.mlp_stream__DOT____Vlvbound_hc2aa3eeb__0 
            = vlSelfRef.mlp_stream__DOT__in_wr_dat_w;
        if ((0x030fU >= (IData)(vlSelfRef.mlp_stream__DOT__in_wr_adr))) {
            __VdlyVal__mlp_stream__DOT__in_mem__v0 
                = vlSelfRef.mlp_stream__DOT____Vlvbound_hc2aa3eeb__0;
            __VdlyDim0__mlp_stream__DOT__in_mem__v0 
                = vlSelfRef.mlp_stream__DOT__in_wr_adr;
            __VdlySet__mlp_stream__DOT__in_mem__v0 = 1U;
        }
    }
    vlSelfRef.mlp_stream__DOT__memadr_1 = vlSelfRef.mlp_stream__DOT__hid_wr_adr;
    vlSelfRef.mlp_stream__DOT__memadr = vlSelfRef.mlp_stream__DOT__in_wr_adr;
    if (vlSelfRef.mlp_stream__DOT__out_val_next_value_ce8) {
        vlSelfRef.mlp_stream__DOT__out_val = vlSelfRef.mlp_stream__DOT__out_val_next_value8;
    }
    if (vlSelfRef.mlp_stream__DOT__error_status_next_value_ce7) {
        vlSelfRef.mlp_stream__DOT__status_2 = vlSelfRef.mlp_stream__DOT__error_status_next_value7;
    }
    if (vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6) {
        vlSelfRef.mlp_stream__DOT__status_1 = vlSelfRef.mlp_stream__DOT__done_status_next_value6;
    }
    if (vlSelfRef.mlp_stream__DOT__last_w_next_value_ce0) {
        vlSelfRef.mlp_stream__DOT__last_w = vlSelfRef.mlp_stream__DOT__last_w_next_value0;
    }
    if (vlSelfRef.mlp_stream__DOT__last_x_next_value_ce1) {
        vlSelfRef.mlp_stream__DOT__last_x = vlSelfRef.mlp_stream__DOT__last_x_next_value1;
    }
    if (vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value_ce1) {
        vlSelfRef.mlp_stream__DOT__row_base_w1 = vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value1;
    }
    if (vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value_ce2) {
        vlSelfRef.mlp_stream__DOT__row_base_w2 = vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value2;
    }
    if (vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4) {
        vlSelfRef.mlp_stream__DOT__weight_addr = vlSelfRef.mlp_stream__DOT__weight_addr_next_value4;
    }
    if (vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value_ce0) {
        vlSelfRef.mlp_stream__DOT__in_x_reg = vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value0;
    }
    if (vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value_ce1) {
        vlSelfRef.mlp_stream__DOT__hid_x_reg = vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value1;
    }
    if (vlSelfRef.mlp_stream__DOT__acc_next_value_ce2) {
        vlSelfRef.mlp_stream__DOT__acc = vlSelfRef.mlp_stream__DOT__acc_next_value2;
    }
    if (vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0) {
        vlSelfRef.mlp_stream__DOT__out_idx = vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0;
    }
    if (vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3) {
        vlSelfRef.mlp_stream__DOT__in_idx = vlSelfRef.mlp_stream__DOT__in_idx_next_value3;
    }
    if (vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5) {
        vlSelfRef.mlp_stream__DOT__hid_idx = vlSelfRef.mlp_stream__DOT__hid_idx_next_value5;
    }
    vlSelfRef.mlp_stream__DOT__resp_valid = 0U;
    if (vlSelfRef.mlp_stream__DOT__wb_cyc) {
        if (vlSelfRef.mlp_stream__DOT__bus_ack) {
            vlSelfRef.mlp_stream__DOT__wb_cyc = 0U;
            vlSelfRef.mlp_stream__DOT__wb_stb = 0U;
            vlSelfRef.mlp_stream__DOT__resp_valid = 1U;
            vlSelfRef.mlp_stream__DOT__resp_rdata = vlSelfRef.mlp_stream__DOT__bus_dat_r;
        }
    } else if (vlSelfRef.mlp_stream__DOT__req_valid) {
        vlSelfRef.mlp_stream__DOT__wb_cyc = 1U;
        vlSelfRef.mlp_stream__DOT__wb_stb = 1U;
        vlSelfRef.mlp_stream__DOT__wb_we = vlSelfRef.mlp_stream__DOT__req_we;
        vlSelfRef.mlp_stream__DOT__wb_adr = vlSelfRef.mlp_stream__DOT__req_addr;
        vlSelfRef.mlp_stream__DOT__wb_dat_w = vlSelfRef.mlp_stream__DOT__req_wdata;
        vlSelfRef.mlp_stream__DOT__wb_sel = 0x0fU;
    }
    vlSelfRef.mlp_stream__DOT__state = vlSelfRef.mlp_stream__DOT__next_state;
    if (vlSelfRef.mlp_stream__DOT__sys_rst_1) {
        vlSelfRef.mlp_stream__DOT__out_val = 0U;
        vlSelfRef.mlp_stream__DOT__status_2 = 0U;
        vlSelfRef.mlp_stream__DOT__status_1 = 0U;
        vlSelfRef.mlp_stream__DOT__last_w = 0U;
        vlSelfRef.mlp_stream__DOT__last_x = 0U;
        vlSelfRef.mlp_stream__DOT__row_base_w1 = 0U;
        vlSelfRef.mlp_stream__DOT__row_base_w2 = 0U;
        vlSelfRef.mlp_stream__DOT__weight_addr = 0U;
        vlSelfRef.mlp_stream__DOT__in_x_reg = 0U;
        vlSelfRef.mlp_stream__DOT__hid_x_reg = 0U;
        vlSelfRef.mlp_stream__DOT__acc = 0ULL;
        vlSelfRef.mlp_stream__DOT__out_idx = 0U;
        vlSelfRef.mlp_stream__DOT__in_idx = 0U;
        vlSelfRef.mlp_stream__DOT__hid_idx = 0U;
        vlSelfRef.mlp_stream__DOT__wb_cyc = 0U;
        vlSelfRef.mlp_stream__DOT__wb_stb = 0U;
        vlSelfRef.mlp_stream__DOT__wb_we = 0U;
        vlSelfRef.mlp_stream__DOT__wb_adr = 0U;
        vlSelfRef.mlp_stream__DOT__wb_dat_w = 0U;
        vlSelfRef.mlp_stream__DOT__wb_sel = 0x0fU;
        vlSelfRef.mlp_stream__DOT__resp_valid = 0U;
        vlSelfRef.mlp_stream__DOT__resp_rdata = 0U;
        vlSelfRef.mlp_stream__DOT__state = 2U;
    }
    if (__VdlySet__mlp_stream__DOT__hid_mem__v0) {
        vlSelfRef.mlp_stream__DOT__hid_mem[__VdlyDim0__mlp_stream__DOT__hid_mem__v0] 
            = __VdlyVal__mlp_stream__DOT__hid_mem__v0;
    }
    if (__VdlySet__mlp_stream__DOT__in_mem__v0) {
        vlSelfRef.mlp_stream__DOT__in_mem[__VdlyDim0__mlp_stream__DOT__in_mem__v0] 
            = __VdlyVal__mlp_stream__DOT__in_mem__v0;
    }
    vlSelfRef.mlp_stream__DOT__hid_wr_dat_r = vlSelfRef.mlp_stream__DOT__hid_mem
        [vlSelfRef.mlp_stream__DOT__memadr_1];
    vlSelfRef.mlp_stream__DOT__in_wr_dat_r = ((0x030fU 
                                               >= (IData)(vlSelfRef.mlp_stream__DOT__memadr))
                                               ? vlSelfRef.mlp_stream__DOT__in_mem
                                              [vlSelfRef.mlp_stream__DOT__memadr]
                                               : 0U);
    vlSelfRef.status_2 = vlSelfRef.mlp_stream__DOT__status_2;
    vlSelfRef.status_1 = vlSelfRef.mlp_stream__DOT__status_1;
    vlSelfRef.mlp_stream__DOT__status_5 = vlSelfRef.mlp_stream__DOT__last_w;
    vlSelfRef.mlp_stream__DOT__status_6 = vlSelfRef.mlp_stream__DOT__last_x;
    vlSelfRef.mlp_stream__DOT__status_3 = (IData)(vlSelfRef.mlp_stream__DOT__acc);
    vlSelfRef.mlp_stream__DOT__status_4 = (0x0000ffffU 
                                           & (IData)(
                                                     (vlSelfRef.mlp_stream__DOT__acc 
                                                      >> 0x00000020U)));
    vlSelfRef.mlp_stream__DOT__shifted = (0x0000ffffffffffffULL 
                                          & VL_SHIFTRS_QQI(48,48,5, vlSelfRef.mlp_stream__DOT__acc, (IData)(vlSelfRef.mlp_stream__DOT__storage_2)));
    vlSelfRef.mlp_stream__DOT__in_rd_dat_r = ((0x030fU 
                                               >= (0x000003ffU 
                                                   & (IData)(vlSelfRef.mlp_stream__DOT__in_idx)))
                                               ? vlSelfRef.mlp_stream__DOT__in_mem
                                              [(0x000003ffU 
                                                & (IData)(vlSelfRef.mlp_stream__DOT__in_idx))]
                                               : 0U);
    vlSelfRef.mlp_stream__DOT__hid_rd_dat_r = vlSelfRef.mlp_stream__DOT__hid_mem
        [(0x0000007fU & (IData)(vlSelfRef.mlp_stream__DOT__hid_idx))];
    vlSelfRef.mlp_stream__DOT__bus_stb = vlSelfRef.mlp_stream__DOT__wb_stb;
    vlSelfRef.mlp_stream__DOT__bus_we = vlSelfRef.mlp_stream__DOT__wb_we;
    vlSelfRef.mlp_stream__DOT__bus_adr = VL_SHIFTR_III(32,32,32, vlSelfRef.mlp_stream__DOT__wb_adr, 2U);
    vlSelfRef.mlp_stream__DOT__bus_dat_w = vlSelfRef.mlp_stream__DOT__wb_dat_w;
    vlSelfRef.mlp_stream__DOT__bus_sel = vlSelfRef.mlp_stream__DOT__wb_sel;
    vlSelfRef.mlp_stream__DOT__complexslicelowerer_slice_proxy0 
        = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
    vlSelfRef.mlp_stream__DOT__complexslicelowerer_slice_proxy1 
        = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
    vlSelfRef.mlp_stream__DOT__rhs_slice_proxy = (0x0000ffffU 
                                                  & vlSelfRef.mlp_stream__DOT__resp_rdata);
    vlSelfRef.mlp_stream__DOT__bus_cyc = vlSelfRef.mlp_stream__DOT__wb_cyc;
    vlSelfRef.mlp_stream__DOT__bus_idle = (1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__wb_cyc)));
    vlSelfRef.mlp_stream__DOT__out_val_next_value_ce8 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_we = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_we = 0U;
    vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 0U;
    vlSelfRef.mlp_stream__DOT__last_x_next_value_ce1 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_adr = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_adr = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_we = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_we = 0U;
    vlSelfRef.mlp_stream__DOT__last_w_next_value_ce0 = 0U;
    vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_adr = 0U;
    vlSelfRef.mlp_stream__DOT__in_wr_adr = 0U;
    vlSelfRef.mlp_stream__DOT__last_w_next_value0 = 0U;
    vlSelfRef.mlp_stream__DOT__status = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value_ce1 = 0U;
    vlSelfRef.mlp_stream__DOT__error_status_next_value_ce7 = 0U;
    vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 = 0U;
    vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 0U;
    vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 0U;
    vlSelfRef.mlp_stream__DOT__error_status_next_value7 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value_ce2 = 0U;
    vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value1 = 0U;
    vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0 = 0U;
    vlSelfRef.mlp_stream__DOT__in_idx_next_value3 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value2 = 0U;
    vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 = 0U;
    vlSelfRef.mlp_stream__DOT__is_ongoing1 = 0U;
    vlSelfRef.mlp_stream__DOT__is_ongoing0 = 0U;
    vlSelfRef.status_5 = vlSelfRef.mlp_stream__DOT__status_5;
    vlSelfRef.status_6 = vlSelfRef.mlp_stream__DOT__status_6;
    vlSelfRef.status_3 = vlSelfRef.mlp_stream__DOT__status_3;
    vlSelfRef.status_4 = vlSelfRef.mlp_stream__DOT__status_4;
    __VdfgRegularize_h6e95ff9d_0_0 = (VL_LTS_IQQ(49, 0x0000000000007fffULL, 
                                                 (0x0001ffffffffffffULL 
                                                  & VL_EXTENDS_QQ(49,48, vlSelfRef.mlp_stream__DOT__shifted)))
                                       ? 0x00007fffU
                                       : (VL_GTS_IQQ(48, 0x0000ffffffff8000ULL, vlSelfRef.mlp_stream__DOT__shifted)
                                           ? 0x00008000U
                                           : (0x0000ffffU 
                                              & (IData)(vlSelfRef.mlp_stream__DOT__shifted))));
    vlSelfRef.bus_stb = vlSelfRef.mlp_stream__DOT__bus_stb;
    vlSelfRef.bus_we = vlSelfRef.mlp_stream__DOT__bus_we;
    vlSelfRef.bus_adr = vlSelfRef.mlp_stream__DOT__bus_adr;
    vlSelfRef.bus_dat_w = vlSelfRef.mlp_stream__DOT__bus_dat_w;
    vlSelfRef.bus_sel = vlSelfRef.mlp_stream__DOT__bus_sel;
    vlSelfRef.mlp_stream__DOT__mac_w_ext = (((- (IData)(
                                                        (1U 
                                                         & ((IData)(vlSelfRef.mlp_stream__DOT__rhs_slice_proxy) 
                                                            >> 0x0000000fU)))) 
                                             << 0x00000010U) 
                                            | (0x0000ffffU 
                                               & vlSelfRef.mlp_stream__DOT__resp_rdata));
    vlSelfRef.bus_cyc = vlSelfRef.mlp_stream__DOT__bus_cyc;
    vlSelfRef.mlp_stream__DOT__req_wdata = 0U;
    vlSelfRef.mlp_stream__DOT__req_wdata = 0U;
    vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value_ce1 = 0U;
    vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value_ce0 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value1 = 0U;
    vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value0 = 0U;
    vlSelfRef.mlp_stream__DOT__req_we = 0U;
    vlSelfRef.mlp_stream__DOT__req_we = 0U;
    vlSelfRef.mlp_stream__DOT__req_valid = 0U;
    vlSelfRef.mlp_stream__DOT__req_valid = 0U;
    vlSelfRef.mlp_stream__DOT__next_state = 0U;
    vlSelfRef.mlp_stream__DOT__next_state = vlSelfRef.mlp_stream__DOT__state;
    vlSelfRef.mlp_stream__DOT__req_addr = 0U;
    vlSelfRef.mlp_stream__DOT__req_addr = 0U;
    if ((8U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__out_val_next_value_ce8 = 1U;
                }
            }
        }
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__hid_wr_we = 1U;
                    vlSelfRef.mlp_stream__DOT__hid_wr_adr 
                        = (0x0000007fU & (IData)(vlSelfRef.mlp_stream__DOT__hid_idx));
                }
            }
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 1U;
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 1U;
                    }
                }
            }
        }
        vlSelfRef.mlp_stream__DOT__status = ((1U & 
                                              (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                                                  >> 2U))) 
                                             || ((1U 
                                                  & (~ 
                                                     ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                                                      >> 1U))) 
                                                 || (1U 
                                                     & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))));
    } else {
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 1U;
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 1U;
                    }
                }
            }
        } else if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                             >> 1U)))) {
            if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 1U;
                    vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 1U;
                }
            } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__acc_next_value_ce2 = 1U;
                vlSelfRef.mlp_stream__DOT__weight_addr_next_value_ce4 = 1U;
            }
        }
        vlSelfRef.mlp_stream__DOT__status = ((1U & 
                                              ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                                               >> 2U)) 
                                             || ((1U 
                                                  & (~ 
                                                     ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                                                      >> 1U))) 
                                                 || (1U 
                                                     & (IData)(vlSelfRef.mlp_stream__DOT__state))));
    }
    vlSelfRef.status = vlSelfRef.mlp_stream__DOT__status;
    if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                  >> 3U)))) {
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__last_x_next_value_ce1 = 1U;
                        vlSelfRef.mlp_stream__DOT__last_w_next_value_ce0 = 1U;
                        vlSelfRef.mlp_stream__DOT__last_w_next_value0 
                            = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
                    }
                    vlSelfRef.mlp_stream__DOT__is_ongoing1 = 1U;
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__last_x_next_value_ce1 = 1U;
                    vlSelfRef.mlp_stream__DOT__last_w_next_value_ce0 = 1U;
                    vlSelfRef.mlp_stream__DOT__last_w_next_value0 
                        = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
                }
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__is_ongoing0 = 1U;
                }
            }
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__re) {
                        vlSelfRef.mlp_stream__DOT__error_status_next_value_ce7 = 1U;
                        vlSelfRef.mlp_stream__DOT__error_status_next_value7 = 0U;
                        if ((((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                              | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                             | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))) {
                            vlSelfRef.mlp_stream__DOT__error_status_next_value_ce7 = 1U;
                            vlSelfRef.mlp_stream__DOT__error_status_next_value7 = 1U;
                        }
                    }
                }
            }
        }
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__in_wr_we = 1U;
                        vlSelfRef.mlp_stream__DOT__in_wr_dat_w 
                            = (0x0000ffffU & vlSelfRef.mlp_stream__DOT__resp_rdata);
                        vlSelfRef.mlp_stream__DOT__in_wr_adr 
                            = (0x000003ffU & (IData)(vlSelfRef.mlp_stream__DOT__in_idx));
                    }
                }
            }
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 1U;
                        vlSelfRef.mlp_stream__DOT__in_idx_next_value3 = 0U;
                    }
                }
            } else if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 1U;
                    vlSelfRef.mlp_stream__DOT__in_idx_next_value3 
                        = (0x0000ffffU & ((IData)(1U) 
                                          + (IData)(vlSelfRef.mlp_stream__DOT__in_idx)));
                }
            }
        } else if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                if (vlSelfRef.mlp_stream__DOT__re) {
                    if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                   | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                  | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                        vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 1U;
                        vlSelfRef.mlp_stream__DOT__in_idx_next_value3 = 0U;
                    }
                }
            }
        } else if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
            if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__in_idx_next_value_ce3 = 1U;
                vlSelfRef.mlp_stream__DOT__in_idx_next_value3 
                    = (0x0000ffffU & ((IData)(1U) + (IData)(vlSelfRef.mlp_stream__DOT__in_idx)));
            }
        }
    }
    vlSelfRef.mlp_stream__DOT__mac_x_raw = ((IData)(vlSelfRef.mlp_stream__DOT__is_ongoing0)
                                             ? (IData)(vlSelfRef.mlp_stream__DOT__in_x_reg)
                                             : ((IData)(vlSelfRef.mlp_stream__DOT__is_ongoing1)
                                                 ? (IData)(vlSelfRef.mlp_stream__DOT__hid_x_reg)
                                                 : 0U));
    vlSelfRef.mlp_stream__DOT__out_write_val = __VdfgRegularize_h6e95ff9d_0_0;
    vlSelfRef.mlp_stream__DOT__hid_write_val = ((0U 
                                                 != 
                                                 ((IData)(vlSelfRef.mlp_stream__DOT__storage_1) 
                                                  & VL_GTS_IQQ(48, 0ULL, vlSelfRef.mlp_stream__DOT__shifted)))
                                                 ? 0U
                                                 : (IData)(__VdfgRegularize_h6e95ff9d_0_0));
    vlSelfRef.mlp_stream__DOT__last_x_next_value1 = 0U;
    if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                  >> 3U)))) {
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                         < (IData)(vlSelfRef.mlp_stream__DOT__storage_3))) {
                        if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                            vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value_ce0 = 1U;
                            vlSelfRef.mlp_stream__DOT__in_x_reg_t_next_value0 
                                = vlSelfRef.mlp_stream__DOT__in_rd_dat_r;
                        }
                    }
                }
            }
        }
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__last_x_next_value1 
                            = vlSelfRef.mlp_stream__DOT__mac_x_raw;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__last_x_next_value1 
                        = vlSelfRef.mlp_stream__DOT__mac_x_raw;
                }
            }
        }
    }
    vlSelfRef.mlp_stream__DOT__mac_x_ext = (((- (IData)(
                                                        (1U 
                                                         & ((IData)(vlSelfRef.mlp_stream__DOT__mac_x_raw) 
                                                            >> 0x0000000fU)))) 
                                             << 0x00000010U) 
                                            | (IData)(vlSelfRef.mlp_stream__DOT__mac_x_raw));
    vlSelfRef.mlp_stream__DOT__out_val_next_value8 = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__hid_wr_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__acc_next_value2 = 0ULL;
    if ((8U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value_ce1 = 1U;
                    vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value1 
                        = (vlSelfRef.mlp_stream__DOT__row_base_w1 
                           + ((IData)(vlSelfRef.mlp_stream__DOT__storage_3) 
                              << 2U));
                    vlSelfRef.mlp_stream__DOT__hid_wr_dat_w 
                        = vlSelfRef.mlp_stream__DOT__hid_write_val;
                }
            }
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 
                            = vlSelfRef.mlp_stream__DOT__row_base_w2;
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 = 0U;
                        vlSelfRef.mlp_stream__DOT__acc_next_value2 
                            = (((QData)((IData)((- (IData)(
                                                           (1U 
                                                            & ((IData)(vlSelfRef.mlp_stream__DOT__complexslicelowerer_slice_proxy1) 
                                                               >> 0x0fU)))))) 
                                << 0x00000010U) | (QData)((IData)(
                                                                  (0x0000ffffU 
                                                                   & vlSelfRef.mlp_stream__DOT__resp_rdata))));
                    }
                }
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                         < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) {
                        if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                            vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value_ce1 = 1U;
                            vlSelfRef.mlp_stream__DOT__hid_x_reg_t_next_value1 
                                = vlSelfRef.mlp_stream__DOT__hid_rd_dat_r;
                        }
                    }
                }
            } else if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 
                    = (0x0000ffffU & ((IData)(1U) + (IData)(vlSelfRef.mlp_stream__DOT__hid_idx)));
            }
        }
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 1U;
                    vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 1U;
                    if (vlSelfRef.mlp_stream__DOT__re) {
                        vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 0U;
                        vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 1U;
                        vlSelfRef.mlp_stream__DOT__next_state = 2U;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__next_state = 9U;
                }
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value_ce2 = 1U;
                        vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0 = 1U;
                        vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value2 
                            = (vlSelfRef.mlp_stream__DOT__row_base_w2 
                               + ((IData)(vlSelfRef.mlp_stream__DOT__storage_4) 
                                  << 2U));
                        vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0 
                            = (0x0000ffffU & ((IData)(1U) 
                                              + (IData)(vlSelfRef.mlp_stream__DOT__out_idx)));
                    }
                }
            } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                    vlSelfRef.mlp_stream__DOT__next_state = 0x0eU;
                }
            } else {
                vlSelfRef.mlp_stream__DOT__next_state = 0x0dU;
            }
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                        vlSelfRef.mlp_stream__DOT__req_wdata 
                            = (((- (IData)((1U & ((IData)(vlSelfRef.mlp_stream__DOT__out_val) 
                                                  >> 0x0fU)))) 
                                << 0x00000010U) | (IData)(vlSelfRef.mlp_stream__DOT__out_val));
                        vlSelfRef.mlp_stream__DOT__req_we = 1U;
                        vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                        vlSelfRef.mlp_stream__DOT__req_addr 
                            = (vlSelfRef.mlp_stream__DOT__storage_11 
                               + ((IData)(vlSelfRef.mlp_stream__DOT__out_idx) 
                                  << 2U));
                    }
                }
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    vlSelfRef.mlp_stream__DOT__out_val_next_value8 
                        = vlSelfRef.mlp_stream__DOT__out_write_val;
                }
            }
        } else if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                     < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) {
                    if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                        vlSelfRef.mlp_stream__DOT__req_we = 0U;
                        vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                        vlSelfRef.mlp_stream__DOT__next_state = 1U;
                        vlSelfRef.mlp_stream__DOT__req_addr 
                            = vlSelfRef.mlp_stream__DOT__weight_addr;
                    }
                } else {
                    vlSelfRef.mlp_stream__DOT__next_state = 0x0cU;
                }
            } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__next_state = 0x0bU;
            }
        } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if (((IData)(vlSelfRef.mlp_stream__DOT__out_idx) 
                 < (IData)(vlSelfRef.mlp_stream__DOT__storage_5))) {
                if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                    vlSelfRef.mlp_stream__DOT__req_we = 0U;
                    vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                    vlSelfRef.mlp_stream__DOT__next_state = 0x0aU;
                    vlSelfRef.mlp_stream__DOT__req_addr 
                        = (vlSelfRef.mlp_stream__DOT__storage_10 
                           + ((IData)(vlSelfRef.mlp_stream__DOT__out_idx) 
                              << 2U));
                }
            } else {
                vlSelfRef.mlp_stream__DOT__next_state = 0x0fU;
            }
        } else {
            vlSelfRef.mlp_stream__DOT__next_state = 5U;
        }
    } else {
        if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                      >> 2U)))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__re) {
                        if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                       | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                      | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                            vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value_ce1 = 1U;
                            vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value_ce2 = 1U;
                            vlSelfRef.mlp_stream__DOT__row_base_w1_f_next_value1 
                                = vlSelfRef.mlp_stream__DOT__storage_7;
                            vlSelfRef.mlp_stream__DOT__row_base_w2_f_next_value2 
                                = vlSelfRef.mlp_stream__DOT__storage_9;
                        }
                        vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 0U;
                        vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 1U;
                        if ((((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                              | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                             | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))) {
                            vlSelfRef.mlp_stream__DOT__done_status_next_value6 = 1U;
                            vlSelfRef.mlp_stream__DOT__done_status_next_value_ce6 = 1U;
                        }
                    }
                }
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                         >= (IData)(vlSelfRef.mlp_stream__DOT__storage_3))) {
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 = 0U;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__re) {
                    if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                   | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                  | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                        vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 = 0U;
                    }
                }
            } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__hid_idx_next_value_ce5 = 1U;
                    vlSelfRef.mlp_stream__DOT__hid_idx_next_value5 
                        = (0x0000ffffU & ((IData)(1U) 
                                          + (IData)(vlSelfRef.mlp_stream__DOT__hid_idx)));
                }
            }
        }
        if ((4U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 
                            = vlSelfRef.mlp_stream__DOT__row_base_w1;
                        vlSelfRef.mlp_stream__DOT__acc_next_value2 
                            = (((QData)((IData)((- (IData)(
                                                           (1U 
                                                            & ((IData)(vlSelfRef.mlp_stream__DOT__complexslicelowerer_slice_proxy0) 
                                                               >> 0x0fU)))))) 
                                << 0x00000010U) | (QData)((IData)(
                                                                  (0x0000ffffU 
                                                                   & vlSelfRef.mlp_stream__DOT__resp_rdata))));
                    }
                }
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                         < (IData)(vlSelfRef.mlp_stream__DOT__storage_3))) {
                        if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                            vlSelfRef.mlp_stream__DOT__req_we = 0U;
                            vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                            vlSelfRef.mlp_stream__DOT__next_state = 0U;
                            vlSelfRef.mlp_stream__DOT__req_addr 
                                = vlSelfRef.mlp_stream__DOT__weight_addr;
                        }
                    } else {
                        vlSelfRef.mlp_stream__DOT__next_state = 8U;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__next_state = 7U;
                }
            } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                     < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) {
                    if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                        vlSelfRef.mlp_stream__DOT__req_we = 0U;
                        vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                        vlSelfRef.mlp_stream__DOT__next_state = 6U;
                        vlSelfRef.mlp_stream__DOT__req_addr 
                            = (vlSelfRef.mlp_stream__DOT__storage_8 
                               + ((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                                  << 2U));
                    }
                } else {
                    vlSelfRef.mlp_stream__DOT__next_state = 9U;
                }
            } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__next_state = 3U;
            }
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__hid_idx) 
                         >= (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) {
                        vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0 = 1U;
                        vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0 = 0U;
                    }
                }
            }
        } else {
            if ((1U & (~ ((IData)(vlSelfRef.mlp_stream__DOT__state) 
                          >> 1U)))) {
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                        vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 
                            = ((IData)(4U) + vlSelfRef.mlp_stream__DOT__weight_addr);
                        vlSelfRef.mlp_stream__DOT__acc_next_value2 
                            = (0x0000ffffffffffffULL 
                               & (vlSelfRef.mlp_stream__DOT__acc 
                                  + VL_MULS_QQQ(48, 
                                                (0x0000ffffffffffffULL 
                                                 & VL_EXTENDS_QI(48,32, vlSelfRef.mlp_stream__DOT__mac_w_ext)), 
                                                (0x0000ffffffffffffULL 
                                                 & VL_EXTENDS_QI(48,32, vlSelfRef.mlp_stream__DOT__mac_x_ext)))));
                    }
                } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__weight_addr_next_value4 
                        = ((IData)(4U) + vlSelfRef.mlp_stream__DOT__weight_addr);
                    vlSelfRef.mlp_stream__DOT__acc_next_value2 
                        = (0x0000ffffffffffffULL & 
                           (vlSelfRef.mlp_stream__DOT__acc 
                            + VL_MULS_QQQ(48, (0x0000ffffffffffffULL 
                                               & VL_EXTENDS_QI(48,32, vlSelfRef.mlp_stream__DOT__mac_w_ext)), 
                                          (0x0000ffffffffffffULL 
                                           & VL_EXTENDS_QI(48,32, vlSelfRef.mlp_stream__DOT__mac_x_ext)))));
                }
            }
            if ((2U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if ((1U & (~ (IData)(vlSelfRef.mlp_stream__DOT__state)))) {
                    if (vlSelfRef.mlp_stream__DOT__re) {
                        if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                       | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                      | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                            vlSelfRef.mlp_stream__DOT__out_idx_f_next_value_ce0 = 1U;
                            vlSelfRef.mlp_stream__DOT__out_idx_f_next_value0 = 0U;
                        }
                    }
                }
                if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                    if (((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                         < (IData)(vlSelfRef.mlp_stream__DOT__storage_3))) {
                        if (vlSelfRef.mlp_stream__DOT__bus_idle) {
                            vlSelfRef.mlp_stream__DOT__req_we = 0U;
                            vlSelfRef.mlp_stream__DOT__req_valid = 1U;
                            vlSelfRef.mlp_stream__DOT__next_state = 4U;
                            vlSelfRef.mlp_stream__DOT__req_addr 
                                = (vlSelfRef.mlp_stream__DOT__storage_6 
                                   + ((IData)(vlSelfRef.mlp_stream__DOT__in_idx) 
                                      << 2U));
                        }
                    } else {
                        vlSelfRef.mlp_stream__DOT__next_state = 5U;
                    }
                } else if (vlSelfRef.mlp_stream__DOT__re) {
                    if ((1U & (~ (((0x0310U < (IData)(vlSelfRef.mlp_stream__DOT__storage_3)) 
                                   | (0x0080U < (IData)(vlSelfRef.mlp_stream__DOT__storage_4))) 
                                  | (0x000aU < (IData)(vlSelfRef.mlp_stream__DOT__storage_5)))))) {
                        vlSelfRef.mlp_stream__DOT__next_state = 3U;
                    }
                }
            } else if ((1U & (IData)(vlSelfRef.mlp_stream__DOT__state))) {
                if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                    vlSelfRef.mlp_stream__DOT__next_state = 0x0bU;
                }
            } else if (vlSelfRef.mlp_stream__DOT__resp_valid) {
                vlSelfRef.mlp_stream__DOT__next_state = 7U;
            }
        }
    }
}

void Vtop___024root___eval_nba(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_nba\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1ULL & vlSelfRef.__VnbaTriggered[0U])) {
        Vtop___024root___nba_sequent__TOP__0(vlSelf);
    }
}

void Vtop___024root___trigger_orInto__act_vec_vec(VlUnpacked<QData/*63:0*/, 1> &out, const VlUnpacked<QData/*63:0*/, 1> &in) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___trigger_orInto__act_vec_vec\n"); );
    // Locals
    IData/*31:0*/ n;
    // Body
    n = 0U;
    do {
        out[n] = (out[n] | in[n]);
        n = ((IData)(1U) + n);
    } while ((0U >= n));
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtop___024root___dump_triggers__act(const VlUnpacked<QData/*63:0*/, 1> &triggers, const std::string &tag);
#endif  // VL_DEBUG

bool Vtop___024root___eval_phase__act(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_phase__act\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    Vtop___024root___eval_triggers_vec__act(vlSelf);
#ifdef VL_DEBUG
    if (VL_UNLIKELY(vlSymsp->_vm_contextp__->debug())) {
        Vtop___024root___dump_triggers__act(vlSelfRef.__VactTriggered, "act"s);
    }
#endif
    Vtop___024root___trigger_orInto__act_vec_vec(vlSelfRef.__VnbaTriggered, vlSelfRef.__VactTriggered);
    return (0U);
}

void Vtop___024root___trigger_clear__act(VlUnpacked<QData/*63:0*/, 1> &out) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___trigger_clear__act\n"); );
    // Locals
    IData/*31:0*/ n;
    // Body
    n = 0U;
    do {
        out[n] = 0ULL;
        n = ((IData)(1U) + n);
    } while ((1U > n));
}

bool Vtop___024root___eval_phase__nba(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_phase__nba\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Locals
    CData/*0:0*/ __VnbaExecute;
    // Body
    __VnbaExecute = Vtop___024root___trigger_anySet__act(vlSelfRef.__VnbaTriggered);
    if (__VnbaExecute) {
        Vtop___024root___eval_nba(vlSelf);
        Vtop___024root___trigger_clear__act(vlSelfRef.__VnbaTriggered);
    }
    return (__VnbaExecute);
}

void Vtop___024root___eval(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Locals
    IData/*31:0*/ __VicoIterCount;
    IData/*31:0*/ __VnbaIterCount;
    // Body
    __VicoIterCount = 0U;
    vlSelfRef.__VicoFirstIteration = 1U;
    do {
        if (VL_UNLIKELY(((0x00000064U < __VicoIterCount)))) {
#ifdef VL_DEBUG
            Vtop___024root___dump_triggers__ico(vlSelfRef.__VicoTriggered, "ico"s);
#endif
            VL_FATAL_MT("../../sim/build/mlp_stream.v", 2, "", "DIDNOTCONVERGE: Input combinational region did not converge after '--converge-limit' of 100 tries");
        }
        __VicoIterCount = ((IData)(1U) + __VicoIterCount);
        vlSelfRef.__VicoPhaseResult = Vtop___024root___eval_phase__ico(vlSelf);
        vlSelfRef.__VicoFirstIteration = 0U;
    } while (vlSelfRef.__VicoPhaseResult);
    __VnbaIterCount = 0U;
    do {
        if (VL_UNLIKELY(((0x00000064U < __VnbaIterCount)))) {
#ifdef VL_DEBUG
            Vtop___024root___dump_triggers__act(vlSelfRef.__VnbaTriggered, "nba"s);
#endif
            VL_FATAL_MT("../../sim/build/mlp_stream.v", 2, "", "DIDNOTCONVERGE: NBA region did not converge after '--converge-limit' of 100 tries");
        }
        __VnbaIterCount = ((IData)(1U) + __VnbaIterCount);
        vlSelfRef.__VactIterCount = 0U;
        do {
            if (VL_UNLIKELY(((0x00000064U < vlSelfRef.__VactIterCount)))) {
#ifdef VL_DEBUG
                Vtop___024root___dump_triggers__act(vlSelfRef.__VactTriggered, "act"s);
#endif
                VL_FATAL_MT("../../sim/build/mlp_stream.v", 2, "", "DIDNOTCONVERGE: Active region did not converge after '--converge-limit' of 100 tries");
            }
            vlSelfRef.__VactIterCount = ((IData)(1U) 
                                         + vlSelfRef.__VactIterCount);
            vlSelfRef.__VactPhaseResult = Vtop___024root___eval_phase__act(vlSelf);
        } while (vlSelfRef.__VactPhaseResult);
        vlSelfRef.__VnbaPhaseResult = Vtop___024root___eval_phase__nba(vlSelf);
    } while (vlSelfRef.__VnbaPhaseResult);
}

#ifdef VL_DEBUG
void Vtop___024root___eval_debug_assertions(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_debug_assertions\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if (VL_UNLIKELY(((vlSelfRef.sys_clk & 0xfeU)))) {
        Verilated::overWidthError("sys_clk");
    }
    if (VL_UNLIKELY(((vlSelfRef.sys_rst & 0xfeU)))) {
        Verilated::overWidthError("sys_rst");
    }
    if (VL_UNLIKELY(((vlSelfRef.bus_ack & 0xfeU)))) {
        Verilated::overWidthError("bus_ack");
    }
    if (VL_UNLIKELY(((vlSelfRef.storage & 0xfeU)))) {
        Verilated::overWidthError("storage");
    }
    if (VL_UNLIKELY(((vlSelfRef.re & 0xfeU)))) {
        Verilated::overWidthError("re");
    }
    if (VL_UNLIKELY(((vlSelfRef.storage_1 & 0xfeU)))) {
        Verilated::overWidthError("storage_1");
    }
    if (VL_UNLIKELY(((vlSelfRef.storage_2 & 0xe0U)))) {
        Verilated::overWidthError("storage_2");
    }
}
#endif  // VL_DEBUG
