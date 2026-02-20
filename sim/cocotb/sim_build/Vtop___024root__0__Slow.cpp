// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vtop.h for the primary calling header

#include "Vtop__pch.h"

VL_ATTR_COLD void Vtop___024root___eval_static(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_static\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.mlp_stream__DOT__wb_cyc = 0U;
    vlSelfRef.mlp_stream__DOT__wb_stb = 0U;
    vlSelfRef.mlp_stream__DOT__wb_we = 0U;
    vlSelfRef.mlp_stream__DOT__wb_adr = 0U;
    vlSelfRef.mlp_stream__DOT__wb_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__wb_sel = 0x0fU;
    vlSelfRef.mlp_stream__DOT__resp_valid = 0U;
    vlSelfRef.mlp_stream__DOT__resp_rdata = 0U;
    vlSelfRef.mlp_stream__DOT__in_idx = 0U;
    vlSelfRef.mlp_stream__DOT__hid_idx = 0U;
    vlSelfRef.mlp_stream__DOT__out_idx = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w1 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w2 = 0U;
    vlSelfRef.mlp_stream__DOT__weight_addr = 0U;
    vlSelfRef.mlp_stream__DOT__acc = 0ULL;
    vlSelfRef.mlp_stream__DOT__last_w = 0U;
    vlSelfRef.mlp_stream__DOT__last_x = 0U;
    vlSelfRef.mlp_stream__DOT__in_x_reg = 0U;
    vlSelfRef.mlp_stream__DOT__hid_x_reg = 0U;
    vlSelfRef.mlp_stream__DOT__out_val = 0U;
    vlSelfRef.mlp_stream__DOT__state = 2U;
    vlSelfRef.__Vtrigprevexpr___TOP__mlp_stream__DOT__sys_clk_1__0 
        = vlSelfRef.mlp_stream__DOT__sys_clk_1;
}

VL_ATTR_COLD void Vtop___024root___eval_static__TOP(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_static__TOP\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.mlp_stream__DOT__wb_cyc = 0U;
    vlSelfRef.mlp_stream__DOT__wb_stb = 0U;
    vlSelfRef.mlp_stream__DOT__wb_we = 0U;
    vlSelfRef.mlp_stream__DOT__wb_adr = 0U;
    vlSelfRef.mlp_stream__DOT__wb_dat_w = 0U;
    vlSelfRef.mlp_stream__DOT__wb_sel = 0x0fU;
    vlSelfRef.mlp_stream__DOT__resp_valid = 0U;
    vlSelfRef.mlp_stream__DOT__resp_rdata = 0U;
    vlSelfRef.mlp_stream__DOT__in_idx = 0U;
    vlSelfRef.mlp_stream__DOT__hid_idx = 0U;
    vlSelfRef.mlp_stream__DOT__out_idx = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w1 = 0U;
    vlSelfRef.mlp_stream__DOT__row_base_w2 = 0U;
    vlSelfRef.mlp_stream__DOT__weight_addr = 0U;
    vlSelfRef.mlp_stream__DOT__acc = 0ULL;
    vlSelfRef.mlp_stream__DOT__last_w = 0U;
    vlSelfRef.mlp_stream__DOT__last_x = 0U;
    vlSelfRef.mlp_stream__DOT__in_x_reg = 0U;
    vlSelfRef.mlp_stream__DOT__hid_x_reg = 0U;
    vlSelfRef.mlp_stream__DOT__out_val = 0U;
    vlSelfRef.mlp_stream__DOT__state = 2U;
}

VL_ATTR_COLD void Vtop___024root___eval_initial(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_initial\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.mlp_stream__DOT__dummy_s = 0U;
    vlSelfRef.mlp_stream__DOT__req_wsel = 0x0fU;
    vlSelfRef.mlp_stream__DOT__t_slice_proxy0 = 0x0000000000007fffULL;
    vlSelfRef.mlp_stream__DOT__f_slice_proxy = 0x0000ffffffff8000ULL;
    vlSelfRef.mlp_stream__DOT__t_slice_proxy1 = 0x0000000000007fffULL;
    vlSelfRef.mlp_stream__DOT__t_slice_proxy2 = 0x0000ffffffff8000ULL;
}

VL_ATTR_COLD void Vtop___024root___eval_initial__TOP(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_initial__TOP\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.mlp_stream__DOT__dummy_s = 0U;
    vlSelfRef.mlp_stream__DOT__req_wsel = 0x0fU;
    vlSelfRef.mlp_stream__DOT__t_slice_proxy0 = 0x0000000000007fffULL;
    vlSelfRef.mlp_stream__DOT__f_slice_proxy = 0x0000ffffffff8000ULL;
    vlSelfRef.mlp_stream__DOT__t_slice_proxy1 = 0x0000000000007fffULL;
    vlSelfRef.mlp_stream__DOT__t_slice_proxy2 = 0x0000ffffffff8000ULL;
}

VL_ATTR_COLD void Vtop___024root___eval_final(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_final\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtop___024root___dump_triggers__stl(const VlUnpacked<QData/*63:0*/, 1> &triggers, const std::string &tag);
#endif  // VL_DEBUG
VL_ATTR_COLD bool Vtop___024root___eval_phase__stl(Vtop___024root* vlSelf);

VL_ATTR_COLD void Vtop___024root___eval_settle(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_settle\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Locals
    IData/*31:0*/ __VstlIterCount;
    // Body
    __VstlIterCount = 0U;
    vlSelfRef.__VstlFirstIteration = 1U;
    do {
        if (VL_UNLIKELY(((0x00000064U < __VstlIterCount)))) {
#ifdef VL_DEBUG
            Vtop___024root___dump_triggers__stl(vlSelfRef.__VstlTriggered, "stl"s);
#endif
            VL_FATAL_MT("../../sim/build/mlp_stream.v", 2, "", "DIDNOTCONVERGE: Settle region did not converge after '--converge-limit' of 100 tries");
        }
        __VstlIterCount = ((IData)(1U) + __VstlIterCount);
        vlSelfRef.__VstlPhaseResult = Vtop___024root___eval_phase__stl(vlSelf);
        vlSelfRef.__VstlFirstIteration = 0U;
    } while (vlSelfRef.__VstlPhaseResult);
}

VL_ATTR_COLD void Vtop___024root___eval_triggers_vec__stl(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_triggers_vec__stl\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.__VstlTriggered[0U] = ((0xfffffffffffffffeULL 
                                      & vlSelfRef.__VstlTriggered[0U]) 
                                     | (IData)((IData)(vlSelfRef.__VstlFirstIteration)));
}

VL_ATTR_COLD bool Vtop___024root___trigger_anySet__stl(const VlUnpacked<QData/*63:0*/, 1> &in);

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtop___024root___dump_triggers__stl(const VlUnpacked<QData/*63:0*/, 1> &triggers, const std::string &tag) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___dump_triggers__stl\n"); );
    // Body
    if ((1U & (~ (IData)(Vtop___024root___trigger_anySet__stl(triggers))))) {
        VL_DBG_MSGS("         No '" + tag + "' region triggers active\n");
    }
    if ((1U & (IData)(triggers[0U]))) {
        VL_DBG_MSGS("         '" + tag + "' region trigger index 0 is active: Internal 'stl' trigger - first iteration\n");
    }
}
#endif  // VL_DEBUG

VL_ATTR_COLD bool Vtop___024root___trigger_anySet__stl(const VlUnpacked<QData/*63:0*/, 1> &in) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___trigger_anySet__stl\n"); );
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

VL_ATTR_COLD void Vtop___024root___stl_sequent__TOP__0(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___stl_sequent__TOP__0\n"); );
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
    vlSelfRef.mlp_stream__DOT__status = 0U;
    vlSelfRef.mlp_stream__DOT__status_3 = (IData)(vlSelfRef.mlp_stream__DOT__acc);
    vlSelfRef.mlp_stream__DOT__status_4 = (0x0000ffffU 
                                           & (IData)(
                                                     (vlSelfRef.mlp_stream__DOT__acc 
                                                      >> 0x00000020U)));
    vlSelfRef.mlp_stream__DOT__bus_cyc = vlSelfRef.mlp_stream__DOT__wb_cyc;
    vlSelfRef.mlp_stream__DOT__sys_clk = vlSelfRef.sys_clk;
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
    vlSelfRef.mlp_stream__DOT__sys_clk_1 = vlSelfRef.mlp_stream__DOT__sys_clk;
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

VL_ATTR_COLD void Vtop___024root___eval_stl(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_stl\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1ULL & vlSelfRef.__VstlTriggered[0U])) {
        Vtop___024root___stl_sequent__TOP__0(vlSelf);
    }
}

VL_ATTR_COLD bool Vtop___024root___eval_phase__stl(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___eval_phase__stl\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Locals
    CData/*0:0*/ __VstlExecute;
    // Body
    Vtop___024root___eval_triggers_vec__stl(vlSelf);
#ifdef VL_DEBUG
    if (VL_UNLIKELY(vlSymsp->_vm_contextp__->debug())) {
        Vtop___024root___dump_triggers__stl(vlSelfRef.__VstlTriggered, "stl"s);
    }
#endif
    __VstlExecute = Vtop___024root___trigger_anySet__stl(vlSelfRef.__VstlTriggered);
    if (__VstlExecute) {
        Vtop___024root___eval_stl(vlSelf);
    }
    return (__VstlExecute);
}

bool Vtop___024root___trigger_anySet__ico(const VlUnpacked<QData/*63:0*/, 1> &in);

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtop___024root___dump_triggers__ico(const VlUnpacked<QData/*63:0*/, 1> &triggers, const std::string &tag) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___dump_triggers__ico\n"); );
    // Body
    if ((1U & (~ (IData)(Vtop___024root___trigger_anySet__ico(triggers))))) {
        VL_DBG_MSGS("         No '" + tag + "' region triggers active\n");
    }
    if ((1U & (IData)(triggers[0U]))) {
        VL_DBG_MSGS("         '" + tag + "' region trigger index 0 is active: Internal 'ico' trigger - first iteration\n");
    }
}
#endif  // VL_DEBUG

bool Vtop___024root___trigger_anySet__act(const VlUnpacked<QData/*63:0*/, 1> &in);

#ifdef VL_DEBUG
VL_ATTR_COLD void Vtop___024root___dump_triggers__act(const VlUnpacked<QData/*63:0*/, 1> &triggers, const std::string &tag) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___dump_triggers__act\n"); );
    // Body
    if ((1U & (~ (IData)(Vtop___024root___trigger_anySet__act(triggers))))) {
        VL_DBG_MSGS("         No '" + tag + "' region triggers active\n");
    }
    if ((1U & (IData)(triggers[0U]))) {
        VL_DBG_MSGS("         '" + tag + "' region trigger index 0 is active: @(posedge mlp_stream.sys_clk_1)\n");
    }
}
#endif  // VL_DEBUG

VL_ATTR_COLD void Vtop___024root___ctor_var_reset(Vtop___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vtop___024root___ctor_var_reset\n"); );
    Vtop__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    const uint64_t __VscopeHash = VL_MURMUR64_HASH(vlSelf->vlNamep);
    vlSelf->sys_clk = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 7302336598091101382ull);
    vlSelf->sys_rst = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 10947334754268556250ull);
    vlSelf->bus_adr = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 17858839340929839705ull);
    vlSelf->bus_dat_w = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 6924757476449402754ull);
    vlSelf->bus_dat_r = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 214280655956423450ull);
    vlSelf->bus_sel = VL_SCOPED_RAND_RESET_I(4, __VscopeHash, 15733095433185429307ull);
    vlSelf->bus_cyc = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 7720519163851615648ull);
    vlSelf->bus_stb = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 15499354301167312229ull);
    vlSelf->bus_ack = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 3975498337686664941ull);
    vlSelf->bus_we = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 11669214654233166897ull);
    vlSelf->storage = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 6967744052328620840ull);
    vlSelf->re = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9839690450034771972ull);
    vlSelf->storage_1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 15654753350756652249ull);
    vlSelf->storage_2 = VL_SCOPED_RAND_RESET_I(5, __VscopeHash, 8314512570075127425ull);
    vlSelf->storage_3 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 13609614500664523707ull);
    vlSelf->storage_4 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 6996378570072859286ull);
    vlSelf->storage_5 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 4841093040490281283ull);
    vlSelf->storage_6 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 6355668315766888373ull);
    vlSelf->storage_7 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 17128327992460221326ull);
    vlSelf->storage_8 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 14041483881935890584ull);
    vlSelf->storage_9 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 14390252004447234486ull);
    vlSelf->storage_10 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 936887366199026829ull);
    vlSelf->storage_11 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 15150156055057353597ull);
    vlSelf->status = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 14822974759303984767ull);
    vlSelf->status_1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 10856283412377070542ull);
    vlSelf->status_2 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 1929888068099819058ull);
    vlSelf->status_3 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 8731597869305588876ull);
    vlSelf->status_4 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 12219572083130337390ull);
    vlSelf->status_5 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 11594572196059826743ull);
    vlSelf->status_6 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 8271444475255561279ull);
    vlSelf->mlp_stream__DOT____Vlvbound_hc2aa3eeb__0 = 0;
    vlSelf->mlp_stream__DOT__sys_clk = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 743889660879309598ull);
    vlSelf->mlp_stream__DOT__sys_rst = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 11494569579262358306ull);
    vlSelf->mlp_stream__DOT__bus_adr = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 10967564531116896528ull);
    vlSelf->mlp_stream__DOT__bus_dat_w = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 12836736911876662290ull);
    vlSelf->mlp_stream__DOT__bus_dat_r = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 16220124419579469223ull);
    vlSelf->mlp_stream__DOT__bus_sel = VL_SCOPED_RAND_RESET_I(4, __VscopeHash, 13426006356904580013ull);
    vlSelf->mlp_stream__DOT__bus_cyc = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 7059453147262138ull);
    vlSelf->mlp_stream__DOT__bus_stb = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 13504345932731907309ull);
    vlSelf->mlp_stream__DOT__bus_ack = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 11857904994648574532ull);
    vlSelf->mlp_stream__DOT__bus_we = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 14017253580095280808ull);
    vlSelf->mlp_stream__DOT__storage = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 14885340479931371154ull);
    vlSelf->mlp_stream__DOT__re = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 17467930453616415163ull);
    vlSelf->mlp_stream__DOT__storage_1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 17127962426492354085ull);
    vlSelf->mlp_stream__DOT__storage_2 = VL_SCOPED_RAND_RESET_I(5, __VscopeHash, 8730103815899738807ull);
    vlSelf->mlp_stream__DOT__storage_3 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 11634041661071586808ull);
    vlSelf->mlp_stream__DOT__storage_4 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 17095183680663903055ull);
    vlSelf->mlp_stream__DOT__storage_5 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 9913075860509811919ull);
    vlSelf->mlp_stream__DOT__storage_6 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 16380370017728405414ull);
    vlSelf->mlp_stream__DOT__storage_7 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 15364622838080751570ull);
    vlSelf->mlp_stream__DOT__storage_8 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 16627611371240731454ull);
    vlSelf->mlp_stream__DOT__storage_9 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 4087428856129331021ull);
    vlSelf->mlp_stream__DOT__storage_10 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 6748256536505188086ull);
    vlSelf->mlp_stream__DOT__storage_11 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 16251450262499608470ull);
    vlSelf->mlp_stream__DOT__status = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8101946289187807134ull);
    vlSelf->mlp_stream__DOT__status_1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9712123833582755161ull);
    vlSelf->mlp_stream__DOT__status_2 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 3312639854953730519ull);
    vlSelf->mlp_stream__DOT__status_3 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 13195353264976900381ull);
    vlSelf->mlp_stream__DOT__status_4 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 8964846287795120589ull);
    vlSelf->mlp_stream__DOT__status_5 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 3448674078629225424ull);
    vlSelf->mlp_stream__DOT__status_6 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 17228843899855250007ull);
    vlSelf->mlp_stream__DOT__sys_clk_1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 818759350302938114ull);
    vlSelf->mlp_stream__DOT__sys_rst_1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 2520984333979319948ull);
    vlSelf->mlp_stream__DOT__in_wr_adr = VL_SCOPED_RAND_RESET_I(10, __VscopeHash, 14468206613096187044ull);
    vlSelf->mlp_stream__DOT__in_wr_dat_r = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 16550296599132696808ull);
    vlSelf->mlp_stream__DOT__in_wr_we = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8977447378913715805ull);
    vlSelf->mlp_stream__DOT__in_wr_dat_w = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 9136543211902115267ull);
    vlSelf->mlp_stream__DOT__in_rd_dat_r = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 647224362690336216ull);
    vlSelf->mlp_stream__DOT__hid_wr_adr = VL_SCOPED_RAND_RESET_I(7, __VscopeHash, 2505130830241777725ull);
    vlSelf->mlp_stream__DOT__hid_wr_dat_r = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 15305281935233564356ull);
    vlSelf->mlp_stream__DOT__hid_wr_we = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9889021959956165835ull);
    vlSelf->mlp_stream__DOT__hid_wr_dat_w = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 16000052984804739491ull);
    vlSelf->mlp_stream__DOT__hid_rd_dat_r = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 10684710225134335832ull);
    vlSelf->mlp_stream__DOT__wb_cyc = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8489117394569909602ull);
    vlSelf->mlp_stream__DOT__wb_stb = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 7234087959716525294ull);
    vlSelf->mlp_stream__DOT__wb_we = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 5294905682114507193ull);
    vlSelf->mlp_stream__DOT__wb_adr = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 16334344881620626846ull);
    vlSelf->mlp_stream__DOT__wb_dat_w = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 11390859308211886859ull);
    vlSelf->mlp_stream__DOT__wb_sel = VL_SCOPED_RAND_RESET_I(4, __VscopeHash, 18078955643057264105ull);
    vlSelf->mlp_stream__DOT__req_valid = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 3632361248950600580ull);
    vlSelf->mlp_stream__DOT__req_we = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 6361398860881878372ull);
    vlSelf->mlp_stream__DOT__req_addr = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 15975928424593571543ull);
    vlSelf->mlp_stream__DOT__req_wdata = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 11066132297808072770ull);
    vlSelf->mlp_stream__DOT__req_wsel = 15U;
    ;
    vlSelf->mlp_stream__DOT__resp_valid = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 18400358715658732643ull);
    vlSelf->mlp_stream__DOT__resp_rdata = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 3174296870544108172ull);
    vlSelf->mlp_stream__DOT__bus_idle = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 1046376125931004320ull);
    vlSelf->mlp_stream__DOT__in_idx = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 8538686124113526491ull);
    vlSelf->mlp_stream__DOT__hid_idx = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 1678641366346851677ull);
    vlSelf->mlp_stream__DOT__out_idx = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 12402866197402446507ull);
    vlSelf->mlp_stream__DOT__row_base_w1 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 1530554012355256947ull);
    vlSelf->mlp_stream__DOT__row_base_w2 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 10926154230485903185ull);
    vlSelf->mlp_stream__DOT__weight_addr = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 16934390908418118427ull);
    vlSelf->mlp_stream__DOT__acc = VL_SCOPED_RAND_RESET_Q(48, __VscopeHash, 7532667846067918527ull);
    vlSelf->mlp_stream__DOT__last_w = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 5335257884531601184ull);
    vlSelf->mlp_stream__DOT__last_x = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 5008868808000045840ull);
    vlSelf->mlp_stream__DOT__mac_w_ext = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 7515888209855577829ull);
    vlSelf->mlp_stream__DOT__mac_x_ext = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 14880691753651924559ull);
    vlSelf->mlp_stream__DOT__mac_x_raw = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 7455409629512823754ull);
    vlSelf->mlp_stream__DOT__in_x_reg = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 11885450473298976423ull);
    vlSelf->mlp_stream__DOT__hid_x_reg = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 1296099024677407605ull);
    vlSelf->mlp_stream__DOT__out_val = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 3417612890174777463ull);
    vlSelf->mlp_stream__DOT__hid_write_val = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 2027115183225457184ull);
    vlSelf->mlp_stream__DOT__out_write_val = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 12001093630402549195ull);
    vlSelf->mlp_stream__DOT__shifted = VL_SCOPED_RAND_RESET_Q(48, __VscopeHash, 4258621097629238995ull);
    vlSelf->mlp_stream__DOT__is_ongoing0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 7221433326492749181ull);
    vlSelf->mlp_stream__DOT__is_ongoing1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 1660526408676853147ull);
    vlSelf->mlp_stream__DOT__state = VL_SCOPED_RAND_RESET_I(4, __VscopeHash, 4595124176527337692ull);
    vlSelf->mlp_stream__DOT__next_state = VL_SCOPED_RAND_RESET_I(4, __VscopeHash, 5277811129207407637ull);
    vlSelf->mlp_stream__DOT__last_w_next_value0 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 2125170414643840889ull);
    vlSelf->mlp_stream__DOT__last_w_next_value_ce0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8638357673924544869ull);
    vlSelf->mlp_stream__DOT__last_x_next_value1 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 3051421890955763231ull);
    vlSelf->mlp_stream__DOT__last_x_next_value_ce1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 4802183520340146660ull);
    vlSelf->mlp_stream__DOT__acc_next_value2 = VL_SCOPED_RAND_RESET_Q(48, __VscopeHash, 12470569883306993093ull);
    vlSelf->mlp_stream__DOT__acc_next_value_ce2 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 13867184836261087973ull);
    vlSelf->mlp_stream__DOT__in_idx_next_value3 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 6315430067245772872ull);
    vlSelf->mlp_stream__DOT__in_idx_next_value_ce3 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 16277459850712367117ull);
    vlSelf->mlp_stream__DOT__weight_addr_next_value4 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 16234955031457530550ull);
    vlSelf->mlp_stream__DOT__weight_addr_next_value_ce4 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9291487171912499346ull);
    vlSelf->mlp_stream__DOT__hid_idx_next_value5 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 12521005309942023621ull);
    vlSelf->mlp_stream__DOT__hid_idx_next_value_ce5 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8952877601569399354ull);
    vlSelf->mlp_stream__DOT__done_status_next_value6 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 13407385388028867788ull);
    vlSelf->mlp_stream__DOT__done_status_next_value_ce6 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 5084521234528535728ull);
    vlSelf->mlp_stream__DOT__error_status_next_value7 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8953588525206385834ull);
    vlSelf->mlp_stream__DOT__error_status_next_value_ce7 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 16163202994551471731ull);
    vlSelf->mlp_stream__DOT__out_idx_f_next_value0 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 12492545329811891107ull);
    vlSelf->mlp_stream__DOT__out_idx_f_next_value_ce0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 14160523355616408796ull);
    vlSelf->mlp_stream__DOT__row_base_w1_f_next_value1 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 17625656953808690114ull);
    vlSelf->mlp_stream__DOT__row_base_w1_f_next_value_ce1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 11176333943594296730ull);
    vlSelf->mlp_stream__DOT__row_base_w2_f_next_value2 = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 7956054264220711303ull);
    vlSelf->mlp_stream__DOT__row_base_w2_f_next_value_ce2 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 607855693703635737ull);
    vlSelf->mlp_stream__DOT__in_x_reg_t_next_value0 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 878676038931460134ull);
    vlSelf->mlp_stream__DOT__in_x_reg_t_next_value_ce0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 923304208706667043ull);
    vlSelf->mlp_stream__DOT__hid_x_reg_t_next_value1 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 4467355643271133428ull);
    vlSelf->mlp_stream__DOT__hid_x_reg_t_next_value_ce1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 17879703651946937522ull);
    vlSelf->mlp_stream__DOT__out_val_next_value8 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 7827150654310666878ull);
    vlSelf->mlp_stream__DOT__out_val_next_value_ce8 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 5642090303362678429ull);
    vlSelf->mlp_stream__DOT__t_slice_proxy0 = 32767U;
    ;
    vlSelf->mlp_stream__DOT__f_slice_proxy = 4294934528U;
    ;
    vlSelf->mlp_stream__DOT__t_slice_proxy1 = 32767U;
    ;
    vlSelf->mlp_stream__DOT__t_slice_proxy2 = 4294934528U;
    ;
    vlSelf->mlp_stream__DOT__rhs_slice_proxy = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 13542134964633548280ull);
    vlSelf->mlp_stream__DOT__complexslicelowerer_slice_proxy0 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 16639789303582660102ull);
    vlSelf->mlp_stream__DOT__complexslicelowerer_slice_proxy1 = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 12703824479776129761ull);
    vlSelf->mlp_stream__DOT__dummy_s = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 15484504354254085575ull);
    vlSelf->mlp_stream__DOT__dummy_d = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 7858456126074178796ull);
    vlSelf->mlp_stream__DOT__dummy_d_1 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 14272535224153797184ull);
    vlSelf->mlp_stream__DOT__dummy_d_2 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 5827959575177560676ull);
    vlSelf->mlp_stream__DOT__dummy_d_3 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 8542017899307880517ull);
    for (int __Vi0 = 0; __Vi0 < 784; ++__Vi0) {
        vlSelf->mlp_stream__DOT__in_mem[__Vi0] = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 5187021059870691192ull);
    }
    vlSelf->mlp_stream__DOT__memadr = VL_SCOPED_RAND_RESET_I(10, __VscopeHash, 2888414645670699470ull);
    for (int __Vi0 = 0; __Vi0 < 128; ++__Vi0) {
        vlSelf->mlp_stream__DOT__hid_mem[__Vi0] = VL_SCOPED_RAND_RESET_I(16, __VscopeHash, 10381074284867096498ull);
    }
    vlSelf->mlp_stream__DOT__memadr_1 = VL_SCOPED_RAND_RESET_I(7, __VscopeHash, 8336513501612378517ull);
    for (int __Vi0 = 0; __Vi0 < 1; ++__Vi0) {
        vlSelf->__VstlTriggered[__Vi0] = 0;
    }
    for (int __Vi0 = 0; __Vi0 < 1; ++__Vi0) {
        vlSelf->__VicoTriggered[__Vi0] = 0;
    }
    for (int __Vi0 = 0; __Vi0 < 1; ++__Vi0) {
        vlSelf->__VactTriggered[__Vi0] = 0;
    }
    vlSelf->__Vtrigprevexpr___TOP__mlp_stream__DOT__sys_clk_1__0 = 0;
    for (int __Vi0 = 0; __Vi0 < 1; ++__Vi0) {
        vlSelf->__VnbaTriggered[__Vi0] = 0;
    }
}
