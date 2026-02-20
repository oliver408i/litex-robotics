// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Model implementation (design independent parts)

#include "Vtop__pch.h"

//============================================================
// Constructors

Vtop::Vtop(VerilatedContext* _vcontextp__, const char* _vcname__)
    : VerilatedModel{*_vcontextp__}
    , vlSymsp{new Vtop__Syms(contextp(), _vcname__, this)}
    , sys_clk{vlSymsp->TOP.sys_clk}
    , sys_rst{vlSymsp->TOP.sys_rst}
    , bus_sel{vlSymsp->TOP.bus_sel}
    , bus_cyc{vlSymsp->TOP.bus_cyc}
    , bus_stb{vlSymsp->TOP.bus_stb}
    , bus_ack{vlSymsp->TOP.bus_ack}
    , bus_we{vlSymsp->TOP.bus_we}
    , storage{vlSymsp->TOP.storage}
    , re{vlSymsp->TOP.re}
    , storage_1{vlSymsp->TOP.storage_1}
    , storage_2{vlSymsp->TOP.storage_2}
    , status{vlSymsp->TOP.status}
    , status_1{vlSymsp->TOP.status_1}
    , status_2{vlSymsp->TOP.status_2}
    , storage_3{vlSymsp->TOP.storage_3}
    , storage_4{vlSymsp->TOP.storage_4}
    , storage_5{vlSymsp->TOP.storage_5}
    , status_5{vlSymsp->TOP.status_5}
    , status_6{vlSymsp->TOP.status_6}
    , bus_adr{vlSymsp->TOP.bus_adr}
    , bus_dat_w{vlSymsp->TOP.bus_dat_w}
    , bus_dat_r{vlSymsp->TOP.bus_dat_r}
    , storage_6{vlSymsp->TOP.storage_6}
    , storage_7{vlSymsp->TOP.storage_7}
    , storage_8{vlSymsp->TOP.storage_8}
    , storage_9{vlSymsp->TOP.storage_9}
    , storage_10{vlSymsp->TOP.storage_10}
    , storage_11{vlSymsp->TOP.storage_11}
    , status_3{vlSymsp->TOP.status_3}
    , status_4{vlSymsp->TOP.status_4}
    , rootp{&(vlSymsp->TOP)}
{
    // Register model with the context
    contextp()->addModel(this);
}

Vtop::Vtop(const char* _vcname__)
    : Vtop(Verilated::threadContextp(), _vcname__)
{
}

//============================================================
// Destructor

Vtop::~Vtop() {
    delete vlSymsp;
}

//============================================================
// Evaluation function

#ifdef VL_DEBUG
void Vtop___024root___eval_debug_assertions(Vtop___024root* vlSelf);
#endif  // VL_DEBUG
void Vtop___024root___eval_static(Vtop___024root* vlSelf);
void Vtop___024root___eval_initial(Vtop___024root* vlSelf);
void Vtop___024root___eval_settle(Vtop___024root* vlSelf);
void Vtop___024root___eval(Vtop___024root* vlSelf);

void Vtop::eval_step() {
    VL_DEBUG_IF(VL_DBG_MSGF("+++++TOP Evaluate Vtop::eval_step\n"); );
#ifdef VL_DEBUG
    // Debug assertions
    Vtop___024root___eval_debug_assertions(&(vlSymsp->TOP));
#endif  // VL_DEBUG
    vlSymsp->__Vm_deleter.deleteAll();
    if (VL_UNLIKELY(!vlSymsp->__Vm_didInit)) {
        VL_DEBUG_IF(VL_DBG_MSGF("+ Initial\n"););
        Vtop___024root___eval_static(&(vlSymsp->TOP));
        Vtop___024root___eval_initial(&(vlSymsp->TOP));
        Vtop___024root___eval_settle(&(vlSymsp->TOP));
        vlSymsp->__Vm_didInit = true;
    }
    VL_DEBUG_IF(VL_DBG_MSGF("+ Eval\n"););
    Vtop___024root___eval(&(vlSymsp->TOP));
    // Evaluate cleanup
    Verilated::endOfEval(vlSymsp->__Vm_evalMsgQp);
}

//============================================================
// Events and timing
bool Vtop::eventsPending() { return false; }

uint64_t Vtop::nextTimeSlot() {
    VL_FATAL_MT(__FILE__, __LINE__, "", "No delays in the design");
    return 0;
}

//============================================================
// Utilities

const char* Vtop::name() const {
    return vlSymsp->name();
}

//============================================================
// Invoke final blocks

void Vtop___024root___eval_final(Vtop___024root* vlSelf);

VL_ATTR_COLD void Vtop::final() {
    Vtop___024root___eval_final(&(vlSymsp->TOP));
}

//============================================================
// Implementations of abstract methods from VerilatedModel

const char* Vtop::hierName() const { return vlSymsp->name(); }
const char* Vtop::modelName() const { return "Vtop"; }
unsigned Vtop::threads() const { return 1; }
void Vtop::prepareClone() const { contextp()->prepareClone(); }
void Vtop::atClone() const {
    contextp()->threadPoolpOnClone();
}
