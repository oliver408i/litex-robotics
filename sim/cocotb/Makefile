# Cocotb test driver.
#
# Default TARGET=core builds verilog/snn_mlp_core.v as the DUT and runs the
# tiny + MNIST tests from test_snn_mlp_core.py. TARGET=loader builds
# verilog/snn_weight_loader.v and runs test_snn_weight_loader.py.
#
# Default core parameters target the tiny 4->2->2 smoke test. Override on
# the command line, e.g.:
#   ./run.sh IN_SIZE=784 HIDDEN=64 OUT_SIZE=10 TIMESTEPS=25
#   ./run.sh TARGET=loader
#
# Invoke through ./run.sh so iverilog (oss-cad-suite) and cocotb (.venv) are
# both on PATH with the right priority.

TOPLEVEL_LANG ?= verilog
SIM           ?= icarus

TARGET ?= core

ifeq ($(TARGET),core)
    TOPLEVEL = snn_mlp_core
    MODULE   = test_snn_mlp_core
    VERILOG_SOURCES = $(abspath $(CURDIR)/../../verilog/snn_mlp_core.v)

    IN_SIZE   ?= 4
    HIDDEN    ?= 2
    OUT_SIZE  ?= 2
    TIMESTEPS ?= 3
    N_MAC     ?= 1

    SIM_BUILD = sim_build_core_$(IN_SIZE)_$(HIDDEN)_$(OUT_SIZE)_$(TIMESTEPS)_$(N_MAC)

    COMPILE_ARGS += -P$(TOPLEVEL).IN_SIZE=$(IN_SIZE)
    COMPILE_ARGS += -P$(TOPLEVEL).HIDDEN=$(HIDDEN)
    COMPILE_ARGS += -P$(TOPLEVEL).OUT_SIZE=$(OUT_SIZE)
    COMPILE_ARGS += -P$(TOPLEVEL).TIMESTEPS=$(TIMESTEPS)
    COMPILE_ARGS += -P$(TOPLEVEL).N_MAC=$(N_MAC)
else ifeq ($(TARGET),loader)
    TOPLEVEL = snn_weight_loader
    MODULE   = test_snn_weight_loader
    VERILOG_SOURCES = $(abspath $(CURDIR)/../../verilog/snn_weight_loader.v)

    WB_DATA_WIDTH ?= 32
    WB_ADDR_WIDTH ?= 32
    N_MAC         ?= 1
    DATA_WIDTH    ?= 16

    SIM_BUILD = sim_build_loader_$(WB_DATA_WIDTH)_$(N_MAC)_$(DATA_WIDTH)

    COMPILE_ARGS += -P$(TOPLEVEL).WB_DATA_WIDTH=$(WB_DATA_WIDTH)
    COMPILE_ARGS += -P$(TOPLEVEL).WB_ADDR_WIDTH=$(WB_ADDR_WIDTH)
    COMPILE_ARGS += -P$(TOPLEVEL).N_MAC=$(N_MAC)
    COMPILE_ARGS += -P$(TOPLEVEL).DATA_WIDTH=$(DATA_WIDTH)
else
    $(error unknown TARGET '$(TARGET)'; use TARGET=core or TARGET=loader)
endif

include $(shell cocotb-config --makefiles)/Makefile.sim
