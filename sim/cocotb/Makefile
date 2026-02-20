TOPLEVEL_LANG = verilog
SIM ?= verilator
TOPLEVEL = mlp_stream
MODULE = test_mlp_stream
VERILOG_SOURCES = ../../sim/build/mlp_stream.v
EXTRA_ARGS += -Wno-COMBDLY -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-INITIALDLY

include $(shell cocotb-config --makefiles)/Makefile.sim
