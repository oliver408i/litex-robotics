VERILOG ?= verilator

.PHONY: verilog sim clean

verilog:
	python3 $(CURDIR)/gen_mlp_stream_verilog.py

sim: verilog
	$(MAKE) -C cocotb SIM=$(VERILOG)

clean:
	$(RM) -r sim/build sim/cocotb/sim_build sim/cocotb/__pycache__
