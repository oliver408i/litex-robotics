MLP Stream Simulation (Verilator + cocotb)

Prereqs:
- OSS CAD Suite in PATH (verilator, iverilog)
- Python with cocotb installed

Install cocotb (one-time):
  python3 -m pip install --user cocotb

Generate Verilog:
  make -C sim verilog

Run the simulation (Verilator):
  make -C sim sim VERILOG=verilator

Optional: run with Icarus:
  make -C sim sim VERILOG=icarus

Notes:
- The testbench emulates a Wishbone SDRAM and compares HW vs SW MLP output.
- The default test uses the same toy model as software/mlp_demo.
