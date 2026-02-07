# FPGA Soft-Core Robotics Controller (Personal Project)

This is a personal FPGA soft-core robotics controller project. It is not a ready-deploy application; it is published for reference and learning.

## What This Is
- A Litex-based soft-core build targeting the Sipeed Tang Nano 20K.
- Gateware + firmware + small host utilities for experimenting with motor control, sensors, and debug workflows.

## Status
- Experimental and evolving.
- Interfaces and pin usage change as hardware evolves.

## Repo Layout (high level)
- `gateware/`: FPGA build and platform integration.
- `software/`: Firmware and supporting code.
- `gui-debugger/` + `scripts/`: Host-side tooling and utilities.
- `verilog/`: Standalone HDL components.
- `pins_used.txt`: Current pin assignments and wiring notes.

## Notes
- This is reference material, not production-ready.
- If you reuse anything, validate pin mappings, electrical constraints, and safety requirements for your own hardware.

