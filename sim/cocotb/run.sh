#!/usr/bin/env bash
# Run the cocotb test for verilog/snn_mlp_core.v.
#
# Sources oss-cad-suite for iverilog, then prepends .venv/bin to PATH so the
# venv's python (3.12) and cocotb-config take priority over oss-cad-suite's
# bundled Python.
set -euo pipefail

cd "$(dirname "$0")"
REPO_ROOT="$(cd ../.. && pwd)"

# We do NOT source oss-cad-suite/environment because it forces LD_LIBRARY_PATH
# and PYTHONHOME to the bundled (old-glibc, py3.11) values, which breaks
# cocotb's icarus VPI when it dlopens system libpython3.12.
# Instead: put .venv/bin and the local vvp shim first, then oss-cad-suite's
# bin for the rest of the tools.
export PATH="$REPO_ROOT/sim/cocotb/vvp-shim:$REPO_ROOT/.venv/bin:$REPO_ROOT/oss-cad-suite/oss-cad-suite/bin:$PATH"
# Pin cocotb to our shim dir so it picks up the patched vvp alongside iverilog.
export ICARUS_BIN_DIR="$REPO_ROOT/sim/cocotb/vvp-shim"

exec make "$@"
