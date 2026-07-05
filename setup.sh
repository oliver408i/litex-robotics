#!/usr/bin/env bash
#
# setup.sh — bootstrap the fpga-mcu dev environment on a fresh machine.
#
# Reconstructs everything that is gitignored and therefore does NOT come with a
# `git clone`: the OSS CAD Suite FPGA toolchain, the pinned LiteX/Migen source
# stack under litex-setup/, the Python 3.12 venv, the liblitespi XIP patch, and
# the LVGL submodule. After this runs, `cd sim/cocotb && ./run.sh` and the
# `icepi_zero_*.py` builds work offline.
#
# Usage:
#   ./setup.sh                 # full core setup (toolchain + litex + venv + patch)
#   ./setup.sh --with-ml       # also install torch/torchvision/snntorch (CPU wheels)
#   ./setup.sh --skip-toolchain  # skip the 2.4 GB oss-cad-suite download
#   ./setup.sh --help
#
# Idempotent: re-running skips work that is already done.

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration (pinned to match the reference machine — change deliberately)
# ---------------------------------------------------------------------------
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3.12}"          # cocotb 2.0.1 + litex want 3.12
OSS_CAD_DATE="${OSS_CAD_DATE:-2026-06-02}"      # release tag
OSS_CAD_FILE="oss-cad-suite-linux-x64-$(echo "$OSS_CAD_DATE" | tr -d '-').tgz"
OSS_CAD_URL="https://github.com/YosysHQ/oss-cad-suite-build/releases/download/${OSS_CAD_DATE}/${OSS_CAD_FILE}"

WITH_ML=0
SKIP_TOOLCHAIN=0
for arg in "$@"; do
  case "$arg" in
    --with-ml)        WITH_ML=1 ;;
    --skip-toolchain) SKIP_TOOLCHAIN=1 ;;
    --help|-h)        sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "Unknown flag: $arg (try --help)"; exit 2 ;;
  esac
done

log()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31mxx\033[0m %s\n' "$*" >&2; exit 1; }

cd "$REPO_ROOT"

# Pinned LiteX/Migen sources: "dir|git-url|commit".  Commits captured from the
# reference machine's `pip freeze`; the top-level build scripts add these dirs
# to sys.path directly, and the venv installs them editable, so the directory
# layout under litex-setup/ is load-bearing.
read -r -d '' LITEX_REPOS <<'EOF' || true
migen|https://github.com/m-labs/migen.git|4c2ae8d
litex|https://github.com/enjoy-digital/litex.git|9748a9b975ceb9d20a5dcc0a73a3aa277ee660bf
litex-boards|https://github.com/litex-hub/litex-boards.git|d5e84d9a4de7186e51e704ea97b9a13e923a9525
litedram|https://github.com/enjoy-digital/litedram.git|1e2af1436309203c2fae61f492e1a589c8bd4649
liteeth|https://github.com/enjoy-digital/liteeth.git|6067ae4fa3a21ea018d6a009df49bd09d7494f03
litei2c|https://github.com/litex-hub/litei2c.git|23764b327ff2ab574121496260cf7d1f3a368487
liteiclink|https://github.com/enjoy-digital/liteiclink.git|679befc2271e64297345b15e974b2d2fdcd8fad5
litejesd204b|https://github.com/enjoy-digital/litejesd204b.git|786ceff2e0b992c5d9205116c7720f8755312e08
litepcie|https://github.com/enjoy-digital/litepcie.git|ef71abce002f88843fa4bb238aab5ee247bb8d64
litesata|https://github.com/enjoy-digital/litesata.git|82b4860ed32155e125dd867fdbba1f6c7b4a282a
litescope|https://github.com/enjoy-digital/litescope.git|8b49f61d2dd4ee70ceb2578f95aa763f1756d92e
litesdcard|https://github.com/enjoy-digital/litesdcard.git|5429f5d7ef57f7e0a4802087033dbde8f1b7a4d5
litespi|https://github.com/litex-hub/litespi.git|54259cad462e901f6e3e4aa486874351822ba4bd
pythondata-cpu-lm32|https://github.com/litex-hub/pythondata-cpu-lm32.git|594f2068e32a6faa84f05f9b443b58c3c4658113
pythondata-cpu-minerva|https://github.com/litex-hub/pythondata-cpu-minerva.git|8db577a903f525758443909c863425549bada9bc
pythondata-cpu-mor1kx|https://github.com/litex-hub/pythondata-cpu-mor1kx.git|4b9d59badcc535fbb96fd5208411a3377a53a45b
pythondata-cpu-naxriscv|https://github.com/litex-hub/pythondata-cpu-naxriscv.git|dfb4bfdf19e864ec773f9c2ac2c90aef5a4961a2
pythondata-cpu-sentinel|https://github.com/litex-hub/pythondata-cpu-sentinel.git|83f93b66ac8c71d5a35773911f8c82e63803067f
pythondata-cpu-serv|https://github.com/litex-hub/pythondata-cpu-serv.git|cdede3b5e35e0c8c385ff10d783905d95cd864ab
pythondata-cpu-vexiiriscv|https://github.com/litex-hub/pythondata-cpu-vexiiriscv.git|f0b0d455b56767a7ff34efb0a8b56ec237ad481d
pythondata-cpu-vexriscv|https://github.com/litex-hub/pythondata-cpu-vexriscv.git|1979a644dbe64d8d32dfbdd970dccee6add63723
pythondata-cpu-vexriscv-smp|https://github.com/litex-hub/pythondata-cpu-vexriscv-smp.git|0a97182aff60dd830d04f8e10b042ecd99638fa3
pythondata-misc-tapcfg|https://github.com/litex-hub/pythondata-misc-tapcfg.git|fbcb02422940bbb97c492f0c970959d262b33632
pythondata-misc-usb_ohci|https://github.com/litex-hub/pythondata-misc-usb_ohci.git|f141aed6d264c01aa7484a2efe0bcb97a2cbbb34
pythondata-software-compiler_rt|https://github.com/litex-hub/pythondata-software-compiler_rt.git|fcb03245613ccf3079cc833a701f13d0beaae09d
pythondata-software-picolibc|https://github.com/litex-hub/pythondata-software-picolibc.git|a5e11229885a87083ec54034f8d693ba52ea2718
valentyusb|https://github.com/litex-hub/valentyusb.git|0b534ccfccfcdcb7a5db219743f0c0b20e5ec412
EOF

# ---------------------------------------------------------------------------
# 0. Preflight: required host tools
# ---------------------------------------------------------------------------
log "Checking host prerequisites"
command -v git  >/dev/null || die "git not found"
command -v curl >/dev/null || command -v wget >/dev/null || die "need curl or wget"
command -v "$PYTHON_BIN" >/dev/null || die "$PYTHON_BIN not found. Install Python 3.12 (Fedora: 'sudo dnf install python3.12'; Debian/Ubuntu: 'sudo apt install python3.12 python3.12-venv')."

if ! command -v riscv64-linux-gnu-gcc >/dev/null; then
  warn "riscv64-linux-gnu-gcc not found — firmware (software/*) will not build until you install it:"
  warn "    Fedora:        sudo dnf install gcc-riscv64-linux-gnu binutils-riscv64-linux-gnu"
  warn "    Debian/Ubuntu: sudo apt install gcc-riscv64-linux-gnu"
  warn "  (gateware bitstream builds and cocotb sims do NOT need it; continuing.)"
else
  log "Found $(riscv64-linux-gnu-gcc --version | head -1)"
fi

# ---------------------------------------------------------------------------
# 1. Git submodules (LVGL)
# ---------------------------------------------------------------------------
log "Syncing git submodules (LVGL)"
git submodule update --init --recursive

# Seed wifi_secrets.h from the template if absent (gitignored; holds your PSK).
WIFI_SECRETS="software/winc_test/wifi_secrets.h"
if [ ! -f "$WIFI_SECRETS" ] && [ -f "${WIFI_SECRETS}.example" ]; then
  cp "${WIFI_SECRETS}.example" "$WIFI_SECRETS"
  warn "Created $WIFI_SECRETS from template — edit it with your SSID/PSK before building the WINC apps."
fi

# ---------------------------------------------------------------------------
# 2. OSS CAD Suite (yosys + nextpnr-ecp5 + iverilog + openFPGALoader)
# ---------------------------------------------------------------------------
if [ "$SKIP_TOOLCHAIN" -eq 1 ]; then
  log "Skipping OSS CAD Suite download (--skip-toolchain)"
elif [ -x "oss-cad-suite/oss-cad-suite/bin/yosys" ]; then
  log "OSS CAD Suite already present ($(oss-cad-suite/oss-cad-suite/bin/yosys --version 2>/dev/null | head -1))"
else
  log "Downloading OSS CAD Suite ${OSS_CAD_DATE} (~2.4 GB)"
  mkdir -p oss-cad-suite
  if command -v curl >/dev/null; then
    curl -fL -o "/tmp/${OSS_CAD_FILE}" "$OSS_CAD_URL"
  else
    wget -O "/tmp/${OSS_CAD_FILE}" "$OSS_CAD_URL"
  fi
  log "Extracting into oss-cad-suite/ (run.sh expects oss-cad-suite/oss-cad-suite/bin)"
  tar -xzf "/tmp/${OSS_CAD_FILE}" -C oss-cad-suite
  rm -f "/tmp/${OSS_CAD_FILE}"
fi

# ---------------------------------------------------------------------------
# 3. Clone pinned LiteX/Migen stack into litex-setup/
# ---------------------------------------------------------------------------
log "Cloning/pinning LiteX + Migen sources under litex-setup/"
mkdir -p litex-setup
while IFS='|' read -r name url commit; do
  [ -z "${name:-}" ] && continue
  dest="litex-setup/$name"
  if [ -d "$dest/.git" ]; then
    have="$(git -C "$dest" rev-parse HEAD 2>/dev/null || echo none)"
    case "$have" in "$commit"*) log "  $name already at $commit"; continue;; esac
    log "  $name: fetching $commit"
    git -C "$dest" fetch --quiet origin "$commit" 2>/dev/null || git -C "$dest" fetch --quiet --all
  else
    log "  $name: cloning"
    git clone --quiet "$url" "$dest"
    git -C "$dest" fetch --quiet origin "$commit" 2>/dev/null || true
  fi
  git -C "$dest" checkout --quiet "$commit" || die "could not checkout $commit in $name"
done <<< "$LITEX_REPOS"

# ---------------------------------------------------------------------------
# 4. Apply the liblitespi XIP patch to the freshly-checked-out litex
# ---------------------------------------------------------------------------
PATCH="patches/litex-spiflash-skip-master-init.patch"
SPIFLASH_C="litex-setup/litex/litex/soc/software/liblitespi/spiflash.c"
if grep -q "SPIFLASH_SKIP_MASTER_INIT" "$SPIFLASH_C" 2>/dev/null; then
  log "liblitespi XIP patch already applied"
elif [ -f "$PATCH" ]; then
  log "Applying liblitespi XIP patch (required for the flash-XIP boot chain)"
  ( cd litex-setup/litex && git apply "$REPO_ROOT/$PATCH" ) \
    || die "patch failed to apply — see docs/boot_chain.md"
else
  warn "Patch $PATCH not found; flash-XIP BIOS will crash without it. See docs/boot_chain.md."
fi

# ---------------------------------------------------------------------------
# 4b. Apply the icepi_zero platform patch (ext_reset pin + gpio/rgb_led/mcp3008)
# ---------------------------------------------------------------------------
IZ_PATCH="patches/litex-boards-icepi-zero-io.patch"
IZ_PLATFORM="litex-setup/litex-boards/litex_boards/platforms/icepi_zero.py"
if grep -q "ext_reset" "$IZ_PLATFORM" 2>/dev/null; then
  log "icepi_zero platform patch already applied"
elif [ -f "$IZ_PATCH" ]; then
  log "Applying icepi_zero platform patch (ext_reset pin required by icepi_zero_base.py)"
  ( cd litex-setup/litex-boards && git apply "$REPO_ROOT/$IZ_PATCH" ) \
    || die "icepi_zero platform patch failed to apply"
else
  warn "Patch $IZ_PATCH not found; builds will fail with 'resource not found ext_reset:None'."
fi

# ---------------------------------------------------------------------------
# 5. Python venv + editable installs
# ---------------------------------------------------------------------------
if [ ! -d ".venv" ]; then
  log "Creating .venv with $PYTHON_BIN"
  "$PYTHON_BIN" -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
log "Using $(python --version) at $(command -v python)"
python -m pip install --quiet --upgrade pip wheel setuptools

log "Installing pinned LiteX/Migen stack (editable)"
# migen + litex first, then the rest (order is harmless for editable installs).
while IFS='|' read -r name url commit; do
  [ -z "${name:-}" ] && continue
  python -m pip install --quiet -e "litex-setup/$name" || die "pip install -e litex-setup/$name failed"
done <<< "$LITEX_REPOS"

log "Installing core Python deps (sim + tools)"
python -m pip install --quiet \
  "cocotb==2.0.1" "pyserial==3.5" "numpy" "pillow" "requests==2.32.5" "pyyaml"

if [ "$WITH_ML" -eq 1 ]; then
  log "Installing ML/training deps (CPU torch wheels — for tools/train_snn*.py)"
  python -m pip install --quiet \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu
  python -m pip install --quiet "snntorch==0.9.4" "scipy" "sympy==1.14.0"
else
  warn "Skipped ML deps (torch/snntorch). Re-run with --with-ml to build/train SNN weights."
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
cat <<DONE

$(log "Setup complete.")
Next steps:
  • Run the cocotb sims:     cd sim/cocotb && ./run.sh
  • Build a bitstream:        .venv/bin/python icepi_zero_all.py --build
  • Build firmware:           make -C software/snn_mnist_demo  (needs riscv64-linux-gnu-gcc)
  • Flash the board:          .venv/bin/python flash.py        (needs the IcePi Zero + FTDI cable)

Note: hardware-in-the-loop steps (flash.py, UART weight streaming, WINC/LCD bring-up)
require the physical board. For WiFi, edit software/winc_test/wifi_secrets.h (seeded from
wifi_secrets.h.example) with your SSID/PSK.
See README.md and docs/ for details.
DONE
