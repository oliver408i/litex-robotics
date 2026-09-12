#!/usr/bin/env bash
# Run an OpenOCD script, retrying across speeds until the AP examines.
#
# The ch347 + IC-grabber link on this bench is marginal: `SWD DPIDR` reads
# reliably but `dap init` fails to bring up CSYSPWRUPACK maybe half the time,
# with no consistent dependence on adapter speed. Retrying is the pragmatic
# fix -- a single attempt proves nothing.
#
#   ./host/swd.sh tools/openocd-flash.cfg
#   ./host/swd.sh /tmp/status.cfg 12

set -u
CFG="${1:?usage: swd.sh <openocd.cfg> [max_attempts]}"
MAX="${2:-10}"
SPEEDS=(200 100 400 50 300)

HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$HERE/../../../.venv/bin/python"
[ -x "$PY" ] || PY=python3

# NRST must be driven high or the DP returns garbage.
"$PY" "$HERE/ch347_nrst.py" --high >/dev/null 2>&1

tmp=$(mktemp /tmp/swd-retry-XXXXXX.cfg)
trap 'rm -f "$tmp"' EXIT

for attempt in $(seq 1 "$MAX"); do
    spd=${SPEEDS[$(( (attempt - 1) % ${#SPEEDS[@]} ))]}
    sed "s/^adapter speed .*$/adapter speed $spd/" "$CFG" > "$tmp"
    out=$(timeout 180 openocd -f "$tmp" 2>&1)
    if ! grep -q "{dap init}" <<<"$out"; then
        echo "$out" | grep -viE "^(Open On-Chip|Licensed|For bug|http)|DEPRECATED"
        echo "--- ok on attempt $attempt at ${spd} kHz ---"
        exit 0
    fi
    echo "attempt $attempt (${spd} kHz): dap init failed, retrying" >&2
done

echo "FAILED after $MAX attempts -- check SWDIO/SWCLK/GND clip contact" >&2
exit 1
