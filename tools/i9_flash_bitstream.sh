#!/bin/bash
# Program the Colorlight i9's SPI flash with a bitstream, over Ethernet.
#
# This is the `--flash` that targets/colorlight_i9/base.py refuses to implement:
# the probe can configure SRAM over JTAG but cannot write the board's flash. It
# arrives by a different road instead -- Etherbone stages the bitstream in SDRAM
# and the BIOS programs it from there, using hardware and firmware the SoC
# already has. No new gateware.
#
#   .venv/bin/litex_server --udp --udp-ip 192.168.0.50 &     # once
#   tools/i9_flash_bitstream.sh build/colorlight_i5/gateware/colorlight_i5.bit
#
# Requires an Etherbone-capable bitstream to be RUNNING on the board (that is
# how the upload gets in), i.e. targets/colorlight_i9/eth.py --with-etherbone.
#
# Order of operations is the safety argument, so do not rearrange it:
#
#   1. upload, and verify the staged copy by CRC BEFORE anything is erased.
#      A bad upload then costs nothing -- the flash is still intact.
#   2. only then erase, program, and verify what actually landed in flash.
#
# ERASE BEFORE WRITE IS NOT OPTIONAL. spiflash_write_stream verifies as it
# goes, and against un-erased flash it prints an error PER BYTE -- a megabyte
# of complaints at 115200 baud, which buries every later reply for the better
# part of an hour and looks exactly like a hang. This script always erases
# first, and aborts if it sees that error anyway.
#
# The flash is NOT a brick risk: if the image is bad the ECP5 simply fails to
# configure at power-up, and the probe still loads a bitstream into SRAM over
# JTAG exactly as before. Flash is the boot path, not the only path.
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1

BIT="${1:-}"
STAGE="${STAGE:-0x40100000}"                     # SDRAM, clear of the BIOS
BACKUP="${BACKUP:-$HOME/i9_flash_backup_2MB.bin}"
# Pace the upload. At full rate (~12 MB/s) a 1.15 MB write into SDRAM has been
# observed to wedge the Wishbone bus: the CPU goes silent -- console dead, so
# the very next `crc` never answers -- while gateware ICMP keeps replying, so
# the board still pings and looks fine. It is intermittent; the same upload
# succeeds more often than not, which is exactly what makes it worth pacing by
# default rather than chasing later. 2 ms per burst costs ~3 s and has been
# reliable. Set PACE=0 to go fast if you are feeling lucky.
PACE="${PACE:-0.002}"
PY=".venv/bin/python"
CON="$PY tools/soc_console.py"
NOT_ERASED='not erased'

die() { echo "FAILED: $*" >&2; exit 1; }

[ -n "$BIT" ] || die "usage: $0 <bitstream.bit>   (env: STAGE, BACKUP)"
[ -f "$BIT" ] || die "no such bitstream: $BIT"

# --- preflight ------------------------------------------------------------
if [ ! -f "$BACKUP" ]; then
    die "no flash backup at $BACKUP.
  Erasing without one throws away whatever the board shipped with, which is
  not recoverable. Take one first (it reads the flash over Etherbone):
      $PY tools/etherbone_backup.py $BACKUP
  or set BACKUP=/path/to/existing to point at one you already have."
fi
echo "backup present : $BACKUP ($(stat -c%s "$BACKUP") bytes)"

ss -lnt 2>/dev/null | grep -q ':1234' \
    || die "litex_server is not listening on :1234.
  Start it first:  $PY -m litex.tools.litex_server --udp --udp-ip <board-ip>
  (and remember it must be restarted after every bitstream reload)."

# --- 1. stage in SDRAM and verify BEFORE touching the flash ---------------
echo
echo "=== 1/4  upload to $STAGE ==="
OUT="$($PY tools/etherbone_upload.py "$BIT" "$STAGE" --pace "$PACE")" \
    || die "upload failed"
echo "$OUT" | grep -v '^RESULT'
read -r _ LEN CRC <<<"$(echo "$OUT" | grep '^RESULT')"
[ -n "${LEN:-}" ] && [ -n "${CRC:-}" ] || die "could not parse the upload result"

echo
echo "=== 2/4  verify the staged copy (nothing erased yet) ==="
GOT="$($CON "crc $STAGE $LEN" --expect CRC32 --timeout 120 \
        | grep -ao 'CRC32: [0-9a-f]\{8\}' | tail -1 | awk '{print $2}')"
echo "  host 0x$CRC   device 0x${GOT:-<no reply>}"
[ "$GOT" = "$CRC" ] || die "staged copy does not match the file.
  NOTHING HAS BEEN ERASED -- the flash is untouched.
  If the console gave no reply at all, the upload most likely wedged the bus:
  ping the board (gateware ICMP answers even then, so that proves little),
  reload the bitstream to reset it, and re-run with a larger PACE."
echo "  match -- safe to erase"

# --- 2. erase, program, verify -------------------------------------------
SECTOR=4096
ERASE=$(( (LEN + SECTOR - 1) / SECTOR * SECTOR ))
echo
echo "=== 3/4  erase $ERASE bytes ($((ERASE / SECTOR)) x 4K sectors) ==="
$CON "flash_erase_range 0 $ERASE" --timeout 300 | tail -3

echo
echo "=== 3/4  program $LEN bytes from $STAGE ==="
WOUT="$($CON "flash_write 0 $STAGE $LEN" --fail "$NOT_ERASED" --timeout 600)"
WRC=$?
echo "$WOUT" | tail -3
if [ "$WRC" = 2 ]; then
    die "the flash was not erased where we are writing.
  The write has been abandoned -- do NOT let it run on, it prints an error per
  byte and will flood the console for the better part of an hour. Reload the
  bitstream to reset the CPU and stop it:
      sg dialout -c '$PY targets/colorlight_i9/eth.py --load --with-etherbone --eth-phy 1'
  then re-run this script."
fi

echo
echo "=== 4/4  verify what is actually in flash ==="
FGOT="$($CON "crc 0x00800000 $LEN" --expect CRC32 --timeout 120 \
        | grep -ao 'CRC32: [0-9a-f]\{8\}' | tail -1 | awk '{print $2}')"
echo "  expected 0x$CRC   flash 0x${FGOT:-<no reply>}"
if [ "$FGOT" != "$CRC" ]; then
    die "flash does not match the bitstream.
  Do NOT reconfigure from it. Restore the backup, or simply re-run this script
  -- the board is still running the design that was loaded into SRAM, and the
  probe can reload one at any time."
fi

echo
echo "FLASHED AND VERIFIED."
echo "  Activate it without a power cycle:  $PY tools/icelink.py --refresh"
