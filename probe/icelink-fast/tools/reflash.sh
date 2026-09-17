#!/bin/bash
# Reflash the probe's firmware over USB, from a running probe.
#
#   sudo probe/icelink-fast/tools/reflash.sh [image.bin]
#
# Wraps the recipe in probe/icelink-fast/README.md so it is one approved
# command instead of four hand-typed ones. It does the part `make usb-flash`
# cannot: putting the probe into its bootloader first, and mounting the
# MAINTENANCE drive WITHOUT the `flush` option that udisks adds by default and
# that DAPLink silently rejects every image through.
#
# Needs root for the mount, which is the only reason this is a script and not
# part of the Makefile.
#
# Recovery, if the new image misbehaves: the probe drops back to MAINTENANCE on
# its own when nothing is listening (the dead-man timer), and the previous
# image can be written the same way. Keep a known-good binary around --
# `make` from a clean checkout reproduces it byte-for-byte.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMG="${1:-$HERE/build/icelink-fast.bin}"
MNT=/mnt/daplink
CONSOLE=/dev/icelink-console
APP_ID="cafe:4001"
BL_ID="0d28:0204"

die() { echo "FAILED: $*" >&2; exit 1; }

[ "$(id -u)" = 0 ] || die "run me with sudo (the mount needs root)"
[ -f "$IMG" ] || die "no image at $IMG (build it: make -C $HERE)"
echo "image: $IMG ($(stat -c%s "$IMG") bytes, md5 $(md5sum "$IMG" | cut -d' ' -f1))"

# --- get into the bootloader ---------------------------------------------
if lsusb | grep -qi "$APP_ID"; then
    echo "probe is running the app; asking it to enter the bootloader ('b')"
    [ -e "$CONSOLE" ] || die "$CONSOLE missing though the app is running"
    printf b > "$CONSOLE" || die "could not write to $CONSOLE"
    for _ in $(seq 1 20); do
        sleep 1
        lsusb | grep -qi "$BL_ID" && break
    done
fi
lsusb | grep -qi "$BL_ID" \
    || die "probe did not enter the bootloader (no $BL_ID on the bus)"
echo "bootloader is up"

# --- mount MAINTENANCE without flush -------------------------------------
DEV=/dev/disk/by-label/MAINTENANCE
for _ in $(seq 1 10); do [ -e "$DEV" ] && break; sleep 1; done
[ -e "$DEV" ] || die "MAINTENANCE volume never appeared"

# Drop any automount first: udisks mounts vfat with `flush`, which interleaves
# metadata among the file's data sectors. DAPLink tracks a transfer by
# contiguous data-sector writes, so it finalises early and rejects the image.
if findmnt -n -S "$(readlink -f "$DEV")" >/dev/null 2>&1; then
    echo "unmounting the automount"
    umount "$(readlink -f "$DEV")" || die "could not unmount the automount"
fi
mkdir -p "$MNT"
mount -t vfat -o rw "$DEV" "$MNT" || die "mount failed"
case ",$(findmnt -n -o OPTIONS --target "$MNT")," in
    *,flush,*) umount "$MNT"; die "mounted with flush; DAPLink would reject this" ;;
esac
echo "mounted $DEV -> $MNT (no flush)"

[ -f "$MNT/FAIL.TXT" ] && { echo "note: a FAIL.TXT from an earlier attempt:"; \
                            sed 's/^/  /' "$MNT/FAIL.TXT"; }

# --- copy, then unmount: the unmount is what commits it -------------------
cp "$IMG" "$MNT/" || { umount "$MNT"; die "copy failed"; }
sync
echo "copied; unmounting to commit (DAPLink programs and resets itself now)"
umount "$MNT" || die "umount failed -- nothing was committed"

for i in $(seq 1 30); do
    sleep 1
    if lsusb | grep -qi "$APP_ID"; then
        echo "PROGRAMMED after ${i}s: probe is running the app"
        lsusb | grep -i "$APP_ID"
        ls -l /dev/icelink-* 2>/dev/null
        exit 0
    fi
done

echo "the probe has not come back after 30 s; looking for a rejection" >&2
if mount -t vfat -o ro "$DEV" "$MNT" 2>/dev/null; then
    [ -f "$MNT/FAIL.TXT" ] && { echo "--- FAIL.TXT ---" >&2; cat "$MNT/FAIL.TXT" >&2; }
    umount "$MNT"
fi
echo "
If FAIL.TXT appeared: DO NOT reset or replug. A rejected write can leave a
valid-looking vector table that hangs before USB starts, which takes the probe
off the bus entirely and leaves only SWD. Recover with 'make status' then
'make erase-app' over SWD." >&2
exit 1
