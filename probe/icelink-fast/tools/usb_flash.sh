#!/usr/bin/env bash
# Copy an image to the DAPLink MAINTENANCE drive and report what the bootloader
# actually did with it.
#
# TWO things this guards, both learned the hard way on 2026-09-11:
#
# 1. The mount must NOT have the `flush` option. udisks/GNOME automounts vfat
#    with flush, which writes metadata eagerly and interleaves FAT/directory
#    sectors among the file's data sectors. DAPLink tracks a transfer by
#    contiguous data-sector writes, so the interleave makes it finalise early
#    and reject the image ("update sent was incomplete"). Five attempts failed
#    that way before the cause was found. Mount it yourself instead:
#
#        sudo mount -t vfat -o rw,uid=$(id -u),gid=$(id -g) /dev/sdX /mnt/daplink
#        make usb-flash MAINT=/mnt/daplink
#        sudo umount /mnt/daplink      # <-- this is what commits it
#
# 2. The bootloader validates AFTER accepting the file, and signals rejection
#    by remounting with FAIL.TXT -- so a plain `cp` reports success on a write
#    that was thrown away. Worse, a rejected write can still have programmed
#    the first page, leaving a valid-looking vector table that the bootloader
#    launches and that hangs before USB comes up. If that happens the probe
#    drops off the bus entirely and only SWD gets it back. So: never reset
#    after a FAIL.TXT.
#
# Commit is asynchronous. With a no-flush mount the image is not programmed
# until the filesystem is unmounted, and DAPLink then takes several seconds to
# program and reset -- do not read an early sample as failure.
set -u

IMG="${1:?usage: usb_flash.sh <image.bin> <maintenance-mount>}"
MNT="${2:?usage: usb_flash.sh <image.bin> <maintenance-mount>}"
WAIT="${3:-30}"

APP_ID="cafe:4001"      # the probe running our firmware
BL_ID="0d28:0204"       # the DAPLink bootloader

[ -f "$IMG" ] || { echo "no such image: $IMG" >&2; exit 1; }
[ -d "$MNT" ] || {
    echo "MAINTENANCE not mounted at $MNT" >&2
    echo "the probe is running an app -- reset into the bootloader first ('b')" >&2
    exit 1
}

# Guard 1: refuse outright on a flush mount. This is not a warning -- it is
# known to fail, and failing here costs nothing while failing there can strand
# the probe.
opts=$(findmnt -n -o OPTIONS --target "$MNT" 2>/dev/null || true)
if [[ ",$opts," == *",flush,"* ]]; then
    echo "REFUSING: $MNT is mounted with 'flush' (udisks default)." >&2
    echo "  DAPLink rejects images written through a flush mount." >&2
    echo "  Remount it by hand, then retry:" >&2
    dev=$(findmnt -n -o SOURCE --target "$MNT" 2>/dev/null)
    echo "    sudo umount $dev" >&2
    echo "    sudo mkdir -p /mnt/daplink" >&2
    echo "    sudo mount -t vfat -o rw,uid=\$(id -u),gid=\$(id -g) $dev /mnt/daplink" >&2
    echo "    make usb-flash MAINT=/mnt/daplink" >&2
    echo "    sudo umount /mnt/daplink" >&2
    exit 1
fi

had_fail=0
[ -f "$MNT/FAIL.TXT" ] && had_fail=1
before=$(grep -i '^Remount count:' "$MNT/DETAILS.TXT" 2>/dev/null | tr -dc '0-9')
before=${before:-0}
[ "$had_fail" = 1 ] && echo "note: a FAIL.TXT from an earlier attempt is present"

echo "copying $(basename "$IMG") ($(stat -c%s "$IMG") B) -> $MNT"
cp "$IMG" "$MNT/" || { echo "copy failed" >&2; exit 1; }
sync
echo "copied. NOW UNMOUNT to commit it:  sudo umount $MNT"
echo "watching for up to ${WAIT}s..."

for _ in $(seq 1 "$WAIT"); do
    sleep 1

    if lsusb 2>/dev/null | grep -qi "$APP_ID"; then
        echo "PROGRAMMED: probe re-enumerated as $APP_ID and is running the app"
        exit 0
    fi

    if [ -d "$MNT" ] && [ -f "$MNT/FAIL.TXT" ]; then
        now=$(grep -i '^Remount count:' "$MNT/DETAILS.TXT" 2>/dev/null | tr -dc '0-9')
        now=${now:-0}
        if [ "$had_fail" = 0 ] || [ "${now:-0}" -gt "${before:-0}" ]; then
            echo "REJECTED by the bootloader:" >&2
            sed 's/^/  /' "$MNT/FAIL.TXT" >&2
            echo "  (remount count $before -> $now)" >&2
            echo >&2
            echo "DO NOT RESET. A rejected write can leave a valid-looking vector" >&2
            echo "table in the app slot; resetting would launch it and it may hang" >&2
            echo "before USB starts, taking the probe off the bus. Recover over SWD" >&2
            echo "with 'make status' then 'make erase-app'." >&2
            exit 1
        fi
    fi
done

echo "no result after ${WAIT}s."
echo "  Did you unmount? The image is not committed until the filesystem is."
echo "  If it is unmounted and the probe still shows $BL_ID, give it another"
echo "  10s before concluding anything -- commit is not instant."
exit 2
