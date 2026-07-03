#!/usr/bin/env python3
"""IcePi Zero system packages ("syspkg").

A syspkg is a single tar archive that captures one *matched* SoC variant:
the bitstream, its BIOS, the loader, the app, the CSR map and (optionally)
the generated C headers -- everything that must agree on CSR layout to boot.

Why this exists: the SoC no longer fits every feature in one build, so each
use case is its own bitstream. Every `icepi_zero_*` top builds to the SAME
`build/icepi_zero/` dir, so building the next variant CLOBBERS the previous
one's matched bitstream/BIOS/csr.csv. `syspkg pack` snapshots a build into a
versioned, self-contained unit right after it is built; `flash.py --syspkg`
flashes the whole matched set in one session.

  ./syspkg.py pack winc --app software/winc_loader_demo/firmware.bin
  ./syspkg.py info  dist/winc.syspkg
  ./syspkg.py headers dist/winc.syspkg -o /tmp/winc_headers   # rebuild apps

The slot offsets / .fbi rules are single-sourced from flash.py's SLOTS so the
two tools can never drift.
"""
import argparse
import datetime
import hashlib
import io
import json
import os
import struct
import sys
import tarfile
import zlib

from flash import SLOTS          # single source of (offset, wrap_fbi, default)

REPO = os.path.dirname(os.path.abspath(__file__))
SYSPKG_VERSION = 1
MANIFEST_NAME = "manifest.json"

# Where each slot's raw image comes from inside a finished build tree. `app` is
# always explicit (apps vary). `loader` is INCLUDED by default: the loader drives
# the aux_spi / LiteSPI-master / winc CSRs, so it is CSR-map-specific to its
# gateware. Flashing a new variant's bitstream without its matching loader leaves
# the stale loader CSR-mismatched against the new fabric -- which is itself the
# brick-the-recovery-path failure. So a syspkg ships the loader built against
# THIS variant; rebuild it (`make -C software/c3_flash`) after the gateware
# build so c3_flash.bin matches (pack warns if it looks stale). Use --no-loader
# only to deliberately keep the existing on-flash loader.
#
# CAVEAT (2026-07-03, post-WINC->C3 pivot): unlike the old winc_loader, which
# was rebuilt as part of EACH deployable variant's own gateware (so its CSR
# addresses always matched that variant's fabric), c3_flash.bin is built
# against the separate, fixed `icepi_zero_c3flash.py` SoC (SPIBone + mailbox
# only, no deployable features) -- it is NOT rebuilt per variant and does NOT
# share baseline's/mnist_lcd's/logger's CSR map. Packing a syspkg for one of
# those variants with today's c3_flash.bin would ship a CSR-mismatched loader
# despite the mtime check below passing (it only compares timestamps, not
# actual CSR maps). Don't trust `--slots loader` for a non-c3flash variant
# until add_c3_spibone/add_c3_mailbox are integrated into each deployable's
# own build -- see [[c3-loader-production-scope]].
SLOT_SOURCES = {
    "bitstream": "{build}/gateware/icepi_zero.bit",
    "bios":      "{build}/software/bios/bios.bin",
    "loader":    "software/c3_flash/c3_flash.bin",
    "app":       None,
}

# Build artifacts that describe the CSR map -- bundled so a syspkg is enough
# to rebuild apps against this exact fabric without rebuilding gateware.
HEADER_SOURCES = {
    "csr.csv":  "{build}/csr.csv",
    "csr.json": "{build}/csr.json",
    "csr.h":    "{build}/software/include/generated/csr.h",
    "soc.h":    "{build}/software/include/generated/soc.h",
}


def _digests(data):
    return {"size": len(data),
            "crc32": f"0x{zlib.crc32(data) & 0xffffffff:08x}",
            "sha256": hashlib.sha256(data).hexdigest()}


def _git_describe():
    try:
        import subprocess
        return subprocess.check_output(
            ["git", "-C", REPO, "describe", "--always", "--dirty"],
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "unknown"


# ---- pack -------------------------------------------------------------------

def pack(args):
    build = os.path.abspath(args.build_dir)
    if not os.path.isdir(build):
        sys.exit(f"build dir not found: {build} (run a variant --build first)")

    want = set(args.slots) if args.slots else {"bitstream", "bios", "loader", "app"}
    if args.no_loader:
        want.discard("loader")
    if args.app:
        want.add("app")
    if "app" in want and not args.app:
        sys.exit("the app slot needs --app PATH (apps vary -- never defaulted)")

    # The loader is CSR-specific to its gateware: a loader older than the
    # bitstream was almost certainly built against a different fabric and will be
    # CSR-mismatched. Warn loudly rather than silently shipping a stale loader.
    if "loader" in want:
        loader_src = os.path.join(REPO, SLOT_SOURCES["loader"])
        bit_src    = SLOT_SOURCES["bitstream"].format(build=build)
        if os.path.exists(loader_src) and os.path.exists(bit_src) and \
           os.path.getmtime(loader_src) < os.path.getmtime(bit_src):
            print("  WARNING: c3_flash.bin is OLDER than the bitstream -- it may be\n"
                  "           built against a different CSR map. Rebuild it to match:\n"
                  "             make -C software/c3_flash\n"
                  "           (or pass --no-loader to keep the existing on-flash loader).")

    members = {}          # arcname -> bytes
    manifest = {
        "syspkg_version": SYSPKG_VERSION,
        "name": args.name,
        "board": "icepi_zero",
        "git": _git_describe(),
        "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "note": args.note or "",
        "slots": {},
        "headers": [],
    }

    # CSR map signature: the anti-mismatch key. sha256 of csr.csv identifies the
    # fabric this BIOS/app were built against.
    csr_csv = os.path.join(build, "csr.csv")
    if os.path.exists(csr_csv):
        with open(csr_csv, "rb") as f:
            manifest["csr_sha256"] = hashlib.sha256(f.read()).hexdigest()
    else:
        manifest["csr_sha256"] = None
        print("  WARNING: no csr.csv in build dir -- package has no CSR signature")

    for slot in ("bitstream", "bios", "loader", "app"):     # flash order
        if slot not in want:
            continue
        if slot == "app":
            src = os.path.abspath(args.app)
        else:
            src = SLOT_SOURCES[slot].format(build=build)
            if not os.path.isabs(src):
                src = os.path.join(REPO, src)
        if not os.path.exists(src):
            sys.exit(f"{slot}: image not found: {src}")
        with open(src, "rb") as f:
            data = f.read()
        offset, wrap_fbi, _ = SLOTS[slot]
        arc = f"slots/{slot}.bin"
        members[arc] = data
        manifest["slots"][slot] = {
            "path": arc, "offset": f"0x{offset:06x}", "fbi": wrap_fbi,
            "source": os.path.relpath(src, REPO), **_digests(data)}

    if not args.no_headers:
        for name, tmpl in HEADER_SOURCES.items():
            src = tmpl.format(build=build)
            if os.path.exists(src):
                with open(src, "rb") as f:
                    members[f"headers/{name}"] = f.read()
                manifest["headers"].append(f"headers/{name}")

    out = args.output or os.path.join(REPO, "dist", f"{args.name}.syspkg")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    mblob = json.dumps(manifest, indent=2).encode()
    with tarfile.open(out, "w") as tar:
        # manifest first so `info`/`flash` can read it without scanning.
        ti = tarfile.TarInfo(MANIFEST_NAME); ti.size = len(mblob)
        tar.addfile(ti, io.BytesIO(mblob))
        for arc, data in members.items():
            ti = tarfile.TarInfo(arc); ti.size = len(data)
            tar.addfile(ti, io.BytesIO(data))

    total = len(mblob) + sum(len(d) for d in members.values())
    print(f"wrote {os.path.relpath(out, REPO)}  ({total/1e3:.1f} kB)")
    print(f"  slots:   {', '.join(manifest['slots'])}")
    print(f"  csr_sha: {manifest['csr_sha256']}")
    print(f"  git:     {manifest['git']}")


# ---- read side (shared with flash.py) --------------------------------------

def read_manifest(path):
    with tarfile.open(path, "r") as tar:
        m = tar.extractfile(MANIFEST_NAME)
        if m is None:
            sys.exit(f"{path}: not a syspkg ({MANIFEST_NAME} missing)")
        manifest = json.load(m)
    if manifest.get("syspkg_version") != SYSPKG_VERSION:
        sys.exit(f"{path}: unsupported syspkg_version "
                 f"{manifest.get('syspkg_version')} (this tool is v{SYSPKG_VERSION})")
    return manifest


def jobs_from_syspkg(path):
    """[(label, offset, data)] ready for flash.py, .fbi applied per manifest.

    Verifies each slot against its recorded crc32 so a truncated/corrupt
    package is caught on the host before anything touches flash."""
    manifest = read_manifest(path)
    jobs = []
    with tarfile.open(path, "r") as tar:
        for slot, meta in manifest["slots"].items():
            f = tar.extractfile(meta["path"])
            if f is None:
                sys.exit(f"{path}: slot {slot} payload {meta['path']} missing")
            data = f.read()
            if f"0x{zlib.crc32(data) & 0xffffffff:08x}" != meta["crc32"]:
                sys.exit(f"{path}: slot {slot} crc mismatch -- package corrupt")
            if meta["fbi"]:
                data = struct.pack("<II", len(data), zlib.crc32(data)) + data
            # label starts with the slot so flash.py's flashed-set detection
            # (label.split(":")[0]) keeps working for the reboot/power-cycle logic.
            jobs.append((f"{slot}:syspkg", int(meta["offset"], 0), data))
    return sorted(jobs, key=lambda j: j[1]), manifest


def info(args):
    manifest = read_manifest(args.path)
    print(f"name:    {manifest['name']}")
    print(f"board:   {manifest['board']}")
    print(f"git:     {manifest['git']}")
    print(f"created: {manifest['created']}")
    print(f"csr_sha: {manifest.get('csr_sha256')}")
    if manifest.get("note"):
        print(f"note:    {manifest['note']}")
    print("slots:")
    for slot, meta in manifest["slots"].items():
        print(f"  {slot:9s} @{meta['offset']}  {meta['size']:>8d} B  "
              f"{meta['crc32']}{'  [fbi]' if meta['fbi'] else ''}")
    if manifest["headers"]:
        print(f"headers: {', '.join(os.path.basename(h) for h in manifest['headers'])}")


def headers(args):
    manifest = read_manifest(args.path)
    if not manifest["headers"]:
        sys.exit("this syspkg carries no headers (packed with --no-headers)")
    dest = os.path.abspath(args.output)
    os.makedirs(dest, exist_ok=True)
    with tarfile.open(args.path, "r") as tar:
        for arc in manifest["headers"]:
            f = tar.extractfile(arc)
            out = os.path.join(dest, os.path.basename(arc))
            with open(out, "wb") as g:
                g.write(f.read())
            print(f"  {os.path.relpath(out, REPO)}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("pack", help="snapshot a build dir into a .syspkg")
    p.add_argument("name", help="package name (e.g. winc, mnist_lcd)")
    p.add_argument("--app", metavar="FILE", help="application .bin for the app slot")
    p.add_argument("--build-dir", default=os.path.join(REPO, "build/icepi_zero"))
    p.add_argument("--slots", nargs="+", metavar="SLOT",
                   help="slots to include (default: bitstream bios loader app)")
    p.add_argument("--no-loader", action="store_true",
                   help="omit the loader slot, keeping the existing on-flash loader "
                        "(default includes a loader matched to this gateware)")
    p.add_argument("--no-headers", action="store_true",
                   help="omit the generated csr.h/soc.h/csr.csv/csr.json")
    p.add_argument("--note", help="freeform note stored in the manifest")
    p.add_argument("-o", "--output", metavar="FILE", help="output path "
                   "(default: dist/<name>.syspkg)")
    p.set_defaults(func=pack)

    p = sub.add_parser("info", help="print a syspkg's manifest")
    p.add_argument("path")
    p.set_defaults(func=info)

    p = sub.add_parser("headers", help="extract the bundled C headers")
    p.add_argument("path")
    p.add_argument("-o", "--output", default=os.path.join(REPO, "syspkg_headers"))
    p.set_defaults(func=headers)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
