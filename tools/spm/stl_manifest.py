"""The SPM meshes, weighed and hashed, before any of them is ever committed.

No binary belongs in a repository on a guess. This names each candidate by size and content, and
writes the manifest that will travel beside them so that a later clone can tell whether the bytes it
got are the bytes that were meant. Of the nine, the translucent overlay is held out as pure display, and
that held-out file carries most of the weight, so the list is a decision about the mechanism and not a
copy of a folder.

    python -B blender_ik_addon/tools/spm/stl_manifest.py            # report
    python -B blender_ik_addon/tools/spm/stl_manifest.py --write     # write the manifest next to them
    python -B blender_ik_addon/tools/spm/stl_manifest.py --check     # verify against the manifest

Nothing is copied and nothing binary is written by this script; --write writes a text file.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys

ROOT = None


def _find_root() -> str:
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(6):
        if os.path.isdir(os.path.join(d, "blender_ik_addon")) and os.path.isdir(os.path.join(d, "SPM")):
            return d
        d = os.path.dirname(d)
    raise RuntimeError("cannot find the workspace root above this script")


ROOT = _find_root()
SOURCE = os.path.join(ROOT, "SPM", "meshes")
#: where they will go, beside the arm's, when the rig that uses them exists: Slice 3's business
DESTINATION = os.path.join("blender_ik_addon", "meshes", "spm")
#: the translucent overlay is the only one held out. StaticBase is the bench: it SHIPS with the set for
#: display, but the rig's mesh table (Platform, three AxisConnectors, three Disks) does not build it, and the
#: tool's own exported def list references neither this nor the overlay - so both are display, not kinematics.
EXCLUDED = ("GhostOverlay.stl",)
NAME = "SPM_STL.sha256.txt"


def digest(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def rows():
    out = [ ]
    for f in sorted(os.listdir(SOURCE)):
        if not f.endswith(".stl"):
            continue
        p = os.path.join(SOURCE, f)
        out.append({"file": f, "bytes": os.path.getsize(p), "sha256": digest(p),
                    "commit": f not in EXCLUDED,
                    "source": os.path.relpath(p, ROOT).replace("\\", "/")})
    return out


def main() -> int:
    if not os.path.isdir(SOURCE):
        print("no SPM/meshes to weigh at", SOURCE)
        return 2
    r = rows()
    print(f"--- the SPM meshes in {os.path.relpath(SOURCE, ROOT)}")
    for e in sorted(r, key=lambda e: e["bytes"]):
        mark = "commit" if e["commit"] else "hold  "
        print(f"    {mark}  {e['bytes']:>10,}  {e['sha256'][:16]}...  {e['file']}")
    keep = [e for e in r if e["commit"]]
    held = [e for e in r if not e["commit"]]
    print(f"    {'':->12}  {len(keep)} committed (the seven the mechanism is built from, plus the StaticBase"
          f" bench for display): {sum(e['bytes'] for e in keep):,} bytes;"
          f"\n    all {len(r)}: {sum(e['bytes'] for e in r):,} bytes."
          f"\n    held out: {sum(e['bytes'] for e in held):,} bytes across {len(held)} file(s) -"
          f" the translucent overlay, which is a display choice and not a kinematic one."
          f"\n    (StaticBase ships but the rig's mesh table does not build it; only the tool draws the bench.)")
    arm = os.path.join(ROOT, "blender_ik_addon", "meshes")
    if os.path.isdir(arm):
        total = sum(os.path.getsize(os.path.join(arm, f)) for f in os.listdir(arm))
        print(f"    for scale: the arm's meshes/ is {total:,} bytes, so this adds "
              f"{100 * sum(e['bytes'] for e in keep) // max(1, total)}% on top of it")
    print(f"    they would go to {DESTINATION}/ when the rig that loads them exists (Slice 3), and"
          f" the manifest\n    ({NAME}) goes there with them, not before the binaries.")

    manifest = os.path.join(ROOT, DESTINATION, NAME)
    if "--write" in sys.argv:
        try:
            os.makedirs(os.path.dirname(manifest), exist_ok=True)
            with open(manifest, "w", encoding="utf-8", newline="\n") as fh:
                fh.write("# sha256  bytes  file - the SPM meshes, as committed. Regenerate with\n"
                         "# python -B blender_ik_addon/tools/spm/stl_manifest.py --write\n")
                for e in sorted(keep, key=lambda e: e["file"]):
                    fh.write(f"{e['sha256']}  {e['bytes']:>10}  {e['file']}\n")
            print("wrote", os.path.relpath(manifest, ROOT))
        except OSError as exc:
            print(f"could not write the manifest there: {type(exc).__name__}: {exc}")
            return 3
    if "--check" in sys.argv:
        if not os.path.isfile(manifest):
            print("no manifest to check against yet:", os.path.relpath(manifest, ROOT))
            return 4
        bad = [ ]
        for line in open(manifest, encoding="utf-8").read().split("\n"):
            if not line or line.startswith("#") or len(line.split()) != 3:
                continue
            want, size, name = line.split()
            p = os.path.join(ROOT, DESTINATION, name)
            if not os.path.isfile(p):
                bad.append(f"{name}: gone")
            elif os.path.getsize(p) != int(size):
                bad.append(f"{name}: {os.path.getsize(p)} bytes where {size} were meant")
            elif digest(p) != want:
                bad.append(f"{name}: not the bytes the manifest describes")
        print("manifest check: " + ("every mesh is the file it says it is" if not bad else "; ".join(bad)))
        return 1 if bad else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
