"""After the copy block, prove the relocated tooling is real rather than merely present.

Four things are checked, and any FAIL is a non-zero exit so the block can gate on it:

  copy     every tool landed in tools/spm or tools/gates, and every _scratch original is STILL
           there. This is a copy, not a move: the untracked originals stay put until RUN_PHASE2
           passes from the new locations, so an absent original here is a real finding.
  imports  the relocated battery imports its sibling by the tracked name, not the underscore one.
           Present-but-broken is the failure a plain file listing hides.
  snapshot the committed HTML byte-for-byte equals the working copy, both digests printed. A
           snapshot that silently trails the tool turns every fixture stamp into a claim.
  stamp    the fixture's _stamp.html_file names the tracked snapshot, or says plainly that it was
           last produced from the working copy and will name the snapshot once it is regenerated
           from it. It names neither is a failure.

    python -B blender_ik_addon/tools/spm/verify_move.py
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))


def _find_root() -> str:
    d = _HERE
    for _ in range(6):
        if os.path.isdir(os.path.join(d, "blender_ik_addon")) and os.path.isdir(os.path.join(d, "SPM")):
            return d
        d = os.path.dirname(d)
    raise RuntimeError("cannot find the workspace root above this script")


ROOT = _find_root()
SPM = _HERE
GATES = os.path.join(ROOT, "blender_ik_addon", "tools", "gates")
SCRATCH = os.path.join(ROOT, "_scratch")
WORKING = os.path.join(ROOT, "SPM", "spm_mechanism_demo_meshes_v1_2.html")
SNAPSHOT = os.path.join(SPM, "spm_mechanism_demo_meshes_v1_2.frozen.html")
FIXTURE = os.path.join(ROOT, "blender_ik_addon", "tests", "fixtures", "spm_golden_poses.json")
ROOT_FIXTURE = os.path.join(ROOT, "spm_golden_poses.json")

#: new home, and the untracked original it was copied from
PAIRS = (
    (os.path.join(SPM, "export_spm_golden.py"), os.path.join(SCRATCH, "_export_spm_golden.py")),
    (os.path.join(SPM, "tip_pin_equivalence.py"), os.path.join(SCRATCH, "_test_tip_pin_equivalence.py")),
    (os.path.join(SPM, "peek_spm_golden.py"), os.path.join(SCRATCH, "_peek_spm_golden.py")),
    (os.path.join(GATES, "run_in_blender.py"), os.path.join(SCRATCH, "_run_in_blender.py")),
    (os.path.join(GATES, "syntax_check.py"), os.path.join(SCRATCH, "_syntax_check.py")),
    (os.path.join(GATES, "neg_options_probe.py"), os.path.join(SCRATCH, "_neg_options_probe.py")),
    (os.path.join(GATES, "regen_docs_in_blender.py"), os.path.join(SCRATCH, "_regen_docs_in_blender.py")),
    (os.path.join(GATES, "RUN_PHASE1.ps1"), os.path.join(ROOT, "RUN_PHASE1.ps1")),
    (os.path.join(GATES, "RUN_PHASE2.ps1"), os.path.join(ROOT, "RUN_PHASE2.ps1")),
    (os.path.join(GATES, "COMMIT_PHASE1_A.ps1"), os.path.join(ROOT, "COMMIT_PHASE1_A.ps1")),
    (os.path.join(GATES, "COMMIT_PHASE1_B.ps1"), os.path.join(ROOT, "COMMIT_PHASE1_B.ps1")),
    (os.path.join(GATES, "COMMIT_PHASE2_0.ps1"), os.path.join(ROOT, "COMMIT_PHASE2_0.ps1")),
)


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def short(p):
    return os.path.relpath(p, ROOT).replace("\\", "/")


def main() -> int:
    failed = [ ]
    print("== the move, verified ==")

    # ---- copy ---------------------------------------------------------------------------
    gone, left = [ ], [ ]
    for new, old in PAIRS:
        if not os.path.isfile(new):
            gone.append(short(new))
        if not os.path.isfile(old):
            left.append(f"{short(old)} (the original is GONE: this was to be a copy)")
        elif os.path.isfile(new) and sha(new) != sha(old):
            left.append(f"{short(new)} differs byte-for-byte from {short(old)}")
    print(f"copy     : {len(PAIRS) - len(gone)} of {len(PAIRS)} in place, "
          f"all byte-for-byte equal to their original" if not (gone or left)
          else "copy     : problems")
    for x in gone:
        print(f"    MISSING  {x}")
    for x in left:
        print(f"    NOTE     {x}")
    failed += gone + left

    # ---- imports: can the relocated battery actually run? --------------------------------
    r = subprocess.run([sys.executable, "-B", os.path.join(SPM, "tip_pin_equivalence.py")],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT)
    out = r.stdout or ""
    line = next((l for l in out.split("\n") if l.startswith("ok:")), "")
    if r.returncode == 0 and line:
        print(f"imports  : the relocated battery ran and agrees ({line.split(':', 1)[1].strip()[:64]})")
    else:
        print(f"imports  : the relocated battery could NOT run (exit {r.returncode})")
        for l in out.strip().split("\n")[-6:]:
            print("    " + l)
        failed.append("the relocated battery")

    # ---- snapshot vs working copy ---------------------------------------------------------
    if os.path.isfile(WORKING) and os.path.isfile(SNAPSHOT):
        hw, hs = sha(WORKING), sha(SNAPSHOT)
        print(f"snapshot : working  {hw}")
        print(f"           snapshot {hs}")
        if hw == hs:
            print("           the committed snapshot is the tool, byte for byte")
        else:
            print("           the snapshot TRAILS the working copy: re-stamp in the same commit as"
                  " the fixture, or the stamp describes bytes the tool no longer is")
            failed.append("snapshot trails the working copy")
    else:
        print(f"snapshot : missing ({short(SNAPSHOT) if not os.path.isfile(SNAPSHOT) else 'working copy'})")
        failed.append("snapshot or working copy missing")

    # ---- what the fixture believes about its own origin -----------------------------------
    path = FIXTURE if os.path.isfile(FIXTURE) else (ROOT_FIXTURE if os.path.isfile(ROOT_FIXTURE) else "")
    if not path:
        print("stamp    : no fixture anywhere; 1B produces it")
        failed.append("no fixture")
    else:
        want = short(SNAPSHOT)
        got = json.load(open(path, encoding="utf-8")).get("_stamp", { }).get("html_file", "")
        here = short(path)
        if got == want:
            print(f"stamp    : {here} names the tracked snapshot ({got})")
        elif got.endswith("spm_mechanism_demo_meshes_v1_2.html"):
            print(f"stamp    : {here} names the working copy ({got}); that is what --allow-working-copy"
                  f"\n             stamps, and 1B will re-stamp it to {want} when it regenerates the"
                  f" fixture from the snapshot")
        else:
            print(f"stamp    : {here} names {got!r}, which is neither the snapshot nor the working copy")
            failed.append("fixture stamps an origin that is neither")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
