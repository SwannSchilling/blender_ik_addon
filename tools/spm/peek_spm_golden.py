"""Print a compact view of the golden fixture, for a human and for the gate table."""
from __future__ import annotations

import json
import os
import sys

def _find_root() -> str:
    """Found rather than counted, so this runs from wherever version control puts it."""
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(6):
        if os.path.isdir(os.path.join(d, "blender_ik_addon")) and os.path.isdir(os.path.join(d, "SPM")):
            return d
        d = os.path.dirname(d)
    raise RuntimeError("cannot find the workspace root above this script")


ROOT = _find_root()


def resolve():
    """Where the fixture actually is: its proper home, the tests/ second-best, or the root.
    Returns the path, or None for a caller that has to know the difference."""
    for p in (os.path.join(ROOT, "blender_ik_addon", "tests", "fixtures", "spm_golden_poses.json"),
              os.path.join(ROOT, "blender_ik_addon", "tests", "spm_golden_poses.json"),
              os.path.join(ROOT, "spm_golden_poses.json")):
        if os.path.isfile(p):
            return p
    print("no fixture yet: run  python -B blender_ik_addon/tools/spm/export_spm_golden.py")
    return None


def rnd(v, n=4):
    return None if v is None else (round(v, n) if isinstance(v, float) else v)


def main() -> int:
    path = resolve()
    if path is None:
        return 2
    d = json.load(open(path, encoding="utf-8"))
    print("fixture:", os.path.relpath(path, ROOT).replace("\\", "/"))
    print("build", d["build"], "| band_mm", d["band_mm"], "| poses", d["counts"]["poses"],
          "| kinds", json.dumps(d["counts"]["by_kind"]))
    print("stamp :", d["_stamp"]["html_sha256"][:16], "|", d["_stamp"]["node_version"], "|",
          d["_stamp"]["html_file"])
    print("pend  :", json.dumps(d.get("pending")))
    print()
    for p in d["poses"]:
        res = rnd(p["closure_residual_max_mm"], 6)
        sig = rnd(p["sigma_min"], 4)
        q = None if p["quat_wxyz"][0] is None else [rnd(v, 5) for v in p["quat_wxyz"]]
        th = [rnd(t) for t in p["theta_rad"]]
        tt = rnd(p["tip_to_pin_mm"][0], 3) if p["tip_to_pin_mm"][0] is not None else None
        qt = p["quat_round_trip_err"]
        qts = "n/a" if qt is None else ("%.1e" % qt)
        print(f"  {p['kind']:<18} t{p['input']['tilt_deg']:>4} p{p['input']['pan_deg']:>4} "
              f"r{p['input']['roll_deg']:>4} valid={str(p['valid']):<24} theta={th} "
              f"sig={str(sig):<9} tip->pin={tt} L={rnd(p['rod_L_mm'],3)} resid={res} "
              f"q={q} qt={qts}")
    g = d["geometry"]
    print("\ngeometry:", {k: rnd(v, 6) for k, v in g.items() if not isinstance(v, list)})
    print("conv    :", d["convention"]["quaternion_order"])
    print("mesh    :", d["convention"]["mesh_frame"], "| scale stl only",
          d["convention"]["scale_for_stl_vertices_only"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
