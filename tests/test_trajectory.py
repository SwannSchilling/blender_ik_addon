"""Headless tests for the trajectory DTO - no bpy, no CAN, any Python.

    python blender_ik_addon/tests/test_trajectory.py

Covers what both Play buttons and the exporters lean on: pack_samples()
(grids, keyframe mapping, velocities) and load_samples() (validation +
normalisation) round-tripping each other.
"""
from __future__ import annotations

import json
import os
import sys
import tempfile

_ADDON = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ADDON not in sys.path:
    sys.path.insert(0, _ADDON)

import trajectory as T                                     # noqa: E402


def main() -> int:
    print("== trajectory DTO: pack / load / keyframe mapping ==")
    fails: list = []

    def check(name: str, cond: bool, detail: str = "") -> None:
        print(("PASS  " if cond else "FAIL  ") + name
              + (("  | " + detail) if (detail and not cond) else ""))
        if not cond:
            fails.append(name)

    # A dense uniform path: 1.0 s at 100 Hz, joint 1 ramps 0 -> 10 deg.
    pts = [(i * 0.01,
            [i * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) for i in range(101)]
    kfs = [0.0, 0.5, 1.0]
    pk = T.pack_samples(pts, kfs)
    check("pack: uniform dt_s", abs(pk["dt_s"] - 0.01) < 1e-9, repr(pk["dt_s"]))
    check("pack: sample count", pk["n_samples"] == 101, repr(pk["n_samples"]))
    check("pack: keyframe_idx maps to the nearest samples",
          pk["keyframe_idx"] == [0, 50, 100], repr(pk["keyframe_idx"]))
    check("pack: central-difference velocities",
          abs(pk["q_vel_deg_s"][50][0] - 10.0) < 1e-6,
          repr(pk["q_vel_deg_s"][50][0]))

    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    try:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(pk, fh)
        pk2 = T.load_samples(path)
        check("load: round trip preserves the path",
              pk2["q_pos_deg"] == pk["q_pos_deg"]
              and abs(pk2["dt_s"] - pk["dt_s"]) < 1e-12)
        check("load: keyframe_idx survives",
              pk2.get("keyframe_idx") == [0, 50, 100],
              repr(pk2.get("keyframe_idx")))

        with open(path, "w", encoding="utf-8") as fh:
            json.dump({k: pk[k] for k in ("dt_s", "q_pos_deg")}, fh)
        pk3 = T.load_samples(path)
        check("load: missing velocities become zeros",
              pk3["q_vel_deg_s"] == [[0.0] * 7 for _ in pts])

        with open(path, "w", encoding="utf-8") as fh:
            json.dump({"dt_s": 0.01, "q_pos_deg": [[1, 2, 3]]}, fh)
        try:
            T.load_samples(path)
            check("load: short rows are refused", False, "no error raised")
        except ValueError as e:
            check("load: short rows are refused",
                  "not 7 joint values" in str(e), str(e))

        with open(path, "w", encoding="utf-8") as fh:
            json.dump({"dt_s": 0, "q_pos_deg": [[0.0] * 7, [1.0] * 7]}, fh)
        try:
            T.load_samples(path)
            check("load: a zero dt_s is refused", False, "no error raised")
        except ValueError as e:
            check("load: a zero dt_s is refused", "dt_s" in str(e), str(e))
    finally:
        os.remove(path)

    try:
        T.load_samples(path + ".missing")
        check("load: a missing file is refused", False, "no error raised")
    except ValueError as e:
        check("load: a missing file is refused",
              "cannot read" in str(e), str(e))

    # segment kinematics on the linear ramp above (10 deg/s on joint 1):
    # keys at 0.0 / 0.5 / 1.0 s -> two segments of 5 deg in 0.5 s.
    kin = T.segment_kinematics(pts, kfs)
    check("kin: one segment per key pair", len(kin) == 2, repr(len(kin)))
    s0 = kin[0]
    check("kin: segment dt matches the key times",
          abs(s0["dt"] - 0.5) < 1e-9, repr(s0["dt"]))
    j0 = s0["joints"][0]
    check("kin: dq across the segment", abs(j0["dq_deg"] - 5.0) < 1e-9,
          repr(j0["dq_deg"]))
    check("kin: mean speed = dq/dt", abs(j0["v_mean_deg_s"] - 10.0) < 1e-6,
          repr(j0["v_mean_deg_s"]))
    check("kin: peak speed on a constant ramp",
          abs(j0["v_peak_deg_s"] - 10.0) < 1e-6, repr(j0["v_peak_deg_s"]))
    check("kin: zero accel on a constant ramp", j0["a_peak_deg_s2"] < 1e-6,
          repr(j0["a_peak_deg_s2"]))
    check("kin: untouched joints stay zero",
          all(abs(r["dq_deg"]) < 1e-12 for r in s0["joints"][1:]))
    check("kin: format produces one line per segment",
          len(T.format_kinematics(kin)) == 2)

    print(("== %d failed ==" % len(fails)) if fails else "== all ok ==")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())