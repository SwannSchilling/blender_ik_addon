"""Headless smoke for the dense trajectory capture (NO hardware touched).

Run from anywhere:
  blender --background --factory-startup --python tests/smoke_dense_capture.py

Checks that _sample_track samples the keyed F-curves on a uniform grid (the
scrubber's own curve), that the evaluator plumbing matches Blender's own frame
evaluation, and that Export writes a valid DTO at the requested rate. Never
calls a motion operator: the host may have the real arm attached.
"""
import os
import sys
import tempfile

_ADDON = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(_ADDON)
for _p in (_ROOT, _ADDON):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import bpy                                                  # noqa: E402
import blender_ik_addon as A                                # noqa: E402
from blender_ik_addon import trajectory as T                 # noqa: E402

FAILS: list = []


def check(name: str, cond: bool, detail: str = "") -> None:
    print(("PASS  " if cond else "FAIL  ") + name
          + (("  |  " + detail) if (detail and not cond) else ""))
    if not cond:
        FAILS.append(name)


A.register()
res = bpy.ops.pickik.build_rig()
check("build_rig works headless", res == {'FINISHED'}, repr(res))
rig = A._state.rig
check("rig is alive", rig is not None and rig.alive())

sc = bpy.context.scene
tgt = rig.target
base = (tgt.location.x, tgt.location.y, tgt.location.z)
for fr, off in ((1, 0.0), (11, 0.02), (21, 0.04)):
    tgt.location = (base[0] + off, base[1], base[2])
    tgt.keyframe_insert(data_path="location", frame=fr)

fps, _base, eff = A._effective_fps(sc)
dt = 0.05
pts, kfs = A._sample_track(bpy.context, dt)
t0, t1 = kfs[0], kfs[-1]
n_expect = max(2, int(round((t1 - t0) / dt)) + 1)
check("dense: sample count matches the uniform grid",
      len(pts) == n_expect, "%d != %d" % (len(pts), n_expect))
check("dense: keyframe times are the authored keys",
      [round(x, 6) for x in kfs]
      == [round(1 / eff, 6), round(11 / eff, 6), round(21 / eff, 6)],
      repr(kfs))
step = (t1 - t0) / (len(pts) - 1)
check("dense: uniform spacing with exact ends",
      abs(pts[0][0] - t0) < 1e-9 and abs(pts[-1][0] - t1) < 1e-9
      and all(abs((pts[i + 1][0] - pts[i][0]) - step) < 1e-9
              for i in range(len(pts) - 1)))
check("dense: every row has 7 joints",
      all(len(row) == 7 for _, row in pts))

mid = pts[len(pts) // 2][1]
check("dense: interior samples are interpolated (motion, not glued keys)",
      any(abs(a - b) > 1e-6 for a, b in zip(mid, pts[0][1]))
      and any(abs(a - b) > 1e-6 for a, b in zip(mid, pts[-1][1])))

pk = T.pack_samples(pts, kfs)
want_mid = min(range(len(pts)), key=lambda i: abs(pts[i][0] - kfs[1]))
check("pack: keyframe_idx lands on the grid ends and middle",
      pk["keyframe_idx"] == [0, want_mid, len(pts) - 1],
      repr(pk["keyframe_idx"]))

evs = A._target_fcurve_evaluators(bpy.context)
check("evaluators found the keyed location curves",
      evs is not None and len(evs) == 3)
if evs:
    ok = True
    for f in (1.0, 5.3, 11.0, 17.8, 21.0):
        got = tuple(ev(f) for ev in evs)
        fi = int(f)
        sc.frame_set(fi, subframe=f - fi)
        bpy.context.view_layer.update()
        want = (tgt.location.x, tgt.location.y, tgt.location.z)
        if any(abs(g - w) > 1e-5 for g, w in zip(got, want)):
            ok = False
            break
    check("evaluators match Blender's own frame evaluation", ok)

out = os.path.join(tempfile.gettempdir(), "pickik_export_smoke.json")
res = bpy.ops.pickik.export_trajectory(filepath=out, fps=50)
check("export writes the file", res == {'FINISHED'} and os.path.isfile(out),
      repr(res))
if os.path.isfile(out):
    pk2 = T.load_samples(out)
    dur = pk2["dt_s"] * (pk2["n_samples"] - 1)
    check("export: duration is exactly the authored span",
          abs(dur - (t1 - t0)) < 1e-6, "%r vs %r" % (dur, t1 - t0))
    check("export: dt is near 1/fps (grid ends anchored)",
          abs(pk2["dt_s"] - 1.0 / 50) < 0.15 / 50, repr(pk2["dt_s"]))
    check("export: keyframe_idx marks every authored key",
          pk2.get("keyframe_idx")
          == [0, min(range(pk2["n_samples"]),
                     key=lambda i: abs(pk2["t_s"][i] - kfs[1])),
              pk2["n_samples"] - 1],
          repr(pk2.get("keyframe_idx")))
    check("export: remembered for 'Play file'",
          sc.pickik.traj_file_path == out, sc.pickik.traj_file_path)

A.unregister()
print("RESULT:", "OK" if not FAILS else "FAILED %s" % (FAILS,))
sys.exit(1 if FAILS else 0)