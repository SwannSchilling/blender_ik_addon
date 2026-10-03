"""Headless smoke for 'Bake FK keys' + the Motion source toggle (NO motion).

Run from anywhere:
  blender --background --factory-startup --python tests/smoke_fk_bake.py

Bakes a keyed IK-target move onto the Arm7_Ji joint channels, then proves the
baked path is captured with ZERO solver calls and that the toggle gates the
solver. Never calls a motion operator: the host may have the real arm attached.
"""
import math
import os
import sys

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
sc = bpy.context.scene
p = sc.pickik
tgt = rig.target

# Author like a user: key the IK target at three frames.
base = (tgt.location.x, tgt.location.y, tgt.location.z)
for fr, off in ((1, 0.0), (11, 0.02), (21, 0.04)):
    tgt.location = (base[0] + off, base[1], base[2])
    tgt.keyframe_insert(data_path="location", frame=fr)

res = bpy.ops.pickik.bake_fk_keys()
check("bake runs", res == {'FINISHED'}, repr(res))
check("bake switches Motion source to Baked FK",
      p.traj_motion_source == 'BAKED', p.traj_motion_source)
check("bake records the authored key frames",
      p.traj_baked_kf != "", repr(p.traj_baked_kf))
check("bake wrote dense joint keys on all 7 empties",
      all(any(fc.data_path == "rotation_euler" and fc.array_index == 2
              for fc in em.animation_data.action.fcurves)
          for em in rig.joint_empties))

# Capturing the baked source must run ZERO solves: stub the solver out.
real_solve = A._sample_solved_pose
calls = {"n": 0}


def _boom(context, target_m):
    calls["n"] += 1
    raise AssertionError("solver ran during a baked capture")


A._sample_solved_pose = _boom
try:
    pts, kfs, src = A._sample_track(bpy.context, 0.05)
finally:
    A._sample_solved_pose = real_solve
check("baked capture uses the joint curves", src == "joints", src)
check("baked capture ran ZERO solver calls", calls["n"] == 0,
      str(calls["n"]))
check("baked capture is dense and uniform",
      len(pts) >= 2
      and all(len(row) == 7 for _, row in pts)
      and abs((pts[-1][0] - pts[0][0]) - (kfs[-1] - kfs[0])) < 1e-9,
      "%d pts" % len(pts))

# The captured curve IS the bake (the frozen solved motion).
mid_f = 1 + (len(pts) // 2) * 20.0 / (len(pts) - 1)
baked = [em.animation_data.action.fcurves.find("rotation_euler", index=2)
         .evaluate(mid_f) for em in rig.joint_empties]
got = pts[len(pts) // 2][1]
check("captured joints equal the baked curves",
      all(abs(math.degrees(b) - g) < 1e-3 for b, g in zip(baked, got)),
      repr(got))

# Kinematics are computed from the baked path.
kin = T.segment_kinematics(pts, kfs)
check("kinematics: one segment per authored key pair",
      len(kin) == max(0, len(sorted(
          int(t) for t in p.traj_baked_kf.split(",") if t)) - 1),
      "%d segs" % len(kin))

# The solver is gated while in Baked FK mode.
res = bpy.ops.pickik.solve()
check("Solve refuses in Baked FK mode", res == {'CANCELLED'}, repr(res))

# Clear bake round-trips back to IK mode.
res = bpy.ops.pickik.clear_bake()
check("clear bake runs", res == {'FINISHED'}, repr(res))
check("clear bake returns Motion source to IK",
      p.traj_motion_source == 'IK', p.traj_motion_source)
check("clear bake removed the joint curves",
      all(not any(fc.data_path == "rotation_euler" and fc.array_index == 2
                  for fc in em.animation_data.action.fcurves)
          if (em.animation_data and em.animation_data.action) else True
          for em in rig.joint_empties))
check("cleared bake has no recorded keys", p.traj_baked_kf == "",
      repr(p.traj_baked_kf))

A.unregister()
print("RESULT:", "OK" if not FAILS else "FAILED %s" % (FAILS,))
sys.exit(1 if FAILS else 0)