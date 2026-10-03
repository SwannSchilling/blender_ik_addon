"""Headless smoke for the 'Play file' button (NO hardware touched).

Run from anywhere:
  blender --background --factory-startup --python tests/smoke_play_traj_file.py

Checks registration of the operator, the trajectory loader, the shared
panel-knob resolver and the panel wiring. Deliberately never calls execute()
on a motion operator: the host may have the real arm attached.
"""
import inspect
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
check("register() accepts the new class", len(A.CLASSES) >= 20,
      "%d classes" % len(A.CLASSES))
check("bpy.ops.pickik.play_trajectory_file exists",
      hasattr(bpy.ops.pickik, "play_trajectory_file"))
check("bpy.ops.pickik.play_trajectory still exists",
      hasattr(bpy.ops.pickik, "play_trajectory"))

path = os.path.join(_ADDON, "standalone", "trajectory.json")
pk = T.load_samples(path)
check("load_samples reads the repo's sample export",
      isinstance(pk, dict) and pk.get("n_samples", 0) > 0 and pk.get("dt_s", 0) > 0,
      repr(pk)[:80])
check("rows are 7 joint floats",
      all(isinstance(r, list) and len(r) == 7
          and all(isinstance(v, float) for v in r) for r in pk["q_pos_deg"]))
check("velocity rows match the sample count",
      len(pk["q_vel_deg_s"]) == pk["n_samples"])
check("keyframe_idx survives the load", "keyframe_idx" in pk)

bad = os.path.join(tempfile.gettempdir(), "pickik_bad_traj.json")
with open(bad, "w", encoding="utf-8") as fh:
    fh.write('{"q_pos_deg": [[1,2,3]]}')
try:
    T.load_samples(bad)
    check("loader rejects short rows", False, "no error raised")
except ValueError as e:
    check("loader rejects short rows", "not 7 joint values" in str(e), str(e))
try:
    T.load_samples(os.path.join(tempfile.gettempdir(), "pickik_missing.json"))
    check("loader rejects a missing file", False, "no error raised")
except ValueError as e:
    check("loader rejects a missing file", "cannot read" in str(e), str(e))

knobs = A._traj_stream_settings(bpy.context.scene.pickik)
check("_traj_stream_settings resolves 10 knobs",
      isinstance(knobs, tuple) and len(knobs) == 10, repr(knobs)[:100])

src = inspect.getsource(A.PICKIK_PT_main.draw) + inspect.getsource(
    A.PICKIK_PT_main._draw)
check("panel draws the Play file button", "pickik.play_trajectory_file" in src)
check("panel draws the Exported file field", "traj_file_path" in src)

A.unregister()
print("RESULT:", "OK" if not FAILS else "FAILED %s" % (FAILS,))
sys.exit(1 if FAILS else 0)