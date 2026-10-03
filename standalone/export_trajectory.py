"""Headless trajectory exporter - run inside Blender (authoring only).

Samples the arm's keyed timeline densely (the scrubber's own curve, sub-frame
F-curve evaluation + IK per sample) and writes a trajectory JSON for the
stand-alone tools/play_trajectory.py player (or the addon's in-GUI Play).

It should be invoked via Blender's headless mode with the addon registers:

    "C:/Program Files/Blender Foundation/Blender 3.4/blender.exe" -b \
        --factory-startup --python tools/export_trajectory.py -- \
        /path/to/scene.blend /path/to/output.json --fps 60

Args after `--`:
    scene.blend    (optional) a .blend to open; if omitted, the current scene
                   at startup is used (e.g. a script already built a rig).
    out.json       output trajectory path.
    --fps N        sample FPS (default 60).

Any keyframes placed on scene.pickik.q_j1..q_j7 (the addon's "Add Keyframe"
writes these) over scene.frame_start..frame_end are captured.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys


def _build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out_json", nargs="?", help="output trajectory JSON path")
    ap.add_argument("--blend", default="",
                    help="scene.blend to open before sampling (optional)")
    ap.add_argument("--fps", type=int, default=60, help="sample FPS (default 60)")
    return ap


def main(argv: list[str]) -> int:
    args = _build_parser().parse_args(argv)
    out = args.out_json or "pickik_trajectory.json"
    fps = max(5, min(500, args.fps))

    import bpy
    if args.blend and os.path.isfile(args.blend):
        bpy.ops.wm.open_mainfile(filepath=os.path.abspath(args.blend))
    # ensure the addon is available for the rig / q_j props
    import blender_ik_addon  # noqa: F401 (registers props + rig)

    # Dense capture of the keyed timeline's own curve at the requested FPS -
    # the written motion is exactly what the scrubber shows (no re-planning).
    from blender_ik_addon import trajectory
    from blender_ik_addon import _sample_track
    try:
        pt, kfs, _src = _sample_track(bpy.context, 1.0 / fps)
    except RuntimeError as e:
        print(f"[export] {e}")
        return 2
    pk = trajectory.pack_samples(pt, kfs)      # embed exact keyframe_idx

    with open(out, "w", encoding="utf-8") as fh:
        json.dump(pk, fh, indent=2)
    print(f"[export] wrote {pk['n_samples']} samples @ {fps} fps -> {out}")
    return 0


if __name__ == "__main__":
    # Blender passes script args after `--`. Parse everything after it.
    if "--" in sys.argv:
        script_args = sys.argv[sys.argv.index("--") + 1:]
    else:
        script_args = sys.argv[1:]
    sys.exit(main(script_args))