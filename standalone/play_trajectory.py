#!/usr/bin/env python
"""Standalone CubeMars trajectory player (no Blender runtime).

Reads a trajectory JSON exported from the pickik addon and streams the smooth
S-curve to the physical CubeMars actuators over CAN, using the same driver the
addon uses. Blender is completely out of the loop: this is a plain python-can
process, so playback is deterministic and does not depend on Blender's
frame/timeline timing.

Run with the Blender/CubeMars Python that has python-can + gs_usb installed:

    python tools/play_trajectory.py path/to/trajectory.json \
        --channel 0 --interface gs_usb --motor-ids 1,2,3,4,5,6,7

--motor-ids is the CAN drive ID for J1..J7 (use 0 for an inactive joint; you
must still list 7 entries, matching the addon's J1..J7 order). --directions
defaults to the addon's CUBEMARS_MOTOR_DIRECTIONS (-1,1,1,1,1,1,1).
"""

from __future__ import annotations

import argparse
import json
import os
import sys

# The addon dir (parent of standalone/) holds cubemars_driver.py / trajectory.py.
_ADDON_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("trajectory", help="path to the exported trajectory JSON")
    ap.add_argument("--interface", default="gs_usb",
                    help="python-can interface (default gs_usb)")
    ap.add_argument("--channel", type=int, default=0,
                    help="CAN channel index (default 0)")
    ap.add_argument("--bitrate", type=int, default=1_000_000,
                    help="CAN bitrate (default 1000000)")
    ap.add_argument("--motor-ids", default="",
                    help="comma-separated J1..J7 CAN drive IDs, 0 = inactive "
                         "(default: 1,2,3,4,5,6,7)")
    ap.add_argument("--directions", default="-1,1,1,1,1,1,1",
                    help="per-joint direction signs, matches the addon")
    ap.add_argument("--hz", type=float, default=100.0,
                    help="trajectory send cadence in Hz (default 100)")
    ap.add_argument("--accel", type=float, default=2000.0,
                    help="accel_erpm_s2 in each Mode-6 packet (default 2000)")
    args = ap.parse_args()

    sys.path.insert(0, os.path.abspath(_ADDON_DIR))
    try:
        from cubemars_driver import CubeMarsDriver
    except ImportError as e:
        print(f"[play] could not import cubemars_driver: {e}\n"
              f"Run with the Python that has python-can installed and the "
              f"repo's blender_ik_addon dir importable.")
        return 2

    if not os.path.isfile(args.trajectory):
        print(f"[play] trajectory file not found: {args.trajectory}")
        return 2
    with open(args.trajectory, "r", encoding="utf-8") as fh:
        samples = json.load(fh)

    if args.motor_ids:
        ids = [int(x) for x in args.motor_ids.split(",")]
    else:
        ids = [1, 2, 3, 4, 5, 6, 7]
    if len(ids) != 7:
        print("[play] --motor-ids must have exactly 7 entries")
        return 2
    dirs = [int(x) for x in args.directions.split(",")]
    if len(dirs) != 7:
        print("[play] --directions must have exactly 7 entries")
        return 2

    drv = CubeMarsDriver(interface=args.interface, channel=args.channel,
                         bitrate=args.bitrate, motor_ids=ids, directions=dirs)

    n = int(samples.get("n_samples", 0))
    print(f"[play] {n} samples @ {args.hz:.0f} Hz; duration ~{n / args.hz:.2f}s")
    print(f"[play] motors J1..J7 ids={ids} on {args.interface} ch{args.channel}")
    try:
        drv.stream_trajectory(samples, send_hz=args.hz,
                              accel_erpm_s2=args.accel)
        print("[play] streaming... press Ctrl+C to stop")
        import time
        while drv.is_active:
            time.sleep(0.05)
        print(f"[play] done ({drv.status})")
        return 0
    except KeyboardInterrupt:
        print("\n[play] stopped by user")
        drv.stop()
        return 130
    except Exception as e:
        print(f"[play] ERROR: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())