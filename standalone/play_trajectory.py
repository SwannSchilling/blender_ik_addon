#!/usr/bin/env python
"""Standalone CubeMars trajectory player (no Blender runtime).

Reads a trajectory JSON exported from the pickik addon and streams the smooth
S-curve to the physical CubeMars actuators over CAN, using the same driver the
addon uses. Blender is completely out of the loop: this is a plain python-can
process, so playback is deterministic and does not depend on Blender's
frame/timeline timing.

By default the player also runs a PER-KEYFRAME FEEDBACK CHECK: it watches the
streaming worker's progress and, the moment the motion reaches each authored
keyframe, it pauses reading long enough to capture one feedback frame per motor
and compares the motors' REPORTED position against the position the keyframe
asked for. Each keyframe prints a per-joint want/actual/err table with a
PASS/FAIL mark, so you can actually confirm the arm reached the poses you
keyed - not just that a file was streamed.

═════════════════════════════════════════════════════════════════════════════
⚠️  HARDWARE CONFIGURATION — THIS ROBOT (Windows / URDF BIO IK arm)
═════════════════════════════════════════════════════════════════════════════

This script runs on **Windows** using the **gs_usb** interface (GS-USB2 adapter
with WinUSB/Zadig driver). The adapter supports exactly ONE open handle at a
time - the driver keeps it alive between sessions. CLOSE the addon's CubeMars
section (or quit Blender) before running this, or the second handle is refused.

Motor IDs for THIS robot (configured in CubeMars Upper Computer app):
  - Motor J1: CAN ID 104 (0x68) — matches AK80-9 from drive_two_motors.py
  - Motor J2: CAN ID 105 (0x69) — matches AK10-9 from drive_two_motors.py
  - Motors J3–J7: CAN IDs 3,4,5,6,7 (placeholder/inactive if only 2 motors)

See drive_two_motors.py for the two-motor bring-up this arm was validated with
(the shared-bus AK80-9 + AK10-9 demo; same IDs, same Mode-6 wire protocol).

To use only 2 motors, set the unused joints to 0 (0 = that joint is inert, it
is never sent a frame and never checked):
    --motor-ids 104,105,0,0,0,0,0   ← J3–J7 inactive

NOTE: the raw default of --motor-ids is 1,2,3,4,5,6,7 (the generic bench
setup). For THIS arm you must pass --motor-ids 104,105,0,0,0,0,0 or no motor on
the bus will answer and nothing will move. The feedback check will tell you so
("no feedback from any motor").

--directions defaults to (-1,1,1,1,1,1,1) matching the addon's
CUBEMARS_MOTOR_DIRECTIONS table. Invert per-joint if the physical mounting
reverses the expected rotation direction.

═════════════════════════════════════════════════════════════════════════════
🐧 Linux / WSL2 note
═════════════════════════════════════════════════════════════════════════════

On Linux or WSL2 the interface is "socketcan" with a named channel like "can0"
(the convention used by can_ak_seriesmvp.py and drive_two_motors.py, which show
the SocketCAN bring-up for these AK motors):

    python3 play_trajectory.py path/to/trajectory.json \\
        --interface socketcan --channel can0 --motor-ids 104,105,0,0,0,0,0

Setup before running on Linux:
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up

═════════════════════════════════════════════════════════════════════════════
🩺  PER-KEYFRAME FEEDBACK — options
═════════════════════════════════════════════════════════════════════════════

  --feedback / --no-feedback   turn the readback check on (default) or off
  --tol-deg  DEG               per-joint PASS window for |actual - want| (3.0)
  --settle   SECONDS           how long to capture feedback at each keyframe
                               so a report actually arrives (0.12)
  --probe-ids                  listen first and print which motor IDs answer
                               before any motion is sent
  --dry-run                    do NOT touch the bus; just print the keyframes
                               and the poses the file asks for (safe, offline)

A keyframe is taken from the exact ``keyframe_idx`` the exporter embeds; older
files without it are handled by finding the holds in the speed profile.

═════════════════════════════════════════════════════════════════════════════
🧯  IT DOES NOT MOVE — CHECK THESE
═════════════════════════════════════════════════════════════════════════════

If nothing moves but the tool runs without error, the feedback line is telling
you one of these (run with --probe-ids to see it up front):

  * Wrong / missing --motor-ids. THIS arm is 104,105 (0x68,0x69). The default
    1..7 matches nothing on the wire → "no feedback from any motor".
  * The GS-USB adapter is single-handle: Blender (the addon's live CubeMars
    stream) is still holding it. Stop the addon's stream / close Blender first.
  * Bus not up or bitrate wrong: must be 1 Mbps (1000000); on Linux bring the
    can0 link up first.
  * CAN_H / CAN_L swapped, or no 120Ω termination, or the motors are unpowered.
  * Directions: if a joint runs to the wrong side, flip its sign in
    --directions.

═════════════════════════════════════════════════════════════════════════════
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

# The addon dir (parent of standalone/) holds cubemars_driver.py / trajectory.py.
_ADDON_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ADDON_DIR not in sys.path:
    sys.path.insert(0, _ADDON_DIR)

# Windows consoles default to a legacy codepage (cp1252-like) that cannot encode
# the emoji / box-drawing glyphs used in this guide, which argparse prints as the
# --help description. Force UTF-8 with replacement so the guide and the feedback
# tables print instead of raising UnicodeEncodeError on such a console.
for _io in (sys.stdout, sys.stderr):
    try:
        _io.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

# Defaults chosen for the feedback readback check.
DEF_TOL_DEG = 3.0        # per-joint PASS window for |actual - want|
DEF_SETTLE_S = 0.12      # how long to capture feedback at a keyframe
DEF_POLL_S = 0.004       # how tightly to watch the streaming progress


# ---------------------------------------------------------------------------
# Pure, offline helpers (bus-free: importable and unit-checkable)
# ---------------------------------------------------------------------------

def resolve_keyframes(samples: dict, active_idx: list[int] | None) -> list[int]:
    """Sample indices to verify at, from the DTO's embedded ``keyframe_idx``
    or, for legacy files, the speed-profile holds. Always in range."""
    n = int(samples.get("n_samples") or 0)
    if n <= 0:
        return []
    try:
        from trajectory import detect_keyframes
        kfs = detect_keyframes(samples, active_idx=active_idx)
    except Exception:
        kfs = samples.get("keyframe_idx")
        if not kfs:
            kfs = [0, n - 1]
    kfs = [int(i) for i in kfs if isinstance(i, (int, float))]
    kfs = sorted({i for i in kfs if 0 <= i < n})
    if 0 not in kfs:
        kfs.insert(0, 0)
    if (n - 1) not in kfs:
        kfs.append(n - 1)
    return kfs


def expected_pose(samples: dict, i: int, directions: list[int]) -> list[float]:
    """Motor-space target (degrees) the keyframe sample ``i`` asks each joint
    to reach - the rig-space angle times the per-joint direction sign, exactly
    as the driver packs it, so the readback compares apples to apples."""
    row = samples["q_pos_deg"][i]
    return [row[j] * directions[j] for j in range(len(row))]


def compare_pose(active_idx: list[int], want: list[float],
                 actual: dict[int, float], tol_deg: float) -> list[dict]:
    """Per active joint: want vs actual (motor degrees) and a PASS/FAIL mark."""
    rows = []
    for j in active_idx:
        a = actual.get(j)
        if a is None:
            rows.append({"j": j, "want": want[j], "got": None,
                         "err": None, "ok": False, "seen": False})
        else:
            err = a - want[j]
            rows.append({"j": j, "want": want[j], "got": a,
                         "err": err, "ok": abs(err) <= tol_deg, "seen": True})
    return rows


def format_keyframe_report(no: int, total: int, i: int, t_s: float,
                           n: int, rows: list[dict], dry: bool = False) -> str:
    """One human-readable feedback block for a single keyframe."""
    all_ok = bool(rows) and all(r["ok"] for r in rows)
    seen_any = any(r["seen"] for r in rows)
    tag = "[plan]" if dry else ("[PASS]" if all_ok else "[FAIL]")
    head = (f"keyframe {no}/{total}  t={t_s:5.2f}s  sample {i + 1}/{n}"
            f"   {tag}")
    lines = [head]
    for r in rows:
        if dry:
            lines.append(f"    J{r['j'] + 1}: want {r['want']:+7.1f} deg")
        elif not r["seen"]:
            lines.append(f"    J{r['j'] + 1}: want {r['want']:+7.1f}   "
                        f"actual ----   (no feedback: ID wrong? unpowered? "
                        f"bus/termination?)")
        else:
            mark = "OK" if r["ok"] else "!!"
            lines.append(f"    J{r['j'] + 1}: want {r['want']:+7.1f}   "
                        f"actual {r['got']:+7.1f}   err {r['err']:+6.2f}  [{mark}]")
    if not seen_any and not dry:
        lines.append("    (no motor answered on this keyframe)")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Live feedback: read the motors' position, on the same (thread-safe) bus the
# worker is only sending on, so there is never a gap in the command stream.
# ---------------------------------------------------------------------------

def read_positions(drv, settle_s: float) -> dict[int, float]:
    """{joint_index: reported_deg} captured over a short window. Returns an
    empty dict if the bus is not open or nothing answered."""
    try:
        return drv._sample_positions(settle_s) or {}
    except Exception:
        return {}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_int_list(text: str, name: str) -> list[int]:
    vals = [int(x) for x in text.split(",") if x.strip() != ""]
    if len(vals) != 7:
        raise ValueError(f"--{name} must have exactly 7 entries "
                         f"(got {len(vals)})")
    return vals


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("trajectory", nargs="?", default="",
                    help="path to the exported trajectory JSON")
    ap.add_argument("--interface", default="gs_usb",
                    help="python-can interface (default gs_usb)")
    ap.add_argument("--channel", default=0,
                    help="CAN channel: index for gs_usb (0), or 'can0' name "
                         "for socketcan (default 0)")
    ap.add_argument("--bitrate", type=int, default=1_000_000,
                    help="CAN bitrate (default 1000000)")
    ap.add_argument("--motor-ids", default="",
                    help="comma-separated J1..J7 CAN drive IDs, 0 = inactive "
                         "(default: 1,2,3,4,5,6,7; THIS arm: 104,105,0,0,0,0,0)")
    ap.add_argument("--directions", default="-1,1,1,1,1,1,1",
                    help="per-joint direction signs, matches the addon")
    ap.add_argument("--hz", type=float, default=100.0,
                    help="trajectory send cadence in Hz (default 100)")
    ap.add_argument("--accel", type=float, default=2000.0,
                    help="accel_erpm_s2 in each Mode-6 packet (default 2000)")
    ap.add_argument("--feedback", dest="feedback", action="store_true",
                    default=True, help="per-keyframe readback check (default on)")
    ap.add_argument("--no-feedback", dest="feedback", action="store_false",
                    help="disable the per-keyframe readback check")
    ap.add_argument("--tol-deg", type=float, default=DEF_TOL_DEG,
                    help=f"PASS window for |actual-want| (default {DEF_TOL_DEG})")
    ap.add_argument("--settle", type=float, default=DEF_SETTLE_S,
                    help=f"seconds to capture feedback per keyframe "
                         f"(default {DEF_SETTLE_S})")
    ap.add_argument("--probe-ids", action="store_true",
                    help="listen first and print which motor IDs answer")
    ap.add_argument("--dry-run", action="store_true",
                    help="do not open the bus; print the keyframes the file asks")
    return ap


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)

    if not args.trajectory:
        ap.error("a trajectory JSON path is required (or use --dry-run -h)")
    if not os.path.isfile(args.trajectory):
        print(f"[play] trajectory file not found: {args.trajectory}")
        return 2
    with open(args.trajectory, "r", encoding="utf-8") as fh:
        samples = json.load(fh)
    if not samples.get("q_pos_deg"):
        print("[play] file has no q_pos_deg samples - is it a trajectory JSON?")
        return 2

    try:
        ids = _parse_int_list(args.motor_ids, "motor-ids") if args.motor_ids \
            else [1, 2, 3, 4, 5, 6, 7]
        dirs = _parse_int_list(args.directions, "directions")
    except ValueError as e:
        print(f"[play] {e}")
        return 2
    dirs = [1 if d >= 0 else -1 for d in dirs]

    n = int(samples.get("n_samples", 0))
    dt = float(samples.get("dt_s") or (1.0 / max(args.hz, 1.0)))
    active_idx = [i for i, m in enumerate(ids) if m != 0]
    kfs = resolve_keyframes(samples, active_idx)
    t_s = samples.get("t_s") or [i * dt for i in range(n)]

    looks_default_ids = (ids == [1, 2, 3, 4, 5, 6, 7])

    # ---- DRY RUN: offline, no bus, just report the plan -------------------
    if args.dry_run:
        _print_header(n, dt, args, ids, dirs, kfs, active_idx, dry=True)
        total = len(kfs)
        for no, i in enumerate(kfs, 1):
            want = expected_pose(samples, i, dirs)
            rows = [{"j": j, "want": want[j], "got": None, "err": None,
                    "ok": True, "seen": False} for j in (active_idx or range(7))]
            print(format_keyframe_report(no, total, i, t_s[i], n, rows, dry=True))
        print(f"[dry-run] {total} keyframe(s) would be verified; bus untouched.")
        return 0

    if not active_idx:
        print("[play] all motor IDs are 0 (inactive) - nothing to drive. "
              "Pass --motor-ids, e.g. 104,105,0,0,0,0,0")
        return 2

    # ---- real run: import the driver --------------------------------------
    sys.path.insert(0, os.path.abspath(_ADDON_DIR))
    try:
        from cubemars_driver import CubeMarsDriver, error_name
    except ImportError as e:
        print(f"[play] could not import cubemars_driver: {e}\n"
              f"Run with the Python that has python-can installed and the "
              f"repo's blender_ik_addon dir importable.")
        return 2

    channel = args.channel
    try:
        channel = int(channel)
    except (TypeError, ValueError):
        pass  # socketcan uses a str name like 'can0'

    drv = CubeMarsDriver(interface=args.interface, channel=channel,
                         bitrate=args.bitrate, motor_ids=ids, directions=dirs)

    _print_header(n, dt, args, ids, dirs, kfs, active_idx, dry=False)
    if looks_default_ids and args.interface == "gs_usb":
        print("[warn] --motor-ids is the default 1..7. THIS arm is "
              "104,105 (0x68,0x69): add --motor-ids 104,105,0,0,0,0,0 "
              "or no motor will answer.")

    # open the bus once so the reader and the worker share it
    try:
        drv.ensure_bus()
    except Exception as e:
        print(f"[play] could not open the CAN bus: {e}\n"
              f"        single-handle adapter? Is Blender still holding it?")
        return 1

    def _probe(label: str, seconds: float = 0.4) -> dict[int, float]:
        act = read_positions(drv, seconds)
        if args.probe_ids or label == "final":
            if not act:
                print(f"[probe] {label}: no feedback from any motor in "
                      f"{seconds:.2f}s. Are --motor-ids correct "
                      f"(this arm: 104,105)? Powered? CAN_H/L? Termination?")
            else:
                got = ", ".join(f"J{j + 1}@0x{ids[j]:02X}={act[j]:+.1f}"
                               for j in sorted(act))
                print(f"[probe] {label}: {len(act)}/{len(active_idx)} answered: {got}")
        return act

    if args.probe_ids:
        print("[probe] listening before any motion ...")
        _probe("idle")

    # ---- start the stream, then watch progress and check each keyframe ----
    try:
        drv.stream_trajectory(samples, send_hz=args.hz,
                              accel_erpm_s2=args.accel)
        print(f"[play] streaming {n} samples "
              f"~{n * dt:.2f}s ... press Ctrl+C to stop")

        results: list[dict] = []
        done: set[int] = set()
        last = kfs[-1] if kfs else n - 1
        total = len(kfs)

        while drv.is_active:
            i = drv.traj_index()
            if args.feedback and i >= 0:
                for k in kfs:
                    if k in done or k == last or i < k:
                        continue
                    _check_one(drv, samples, k, t_s, n, dirs, ids, active_idx,
                              args.tol_deg, args.settle, results,
                              kfs.index(k) + 1, total)
                    done.add(k)
            time.sleep(DEF_POLL_S)

        # the stream ended: give the arm a beat to settle, then read the last
        if args.feedback and last not in done:
            time.sleep(max(0.25, args.settle))
            _check_one(drv, samples, last, t_s, n, dirs, ids, active_idx,
                      args.tol_deg, max(args.settle, 0.35), results,
                      kfs.index(last) + 1, total)
            done.add(last)

        # one authoritative settled read of the final pose
        if args.feedback:
            final_act = _probe("final", seconds=max(0.4, args.settle))
            print(_final_verdict(results, final_act, samples, last, dirs,
                                active_idx, args.tol_deg, error_name, ids))
        else:
            print(f"[play] done ({drv.status})")
        return _exit_code(results, args)

    except KeyboardInterrupt:
        print("\n[play] stopped by user")
        return 130
    except Exception as e:
        print(f"[play] ERROR: {e}")
        return 1
    finally:
        # ALWAYS shut down the driver and release the CAN bus handle,
        # regardless of whether playback succeeded or failed. This prevents
        # "GsUsbBus was not properly shut down" warnings on Windows.
        try:
            drv.stop()
        except Exception:
            pass
        try:
            drv.disconnect()
        except Exception:
            pass


def _check_one(drv, samples, i, t_s, n, dirs, ids, active_idx, tol_deg,
               settle, results, no, total) -> None:
    """Capture one feedback frame at keyframe sample ``i`` and print + record
    the want/actual comparison."""
    want = expected_pose(samples, i, dirs)
    actual = read_positions(drv, settle)
    rows = compare_pose(active_idx, want, actual, tol_deg)
    print(format_keyframe_report(no, total, i, t_s[i], n, rows))
    results.append({"i": i, "rows": rows,
                    "ok": bool(rows) and all(r["ok"] for r in rows)})


def _print_header(n, dt, args, ids, dirs, kfs, active_idx, dry) -> None:
    tag = "dry-run" if dry else "play"
    dur = n * dt if n else 0.0
    print(f"[{tag}] {n} samples @ {args.hz:.0f} Hz; duration ~{dur:.2f}s "
          f"(dt={dt:.4f}s)")
    chans = f"{args.interface} ch{args.channel}"
    print(f"[{tag}] motors J1..J7 ids={ids} on {chans}")
    act = ",".join(f"J{i + 1}@{ids[i]}" for i in active_idx) or "none"
    print(f"[{tag}] active joints: {act}")
    print(f"[{tag}] keyframes to verify: {len(kfs)} at samples {kfs}")


def _final_verdict(results, final_act, samples, last, dirs, active_idx,
                   tol_deg, error_name, ids) -> str:
    """The report card: tally PASS/FAIL over the keyframes plus a settled read
    of the final pose."""
    if not results:
        return "[play] no keyframe was checked"
    passed = sum(1 for r in results if r["ok"])
    total = len(results)
    lines = [f"\n[play] verdict: {passed}/{total} keyframes within "
             f"{tol_deg:.1f} deg"]
    # settled final pose per active joint
    want = expected_pose(samples, last, dirs)
    if not final_act:
        lines.append("[play] the final pose could NOT be read back - no motor "
                     "answered. If nothing moved, that is the reason: check "
                     "--motor-ids (this arm: 104,105), the single-handle "
                     "adapter (is Blender holding it?), power, and termination.")
    else:
        for j in active_idx:
            a = final_act.get(j)
            if a is None:
                lines.append(f"    J{j + 1}: final wanted {want[j]:+7.1f} "
                            f"but NO feedback")
            else:
                err = a - want[j]
                mark = "OK" if abs(err) <= tol_deg else "!! (did NOT reach pose)"
                lines.append(f"    J{j + 1}: final {a:+7.1f}  "
                             f"wanted {want[j]:+7.1f}  err {err:+6.2f}  [{mark}]")
    return "\n".join(lines)


def _exit_code(results, args) -> int:
    if not args.feedback or not results:
        return 0
    return 0 if all(r["ok"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
