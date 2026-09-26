"""Trajectory planning + timeline sampling for smooth, jitter-free camera moves.

The physical limit is the source: Blender's timeline is evaluated in discrete
frame steps, so re-sampling the F-curves *during playback* can never be truly
smooth. Instead we author in Blender (F-curves), sample the (already smooth,
keyframe-interpolated) curve into a dense time grid, and re-plan it as a
jerk-shaped (S-curve) motion replayed on the motor's own cadence with
per-sample pos/vel/accel. The motion is fully time-parameterized, so there is
no frame-step jitter left.

Nothing here touches bpy or the CAN bus, so it is unit-testable headless.
All angles are in DEGREES (matching the driver's rig-space conventions).

Two independent pieces:

  * ``resample_curve`` - turn a (t, q) polyline into uniform-dt pos samples.
  * ``plan_s_curve``   - shape a per-joint movement with smooth ease-in/out
    (zero velocity at both ends of every segment) and resample to any dt.
  * ``sample_velocity``/``pack_samples`` - derive per-sample velocity and
    bundle into the replay DTO the driver streams (pos + vel per CAN frame).
"""

from __future__ import annotations

import math
from bisect import bisect_left
from typing import Sequence


# ---------------------------------------------------------------------------
# Timed position samples
# ---------------------------------------------------------------------------

def resample_curve(times: Sequence[float], joints: Sequence[Sequence[float]],
                   dt: float) -> list[tuple[float, list[float]]]:
    """Linear-resample a (t, q[]) polyline onto a uniform ``dt`` grid.

    ``times`` must be strictly increasing; each inner ``joints[i]`` has the
    same length (per-joint position, degrees). Returns ``[(t, q_deg)]`` evenly
    spaced from times[0] to times[-1] inclusive. This is an authoring fallback;
    ``plan_s_curve`` smooths the result further so linear kinks are removed.
    """
    n = len(times)
    if n < 2:
        raise ValueError("need at least two samples")
    J = len(joints[0])
    t0, t1 = float(times[0]), float(times[-1])
    if t1 <= t0:
        raise ValueError("times must be strictly increasing")
    if dt <= 0.0:
        raise ValueError("dt must be > 0")
    pts: list[tuple[float, list[float]]] = []
    steps = int(math.ceil((t1 - t0) / dt))
    for s in range(0, steps + 1):
        t = t0 + dt * s
        if t > t1 + 1e-12:
            break
        i = 0
        while i < n - 2 and times[i + 1] < t:
            i += 1
        ta, tb = times[i], times[i + 1]
        frac = (t - ta) / (tb - ta) if tb > ta else 0.0
        q = [joints[i][j] + frac * (joints[i + 1][j] - joints[i][j])
             for j in range(J)]
        pts.append((t, q))
    if pts and abs(pts[-1][0] - t1) > 1e-9:
        pts.append((t1, list(joints[-1])))
    return pts


# ---------------------------------------------------------------------------
# S-curve / smooth-easing planning
# ---------------------------------------------------------------------------

def _ease_s(x: float) -> float:
    """Zero-derivative-at-ends easing (smootherstep): 6t^5 - 15t^4 + 10t^3.

    Every waypoint segment starts and ends with zero velocity, matching the
    S-curve jerk profile well enough for camera work while guaranteeing C1
    continuity across the whole path. (A full 7-segment jerk profile per
    *joint* with accel/decel phases is a later refinement; the end result is
    qualitatively the same smoothness for a camera move.)"""
    x = max(0.0, min(1.0, x))
    return 6 * x ** 5 - 15 * x ** 4 + 10 * x ** 3


def plan_s_curve(points: Sequence[Sequence[float]], dt: float
                 ) -> list[tuple[float, list[float]]]:
    """Shape a jagged/linear keyframe path into a smooth S-curve motion.

    ``points`` is an ordered list of joint-angle sets (each a list of J
    floats, degrees). Every consecutive pair becomes a segment; each segment
    uses the same duration (1 "unit" of time) and is eased with a smooth
    start/finish so velocity is zero at every waypoint and continuous
    throughout. The result is resampled to ``dt`` over the full path.
    Use :func:`plan_s_curve_waypoints` when the keyframes carry real times.

    Return ``[(t, q_deg)]`` from t=0 to t=(N-1) (segment count)."""
    if len(points) < 2:
        raise ValueError("need at least two waypoints")
    J = len(points[0])
    for p in points:
        if len(p) != J:
            raise ValueError("all waypoints must have the same joint count")
    Tf = float(len(points) - 1)          # one time unit per segment
    if dt <= 0.0:
        raise ValueError("dt must be > 0")
    steps = int(math.ceil(Tf / dt))
    out: list[tuple[float, list[float]]] = []
    for s in range(0, steps + 1):
        t = dt * s
        if t > Tf + 1e-12:
            break
        frac = t - math.floor(t) if math.floor(t) < len(points) - 1 else 1.0
        i = min(int(math.floor(t)), len(points) - 2)
        e = _ease_s(frac)
        qa = points[i]
        qb = points[i + 1]
        q = [qa[j] + e * (qb[j] - qa[j]) for j in range(J)]
        out.append((t, q))
    # land exactly on the final waypoint
    if out and abs(out[-1][0] - Tf) > 1e-9:
        out.append((Tf, list(points[-1])))
    return out


def plan_s_curve_waypoints(waypoints: Sequence[tuple[float, Sequence[float]]],
                           dt: float) -> list[tuple[float, list[float]]]:
    """Plan a smooth S-curve through keyframes that carry real times.

    ``waypoints`` is an ordered list of ``(t_seconds, q_deg)`` (e.g. from a
    Blender timeline where frame/fps gives real seconds). Each segment spans
    its own real duration and is eased with a smooth start/finish, so the
    returned path covers exactly the authored timeline duration - a short
    move stays short. Resampled to ``dt``.
    """
    if len(waypoints) < 2:
        raise ValueError("need at least two keyframed waypoints")
    J = len(waypoints[0][1])
    for _t, q in waypoints:
        if len(q) != J:
            raise ValueError("all waypoints must have the same joint count")
    t0 = waypoints[0][0]
    tf = waypoints[-1][0]
    if tf <= t0:
        # Degenerate: single time. Just return it.
        return [(t0, list(waypoints[0][1]))]
    if dt <= 0.0:
        raise ValueError("dt must be > 0")
    steps = int(math.ceil((tf - t0) / dt))
    out: list[tuple[float, list[float]]] = []
    for s in range(0, steps + 1):
        t = t0 + dt * s
        if t > tf + 1e-9:
            break
        # locate segment
        i = 0
        while i < len(waypoints) - 2 and waypoints[i + 1][0] < t:
            i += 1
        ta, qa = waypoints[i]
        tb, qb = waypoints[i + 1]
        _span = tb - ta
        frac = (t - ta) / _span if _span > 1e-12 else 1.0
        e = _ease_s(frac)
        q = [qa[j] + e * (qb[j] - qa[j]) for j in range(J)]
        out.append((t, q))
    # land exactly on the final waypoint
    if out and abs(out[-1][0] - tf) > 1e-9:
        out.append((tf, list(waypoints[-1][1])))
    return out


def sample_velocity(pts: Sequence[tuple[float, list[float]]]
                    ) -> list[list[float]]:
    """Central-difference per-sample velocity (deg/s) from a dense ``[(t, q)]``
    path. Endpoints are zero (the S-curve starts and ends at rest)."""
    if not pts:
        return []
    J = len(pts[0][1])
    out: list[list[float]] = []
    n = len(pts)
    for i in range(n):
        if i == 0 or i == n - 1:
            out.append([0.0] * J)
            continue
        t0, _ = pts[i - 1]
        t2, _ = pts[i + 1]
        dt = t2 - t0
        if dt <= 0:
            out.append([0.0] * J)
        else:
            out.append([(pts[i + 1][1][j] - pts[i - 1][1][j]) / dt
                        for j in range(J)])
    return out


def _nearest_index(times: Sequence[float], t: float) -> int:
    """Index of the sample in the (sorted) ``times`` grid closest to ``t``."""
    n = len(times)
    if n == 0:
        return 0
    j = bisect_left(times, t)
    best = None
    for c in (j - 1, j):
        if 0 <= c < n:
            if best is None or abs(times[c] - t) < abs(times[best] - t):
                best = c
    if best is None:
        best = 0 if t < times[0] else n - 1
    return best


def _clean_idx(idxs: Sequence[int], n: int) -> list[int]:
    """Clamp to [0, n-1], drop duplicates keeping order, force endpoints."""
    if n <= 0:
        return []
    out: list[int] = []
    for i in idxs:
        i = int(max(0, min(n - 1, int(i))))
        if not out or i > out[-1]:
            out.append(i)
    if not out or out[0] != 0:
        out.insert(0, 0)
    if out[-1] != n - 1:
        out.append(n - 1)
    return out


def _keyframe_indices(kf_times: Sequence[float], times: Sequence[float]
                     ) -> list[int]:
    """Map each authored keyframe TIME to its nearest planned sample index."""
    return _clean_idx([_nearest_index(times, float(t)) for t in kf_times],
                      len(times))


def detect_keyframes(samples: dict, active_idx: Sequence[int] | None = None,
                     ratio: float = 0.06, min_gap_frac: float = 0.04
                     ) -> list[int]:
    """Sample indices that mark the authored keyframes ("holds") of a packed
    trajectory - purely from the DTO, no bus, no bpy.

    Prefers the exact ``keyframe_idx`` embedded by :func:`pack_samples` (from
    an Export). For older files without it, falls back to finding the local
    minima of the per-sample joint speed: the S-curve eases every waypoint to
    (near) zero speed, so a waypoint is where the active joints are all
    simultaneously slowest. Endpoints are always included."""
    n = int(samples.get("n_samples") or 0)
    if n <= 0:
        return []
    if n == 1:
        return [0]
    kfi = samples.get("keyframe_idx")
    if isinstance(kfi, (list, tuple)) and len(kfi) >= 2:
        try:
            return _clean_idx([int(i) for i in kfi], n)
        except (TypeError, ValueError):
            pass
    vel = samples.get("q_vel_deg_s")
    if not vel:
        return [0, n - 1]
    joints = list(active_idx) if active_idx else list(range(len(vel[0])))

    def spd(i: int) -> float:
        row = vel[i] if i < len(vel) else []
        vals = [abs(row[j]) for j in joints if j < len(row)]
        return max(vals) if vals else 0.0

    sp = [spd(i) for i in range(n)]
    peak = max(sp) if sp else 0.0
    if peak <= 1e-9:                      # a still trajectory: just endpoints
        return [0, n - 1]
    thr = peak * max(0.0, ratio)
    w = max(1, int(n * min_gap_frac * 0.5))
    cands = []
    for i in range(1, n - 1):
        if sp[i] > thr:
            continue
        lo, hi = max(0, i - w), min(n, i + w + 1)
        if sp[i] <= min(sp[lo:hi]) + 1e-12:
            cands.append(i)
    gap = max(1, int(n * min_gap_frac))
    kept: list[int] = []
    k = 0
    while k < len(cands):
        j, best, bestv = k, cands[k], sp[cands[k]]
        while j < len(cands) and cands[j] - cands[k] <= gap:
            if sp[cands[j]] < bestv:
                best, bestv = cands[j], sp[cands[j]]
            j += 1
        kept.append(best)
        k = j
    return _clean_idx([0] + kept + [n - 1], n)


def pack_samples(pts: Sequence[tuple[float, list[float]]],
                 keyframes: Sequence[object] | None = None
                 ) -> dict[str, object]:
    """Bundle an S-curve path into a replay-ready DTO for the driver.

    The driver replays equal-dt frames, sending each motor's per-sample
    position and velocity in its Mode-6 packet.

    ``keyframes`` (optional) is the ORIGINAL authored waypoint list ``[(t, q)]``
    (the same list handed to :func:`plan_s_curve_waypoints`) or a plain list of
    waypoint times. When given, the nearest planned sample index of every
    keyframe is recorded under ``keyframe_idx`` so the stand-alone player can
    pause-and-verify exactly at the authored keyframes instead of guessing them
    from the speed profile."""
    if not pts:
        raise ValueError("no samples")
    J = len(pts[0][1])
    times = [p[0] for p in pts]
    pos = [list(p[1]) for p in pts]
    vel = sample_velocity(pts)
    out: dict[str, object] = {
        "dt_s": float(1e-3 if len(times) < 2 else times[1] - times[0]),
        "t_s": [round(x, 6) for x in times],
        "q_pos_deg": [[float(q) for q in row] for row in pos],
        "q_vel_deg_s": [[float(v) for v in row] for row in vel],
        "n_joints": J,
        "n_samples": len(pts),
    }
    if keyframes:
        kf_times: list[float] = []
        for kf in keyframes:
            if isinstance(kf, (list, tuple)) and len(kf) >= 1:
                kf_times.append(float(kf[0]))
            else:
                kf_times.append(float(kf))
        out["keyframe_idx"] = _keyframe_indices(kf_times, times)
    return out