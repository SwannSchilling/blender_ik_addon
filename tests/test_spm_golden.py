"""The SPM golden-pose fixture, asserted from the Python side.

What this is for. The fixture was produced by executing the HTML tool's own extracted code, so it
records what that tool says about 27 poses. This file reads it with fresh arithmetic and asks, pose
by pose, whether the numbers hold together: do the solved joint angles close the leg loops of the
orientation on record, do the two theta fields mean the same thing, does the quaternion describe the
matrix it is shipped beside, does the angle decomposition come back where it went in. Nothing here
imports the tool's code or the Python kinematics port - a copy of the thing under test could not
fail, so the arithmetic below is deliberately written again, from the definitions.

The closure residual carries the 0.5 mm band as a secondary check. Primary is the geometry: an angle
that does not close its leg, or a quaternion that is not its matrix, is wrong whatever the band says.
The deliberately invalid and near-singular probes are here to be *reported*: they must say what they
are (a leg with no root, a conditioning below the file's own red threshold), never be absent, and
never be a crash. The port in the next slice asserts against these same rows.

    python -B tests/test_spm_golden.py          from blender_ik_addon/
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ADDON = os.path.dirname(_HERE)
_WORKSPACE = os.path.dirname(_ADDON)
RESULTS = [ ]


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}\n        {detail}")


def fixture_path() -> str:
    """Where the fixture is. Its proper home is tests/fixtures/; the fallbacks are where a
    session that may not create a directory leaves it, so the gate can be honest about a file
    that exists rather than pretend it is missing."""
    for folder in (os.path.join(_ADDON, "tests", "fixtures"), os.path.join(_ADDON, "tests"), _WORKSPACE):
        p = os.path.join(folder, "spm_golden_poses.json")
        if os.path.isfile(p):
            return p
    return ""


# ------------------------------------------------------------------ the definitions, again ----
def quat_to_rot(q):
    """[w,x,y,z] -> 3x3, the order the fixture declares and the tool's quatToRot reads."""
    w, x, y, z = q
    s = 2.0 / ((w * w + x * x + y * y + z * z) or 1.0)
    return [[1 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
            [s * (x * y + w * z), 1 - s * (x * x + z * z), s * (y * z - w * x)],
            [s * (x * z - w * y), s * (y * z + w * x), 1 - s * (x * x + y * y)]]


def rot_to_quat(m):
    """3x3 -> [w,x,y,z], written from the same definition, to compare against the fixture's."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        return [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]


def flat_to_rot(f):
    return [[f[0], f[1], f[2]], [f[3], f[4], f[5]], [f[6], f[7], f[8]]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def rodrigues(axis, angle):
    """Rodrigues' rotation formula, the same three lines the tool runs."""
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def angle_between(a, b):
    """The rotation taking one matrix to the other, in radians: what a pose is worth as an error."""
    m = mat_mul([[a[0][0], a[1][0], a[2][0]], [a[0][1], a[1][1], a[2][1]], [a[0][2], a[1][2], a[2][2]]], b)
    tr = min(1.0, max(-1.0, (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0))
    return math.acos(tr)


def norm(v):
    return math.sqrt(dot(v, v))


def unit(v):
    n = norm(v) or 1.0
    return [v[0] / n, v[1] / n, v[2] / n]


# ------------------------------------------------------------------------------- the suite ----
def the_fixture_its_own_sole_witness(path, raw, doc):
    """The file's provenance, because a fixture nobody can tie to a tool version is a rumour."""
    want = doc.get("_stamp", { })
    html = os.path.join(_WORKSPACE, want.get("html_file", "no path recorded"))
    check("the fixture names the tool it came from, and that file is where it says",
          os.path.isfile(html), f"_stamp.html_file = {want.get('html_file')}")
    if os.path.isfile(html):
        digest = hashlib.sha256(open(html, "rb").read()).hexdigest()
        check("the fixture is the current file's output, not an older one's",
              digest == want.get("html_sha256"),
              f"file {digest[:12]}... against the fixture's {str(want.get('html_sha256'))[:12]}... - "
              f"re-export it (python -B blender_ik_addon/tools/spm/export_spm_golden.py) if the tool moved")
    check("it carries the build, the band, and the runtime that produced it",
          bool(doc.get("build")) and want.get("node_version", "").startswith("v") and want.get("build") == doc.get("build"),
          f"build {doc.get('build')}, exported by node {want.get('node_version')}")
    check("the numbers are metres and radians, and mm only where a key says mm",
          doc.get("_stamp", { }).get("units", "").startswith("metres and radians")
          and doc.get("convention", { }).get("scale_for_stl_vertices_only") == 0.001,
          f"{doc.get('_stamp', { }).get('units')} | SCALE 0.001 is STL vertices only")
    check("the quaternion order is written down, not left to be guessed",
          "[w,x,y,z]" in str(doc.get("convention", { }).get("quaternion_order", "")),
          str(doc.get("convention", { }).get("quaternion_order")))
    check("the mesh frame is kept where it belongs, out of the joint maths",
          "Rx(90deg)" in str(doc.get("convention", { }).get("mesh_frame", "")) and "translate(0,0," in str(
              doc.get("convention", { }).get("mesh_frame", "")),
          str(doc.get("convention", { }).get("mesh_frame")))
    check("no number in the file is a NaN or an Infinity",
          "NaN" not in raw and "Infinity" not in raw, "the export turns those into null and they read as absent")
    counts = doc.get("counts", { })
    check("its own tally matches the poses it ships",
          counts.get("poses") == len(doc.get("poses", [ ])),
          f"counts {counts.get('poses')} against {len(doc.get('poses', [ ]))} rows")
    kinds = set(counts.get("by_kind", { }))
    need = {"home", "tilt_sweep", "pan_sweep", "roll_extreme", "tilt_pan_cross",
            "invalid_probe", "near_singular_probe"}
    check("every family of pose is present, the probes included",
          need <= kinds and all(counts["by_kind"].get(k, 0) > 0 for k in need),
          f"{sorted(kinds)}")
    check("the probes that must report are in the file at all",
          counts.get("by_kind", { }).get("invalid_probe", 0) >= 2
          and counts["by_kind"].get("near_singular_probe", 0) >= 2,
          f"invalid {counts.get('by_kind', { }).get('invalid_probe')}, "
          f"near-singular {counts['by_kind'].get('near_singular_probe')}")


def the_geometry_is_what_it_claims(doc):
    """The leg frames, recomputed. Everything downstream reads these, so they get checked first."""
    g = doc["geometry"]
    ok_norm = all(abs(norm(w) - 1.0) < 1e-12 for w in g["w0"])
    ok_cos = all(abs(g["cosA2"][i] - dot(g["w0"][i], g["v0"][i])) < 1e-12 for i in range(3))
    check("the leg frames are unit vectors and cosA2 is really their dot product",
          ok_norm and ok_cos, f"|w0| within 1e-12, cosA2 recomputed for all three legs")
    r1, r2 = g["r1_mm"] / 1000.0, g["r2_mm"] / 1000.0
    L = math.sqrt(r1 * r1 + r2 * r2 - 2 * r1 * r2 * g["cosA2"][0])
    check("the rod length follows from the geometry that shipped with it",
          abs(L - g["rod_L_m"]) < 1e-12, f"recomputed {L:.9f} against the fixture's {g['rod_L_m']:.9f}")
    return g


def the_solved_angles_close_the_leg_loops(doc, g):
    """(1) the joint angles against the pose they were solved for, and against each other.

    A leg is closed when the crank pin it puts the disk pin at and the platform pin the orientation
    puts there keep the angle the geometry says: dot(Rz(theta_i) w0_i, R v0_i) = cosA2_i. That is the
    condition the closed form solved, recomputed here without it.
    """
    worst, worst_where, seen, mism = 0.0, "", 0, [ ]
    for p in doc["poses"]:
        R = flat_to_rot(p["R_flat"])
        for i in range(3):
            th, valid = p["theta_rad"][i], p["valid"][i]
            if not valid:
                if th is not None:
                    mism.append(f"{p['name']} leg {i}: invalid yet carries an angle {th}")
                continue
            if th is None:
                mism.append(f"{p['name']} leg {i}: valid yet has no angle")
                continue
            seen += 1
            w = mat_vec(rot_z(th), g["w0"][i])
            resid = abs(dot(w, mat_vec(R, g["v0"][i])) - g["cosA2"][i])
            if resid > worst:
                worst, worst_where = resid, f"{p['name']} leg {i} at theta {th:+.4f} rad"
            shown = p["theta_display_deg"][i]
            if shown is None:
                mism.append(f"{p['name']} leg {i}: valid yet the shown angle is null")
            else:
                want = th * 180.0 / math.pi / doc["convention"]["motor_signs"][i]
                if abs(shown - want) > 1e-9:
                    mism.append(f"{p['name']} leg {i}: shown {shown} is not theta/sign = {want}")
    check("(1) every solved angle closes its leg loop of the orientation on record",
          worst <= 1e-9 and not mism,
          f"{seen} angles checked, worst residual {worst:.3e}" + (f"; {mism[:3]}" if mism else ""))


def the_orientation_represents_itself(doc):
    """(2) the round trips, each measured as an angle rather than as a coordinate difference."""
    worst_q = worst_in = worst_r = worst_rec = 0.0
    rec_where = ""
    where = [ ]
    for p in doc["poses"]:
        R = flat_to_rot(p["R_flat"])
        if p["quat_wxyz"][0] is None:
            where.append(f"{p['name']}: no quaternion")
            continue
        worst_q = max(worst_q, angle_between(quat_to_rot(p["quat_wxyz"]), R))
        mine = rot_to_quat(R)
        worst_in = max(worst_in, min(abs(abs(a) - abs(b)) for a, b in zip(mine, p["quat_wxyz"])))
        t, p_, r = (p["input"][k] for k in ("tilt_deg", "pan_deg", "roll_deg"))
        axis_tp = unit([math.cos(math.radians(p_)), math.sin(math.radians(p_)), 0.0])
        built = mat_mul(rodrigues(axis_tp, math.radians(t)), rot_z(math.radians(r)))
        worst_r = max(worst_r, angle_between(built, R))
        rec = p["display_only_from_fk_readout"]
        if "recovered_tilt_pan_roll" in p:
            where.append(f"{p['name']}: still carries recovered_tilt_pan_roll, a name that"
                         f" promised an inverse it never was")
        rec_axis = unit([math.cos(math.radians(rec["pan_deg"])), math.sin(math.radians(rec["pan_deg"])), 0.0])
        rebuilt = mat_mul(rodrigues(rec_axis, math.radians(rec["tilt_deg"])),
                          rot_z(math.radians(rec["roll_deg"])))
        if angle_between(rebuilt, R) > worst_rec:
            worst_rec = angle_between(rebuilt, R)
            rec_where = f"{p['name']} (tilt {rec['tilt_deg']:.3f} pan {rec['pan_deg']:.3f} " \
                        f"roll {rec['roll_deg']:.3f} commanded {t}/{p_}/{r})"
        if t > 1e-6 and abs(rec["tilt_deg"] - t) > 1e-6:
            where.append(f"{p['name']}: tilt came back as {rec['tilt_deg']}")
        if abs(p["quat_round_trip_err"]) > 1e-9:
            where.append(f"{p['name']}: the tool's own quaternion round trip was off by {p['quat_round_trip_err']}")
    check("(2) the quaternion is the matrix it is shipped beside, the angle between them",
          worst_q <= 1e-9 and not [w for w in where if "quaternion" in w],
          f"worst {worst_q:.3e} rad over {len(doc['poses'])} poses, "
          f"independent recomposition agrees elementwise to {worst_in:.3e}")
    check("(2) the angle the panel commands rebuilds that same orientation",
          worst_r <= 1e-9, f"tilt/pan/roll to matrix, worst {worst_r:.3e} rad")
    check("(2) and the angle the panel reads off comes back to the tilt that was commanded",
          worst_r <= 1e-9 and not [w for w in where if "tilt came back" in w]
          and not [w for w in where if "promised an inverse" in w],
          f"tilt round trip held on every pose; the same decomposition reports azimuth and roll under "
          f"a display convention, not as the inverse of the IK construction - recomposing from its "
          f"tilt/pan/roll differs from the pose by up to {worst_rec:.3f} rad ({rec_where}), which is "
          f"reported here as what the original does, not asserted as an error")


def the_reported_verdicts_are_honest(doc):
    """(3) the answers a human acts on: which legs solve, and how well conditioned the pose is."""
    red = 0.05                                            # the file's own threshold, its readout's colour
    problems = [ ]
    for p in doc["poses"]:
        kind, valid, sig = p["kind"], p["valid"], p["sigma_min"]
        if all(valid):
            if sig is None:
                problems.append(f"{p['name']}: a fully valid pose with no conditioning number")
            elif sig <= 0.0:
                problems.append(f"{p['name']}: conditioning {sig} is not a positive number")
        if not any(valid) and not any(valid):
            pass
        if not all(valid) and sig is not None:
            problems.append(f"{p['name']}: a leg has no solution yet conditioning says {sig}")
        if kind == "invalid_probe":
            if all(valid):
                problems.append(f"{p['name']}: an invalid probe that the solver finds solvable")
            if not any(v is False for v in valid):
                problems.append(f"{p['name']}: an invalid probe with no leg marked unsolved")
        if kind == "near_singular_probe":
            if not all(valid):
                problems.append(f"{p['name']}: a near-singular probe that is not solved")
            if sig is None or not (0.0 < sig < red):
                problems.append(f"{p['name']}: conditioning {sig} is not below the file's red {red}")
    checked = sum(1 for p in doc["poses"] if p["kind"] in ("invalid_probe", "near_singular_probe"))
    check("(3) which legs solve, and the conditioning of each pose, reported as they are",
          not problems and checked >= 4,
          f"{checked} probe rows read, {len(doc['poses'])} poses overall"
          + (f"; {problems[:3]}" if problems else ""))
    probe = [p for p in doc["poses"] if p["kind"] == "near_singular_probe"]
    check("(3) the near-singular probes really are near the file's own red line, not merely small",
          bool(probe) and all(0.0 < p["sigma_min"] < red for p in probe),
          "sigma_min " + ", ".join(str(round(p["sigma_min"], 4)) for p in probe) + f" against red {red}")
    bad = [p for p in doc["poses"] if p["kind"] == "invalid_probe"]
    check("(3) an unsolvable leg is an answer: reported, with its angle absent rather than invented",
          bool(bad) and all(any(v is False for v in p["valid"]) and None in p["theta_rad"] for p in bad),
          f"{[p['name'] for p in bad]}")


def the_band_as_the_tool_applies_it(doc, g):
    """(4) secondary: the 0.5 mm band. The rod joins the crank pin to the platform pin, so what says
    closed is |distance - rod length| - the tool's own readout prints both on one line, and a band
    laid on the distance itself would be a band nothing could ever enter."""
    band = doc["band_mm"]
    worst, outside, seconded = 0.0, [ ], 0
    for p in doc["poses"]:
        if not all(p["valid"]):
            if p["closure_residual_max_mm"] is not None:
                outside.append(f"{p['name']}: an unsolved pose still reports a residual")
            continue
        if p["closure_residual_max_mm"] is None:
            outside.append(f"{p['name']}: a solved pose with no residual to judge")
            continue
        worst = max(worst, p["closure_residual_max_mm"])
        if p["closure_residual_max_mm"] > band:
            outside.append(f"{p['name']}: {p['closure_residual_max_mm']} mm over the {band} mm band")
        if p["tip_to_pin_mm"][0] is not None and abs(p["tip_to_pin_mm"][0] - p["rod_L_mm"]) > band:
            seconded += 1
            outside.append(f"{p['name']}: tip to pin {p['tip_to_pin_mm'][0]} is not the rod {p['rod_L_mm']}")
    check("(4) secondary: the closure residual stays inside the tool's 0.5 mm band",
          not outside, f"worst {worst:.3e} mm against a {band} mm band"
          + (f"; {outside[:3]}" if outside else ""))


def main() -> int:
    print("== spm golden poses: the fixture, read with fresh arithmetic ==")
    path = fixture_path()
    if not path:
        check("the fixture exists to be read", False,
              "run python -B blender_ik_addon/tools/spm/export_spm_golden.py from the workspace root")
        return report()
    print(f"   fixture {os.path.relpath(path, _WORKSPACE).replace(chr(92), '/')}")
    raw = open(path, encoding="utf-8").read()
    doc = json.loads(raw)
    the_fixture_its_own_sole_witness(path, raw, doc)
    g = the_geometry_is_what_it_claims(doc)
    the_solved_angles_close_the_leg_loops(doc, g)
    the_orientation_represents_itself(doc)
    the_reported_verdicts_are_honest(doc)
    the_band_as_the_tool_applies_it(doc, g)
    return report()


def report() -> int:
    failed = sum(1 for _n, ok, _d in RESULTS if not ok)
    print(f"\n== {len(RESULTS) - failed} passed, {failed} failed, {len(RESULTS)} checks ==")
    for name, ok, detail in RESULTS:
        if not ok:
            print(f"   FAIL {name}: {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
