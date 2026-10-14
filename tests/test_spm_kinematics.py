# SPDX-License-Identifier: BSD-3-Clause
"""The Slice-2 port, asserted against the committed golden fixture - run with plain CPython, no Blender.

    python -B tests/test_spm_kinematics.py        # from blender_ik_addon/

Why this is the mirror image of test_spm_golden.py and not a copy of it. That file reads the fixture
with freshly rewritten arithmetic and never imports the port, because a copy of the thing under test
could not fail. THIS file does the opposite on purpose: it imports spm_kinematics and spm_driver and
holds them up against the very rows the tool signed. Every pose number is the tool's own; if the port
disagrees on any of the 27, the port is wrong, full stop.

Two directions are proven for all 27 rows, not the easy one:
  * orientation -> theta (the closed-form solve, the +acos root, both via the matrix and via the
    order-tested quaternion, so an entry-point could not quietly change the answer), and
  * theta -> orientation (the numerical FK), which must land back on the recorded orientation.
The invalid and near-singular probes are here to be REPORTED: an unreachable pose must yield a leg
with no root and a skipped FK, never a traceback; a near-singular one must sit under the file's own red
conditioning. Sign and offset parity is checked against spm_driver so those never bleed into the
kinematics, which must be able to reproduce the tool without them.
"""
from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))          # blender_ik_addon/tests
_ADDON = os.path.dirname(_HERE)                              # blender_ik_addon (where the modules are)
_WORKSPACE = os.path.dirname(_ADDON)
for _p in (_ADDON, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import spm_kinematics as K                                    # noqa: E402
import spm_driver as D                                        # noqa: E402

PASS, FAIL = [], []
WS_RED = 0.05                                                 # the tool's own red conditioning threshold

THETA_TOL = 1e-9
SIGMA_TOL = 1e-9
ANGLE_TOL = 1e-6                                              # rad, the FK round trip lands this close
RESID_TOL = 1e-9                                              # the closure residual the GN drives to
BAND_MM = 0.5                                                 # secondary: the on-screen "ok" colour band


def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}\n        {detail}")
    return ok


def fixture_path():
    for folder in (os.path.join(_ADDON, "tests", "fixtures"), os.path.join(_ADDON, "tests"), _WORKSPACE):
        p = os.path.join(folder, "spm_golden_poses.json")
        if os.path.isfile(p):
            return p
    return ""


def is_none(x):
    return x is None or (isinstance(x, float) and not math.isfinite(x))


def close(a, b, tol):
    if is_none(a) and is_none(b):
        return True
    if is_none(a) != is_none(b):
        return False
    return abs(a - b) <= tol


def main():
    import json

    path = fixture_path()
    if not path:
        check("the fixture is present", False, "no spm_golden_poses.json in any home; run the export")
        return tally()
    doc = json.load(open(path, encoding="utf-8"))
    check("the fixture is present", True, os.path.relpath(path, _WORKSPACE).replace("\\", "/"))

    geom, conv = doc["geometry"], doc["convention"]
    home_az = conv["home_azimuth_rad"]
    mech = K.mech_from(geom["r1_mm"], geom["h_mm"], geom["la_deg"], geom["b_deg"], home_az)

    kinds = set(p["kind"] for p in doc["poses"])
    want = {"home", "tilt_sweep", "pan_sweep", "roll_extreme", "tilt_pan_cross",
            "invalid_probe", "near_singular_probe"}
    check("every pose kind the fixture carries is a known one", kinds <= want,
          f"kinds={sorted(kinds)} counts={doc['counts']['by_kind']}")
    check("the invalid and near-singular probes are both present",
          any(p["kind"] == "invalid_probe" for p in doc["poses"])
          and any(p["kind"] == "near_singular_probe" for p in doc["poses"]),
          f"{doc['counts']['by_kind'].get('invalid_probe')} invalid, "
          f"{doc['counts']['by_kind'].get('near_singular_probe')} near-singular")

    # ---- the mechanism model, reproduced from the tool's own geometry fields ------------------------
    wg = max(max(abs(mine[i] - theirs[i]) for i in range(3)) for mine, theirs in
             zip(mech["w0"], geom["w0"])) if geom.get("w0") else 0.0
    vg = max(max(abs(mine[i] - theirs[i]) for i in range(3)) for mine, theirs in
             zip(mech["v0"], geom["v0"])) if geom.get("v0") else 0.0
    cg = max(abs(mech["cosA2"][i] - geom["cosA2"][i]) for i in range(3))
    lg = abs(mech["L"] - geom["rod_L_m"]) if geom.get("rod_L_m") else 0.0
    check("mech_from reproduces the tool's leg frames and rod length",
          max(wg, vg, cg, lg) < 1e-12,
          f"max |w0| {wg:.2e}, |v0| {vg:.2e}, |cosA2| {cg:.2e}, |L| {lg:.2e}")

    # ---- the quaternion helper is genuinely order-sensitive ---------------------------------------
    right, wrong = K.quat_order_probe()
    check("the quaternion helper reads slot 0 as w and would mis-read an [x,y,z,w] handoff",
          right is True and wrong is False,
          f"wxyz round-trips={right}, xyzw accepted={wrong} (want True, False)")

    # ---- per-row, both directions -------------------------------------------------------------------
    wth = wvt = wsg = wq = wfk = welem = wres = wclo = 0.0
    atth = atvt = atsg = atq = atfk = atelem = atres = atclo = ""
    fk_skipped = 0
    bad = []

    for p in doc["poses"]:
        name = p["name"]
        try:
            R_target = K.nested_r(p["R_flat"])
            sol_m = K.solve_from(R_target, mech)
            sol_q = K.solve_from(p["quat_wxyz"], mech, quat_order="wxyz") if p.get("quat_wxyz") else sol_m

            for i in range(3):
                # theta: the closed-form solve against the tool's recorded angle
                d = 0.0 if (is_none(p["theta_rad"][i]) and not math.isfinite(sol_m["theta"][i])) else (
                    1e9 if is_none(p["theta_rad"][i]) != (not math.isfinite(sol_m["theta"][i]))
                    else abs(sol_m["theta"][i] - p["theta_rad"][i]))
                if d > wth:
                    wth, atth = d, f"{name} leg {i}"
                if sol_m["valid"][i] != p["valid"][i]:
                    bad.append(f"{name} leg {i} valid {sol_m['valid'][i]} != {p['valid'][i]}")
                    if not atvt:
                        atvt = name
                    wvt = 1.0
                # sigma_min against the tool's recorded conditioning
                ds = 0.0 if close(sol_m["sigma_min"], p["sigma_min"], SIGMA_TOL) else (
                    1e9 if is_none(sol_m["sigma_min"]) != is_none(p["sigma_min"])
                    else abs(sol_m["sigma_min"] - p["sigma_min"]))
                if ds > wsg:
                    wsg, atsg = ds, name
                # the quaternion entry point must solve the same angles as the matrix one
                dq = 0.0 if (is_none(sol_q["theta"][i]) and not math.isfinite(sol_m["theta"][i])) else (
                    abs(sol_q["theta"][i] - sol_m["theta"][i]))
                if dq > wq:
                    wq, atq = dq, f"{name} leg {i}"

            # the numerical FK: theta -> orientation must land back on the recorded orientation
            if all(math.isfinite(t) for t in sol_m["theta"]):
                fk = K.fk_to_orientation(sol_m["theta"], mech)
                ang = K.angle_between(fk["R"], R_target)
                if ang > wfk:
                    wfk, atfk = ang, name
                # the elementwise figure is the non-collapsing witness: the geodesic angle floors to 0.0
                # near a match, so a bare 0.0 cannot be told from a snap on its own; this can.
                elem = max(abs(fk["R"][r][c] - R_target[r][c]) for r in range(3) for c in range(3))
                if elem > welem:
                    welem, atelem = elem, name
                if fk["residual_max"] > wres:
                    wres, atres = fk["residual_max"], name
            else:
                fk_skipped += 1

            # secondary: the closure residual the on-screen band actually colours
            tp = K.tips_and_pins(mech, sol_m["theta"], K.flat_r(R_target))
            pin = K.pin_errors_mm(tp["crankTips"], tp["platPts"])
            clo = K.closure_residual_mm(pin, mech["L"])
            if p.get("closure_residual_mm"):
                dc = max(abs(clo[i] - p["closure_residual_mm"][i])
                         for i in range(3) if p["closure_residual_mm"][i] is not None) if any(
                    x is not None for x in p["closure_residual_mm"]) else 0.0
                if dc > wclo:
                    wclo, atclo = dc, name
        except Exception as exc:                                          # a probe must never crash the gate
            bad.append(f"{name}: raised {type(exc).__name__}: {exc}")

    check("orientation -> theta matches the tool's angles on all 27 rows", wth < THETA_TOL,
          f"worst |theta| {wth:.2e} rad at {atth or 'none'}")
    check("valid[] flags match the tool on all 27 rows", wvt < 0.5 and not any(v for v in bad if "valid" in v),
          f"{len([b for b in bad if 'valid' in b])} mismatch(es){(' e.g. ' + atvt) if atvt else ''}")
    check("sigma_min matches the tool's conditioning on all 27 rows", wsg < SIGMA_TOL,
          f"worst |sigma_min| {wsg:.2e} at {atsg or 'none'}")
    check("the quaternion entry point solves the same angles as the matrix entry point", wq < THETA_TOL,
          f"worst |quat-theta| {wq:.2e} at {atq or 'none'}")
    check("theta -> orientation (numerical FK) returns the recorded orientation",
          wfk < ANGLE_TOL and welem < 1e-9,
          f"worst geodesic angle {wfk:.2e} rad at {atfk or 'none'}, and the elementwise max "
          f"{welem:.2e} at {atelem or 'none'} (the angle floors to 0 near a match; this elementwise "
          f"figure is what shows the match is real, not a snap); FK skipped on {fk_skipped} legless probe(s)")
    check("the numerical FK drives the closure residuals to zero", wres < RESID_TOL,
          f"worst residual {wres:.2e} at {atres or 'none'}")
    check("the closure residual matches the fixture (secondary 0.5 mm band)", wclo < 1e-6,
          f"worst |residual| {wclo:.2e} mm at {atclo or 'none'}")

    # ---- the probes must report what they are --------------------------------------------------------
    inv = [p for p in doc["poses"] if p["kind"] == "invalid_probe"]
    inv_ok = all(
        (False in p["valid"])
        and (False in K.solve_from(K.nested_r(p["R_flat"]), mech)["valid"])
        and any(is_none(p["theta_rad"][i]) and not math.isfinite(K.solve_from(K.nested_r(p["R_flat"]), mech)["theta"][i])
                for i in range(3))
        for p in inv)
    check("an invalid probe reports a leg with no root and no angle, never a value", inv_ok,
          f"{len(inv)} invalid probe(s): {[p['name'] for p in inv]}")

    sig = [p for p in doc["poses"] if p["kind"] == "near_singular_probe"]
    sig_ok = all((not is_none(p["sigma_min"])) and p["sigma_min"] < WS_RED
                 and (not math.isfinite(K.solve_from(K.nested_r(p["R_flat"]), mech)["sigma_min"]) is False)
                 for p in sig)
    sig_red = all(K.solve_from(K.nested_r(p["R_flat"]), mech)["sigma_min"] < WS_RED for p in sig)
    check("a near-singular probe sits under the tool's own red conditioning", sig_ok and sig_red,
          f"{len(sig)} probe(s), worst recomputed sigma_min "
          f"{max((K.solve_from(K.nested_r(p['R_flat']), mech)['sigma_min']) for p in sig) if sig else float('nan'):.4f} < {WS_RED}")

    home = next((p for p in doc["poses"] if p["kind"] == "home"), None)
    check("the home pose reproduces the tool's conditioning (0.5774)",
          home is not None and not is_none(K.solve_from(K.nested_r(home["R_flat"]), mech)["sigma_min"])
          and abs(K.solve_from(K.nested_r(home["R_flat"]), mech)["sigma_min"] - home["sigma_min"]) < SIGMA_TOL,
          f"recomputed {K.solve_from(K.nested_r(home['R_flat']), mech)['sigma_min'] if home else 'n/a'}")

    # ---- the driver boundary: signs and offsets live there, not in the kinematics --------------------
    cmd_ok = cmd_disp = cmd_rt = True
    for p in doc["poses"]:
        if not (p["kind"] == "home" or p["kind"] in ("tilt_sweep", "pan_sweep", "roll_extreme")):
            continue
        if any(is_none(t) for t in p["theta_rad"]):
            continue
        inp = p["input"]
        ds = D.solve_command(inp["tilt_deg"], inp["pan_deg"], inp["roll_deg"], mech)
        for i in range(3):
            if is_none(p["theta_rad"][i]):
                continue
            if abs(ds["theta"][i] - p["theta_rad"][i]) > THETA_TOL:
                cmd_ok = False
            disp = D.theta_to_display_deg(p["theta_rad"])
            tf = p["theta_display_deg"][i]
            if is_none(disp[i]) != is_none(tf) or (not is_none(disp[i]) and abs(disp[i] - tf) > 1e-9):
                cmd_disp = False
            back = D.theta_from_display_deg(p["theta_display_deg"])[i]
            if abs(back - p["theta_rad"][i]) > THETA_TOL:
                cmd_rt = False
    check("commanding a pose equals solving it directly (the driver adds no error)", cmd_ok,
          "solve_command(tilt,pan,roll) reproduces theta for every reachable commanded row")
    check("theta_to_display divides by the motor sign exactly as theta_display_deg does", cmd_disp,
          f"MOTOR_SIGNS={D.MOTOR_SIGNS}; a leg shown as its own angle would trip this")
    check("theta_from_display inverts theta_to_display", cmd_rt,
          "display -> theta round-trips within the angle tolerance")

    if bad:
        check("no unexpected exceptions or valid[] mismatches during the sweep", False, "; ".join(bad[:6]))
    else:
        check("no unexpected exceptions or valid[] mismatches during the sweep", True, "all rows clean")

    return tally()


def tally():
    print(f"\n== {len(PASS)} passed, {len(FAIL)} failed, {len(PASS) + len(FAIL)} checks ==")
    if FAIL:
        print("failed: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
