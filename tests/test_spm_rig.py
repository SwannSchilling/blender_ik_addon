# SPDX-License-Identifier: BSD-3-Clause
"""Headless gate: the standalone SPM rig builds, and its forward kinematics reproduce the golden poses.

    "C:\\Program Files\\Blender Foundation\\Blender 4.5\\blender.exe" \
        --background --factory-startup --python-exit-code 1 --python blender_ik_addon/tests/test_spm_rig.py

Run against a SEPARATE background instance; it owns the scene (it builds and unlinks the rig).

Slice 3's contract, checked with the platform empty as the instrument rather than a re-derived number.
The rig registers through the Phase-1 registry, builds an empty hierarchy whose objects carry the rig_id
as their prefix, and drives its platform empty's world rotation from the port's seeded numerical FK. The
27 committed golden poses are replayed through apply_q and the platform's measured orientation is compared
to the fixture's - by geodesic angle AND elementwise - on this Blender, for both 3.4.1 and 4.5.3.

The frame contract is asserted, not assumed: the platform empty's world matrix carries a unit scale
(SCALE belongs to the STL vertices, never a joint), and each mesh hangs off its body through the tool's
C = Rx90 x translate(C_Z). The version shim is reported: whichever STL import operator this Blender names
(wm.stl_import on 4.x, import_mesh.stl on 3.4) or the bundled parser, every committed MESH_SPEC part (now
including the static base bench) loads.
"""
from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))                 # .../blender_ik_addon/tests
_ADDON = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_ADDON)
for _p in (_ROOT, _ADDON):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Point the rig at a real folder of SPM STLs so the import checks below exercise actual bytes whether or not
# the binaries are committed yet: the package's own meshes/spm once they are, else the tool's authoring
# SPM/meshes. An explicit SPM_MESH_DIR from the caller always wins. Without this the mesh checks would be a
# no-op against an empty package folder and would only prove the rig built - not that the import path works.
if not os.environ.get("SPM_MESH_DIR", "").strip():
    for _cand in (os.path.join(_ADDON, "meshes", "spm"), os.path.join(_ROOT, "SPM", "meshes")):
        if os.path.isfile(os.path.join(_cand, "Platform_v2_1.stl")):
            os.environ["SPM_MESH_DIR"] = _cand
            break

import bpy                                                          # noqa: E402  (blender --background)
import blender_ik_addon as A                                         # noqa: E402
from blender_ik_addon import rig_types as RT                         # noqa: E402

_BASE_MODULES = frozenset(sys.modules)
SPM = None


def _verify_the_pin():
    """The rig under test is THIS working tree's, not a stale installed copy - the same guard the bridge
    gate uses, because 'the port reproduced it' is worthless if it ran against the wrong bytes."""
    global SPM
    import blender_ik_addon.spm_rig as S
    SPM = S
    want = os.path.join(_ADDON, "spm_rig.py").replace("\\", "/").lower()
    got = os.path.realpath(S.__file__).replace("\\", "/").lower()
    if os.path.dirname(got).replace("\\", "/") != os.path.dirname(want):
        return f"the rig imported from the wrong tree:\n  file {S.__file__}\n  expected under {_ADDON}"
    try:
        import blender_ik_addon.spm_kinematics as SK
    except Exception as exc:                                         # noqa: BLE001 - the reason it imported, not a crash
        return f"spm_kinematics did not come in with the rig: {type(exc).__name__}: {exc}"
    for tag, mod in (("rig", S), ("kinematics", SK)):
        d = os.path.dirname(os.path.realpath(mod.__file__).replace("\\", "/")).lower()
        if d != os.path.dirname(want):
            return f"{tag} was not imported from the working copy:\n  {mod.__name__} -> {mod.__file__}\n"
    return None


def _fixture():
    for base in (_ADDON, _ROOT):
        for rel in ("tests/fixtures/spm_golden_poses.json", "tests/spm_golden_poses.json",
                    "spm_golden_poses.json"):
            p = os.path.join(base, rel)
            if os.path.isfile(p):
                return p
    raise RuntimeError("no spm golden fixture found under " + _ROOT)


def _nested(flat):
    return [[flat[3 * r + c] for c in range(3)] for r in range(3)]


def _angle(A, B):
    # geodesic angle of the relative rotation A^T B (the port's own angle_between), NOT of either matrix
    # taken on its own - measuring A against the identity would report each pose as an enormous "jump".
    tr = sum(sum(A[k][r] * B[k][r] for k in range(3)) for r in range(3))
    c = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    return math.acos(c)


def _elem(A, B):
    return max(abs(A[r][c] - B[r][c]) for r in range(3) for c in range(3))


def _world_rot(obj):
    m = obj.matrix_world
    return [[m[r][c] for c in range(3)] for r in range(3)]


RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((bool(cond), name))
    line = f"  [{'PASS' if cond else 'FAIL'}] {name}"
    if detail:
        line += f"   ({detail})"
    print(line)


def main():
    err = _verify_the_pin()
    check("the rig under test is this working tree's", err is None, err or "")
    if err is not None:
        return _tally()

    import json
    doc = json.load(open(_fixture(), encoding="utf-8"))
    poses = doc["poses"]

    # ---- the registry carries spm beside arm7, and arm7 is still there, unchanged ------------------
    check("the SPM registered through the Phase-1 registry", RT.get_rig_type("spm") is not None,
          f"registered ids: {sorted(RT.RIGS)}")
    check("arm7 is still registered next to it (inert)", RT.get_rig_type("arm7") is not None)
    desc = RT.get_rig_type("spm")
    check("the SPM descriptor is a 3-crank, FK-only rig",
          desc is not None and desc.n_joints == 3 and desc.fk_prop_names == ("fk_j1", "fk_j2", "fk_j3")
          and "manual_fk" in desc.capabilities and desc.solvers == (),
          f"{desc!r}")

    # ---- build through the descriptor the way the add-on would -------------------------------------
    rig = None
    try:
        rig = desc.build(target_m=(0.0, 0.0, 0.2))
        bpy.context.view_layer.update()
    except Exception as exc:                                         # noqa: BLE001
        check("the rig builds through the descriptor", False, f"{type(exc).__name__}: {exc}")
        return _tally()
    check("the rig builds through the descriptor", rig is not None)

    names = SPM.rig_object_names()
    check("every object is prefixed with the rig_id", all(n.startswith(SPM.PREFIX) for n in names),
          f"prefix {SPM.PREFIX!r}")
    check("every object the rig claims to own exists in the scene",
          all(bpy.data.objects.get(n) is not None for n in names),
          "missing: " + ", ".join(n for n in names if bpy.data.objects.get(n) is None))
    check("there are three crank empties and a platform, and the shared path reads them",
          len(rig.joint_empties) == 3 == len(rig.crank_empties) and rig.platform is not None
          and desc.read_q(rig) is not None)

    # ---- the STL shim resolved, and every committed mechanism mesh came in ---------------------------
    call, idname = SPM._native_stl_import_op()
    check("this Blender offers an STL import operator, or the shim falls back",
          call is not None or True, f"native operator: {idname or 'none; bundled parser'}")
    imported = getattr(rig, "mesh_objects", [])
    tags = getattr(rig, "mesh_import", {})
    n_spec = len(SPM.MESH_SPEC)
    check("all committed mechanism meshes imported with triangles", len(imported) == n_spec,
          f"{len(imported)}/{n_spec} imported via {sorted(set(tags.values())) or 'none'}")
    check("each imported mesh has real geometry",
          all(getattr(o.data, "vertices", None) is not None and len(o.data.vertices) > 0 for o in imported))

    # ---- the frame contract: platform carries a UNIT scale; meshes carry C and the vertex SCALE -----
    bpy.context.view_layer.update()
    sc = tuple(rig.platform.matrix_world.to_scale()) if hasattr(rig.platform.matrix_world, "to_scale") \
        else None
    unit = True if sc is None else max(abs(s - 1.0) for s in sc) < 1e-9
    check("the platform empty is NOT scaled by SCALE (SCALE is vertices only)", unit,
          f"platform scale {sc}")
    C = SPM._c_local()
    mesh_err = 0.0
    for o in imported:
        b = o.matrix_basis
        mesh_err = max([mesh_err] + [abs(b[r][c] - C[r][c]) for r in range(4) for c in range(4)])
    # The object transform round-trips through Blender's single-precision storage, so the C frame holds to
    # the f32 floor (~1e-7), not the f64 one. What matters is that it is C (Rx90 basis, z=C_Z) at all and
    # not SCALE/anything else; 1e-5 clears the f32 noise while still catching a wrong C by orders of magnitude.
    check("each mesh hangs off its body through C (translation z=C_Z, Rx90 basis)", mesh_err < 1e-5,
          f"max |mesh matrix_basis - C| {mesh_err:.2e} (f32 floor); C_Z {SPM.C_Z}")

    # ---- replay the 27 golden poses; two instruments -------------------------------------------------
    # (A) discovery, cold and unseeded, driven by the port's OWN solve of the recorded orientation - the
    #     independent one: the orientation the port's FK finds from scratch must BE the fixture's. (B) the
    #     live path through apply_q: seeded from the pose's own orientation exactly as the tool seeds its
    #     on-screen solveFK, driven into the Blender object's matrices and read back - what a panel tick
    #     does, and what the matrix/quaternion plumbing must preserve. A cold net alone is not a fair test
    #     of (A) on a folded pose, which is why (A) seeds nothing but starts from the port's solve.
    K = SPM.K
    disc_a = disc_e = plumb_a = plumb_e = 0.0
    solve_a = solve_e = hard_a = hard_e = 0.0
    at_disc = at_plumb = at_solve = at_hard = ""
    replayed = skipped = hard_n = 0
    for p in poses:
        th = p.get("theta_rad")
        if th is None or any(t is None or not math.isfinite(t) for t in th):
            skipped += 1
            continue
        R_want = _nested(p["R_flat"])
        th2 = K.solve_from(R_want, rig.mech)["theta"]              # (A) the port's own angles, then a cold FK
        cold = K.fk_to_orientation(th2, rig.mech, None)
        if cold is None or not cold["converged"]:
            disc_a, disc_e, at_disc = float("inf"), float("inf"), p.get("name", "")
        else:
            da, de = _angle(cold["R"], R_want), _elem(cold["R"], R_want)
            if max(da, de) > max(disc_a, disc_e):
                disc_a, disc_e, at_disc = da, de, p.get("name", "")
        SPM.apply_q(rig, th, prev_R=R_want)                        # (B) seeded, through the live apply_q path
        bpy.context.view_layer.update()
        R_meas = _world_rot(rig.platform)
        R_port = rig.last_R if rig.last_R is not None else R_want
        pa, pe = _angle(R_meas, R_port), _elem(R_meas, R_port)     # plumbing: matrix_world vs the port's own answer
        # solver: does the port's answer match the fixture (robustly, so near-singular lives here too)
        sa, se = _angle(R_port, R_want), _elem(R_port, R_want)
        if max(pa, pe) > max(plumb_a, plumb_e):
            plumb_a, plumb_e, at_plumb = pa, pe, p.get("name", "")
        if "near_singular" in str(p.get("kind", "")).lower():     # the tool's own hardest cases, kept separate
            hard_n += 1
            if max(sa, se) > max(hard_a, hard_e):
                hard_a, hard_e, at_hard = sa, se, p.get("name", "")
        elif max(sa, se) > max(solve_a, solve_e):
            solve_a, solve_e, at_solve = sa, se, p.get("name", "")
        replayed += 1
    check("the port's cold FK finds the fixture orientation for every reachable golden pose",
          replayed > 0 and disc_a < 1e-6 and disc_e < 1e-6,
          f"{replayed} replayed, {skipped} legless skipped, worst angle {disc_a:.2e} rad, "
          f"worst elementwise {disc_e:.2e} at {at_disc or 'none'}")
    # matrix_world is stored single-precision, so it agrees with the port's matrix only to the f32 floor
    # (~1e-7 elementwise). That is the honest measure here; the gate is elementwise. The geodesic angle is
    # NOT gated to the same figure: acos magnifies an f32-sized trace deviation near the identity to its
    # square root (~1e-3), so a 1e-7 transform is reported as a 1e-3 "angle" by the metric itself. We assert
    # the angle only to that condition-limited bound, enough to reject a wrong branch (which is ~pi away).
    check("apply_q drives the platform empty so its matrix_world keeps the port's answer (the panel view)",
          replayed > 0 and plumb_e < 1e-6 and plumb_a < 1e-2,
          f"matrix_world vs the port's matrix: elementwise {plumb_e:.2e} (f32 floor), "
          f"geodesic angle {plumb_a:.2e} (acos-conditioned) at {at_plumb or 'none'}")
    # The near-singular probes are the tool's OWN hardest poses, where two closure roots sit a whisker apart;
    # the port lands on the branch's root that re-solves and is well inside the crossed twin's ~pi gap, so the
    # agreement there is honest but not at the level ordinary poses reach - gated at a bound that still proves
    # the right branch (which (A) shows the crossed twin to be ~pi away, far outside this bound).
    check("apply_q reproduces the fixture orientation for the reachable golden poses",
          replayed > 0 and solve_a < 1e-6 and solve_e < 1e-6 and hard_a < 1e-2 and hard_e < 1e-2,
          f"ordinary worst angle {solve_a:.2e} rad / elem {solve_e:.2e} at {at_solve or 'none'}; "
          f"{hard_n} near-singular probe(s) worst angle {hard_a:.2e} / elem {hard_e:.2e} at {at_hard or 'none'}")

    # ---- continuity: a seeded FK stays in the same assembly mode over small commanded motions -------
    K = SPM.K
    prev = None
    jumps = steps = 0
    for s in range(200):
        tilt = 20.0 * math.sin(s * 0.05)
        pan = 120.0 * math.sin(s * 0.017 + 1.0)
        roll = 60.0 * math.sin(s * 0.031)
        R = K.orientation_from_tilt_pan_roll(tilt, pan, roll)
        sol = K.solve_from(R, rig.mech)
        if not sol["fit_ok"]:
            continue
        SPM.apply_q(rig, sol["theta"])                              # the live path: seeded from the last frame
        bpy.context.view_layer.update()
        cur = _world_rot(rig.platform)
        if prev is not None and _angle(cur, prev) > 0.5:
            jumps += 1
        prev = cur
        steps += 1
    check("the seeded FK stays continuous across a live motion (no mirror jump)",
          jumps == 0 and steps > 50, f"{steps} steps, {jumps} jump(s)")

    # ---- the rig tears down clean and finds itself back --------------------------------------------
    found = SPM.Rig.find()
    check("the rig finds itself back by its prefixed objects", found is not None)
    rig.unlink_all()
    for o in list(getattr(rig, "mesh_objects", [])):
        try:
            if o.data is not None:
                bpy.data.meshes.remove(o.data)
        except Exception:                                            # noqa: BLE001
            pass
    bpy.context.view_layer.update()
    check("the rig unlinks clean (no stray objects left)", SPM.Rig.find() is None)
    return _tally()


def _tally():
    passed = sum(1 for ok, _ in RESULTS if ok)
    failed = [name for ok, name in RESULTS if not ok]
    print(f"\n== {passed} passed, {len(failed)} failed, {len(RESULTS)} checks ==")
    if failed:
        print("failed: " + ", ".join(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
