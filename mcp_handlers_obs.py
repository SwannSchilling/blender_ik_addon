# SPDX-License-Identifier: BSD-3-Clause
"""mcp_handlers_obs — command handlers for the observe / rig / IK-FK group (MVP, Phase 1).

Runs inside Blender. Reuses the add-on's own tested paths (its operators, `arm7_rig`, and the
shared `ik_core.Core`) so this layer adds transport + policy, never a second model or a second FK
(mcp_protocol_plan §1). Every handler has the contract `fn(args, bridge) -> dict`, raising
`mcp_protocol.McpError(code, msg)` to report a classified failure.

Threading rule made concrete: any use of `bpy` goes through `bridge.call_on_main`, which runs
inline when the job is already on the main thread (read/write) and posts-and-waits when the job is
an off-thread worker (memetic apply). `pure` handlers deliberately avoid `call_on_main` so a
what-if loop really runs off the tick — they cache the DLL handle once and then stay off-thread.
"""
from __future__ import annotations

import importlib
import math
import sys

import bpy

try:                                                   # loaded as part of the add-on package (the normal path)
    from . import mcp_protocol as P
except ImportError:                                    # bare top-level import (probe / headless harness)
    import mcp_protocol as P

_ADDON = None
_CORE = None
_LIMITS = None                                     # ((lower_rad,...), (upper_rad,...))


def _addon():
    """Import the pickik add-on package (it owns the rig, the operators and the Core). Resolved
    through *this* module's own package so a stale copy in Blender's addons dir can never be
    silently preferred over the working tree the harness pinned onto sys.path."""
    global _ADDON
    if _ADDON is None:
        mod = sys.modules.get("blender_ik_addon")
        if mod is None:
            mod = importlib.import_module((__package__ or "blender_ik_addon").split(".")[0])
        _ADDON = mod
    return _ADDON


def _core_of(bridge):
    """The shared Core (one loaded pick_ik_c.dll). Prefetched once on the main thread, then cached
    so a `pure` worker never has to post back to the main thread for it."""
    global _CORE
    if _CORE is None:
        A = _addon()
        _CORE = bridge.call_on_main(lambda: A._core_or_die())
    return _CORE


def _limits_of(bridge):
    """The Design-B joint limits, read from the add-on's own FloatProperty definitions — the single
    source (§7.3). We do NOT restate the numbers here; we read the same hard min/max the sliders and
    the operator use, so the model cannot silently diverge from the rig."""
    global _LIMITS
    if _LIMITS is None:
        A = _addon()
        def _read():
            # `from __future__ import annotations` stringifies __annotations__ (PEP 563), so the
            # FloatProperty objects are NOT reachable from the class — they live only in the registered
            # RNA. Ask the RNA itself: assign far outside the range on the live readout instance and
            # read back what the hard min/max clamp it to. q_j* have NO update= callback (unlike fk_j*),
            # so this neither moves the arm nor fires any handler, and it is restored below. Same table
            # the panel clamps against → no second source of truth (§7.3).
            p = bpy.context.scene.pickik
            saved = [getattr(p, f"q_j{i}") for i in range(1, 8)]
            low, up = [], []
            try:
                for i in range(1, 8):
                    setattr(p, f"q_j{i}", 1e9); up.append(float(getattr(p, f"q_j{i}")))
                    setattr(p, f"q_j{i}", -1e9); low.append(float(getattr(p, f"q_j{i}")))
            finally:
                for i in range(1, 8):                        # restore exactly; leave the scene untouched
                    setattr(p, f"q_j{i}", saved[i - 1])
            for i in range(7):
                if not (low[i] < 0.0 < up[i]) or abs(up[i]) > 7.0:   # a radian bound must be sane
                    raise P.McpError(P.ERR.STATE,
                                    f"Design-B limit q_j{i + 1} read back non-sane; refusing to assume")
            return (low, up)
        _LIMITS = bridge.call_on_main(_read)
    return _LIMITS


def _in_bounds(q, lower, upper):
    bad = []
    for i, v in enumerate(q):
        if not (lower[i] <= v <= upper[i] + 1e-9):
            bad.append({"index": i, "value": v, "lower": lower[i], "upper": upper[i]})
    return bad


def _sync_view() -> None:
    """Recompute matrix_world before reading any empty's world transform; without this a fresh
    move/set reads back the previous frame's value (the stale-empty bug get_target hit)."""
    bpy.context.view_layer.update()


# --------------------------------------------------------------------------- observe (§6.1) ----
def h_status(args, bridge):
    A = _addon()
    def _r():
        p = bpy.context.scene.pickik
        rig = A._state.rig
        return {"addon": A.bl_info.get("name", "pickik"), "blender": bpy.app.version_string,
               "dll_loaded": A._state.core is not None, "dll_error": A._state.dll_error or None,
               "rig_present": rig is not None and rig.alive(), "solver": p.solver,
               "continuous": bool(p.continuous), "busy": bool(A._state.busy),
               "status": p.status or None, "bridge": bridge.status_dict()}
    return bridge.call_on_main(_r)


def h_get_state(args, bridge):
    A = _addon()
    def _r():
        p = bpy.context.scene.pickik
        rig = A._rig_or_die()
        return {"q_rad": [float(v) for v in rig.last_q],
               "target_xyz_mm": [p.target_x_mm, p.target_y_mm, p.target_z_mm]}
    snap = bridge.call_on_main(_r)
    q = snap["q_rad"]
    core = _core_of(bridge)
    t0, _frames = core.fk_tool0(q)                              # the C-ABI FK, the model itself
    lower, upper = _limits_of(bridge)
    return {"q_rad": q, "q_deg": [math.degrees(v) for v in q],
           "target_xyz_mm": snap["target_xyz_mm"],
           "tool0_xyz_mm": [t0[0] * 1e3, t0[1] * 1e3, t0[2] * 1e3],
           "valid": not _in_bounds(q, lower, upper), "fk_source": "c-abi"}


def h_get_robot_info(args, bridge):
    lower, upper = _limits_of(bridge)
    return {"n_joints": 7, "joint_names": [f"J{i}" for i in range(1, 8)],
           "units": "rad, m, mm", "limits": {"lower": list(lower), "upper": list(upper)},
           "solvers": ["ccd", "gradient", "memetic"]}


def h_validate_pose(args, bridge):
    """`pure`: no bpy, no call_on_main — safe to run on the off-tick pool, concurrently."""
    if "q_rad" in args:
        q = [float(v) for v in args["q_rad"]]
    elif "q_deg" in args:
        q = [math.radians(float(v)) for v in args["q_deg"]]
    else:
        raise P.McpError(P.ERR.INVAL, "validate_pose needs q_rad or q_deg")
    if len(q) != 7:
        raise P.McpError(P.ERR.INVAL, f"expected 7 joints, got {len(q)}")
    core = _core_of(bridge)
    t0, _ = core.fk_tool0(q)
    lower, upper = _limits_of(bridge)
    bad = _in_bounds(q, lower, upper)
    return {"in_bounds": not bad, "violations": bad,
           "tool0_xyz_mm": [t0[0] * 1e3, t0[1] * 1e3, t0[2] * 1e3], "fk_source": "c-abi"}


# --------------------------------------------------------------------------- rig & urdf (§6.2) --
def h_build_rig(args, bridge):
    A = _addon()
    def _do():
        rebuild = bool(args.get("rebuild", True))
        if not rebuild and A._state.rig is not None and A._state.rig.alive():
            pass
        else:
            bpy.ops.pickik.build_rig()                          # the add-on's own operator
        rig = A._rig_or_die()
        if args.get("q_deg") is not None:                        # software, run the manual-FK path
            q = [math.radians(float(v)) for v in args["q_deg"]]
            if len(q) != 7:
                raise P.McpError(P.ERR.INVAL, "q_deg needs 7 values")
            for i in range(7):                                   # set fields; RNA clamps to hard limits
                setattr(bpy.context.scene.pickik, f"fk_j{i + 1}", math.degrees(q[i]))
            bpy.ops.pickik.apply_fk()
        return {"rig_present": True, "objects": A.arm7_rig.rig_object_names()}
    return bridge.call_on_main(_do)


def h_delete_rig(args, bridge):                                # gate=confirm is enforced by the bridge
    A = _addon()
    def _do():
        rig = A._state.rig
        removed = []
        if rig is not None:
            for name in A.arm7_rig.rig_object_names():
                ob = bpy.data.objects.get(name)
                if ob is not None:
                    removed.append(name)
                    bpy.data.objects.remove(ob)
            A._state.rig = None
        return {"removed": removed}
    return bridge.call_on_main(_do)


def h_export_urdf(args, bridge):
    A = _addon()
    sub = args.get("directory") or "."
    root = bridge.export_root
    if root:                                                     # §9.5 sandbox: never an arbitrary sink
        target = P.sandbox_under(root, sub)
    else:
        target = None
    def _do():
        bpy.ops.pickik.save_urdf(directory=target or ".")
        return {"directory": target or ".", "exported": True}
    return bridge.call_on_main(_do)


# --------------------------------------------------------------------------- IK / FK (§6.3) ----
def _solve_kwargs(args):
    """Map the tool `options` object onto Core.solve's keyword arguments (option parsing)."""
    o = args.get("options") if isinstance(args.get("options"), dict) else {}
    kw = {}
    if o.get("md_weight") is not None:
        kw["md_weight"] = float(o["md_weight"])
    if o.get("jt_weight") is not None:
        kw["jt_weight"] = float(o["jt_weight"])
    if o.get("la_weight") is not None:
        kw["la_weight"] = float(o["la_weight"])
    if o.get("joint_targets") is not None:
        kw["joint_targets"] = tuple(o["joint_targets"])
    la = o.get("look_at")
    if isinstance(la, dict) and la.get("point") is not None:
        kw["look_at_point_m"] = tuple(la["point"])
        if la.get("axis") is not None:
            kw["look_at_axis"] = tuple(la["axis"])
    quat = args.get("quaternion")
    if quat is not None:
        kw["orientation_quat"] = tuple(quat); kw["position_only"] = False
    return kw


def h_solve_ik(args, bridge):
    """One IK solve. class is derived by mcp_protocol.classify() from the args, so this one handler
    serves all three shapes; it just routes its bpy-touching steps through call_on_main."""
    A = _addon()
    tgt = args.get("target_xyz_mm")
    if not isinstance(tgt, (list, tuple)) or len(tgt) != 3:
        raise P.McpError(P.ERR.INVAL, "target_xyz_mm must be [x,y,z] in millimetres")
    target_m = (float(tgt[0]) / 1e3, float(tgt[1]) / 1e3, float(tgt[2]) / 1e3)
    kind = str(args.get("solver", "gradient")).strip().lower()
    if kind not in ("ccd", "gradient", "memetic"):
        raise P.McpError(P.ERR.INVAL, f"unknown solver {kind!r}")
    seeded = isinstance(args.get("seed_q"), (list, tuple)) and len(args["seed_q"]) == 7
    seed = [float(v) for v in args["seed_q"]] if seeded else None
    execute = bool(args.get("execute")) and not bool(args.get("dry_run"))
    core = _core_of(bridge)

    if seed is None:                                             # current pose is the seed
        def _seed():
            rig = A._rig_or_die()
            return [float(v) for v in rig.last_q]
        seed = bridge.call_on_main(_seed)                        # a main-thread read of the rig

    result = core.solve(kind, target_m, seed, **_solve_kwargs(args))   # heavy work, current thread

    if not execute:
        applied = False
    else:
        result = dict(result); result["solver_name"] = kind
        def _apply():
            rig = A._rig_or_die()
            A._apply_result(rig, result)                          # apply_q + refresh the exposed props
            return True
        applied = bridge.call_on_main(_apply)                    # the scene mutation, on the main thread

    pe = float(result.get("position_error", -1.0))
    oe = float(result.get("orientation_error", -1.0))
    return {"success": bool(result["success"]), "q": list(result["q"]),
           "q_deg": [math.degrees(v) for v in result["q"]],
           "error_pos_mm": pe * 1e3, "error_orient_rad": oe,
           "solver": kind, "dry_run": not execute, "applied": applied,
           "time_ms": float(result.get("time_ms", 0.0))}


def h_set_target(args, bridge):
    A = _addon()
    xyz = args.get("target_xyz_mm") or args.get("xyz")
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        raise P.McpError(P.ERR.INVAL, "set_target needs target_xyz_mm=[x,y,z]")
    def _do():
        p = bpy.context.scene.pickik
        p.target_x_mm, p.target_y_mm, p.target_z_mm = float(xyz[0]), float(xyz[1]), float(xyz[2])
        rig = A._rig_or_die()
        rig.target.location = (float(xyz[0]) / 1e3, float(xyz[1]) / 1e3, float(xyz[2]) / 1e3)
        _sync_view()
        return {"target_xyz_mm": [p.target_x_mm, p.target_y_mm, p.target_z_mm]}
    return bridge.call_on_main(_do)


def h_get_target(args, bridge):
    A = _addon()
    def _r():
        rig = A._rig_or_die()
        _sync_view()                                             # the empty is the source of truth — refresh it first
        tp = A.arm7_rig.target_position(rig)
        return {"target_xyz_mm": [tp[0] * 1e3, tp[1] * 1e3, tp[2] * 1e3]}
    return bridge.call_on_main(_r)


def h_set_joint_angles(args, bridge):                            # manual FK ("software")
    A = _addon()
    degs = args.get("angles_deg")
    if not isinstance(degs, (list, tuple)) or len(degs) != 7:
        raise P.McpError(P.ERR.INVAL, "set_joint_angles needs angles_deg=[7]")
    def _do():
        p = bpy.context.scene.pickik
        for i in range(7):
            setattr(p, f"fk_j{i + 1}", float(degs[i]))            # assignment clamps to the hard limits
        bpy.ops.pickik.apply_fk()
        rig = A._rig_or_die()
        q = [float(v) for v in rig.last_q]
        return {"q_rad": q, "q_applied_deg": [getattr(p, f"fk_j{i + 1}") for i in range(7)]}
    return bridge.call_on_main(_do)


def h_set_solver(args, bridge):
    s = str(args.get("solver", "")).strip().lower()
    if s not in ("ccd", "gradient", "memetic"):
        raise P.McpError(P.ERR.INVAL, f"unknown solver {s!r}")
    def _do():
        bpy.context.scene.pickik.solver = s
        return {"solver": bpy.context.scene.pickik.solver}
    return bridge.call_on_main(_do)


def h_get_solver(args, bridge):
    def _r():
        p = bpy.context.scene.pickik
        return {"solver": p.solver, "md_weight": p.md_weight, "jt_weight": p.jt_weight,
               "la_weight": p.la_weight}
    return bridge.call_on_main(_r)


def h_set_solver_config(args, bridge):
    def _do():
        p = bpy.context.scene.pickik
        for k in ("md_weight", "jt_weight", "la_weight"):
            if args.get(k) is not None:
                setattr(p, k, float(args[k]))
        return {"md_weight": p.md_weight, "jt_weight": p.jt_weight, "la_weight": p.la_weight}
    return bridge.call_on_main(_do)


def h_set_continuous(args, bridge):
    on = bool(args.get("on"))
    def _do():
        p = bpy.context.scene.pickik
        if bool(p.continuous) != on:
            bpy.ops.pickik.toggle_continuous()                    # the operator owns the timer lifecycle
        return {"continuous": bool(bpy.context.scene.pickik.continuous)}
    return bridge.call_on_main(_do)


#: the allow-list the bridge dispatches through; keys MUST equal mcp_protocol.COMMANDS keys
HANDLERS = {
    "status": h_status, "get_state": h_get_state, "get_robot_info": h_get_robot_info,
    "validate_pose": h_validate_pose, "build_rig": h_build_rig, "delete_rig": h_delete_rig,
    "export_urdf": h_export_urdf, "solve_ik": h_solve_ik, "set_target": h_set_target,
    "get_target": h_get_target, "set_joint_angles": h_set_joint_angles, "set_solver": h_set_solver,
    "set_solver_config": h_set_solver_config, "set_continuous": h_set_continuous,
}
