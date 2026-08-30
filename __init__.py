"""PickIK arm7 — native IK for the 7-DOF desktop arm (Blender 4.x).

Loads the pick_ik_c C ABI shared library (libpick-ik-core) via ctypes and
drives the compiled-in arm7 model: millisecond-class FK/IK, no Python FK
pump, no CPython version coupling.

UI (3D sidebar > PickIK):
  - Build rig: creates the arm7 empty hierarchy + a movable target empty;
  - Solve: current pose as seed, target = the empty's position (or the
    X/Y/Z fields), solver dropdown (ccd / gradient / memetic);
  - Continuous: a background timer re-solves (150 ms debounce, busy
    guarded) whenever the target or the options change — the arm chases
    the gizmo. CCD/gradient are ms-class on the UI thread; a memetic
    solve runs on a background thread and snaps in when it finishes.

Threading: ctypes releases the GIL for the whole pickik_solve call, so
the background memetic thread genuinely runs in parallel. Only the
result is applied on the UI thread (via a bpy timer).
"""

from __future__ import annotations

import threading

import bpy
from bpy.props import (BoolProperty, EnumProperty, FloatProperty, StringProperty)

from . import arm7_rig
from . import ik_core

bl_info = {
    "name": "PickIK arm7 (native C ABI)",
    "author": "Swann Schilling",
    "version": (0, 1, 0),
    # Verified on 3.4.1 and 4.5.3 (register/unregister + rig build, headless).
    "blender": (3, 4, 0),
    "location": "View > Sidebar > PickIK",
    "description": "Native inverse kinematics for the 7-DOF arm7 desktop arm "
                   "(PickIK core via the pick_ik_c C ABI)",
    "category": "Rigging",
}

SOLVER_ITEMS = (
    ("gradient", "Gradient (fast, deterministic)", ""),
    ("ccd", "CCD (fast, local)", ""),
    ("memetic", "Memetic (global, background)", ""),
)


# ---------------------------------------------------------------------------
# Scene properties
# ---------------------------------------------------------------------------

class PickIKProps(bpy.types.PropertyGroup):
    dll_path: StringProperty(
        name="DLL path", description="Path to pick_ik_c.dll (empty = auto-find:"
        " $PICKIK_C_DLL, next to this add-on, or the sibling libpick-ik-core "
        "build tree)", default="", subtype="FILE_PATH")
    solver: EnumProperty(name="Solver", items=SOLVER_ITEMS, default="gradient")
    target_x_mm: FloatProperty(name="X (mm)", default=300.0)
    target_y_mm: FloatProperty(name="Y (mm)", default=150.0)
    target_z_mm: FloatProperty(name="Z (mm)", default=300.0)
    md_weight: FloatProperty(name="Minimal displacement", default=0.0, min=0.0, step=0.01,
                             description="Pull the solution toward the seed (0 = off)")
    jt_weight: FloatProperty(name="Joint targets weight", default=0.0, min=0.0, step=0.05,
                             description="Pull named joints toward their targets (0 = off)")
    la_weight: FloatProperty(name="Look-at weight", default=0.0, min=0.0, step=0.01,
                             description="Point the tip axis at the look-at point (0 = off)")
    continuous: BoolProperty(name="Continuous", default=False,
                             description="Re-solve (debounced) whenever the target or"
                                         " options change")
    status: StringProperty(name="Status", default="")


class _CoreState:
    """Module-level runtime state (not scene data)."""
    core: ik_core.Core | None = None
    dll_error: str = ""
    rig: arm7_rig.Rig | None = None
    busy: bool = False
    last_key: str = ""
    pending_result: dict | None = None  # set by bg thread, consumed by timer


_state = _CoreState()


def _core_or_die() -> ik_core.Core:
    if _state.core is None:
        try:
            scene = bpy.context.scene
            path = ik_core.find_dll(scene.pickik.dll_path)
            _state.core = ik_core.Core(path)
            scene.pickik.status = f"loaded {path.rsplit(chr(92), 1)[-1]}" \
                                  if chr(92) in path else f"loaded {path.split('/')[-1]}"
        except ik_core.CoreError as e:
            _state.dll_error = str(e)
            raise
    return _state.core


def _status(msg: str) -> None:
    bpy.context.scene.pickik.status = msg


def _rig_or_die() -> arm7_rig.Rig:
    if _state.rig is None:
        _state.rig = arm7_rig.build()
    return _state.rig


def _sync_target_from_fields(scene) -> None:
    """If the user typed into the mm fields, move the target empty."""
    rig = _state.rig
    if rig is None:
        return
    p = rig.target.matrix_world.to_translation()
    want = (scene.pickik.target_x_mm / 1000.0, scene.pickik.target_y_mm / 1000.0,
            scene.pickik.target_z_mm / 1000.0)
    if any(abs(want[i] - p[i]) > 1e-9 for i in range(3)):
        rig.target.location = want


def _sync_fields_from_target(scene) -> None:
    rig = _state.rig
    if rig is None:
        return
    p = rig.target.matrix_world.to_translation()
    scene.pickik.target_x_mm = p.x * 1000.0
    scene.pickik.target_y_mm = p.y * 1000.0
    scene.pickik.target_z_mm = p.z * 1000.0


def _solve_options_key(scene) -> str:
    """Key over everything that changes a solve: solver, secondary weights,
    and the target position (from the EMPTY — the fields may lag while the
    user drags the gizmo)."""
    p = scene.pickik
    if _state.rig is not None:
        t = _state.rig.target.matrix_world.to_translation()
        tpart = f"{t.x:.4f}|{t.y:.4f}|{t.z:.4f}"
    else:
        tpart = f"{p.target_x_mm:.3f}|{p.target_y_mm:.3f}|{p.target_z_mm:.3f}"
    return f"{p.solver}|{p.md_weight:.4f}|{p.jt_weight:.4f}|{p.la_weight:.4f}|{tpart}"


# ---------------------------------------------------------------------------
# Operators
# ---------------------------------------------------------------------------

class PICKIK_OT_build_rig(bpy.types.Operator):
    bl_idname = "pickik.build_rig"
    bl_label = "Build arm7 rig"
    bl_description = "Create the arm7 empty hierarchy + target empty"

    def execute(self, context) -> set[str]:
        try:
            core = _core_or_die()  # validate the DLL too
        except ik_core.CoreError as e:
            _status(f"DLL error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        scene = context.scene
        p = scene.pickik
        _sync_target_from_fields(scene)
        target = (p.target_x_mm / 1000.0, p.target_y_mm / 1000.0, p.target_z_mm / 1000.0)
        _state.rig = arm7_rig.build(target_m=target)
        _status("rig built (7 joints + tool0 + target empty)")
        # FINISHED (not RUNNING_EXECUTABLE): the latter only exists in
        # Blender 4.x and makes 3.x raise a RuntimeError after execute().
        return {'FINISHED'}


class PICKIK_OT_solve(bpy.types.Operator):
    bl_idname = "pickik.solve"
    bl_label = "Solve IK"
    bl_description = "Solve one IK step (current pose as seed, target empty as goal)"

    def execute(self, context) -> set[str]:
        try:
            core = _core_or_die()
        except ik_core.CoreError as e:
            _status(f"DLL error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        rig = _rig_or_die()
        scene = context.scene
        p = scene.pickik
        _sync_target_from_fields(scene)
        # The sync (or a fresh rig) may have written matrices this tick;
        # make them readable before taking the target.
        bpy.context.view_layer.update()

        target_m = rig.target.matrix_world.to_translation()
        seed = list(rig.last_q)
        solver = p.solver

        if solver == "memetic":
            # Global/recover solve: background thread, snap the result in.
            if _state.busy:
                _status("previous solve still running")
                return {'CANCELLED'}
            _state.busy = True
            _status(f"memetic running (background) for target "
                    f"({target_m.x*1e3:.0f}, {target_m.y*1e3:.0f}, {target_m.z*1e3:.0f}) mm ...")
            threading.Thread(
                target=lambda: _bg_solve("memetic", tuple(target_m), p.md_weight),
                daemon=True).start()
            bpy.app.timers.register(_drain_pending, first_interval=0.05)
            return {'FINISHED'}

        # CCD / gradient: ms-class, synchronous on the UI thread.
        _state.busy = True
        try:
            result = core.solve(solver, tuple(target_m), seed,
                                md_weight=p.md_weight,
                                joint_targets=(None,) * 7 if p.jt_weight == 0.0
                                else _read_joint_targets(),
                                jt_weight=p.jt_weight,
                                la_weight=p.la_weight,
                                look_at_point_m=None,)
        except ik_core.CoreError as e:
            _state.busy = False
            _status(f"solve error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        _state.busy = False
        result["solver_name"] = solver  # _apply_result's status line
        _apply_result(rig, result)
        _state.last_key = _solve_options_key(scene)
        return {'FINISHED'}


def _read_joint_targets():
    """Joint targets: for v1 the targets are the seed's own angles is not
    meaningful — the panel exposes only the weight; per-joint inputs come
    with the v1.1 options block. Return None-tuple (no per-joint target)."""
    return (None,) * 7


def _apply_result(rig: arm7_rig.Rig, result: dict) -> None:
    p = bpy.context.scene.pickik
    if result["success"]:
        arm7_rig.apply_q(rig, result["q"])
        p.status = (f"{result['solver_name']} OK  pos err "
                    f"{result['position_error']*1e3:.4f} mm  "
                    f"({result['time_ms']:.1f} ms)")
    else:
        p.status = (f"no solution  (pos err "
                    f"{result['position_error']*1e3:.1f} mm, "
                    f"{result['time_ms']:.1f} ms)")
    qdeg = ", ".join(f"{v*57.29578:.1f}" for v in result["q"])
    p.status += f"\nq[deg] = [{qdeg}]"


def _drain_pending() -> float | None:
    """UI-thread consumer of background-solve results."""
    pending = _state.pending_result
    if pending is None:
        return None  # unregister
    _state.pending_result = None
    rig = _state.rig
    if "error" in pending:
        _status(f"solve error: {pending['error']}")
        return None
    if rig is not None:
        r = dict(pending["result"])
        r["solver_name"] = bpy.context.scene.pickik.solver
        _apply_result(rig, r)
        _state.last_key = _solve_options_key(bpy.context.scene)
    return None


# ---------------------------------------------------------------------------
# Continuous mode
# ---------------------------------------------------------------------------

_continuous_timer_registered = False


def _continuous_tick() -> float | None:
    """50 ms cadence. Drains background-solve results (UI thread only), then
    re-solves when the options key changed (target moved or solver/weights
    changed). Busy-guarded; the synchronous solvers (ccd/gradient) are the
    intended continuous solvers — they are ms-class."""
    # 1) drain a finished background solve, if any.
    if _state.pending_result is not None:
        pending = _state.pending_result
        _state.pending_result = None
        if "error" in pending:
            _status(f"solve error: {pending['error']}")
        elif _state.rig is not None:
            r = dict(pending["result"])
            r["solver_name"] = bpy.context.scene.pickik.solver
            _apply_result(_state.rig, r)
            _state.last_key = _solve_options_key(bpy.context.scene)

    scene = bpy.context.scene
    p = scene.pickik
    if not p.continuous:
        _unregister_continuous()
        return None
    if _state.rig is None or _state.core is None or _state.busy:
        return 0.05
    bpy.context.view_layer.update()  # target reads below need fresh matrices
    key = _solve_options_key(scene)
    if key == _state.last_key:
        return 0.05
    rig = _state.rig
    target_m = rig.target.matrix_world.to_translation()
    if p.solver == "memetic":
        # Continuous + memetic: background solve; the tick drains the result.
        _state.busy = True
        threading.Thread(
            target=lambda: _bg_solve("memetic", tuple(target_m), p.md_weight),
            daemon=True).start()
        return 0.05
    _state.busy = True
    try:
        result = _state.core.solve(p.solver, tuple(target_m), list(rig.last_q),
                                   md_weight=p.md_weight)
        _apply_result(rig, result)
    except ik_core.CoreError as e:
        _status(f"continuous error: {e}")
    finally:
        _state.busy = False
    _state.last_key = key
    return 0.05


def _bg_solve(kind: str, target_m: tuple[float, float, float], md_weight: float) -> None:
    """Background solve worker (memetic). Result lands in _state.pending_result;
    the UI thread applies it (timer or continuous tick)."""
    try:
        result = _state.core.solve(kind, target_m, list(_state.rig.last_q),
                                  md_weight=md_weight)
        _state.pending_result = {"q": result["q"], "result": result, "from_bg": True}
    except ik_core.CoreError as e:
        _state.pending_result = {"error": str(e), "from_bg": True}
    finally:
        _state.busy = False


def _register_continuous() -> None:
    global _continuous_timer_registered
    if not _continuous_timer_registered:
        bpy.app.timers.register(_continuous_tick, first_interval=0.05)
        _continuous_timer_registered = True


def _unregister_continuous() -> None:
    global _continuous_timer_registered
    if _continuous_timer_registered:
        _continuous_timer_registered = False


class PICKIK_OT_toggle_continuous(bpy.types.Operator):
    bl_idname = "pickik.toggle_continuous"
    bl_label = "Toggle continuous"

    def execute(self, context) -> set[str]:
        p = context.scene.pickik
        p.continuous = not p.continuous
        if p.continuous:
            _register_continuous()
            p.status = "continuous ON (arm chases the target)"
        else:
            _unregister_continuous()
            p.status = "continuous OFF"
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------

class PICKIK_PT_main(bpy.types.Panel):
    bl_label = "PickIK arm7"
    bl_idname = "PICKIK_PT_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "PickIK"

    @classmethod
    def poll(cls, context) -> bool:
        return context.space_data.shading.type in {'SOLID', 'MATERIAL', 'RENDERED'}

    def draw(self, context) -> None:
        layout = self.layout
        scene = context.scene
        p = scene.pickik

        for i, line in enumerate(p.status.split("\n")):
            layout.label(text=line, icon='INFO' if i == 0 else 'BLANK')

        box = layout.box()
        box.prop(p, "dll_path", text="")
        row = box.row()
        row.operator("pickik.build_rig", text="Build rig")
        if _state.rig is None:
            box.label(text="(build the rig first)", icon='ERROR')

        box = layout.box()
        box.label(text="Target (mm) — or move the target empty")
        row = box.row()
        row.prop(p, "target_x_mm", text="X")
        row.prop(p, "target_y_mm", text="Y")
        row.prop(p, "target_z_mm", text="Z")
        row = box.row()
        row.prop(p, "solver", text="")
        row.operator("pickik.solve", text="Solve", icon='MESH_DATA')

        box = layout.box()
        box.prop(p, "continuous")
        row = box.row()
        row.operator("pickik.toggle_continuous", text="Toggle continuous")
        box.prop(p, "md_weight")
        box.label(text="Per-joint targets + look-at land in v1.1")


def _sync_target_property(scene, context) -> None:
    """Keep the mm fields in step when the target empty moves."""
    if _state.rig is None:
        return
    _sync_fields_from_target(scene)


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

CLASSES = (PickIKProps, PICKIK_OT_build_rig, PICKIK_OT_solve,
           PICKIK_OT_toggle_continuous, PICKIK_PT_main)


def register() -> None:
    global _state
    bpy.utils.register_class(PickIKProps)
    bpy.types.Scene.pickik = bpy.props.PointerProperty(type=PickIKProps)
    for cls in CLASSES[1:]:
        bpy.utils.register_class(cls)
    _state = _CoreState()  # fresh state per register


def unregister() -> None:
    _unregister_continuous()
    for cls in reversed(CLASSES[1:]):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.pickik
    bpy.utils.unregister_class(PickIKProps)


if __name__ == "__main__":
    register()
