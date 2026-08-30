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
  - FK/manual: J1..J7 degree sliders pose the arm without a solver
    (Apply FK / Sync from arm).

  - **Target authority**: the target empty object is the sole position
    authority. Dragging the gizmo moves it directly; the mm
    fields/sliders are live editors (typing/dragging moves the target
    immediately) and display mirrors (they update from the empty on
    Build/Solve/continuous tick). There is never a snap-back.

FK values exposed in the scene (readable/wireable elsewhere):
  - each joint angle = `Arm7_Ji.rotation_euler.z` (radians — item panel,
    drivers, keyframes, scripts);
  - `bpy.context.scene.pickik.q_j1 ... q_j7` (radians, updated after every
    pose change);
  - `bpy.context.scene.pickik.tool0_x_mm / _y_mm / _z_mm` (end-effector);
  - custom property `ik_q_deg` on every joint empty (object-local readout).

Threading: ctypes releases the GIL for the whole pickik_solve call, so
the background memetic thread genuinely runs in parallel. Only the
result is applied on the UI thread (via a bpy timer).
"""

from __future__ import annotations

import math
import os
import threading
import traceback

import bpy
from mathutils import Vector
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

def _target_field_update(self: bpy.types.PropertyGroup,
                         context: bpy.types.Context) -> None:
    """Move the target empty when the user types/drags a mm field (live)."""
    if _state.mirroring_fields:
        return
    rig = _state.rig
    if rig is None or not rig.alive():
        return
    p = context.scene.pickik
    want = Vector((p.target_x_mm / 1e3, p.target_y_mm / 1e3,
                   p.target_z_mm / 1e3))
    cur = rig.target.location  # unparented empty: local == world
    if (want - cur).length > 1e-9:
        rig.target.location = want


class PickIKProps(bpy.types.PropertyGroup):
    dll_path: StringProperty(
        name="DLL path", description="Path to pick_ik_c.dll. Leave empty to "
        "auto-find ($PICKIK_C_DLL, next to this add-on, or the sibling "
        "libpick-ik-core build tree) — after the first load the found path "
        "is pre-selected here and reused; a path you type yourself always "
        "takes priority and is never overwritten", default="", subtype="FILE_PATH")
    solver: EnumProperty(name="Solver", items=SOLVER_ITEMS, default="gradient")
    # Slider ranges mirror the ik-service web demo (x/y -550..550, z 0..650).
    target_x_mm: FloatProperty(name="X (mm)", default=300.0, min=-550.0, max=550.0,
                            step=1, update=_target_field_update,
                            description="Move the target empty; the arm chases it in"
                                        " continuous mode")
    target_y_mm: FloatProperty(name="Y (mm)", default=150.0, min=-550.0, max=550.0,
                            step=1, update=_target_field_update,
                            description="Move the target empty")
    target_z_mm: FloatProperty(name="Z (mm)", default=300.0, min=0.0, max=650.0,
                            step=1, update=_target_field_update,
                            description="Move the target empty")
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

    # FK/manual mode: degree sliders for direct posing (no solver).
    fk_j1: FloatProperty(name="J1 (deg)", default=0.0, min=-180.0, max=180.0, step=1)
    fk_j2: FloatProperty(name="J2 (deg)", default=0.0, min=-119.7455, max=119.7455, step=1)
    fk_j3: FloatProperty(name="J3 (deg)", default=0.0, min=-180.0, max=180.0, step=1)
    fk_j4: FloatProperty(name="J4 (deg)", default=0.0, min=-119.7455, max=119.7455, step=1)
    fk_j5: FloatProperty(name="J5 (deg)", default=0.0, min=-180.0, max=180.0, step=1)
    fk_j6: FloatProperty(name="J6 (deg)", default=0.0, min=-119.7455, max=119.7455, step=1)
    fk_j7: FloatProperty(name="J7 (deg)", default=0.0, min=-180.0, max=180.0, step=1)
    # Exposed FK values (radians / mm) — updated after every pose change.
    q_j1: FloatProperty(name="q J1 (rad)", default=0.0, min=-3.14159265, max=3.14159265,
                        step=0.001, precision=4,
                        description="Current J1 angle, radians (scriptable)")
    q_j2: FloatProperty(name="q J2 (rad)", default=0.0, min=-2.09, max=2.09,
                        step=0.001, precision=4)
    q_j3: FloatProperty(name="q J3 (rad)", default=0.0, min=-3.14159265, max=3.14159265,
                        step=0.001, precision=4)
    q_j4: FloatProperty(name="q J4 (rad)", default=0.0, min=-2.09, max=2.09,
                        step=0.001, precision=4)
    q_j5: FloatProperty(name="q J5 (rad)", default=0.0, min=-3.14159265, max=3.14159265,
                        step=0.001, precision=4)
    q_j6: FloatProperty(name="q J6 (rad)", default=0.0, min=-2.09, max=2.09,
                        step=0.001, precision=4)
    q_j7: FloatProperty(name="q J7 (rad)", default=0.0, min=-3.14159265, max=3.14159265,
                        step=0.001, precision=4)
    tool0_x_mm: FloatProperty(name="tool0 X (mm)", default=0.0, step=0.1, precision=1,
                              description="End-effector X, updated after every pose")
    tool0_y_mm: FloatProperty(name="tool0 Y (mm)", default=0.0, step=0.1, precision=1)
    tool0_z_mm: FloatProperty(name="tool0 Z (mm)", default=675.0, step=0.1, precision=1)


class _CoreState:
    """Module-level runtime state (not scene data)."""
    core: ik_core.Core | None = None
    dll_error: str = ""
    rig: arm7_rig.Rig | None = None
    busy: bool = False
    last_key: str = ""
    pending_result: dict | None = None  # set by bg thread, consumed by timer
    mirroring_fields: bool = False  # suppress the callback during display mirror


_state = _CoreState()


def _core_or_die() -> ik_core.Core:
    if _state.core is None:
        try:
            scene = bpy.context.scene
            had_explicit = bool(scene.pickik.dll_path.strip())
            path = ik_core.find_dll(scene.pickik.dll_path)
            _state.core = ik_core.Core(path)
            if not had_explicit:
                # Pre-select the auto-found location in the panel field so
                # the user sees (and pins) the DLL actually in use. A
                # user-typed path already took priority in find_dll and is
                # never overwritten. A stale pre-fill can't break discovery:
                # find_dll treats it as the first candidate, not a
                # requirement.
                scene.pickik.dll_path = path
            scene.pickik.status = f"loaded {os.path.basename(path)}"
        except ik_core.CoreError as e:
            _state.dll_error = str(e)
            raise
    return _state.core


def _status(msg: str) -> None:
    bpy.context.scene.pickik.status = msg


def _rig_or_die() -> arm7_rig.Rig:
    """A live rig, self-healing: if the rig's objects were deleted in the
    viewport (stale StructRNA), rebuild from the panel's target fields.
    `arm7_rig.build()` cleans up any surviving partial-rig objects, so this
    never leaves a duplicated half-arm behind."""
    if _state.rig is not None and not _state.rig.alive():
        _state.rig = None
    if _state.rig is None:
        p = bpy.context.scene.pickik
        _state.rig = arm7_rig.build(target_m=(p.target_x_mm / 1e3,
                                              p.target_y_mm / 1e3,
                                              p.target_z_mm / 1e3))
    return _state.rig


def _sync_fields_from_target(scene) -> None:
    """Mirror the target empty's position into the mm fields (display).
    Uses the mirroring_fields flag so the update callback doesn't loop."""
    rig = _state.rig
    if rig is None or not rig.alive():
        return
    p = rig.target.matrix_world.to_translation()
    _state.mirroring_fields = True
    try:
        scene.pickik.target_x_mm = p.x * 1000.0
        scene.pickik.target_y_mm = p.y * 1000.0
        scene.pickik.target_z_mm = p.z * 1000.0
    finally:
        _state.mirroring_fields = False


def _solve_options_key(scene) -> str:
    """Key over everything that changes a solve: solver, secondary weights,
    and the target position (from the EMPTY — the fields may lag while the
    user drags the gizmo)."""
    p = scene.pickik
    if _state.rig is not None and _state.rig.alive():
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
            try:
                core = _core_or_die()  # validate the DLL too
            except ik_core.CoreError as e:
                _status(f"DLL error: {e}")
                self.report({'ERROR'}, str(e))
                return {'CANCELLED'}
            scene = context.scene
            p = scene.pickik
            target = (p.target_x_mm / 1000.0, p.target_y_mm / 1000.0,
                      p.target_z_mm / 1000.0)
            _state.rig = arm7_rig.build(target_m=target)
            _update_fk_props(_state.rig, _state.rig.last_q)
            _status("rig built (7 joints + tool0 + target empty)")
            # FINISHED (not RUNNING_EXECUTABLE): the latter only exists in
            # Blender 4.x and makes 3.x raise a RuntimeError after execute().
            return {'FINISHED'}
        except Exception as e:
            # Never let an exception escape execute(): a raw traceback into
            # the UI redraw cycle is what collapses the panel. Report +
            # status instead.
            _status(f"error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_solve(bpy.types.Operator):
    bl_idname = "pickik.solve"
    bl_label = "Solve IK"
    bl_description = "Solve one IK step (current pose as seed, target empty as goal)"

    def execute(self, context) -> set[str]:
        try:
            try:
                core = _core_or_die()
            except ik_core.CoreError as e:
                _status(f"DLL error: {e}")
                self.report({'ERROR'}, str(e))
                return {'CANCELLED'}
            rig = _rig_or_die()  # self-heals if the rig was deleted
            scene = context.scene
            p = scene.pickik
            bpy.context.view_layer.update()
            _sync_fields_from_target(scene)  # fields mirror the empty (display)
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
        except Exception as e:
            _state.busy = False
            _status(f"error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


def _read_joint_targets():
    """Joint targets: for v1 the targets are the seed's own angles is not
    meaningful — the panel exposes only the weight; per-joint inputs come
    with the v1.1 options block. Return None-tuple (no per-joint target)."""
    return (None,) * 7


def _update_fk_props(rig: arm7_rig.Rig, q) -> None:
    """Expose FK values: scene properties (scriptable) + per-joint custom
    properties (object-local readout). Call after any pose change."""
    p = bpy.context.scene.pickik
    for i in range(7):
        setattr(p, f"q_j{i + 1}", q[i])
        rig.joint_empties[i]["ik_q_deg"] = math.degrees(q[i])
    bpy.context.view_layer.update()
    t = rig.tool.matrix_world.to_translation()
    p.tool0_x_mm = t.x * 1e3
    p.tool0_y_mm = t.y * 1e3
    p.tool0_z_mm = t.z * 1e3


def _apply_result(rig: arm7_rig.Rig, result: dict) -> None:
    p = bpy.context.scene.pickik
    if result["success"]:
        arm7_rig.apply_q(rig, result["q"])
        _update_fk_props(rig, result["q"])
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
    if rig is not None and not rig.alive():
        _state.rig = None
        rig = None
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
        elif _state.rig is not None and _state.rig.alive():
            r = dict(pending["result"])
            r["solver_name"] = bpy.context.scene.pickik.solver
            _apply_result(_state.rig, r)
            _state.last_key = _solve_options_key(bpy.context.scene)

    scene = bpy.context.scene
    p = scene.pickik
    if not p.continuous:
        _unregister_continuous()
        return None
    if _state.rig is not None and not _state.rig.alive():
        _state.rig = None
    if _state.rig is None or _state.core is None or _state.busy:
        return 0.05
    bpy.context.view_layer.update()  # target reads below need fresh matrices
    _sync_fields_from_target(scene)    # fields mirror the empty (display)
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
    except Exception as e:
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
    except Exception as e:
        # e.g. the user deleted the rig mid-solve (stale StructRNA in
        # _state.rig.last_q): report, never crash the worker thread.
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
        try:
            p = context.scene.pickik
            p.continuous = not p.continuous
            if p.continuous:
                _register_continuous()
                p.status = "continuous ON (arm chases the target)"
            else:
                _unregister_continuous()
                p.status = "continuous OFF"
            return {'FINISHED'}
        except Exception as e:
            _status(f"error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


# ---------------------------------------------------------------------------
# FK / manual operators
# ---------------------------------------------------------------------------

class PICKIK_OT_apply_fk(bpy.types.Operator):
    bl_idname = "pickik.apply_fk"
    bl_label = "Apply FK (manual)"
    bl_description = "Pose the arm from the J1..J7 sliders (no solver)"

    def execute(self, context) -> set[str]:
        try:
            rig = _rig_or_die()
            p = context.scene.pickik
            q = [math.radians(getattr(p, f"fk_j{i + 1}")) for i in range(7)]
            arm7_rig.apply_q(rig, q)
            _update_fk_props(rig, q)
            qdeg = ", ".join(f"{getattr(p, f'fk_j{i + 1}'):.1f}" for i in range(7))
            _status(f"manual FK applied  q[deg] = [{qdeg}]")
            return {'FINISHED'}
        except Exception as e:
            _status(f"error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_sync_fk(bpy.types.Operator):
    bl_idname = "pickik.sync_fk"
    bl_label = "Sync sliders from arm"
    bl_description = "Copy the arm's current joint angles into the J1..J7 sliders"

    def execute(self, context) -> set[str]:
        try:
            rig = _rig_or_die()
            p = context.scene.pickik
            bpy.context.view_layer.update()
            for i, em in enumerate(rig.joint_empties):
                setattr(p, f"fk_j{i + 1}", math.degrees(em.rotation_euler.z))
            _status("J1..J7 sliders synced from the arm")
            return {'FINISHED'}
        except Exception as e:
            _status(f"error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


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
        return True  # the arm + target are useful in any shading mode

    def draw(self, context) -> None:
        """Self-diagnosing draw: if the panel body fails for any reason,
        log the full traceback to ~/pickik_addon_draw_errors.log and show
        an explicit error line instead of letting Blender collapse the
        panel to whatever was drawn so far."""
        try:
            self._draw(context)
        except Exception:
            tb = traceback.format_exc()
            try:
                log = os.path.join(os.path.expanduser("~"),
                                   "pickik_addon_draw_errors.log")
                with open(log, "a", encoding="utf-8") as f:
                    f.write("=== draw error ===\n" + tb + "\n")
            except Exception:
                pass
            self.layout.label(text="PickIK: panel draw error — see "
                              "pickik_addon_draw_errors.log in your home dir",
                             icon='ERROR')

    def _draw(self, context) -> None:
        layout = self.layout
        scene = context.scene
        p = scene.pickik

        # 'BLANK' is a Blender 4.x-only icon enum (3.4 has BLANK1); on 3.4
        # requesting it raises TypeError mid-draw and collapses the panel
        # after Solve (when the status grows to two lines). No icon on
        # continuation lines instead.
        status_lines = p.status.split("\n")
        layout.label(text=status_lines[0], icon='INFO')
        for line in status_lines[1:]:
            layout.label(text=line)

        box = layout.box()
        box.prop(p, "dll_path", text="")
        row = box.row()
        row.operator("pickik.build_rig", text="Build rig")
        if _state.rig is None or not _state.rig.alive():
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

        box = layout.box()
        box.label(text="FK / manual (no solver)")
        for i in range(7):
            row = box.row()
            row.label(text=f"J{i + 1}")
            row.prop(p, f"fk_j{i + 1}", text="")
        row = box.row()
        row.operator("pickik.apply_fk", text="Apply FK")
        row.operator("pickik.sync_fk", text="Sync from arm")

        box = layout.box()
        box.label(text="FK values (updated after every pose)")
        row = box.row()
        row.prop(p, "tool0_x_mm", text="tool0 X")
        row.prop(p, "tool0_y_mm", text="Y")
        row.prop(p, "tool0_z_mm", text="Z")
        box.label(text="Per-joint targets + look-at land in v1.1")


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

CLASSES = (PickIKProps, PICKIK_OT_build_rig, PICKIK_OT_solve,
           PICKIK_OT_toggle_continuous, PICKIK_OT_apply_fk, PICKIK_OT_sync_fk,
           PICKIK_PT_main)


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
