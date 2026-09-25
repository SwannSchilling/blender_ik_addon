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
  - FK/manual: J1..J7 degree sliders pose the arm live (no solver needed;
    Apply FK and Sync from arm are also available).

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
import time
import traceback

import bpy
from mathutils import Vector
from bpy.props import (BoolProperty, EnumProperty, FloatProperty,
                       IntProperty, StringProperty)

from . import arm7_rig
from . import ik_core
from . import cubemars_driver
from . import mcp_bridge                                        # the wire (MCP_INTEGRATION_PLAN.md)
from . import mcp_handlers_obs                                  # what the wire is allowed to answer

bl_info = {
    "name": "PickIK arm7 (native C ABI)",
    "author": "Swann Schilling",
    "version": (0, 2, 12),
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


def _fk_field_update(self: bpy.types.PropertyGroup,
                     context: bpy.types.Context) -> None:
    """Pose the arm directly when the user drags an FK slider (no solver)."""
    if _state.mirroring_fields:
        return
    rig = _state.rig
    if rig is None or not rig.alive():
        return
    p = context.scene.pickik
    q = [math.radians(getattr(p, f"fk_j{i + 1}")) for i in range(7)]
    arm7_rig.apply_q(rig, q)
    _update_fk_props(rig, q)


def _cubemars_live_update(self: bpy.types.PropertyGroup,
                          context: bpy.types.Context) -> None:
    """Checkbox 'Live update': on -> start the pose-following stream and
    its tracker timer; off -> stop the stream (motors disabled)."""
    p = context.scene.pickik
    if p.cubemars_enabled and p.cubemars_live:
        _cubemars_live_start()
    else:
        _cubemars_live_stop()


def _cubemars_enabled_update(self: bpy.types.PropertyGroup,
                            context: bpy.types.Context) -> None:
    """Disabling the CubeMars section must also stop a live stream."""
    p = context.scene.pickik
    if not p.cubemars_enabled and p.cubemars_live:
        _cubemars_live_stop()
        p.cubemars_live = False


class PickIKProps(bpy.types.PropertyGroup):
    dll_path: StringProperty(
        name="DLL path", description="Path to pick_ik_c.dll. Leave empty to "
        "auto-find ($PICKIK_C_DLL, next to this add-on, or the sibling "
        "libpick-ik-core build tree) — after the first load the found path "
        "is pre-selected here and reused; a path you type yourself always "
        "takes priority and is never overwritten", default="", subtype="FILE_PATH")
    meshes_path: StringProperty(
        name="Meshes dir", description="Directory with the joint STL visual "
        "meshes (J1_baseyaw_Z.stl etc.). Leave empty for the add-on's own "
        "meshes/ folder. Once auto-found, the path is pre-selected here; "
        "a typed path always takes priority", default="", subtype="DIR_PATH")
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
    fk_j1: FloatProperty(name="J1 (deg)", default=0.0, min=-180.0, max=180.0,
                          step=1, update=_fk_field_update,
                          description="Pose the arm — the FK sliders move the arm live")
    fk_j2: FloatProperty(name="J2 (deg)", default=0.0, min=-119.7455, max=119.7455,
                          step=1, update=_fk_field_update,
                          description="Pose the arm — live")
    fk_j3: FloatProperty(name="J3 (deg)", default=0.0, min=-180.0, max=180.0,
                          step=1, update=_fk_field_update,
                          description="Pose the arm — live")
    fk_j4: FloatProperty(name="J4 (deg)", default=0.0, min=-119.7455, max=119.7455,
                          step=1, update=_fk_field_update,
                          description="Pose the arm — live")
    fk_j5: FloatProperty(name="J5 (deg)", default=0.0, min=-180.0, max=180.0,
                          step=1, update=_fk_field_update,
                          description="Pose the arm — live")
    fk_j6: FloatProperty(name="J6 (deg)", default=0.0, min=-119.7455, max=119.7455,
                          step=1, update=_fk_field_update,
                          description="Pose the arm — live")
    fk_j7: FloatProperty(name="J7 (deg)", default=0.0, min=-180.0, max=180.0,
                          step=1, update=_fk_field_update,
                          description="Pose the arm — live")
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

    # -- CubeMars actuator properties ---------------------------------------
    cubemars_enabled: BoolProperty(
        name="CubeMars Actuators", default=False,
        update=_cubemars_enabled_update,
        description="Enable sending joint positions to CubeMars actuators via CAN")
    cubemars_interface: StringProperty(
        name="CAN interface", default="gs_usb",
        description="python-can backend: gs_usb (UCAN/canable), slcan (serial), "
                    "pcan, kvaser, neousys, socketcan (Linux), etc.")
    cubemars_channel: IntProperty(
        name="Channel", default=0, min=0, max=7,
        description="Bus/channel index (0 for the first adapter)")
    cubemars_j1_id: IntProperty(
        name="J1 CAN ID", default=0x68, min=1, max=127,
        description="CAN drive ID for the J1 actuator (0x68 = AK80-9)")
    cubemars_j2_id: IntProperty(
        name="J2 CAN ID", default=0x69, min=1, max=127,
        description="CAN drive ID for the J2 actuator (0x69 = AK10-9)")
    cubemars_j3_id: IntProperty(
        name="J3 CAN ID", default=0, min=0, max=127,
        description="CAN drive ID for J3 (0 = inactive)")
    cubemars_j4_id: IntProperty(
        name="J4 CAN ID", default=0, min=0, max=127,
        description="CAN drive ID for J4 (0 = inactive)")
    cubemars_j5_id: IntProperty(
        name="J5 CAN ID", default=0, min=0, max=127,
        description="CAN drive ID for J5 (0 = inactive)")
    cubemars_j6_id: IntProperty(
        name="J6 CAN ID", default=0, min=0, max=127,
        description="CAN drive ID for J6 (0 = inactive)")
    cubemars_j7_id: IntProperty(
        name="J7 CAN ID", default=0, min=0, max=127,
        description="CAN drive ID for J7 (0 = inactive)")
    cubemars_speed_erpm: FloatProperty(
        name="Max speed (ERPM)", default=2000.0, min=100.0, max=6000.0, step=100,
        description="Maximum speed for actuator moves")
    cubemars_accel_erpm_s2: FloatProperty(
        name="Max accel (ERPM/s^2)", default=2000.0, min=100.0, max=10000.0, step=100,
        description="Maximum acceleration for actuator moves")
    cubemars_slew_deg_per_tick: FloatProperty(
        name="Follow rate (deg/tick)", default=1.0, min=0.1, max=10.0, step=0.1,
        description="Maximum angular step the live update pushes toward the "
                    "measured pose per 50 ms tick (~20 deg/s at the default "
                    "1.0). Lower = smoother, more laggy follow on keyframe "
                    "playback; raise it for a snappier arm. Applies live "
                    "while 'Live update' is on.")
    cubemars_live: BoolProperty(
        name="Live update", default=False,
        update=_cubemars_live_update,
        description="Stream the current J1..J7 joint angles to the "
                    "actuators continuously while the arm moves - the "
                    "motors follow the pose in real time (50 Hz). The "
                    "motors are disabled when this is switched off or "
                    "the bus is disconnected")
    cubemars_status: StringProperty(name="Status", default="")
    cubemars_detail: StringProperty(name="Detail", default="",
        description="Multi-line detail for the current CubeMars task "
                    "(telemetry readout / driver-check diagnostics)")
    traj_fps: IntProperty(
        name="Trajectory sample FPS", default=60, min=5, max=500,
        description="Output sample rate for the exported/played arm "
                    "trajectory (finer = smoother motor stream, more frames)")


class _CoreState:
    """Module-level runtime state (not scene data)."""
    core: ik_core.Core | None = None
    dll_error: str = ""
    rig: arm7_rig.Rig | None = None
    busy: bool = False
    last_key: str = ""
    pending_result: dict | None = None  # set by bg thread, consumed by timer
    mirroring_fields: bool = False  # suppress the callback during display mirror
    cubemars: cubemars_driver.CubeMarsDriver | None = None
    cubemars_deps: dict | None = None  # cached check_dependencies() for the panel


_state = _CoreState()


def _resolve_meshes_dir(explicit: str = "") -> str:
    """Resolve the meshes directory. Order: explicit path > MESH_DIR
    (add-on's own meshes/). Never fails — always returns a path.
    Falls back to the add-on's meshes/ folder."""
    if explicit.strip():
        return explicit.strip()
    # Add-on's own meshes/ folder (from __init__.py parent dir)
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")


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
            # Pre-fill meshes_path similarly — auto-found, not overwriting
            # an explicit user-entered path.
            had_meshes_explicit = bool(scene.pickik.meshes_path.strip())
            if not had_meshes_explicit:
                scene.pickik.meshes_path = _resolve_meshes_dir()
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
        result["solver_name"] = p.solver
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
            _state.mirroring_fields = True
            try:
                for i, em in enumerate(rig.joint_empties):
                    setattr(p, f"fk_j{i + 1}", math.degrees(em.rotation_euler.z))
            finally:
                _state.mirroring_fields = False
            _status("J1..J7 sliders synced from the arm")
            return {'FINISHED'}
        except Exception as e:
            _status(f"error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


# ---------------------------------------------------------------------------
# CubeMars actuator operators
# ---------------------------------------------------------------------------

_cubemars_timer_registered = False


def _cubemars_status_tick() -> float | None:
    """Blender timer: poll the driver's status (and any running dep
    install / driver-check / telemetry task) into the scene property.
    Unregisters itself when nothing is active; the last task result
    (status + detail) stays on the panel (same persistence as the main
    status property)."""
    global _cubemars_timer_registered
    drv = _state.cubemars
    p = bpy.context.scene.pickik
    task_active, task_text, task_detail = _cubemars_task_poll()
    if task_active:
        p.cubemars_status = task_text
        p.cubemars_detail = task_detail
        return 0.1
    if drv is not None and drv.is_active:
        p.cubemars_status = drv.status
        return 0.1
    if task_text:
        p.cubemars_status = task_text
        p.cubemars_detail = task_detail
    _cubemars_timer_registered = False
    return None


_cubemars_task_lock = threading.Lock()
_cubemars_task_active = False
_cubemars_task_text = ""
_cubemars_task_detail = ""


def _cubemars_task_start(text: str, detail: str = "") -> None:
    """Mark a background dep/driver task as running with a progress line.
    The status timer mirrors text + detail into the scene properties;
    the thread never touches bpy data."""
    global _cubemars_task_active, _cubemars_task_text, _cubemars_task_detail
    with _cubemars_task_lock:
        _cubemars_task_active = True
        _cubemars_task_text = text
        _cubemars_task_detail = detail


def _cubemars_task_finish(text: str, detail: str = "") -> None:
    global _cubemars_task_active, _cubemars_task_text, _cubemars_task_detail
    with _cubemars_task_lock:
        _cubemars_task_active = False
        _cubemars_task_text = text
        _cubemars_task_detail = detail


def _cubemars_task_poll() -> tuple[bool, str, str]:
    """(task running, status text, detail lines)"""
    with _cubemars_task_lock:
        return (_cubemars_task_active, _cubemars_task_text,
                _cubemars_task_detail)


def _register_cubemars_timer() -> None:
    global _cubemars_timer_registered
    if not _cubemars_timer_registered:
        bpy.app.timers.register(_cubemars_status_tick, first_interval=0.1)
        _cubemars_timer_registered = True


_cubemars_live_timer_registered = False
_cubemars_last_live: tuple | None = None
_cubemars_last_live_print_ts: float = 0.0
_LIVE_POSE_EPS_DEG = 0.005  # ignore sub-0.005 deg float noise in q_j*

# ---------------------------------------------------------------------------
# Smooth live-target follow.
# The driver worker streams the *current* live target at 50 Hz and applies
# per-joint direction; it performs no conditioning itself. The timer that
# feeds it (below) used to push the rig pose verbatim on a 50 ms cadence, so
# a keyframed F-curve - which Blender evaluates once per *frame* - reached
# the motors as discrete per-frame jumps -> visible jitter on playback.
# We rate-limit the target here so the motor's velocity controller only ever
# sees a bounded per-tick delta (time-phased to the tick cadence), which it
# can follow smoothly. Manual dragging is unaffected: continuous pose motion
# generally stays under the slew limit, and when it exceeds it the follower
# simply catches up at the capped rate instead of snapping.
# ---------------------------------------------------------------------------
_cubemars_smooth_target: tuple | None = None   # last target pushed (rig deg)
_cubemars_smooth_at: float = 0.0               # perf_counter when it was pushed
# Default max angular slew pushed toward the measured pose per tick when no
# preference is available yet: 20 deg/s at the 50 ms timer cadence = 1.0 deg
# per tick. A hard cap keeps a full-frame jump from reaching the motor as one
# step. The panel's "Follow rate (deg/tick)" slider overrides this live.
_LIVE_MAX_SLEW_DEG_PER_TICK = 1.0


def _cubemars_slew_target(degs: tuple[float, ...], now: float,
                          slew_deg_per_tick: float = _LIVE_MAX_SLEW_DEG_PER_TICK
                          ) -> tuple[float, ...]:
    """Rate-limit the rig pose toward the last pushed target.

    Returns the target that should be pushed this tick, so the motor's
    position-velocity controller never sees a step larger than one slew tick.
    The first call (no previous target) adopts the pose immediately so the
    arm settles where the user left it, and a teleport (e.g. Set Origin,
    build_rig re-home, or a solver jump intended to be absolute) is allowed
    through on a big-enough gap and then re-synced. Pure float noise below
    _LIVE_POSE_EPS_DEG never re-pushes.
    """
    global _cubemars_smooth_target, _cubemars_smooth_at
    prev = _cubemars_smooth_target
    if prev is None:
        _cubemars_smooth_target = tuple(degs)
        _cubemars_smooth_at = now
        return tuple(degs)
    dt = max(now - _cubemars_smooth_at, 0.0)
    _cubemars_smooth_at = now
    # dt is clamped to the timer cadence so an irregular wake-up (e.g. a long
    # GC or modal pause) doesn't let the whole gap through in one step; the
    # cap is then the configured per-tick slew.
    dt = min(dt, 0.2)
    max_delta = max(slew_deg_per_tick, 0.0) * (dt / 0.05)
    out = []
    any_move = False
    for a, b in zip(prev, degs):
        d = b - a
        if abs(d) <= _LIVE_POSE_EPS_DEG:
            out.append(a)          # hold (or already-settled) target
            continue
        if abs(d) <= max_delta:
            out.append(b)          # small step: adopt directly
            any_move = any_move or True
        else:
            out.append(a + math.copysign(max_delta, d))  # slew toward pose
            any_move = True
    target = tuple(out)
    if any_move:
        _cubemars_smooth_target = target
    return target


def _cubemars_live_tick() -> float | None:
    """Blender timer for live update (50 ms cadence): push the arm's
    current joint angles into the driver's live stream, rate-limited so a
    keyframed F-curve (evaluated once per frame) never reaches the motors as
    a per-frame jump - the source of playback jitter. The actual 50 Hz CAN
    pacing happens in the driver worker; this only moves the target it
    tracks, smoothly.

    Diagnostics: the driver's status line shows 'tgt N s old' (age of
    the pushed target - it should stay near 0 while the arm moves) and
    each push is traced in the system console (rate-limited)."""
    global _cubemars_live_timer_registered, _cubemars_last_live
    global _cubemars_last_live_print_ts
    try:
        p = bpy.context.scene.pickik
        drv = _state.cubemars
        if not (p.cubemars_enabled and p.cubemars_live):
            _cubemars_live_timer_registered = False
            return None
        if drv is None:
            _cubemars_live_timer_registered = False
            return None
        if not drv.is_active:
            # The stream died (e.g. bus open failed at start): report
            # the driver's error and switch the toggle back off.
            p.cubemars_status = drv.status or "live update stopped"
            _cubemars_last_live = None
            _cubemars_live_timer_registered = False
            p.cubemars_live = False
            return None
        degs = tuple(math.degrees(getattr(p, f"q_j{i}")) for i in range(1, 8))
        now = time.time()
        # Tuning slider is read live so a change applies without a restart.
        slew = getattr(p, "cubemars_slew_deg_per_tick",
                       _LIVE_MAX_SLEW_DEG_PER_TICK)
        target = _cubemars_slew_target(degs, now, slew)
        prev = _cubemars_last_live
        if prev is None or any(abs(a - b) > _LIVE_POSE_EPS_DEG
                               for a, b in zip(prev, target)):
            _cubemars_last_live = target
            drv.update_live_targets(list(target))
            if now - _cubemars_last_live_print_ts > 0.5:
                _cubemars_last_live_print_ts = now
                print("[PickIK] CubeMars live: targets -> "
                      + " | ".join(f"J{i + 1}={v:.1f}"
                                   for i, v in enumerate(target)))
        return 0.05
    except Exception as e:
        # Never let the timer die silently - a dead timer means the
        # targets stop moving while the stream keeps sending stale ones.
        try:
            p = bpy.context.scene.pickik
            p.cubemars_status = f"live update tick error: {e}"
        except Exception:
            pass
        print("[PickIK] CubeMars: live tick error: " + str(e))
        return 0.2


def _cubemars_live_start() -> None:
    """Start the live-update stream + pose tracker (main thread)."""
    global _cubemars_live_timer_registered, _cubemars_last_live
    global _cubemars_smooth_target, _cubemars_smooth_at
    p = bpy.context.scene.pickik
    drv = _get_cubemars_driver()
    p.cubemars_detail = ""
    try:
        degs = [math.degrees(getattr(p, f"q_j{i}")) for i in range(1, 8)]
        drv.start_live_streaming(
            degs,
            speed_erpm=p.cubemars_speed_erpm,
            accel_erpm_s2=p.cubemars_accel_erpm_s2,
        )
        now = time.time()
        _cubemars_last_live = tuple(degs)
        # Adopt the current pose as the smoothed starting target so a fresh
        # start doesn't slew through the whole in-between range needlessly.
        _cubemars_smooth_target = tuple(degs)
        _cubemars_smooth_at = now
        p.cubemars_status = "live update: starting..."
        print("[PickIK] CubeMars: live update started "
              "(arm pose -> actuators)")
    except Exception as e:
        p.cubemars_status = f"live update error: {e}"
        print("[PickIK] CubeMars: live start failed: " + str(e))
        return
    _register_cubemars_timer()
    if not _cubemars_live_timer_registered:
        bpy.app.timers.register(_cubemars_live_tick, first_interval=0.05)
        _cubemars_live_timer_registered = True


def _cubemars_live_stop() -> None:
    """Stop the live-update stream and its tracker (main thread).
    Idempotent; also used when the section is disabled or unregistered."""
    global _cubemars_live_timer_registered, _cubemars_last_live
    global _cubemars_smooth_target, _cubemars_smooth_at
    drv = _state.cubemars
    if drv is not None and (drv.is_active or drv.is_live):
        drv.stop()
    _cubemars_last_live = None
    _cubemars_smooth_target = None
    _cubemars_smooth_at = 0.0
    if _cubemars_live_timer_registered:
        try:
            bpy.app.timers.unregister(_cubemars_live_tick)
        except Exception:
            pass
        _cubemars_live_timer_registered = False
    p = bpy.context.scene.pickik
    if drv is not None:
        p.cubemars_status = drv.status or "live update stopped"
        _register_cubemars_timer()
    print("[PickIK] CubeMars: live update stopped")


def _cubemars_deps() -> dict:
    """Cached python-can/gs_usb availability for the panel; re-probed
    after an in-app install (or any driver op) so the line stays fresh."""
    if _state.cubemars_deps is None:
        _state.cubemars_deps = cubemars_driver.check_dependencies()
    return _state.cubemars_deps


# Per-joint motor direction for this install (+1 = the motor's +
# rotation matches the rig joint, -1 = inverted). J1's motor is mounted
# so that the rig's + angle drives it the other way, so J1 is hardcoded
# inverted. Per-joint invert toggles in the panel are roadmap (v1.3.0);
# until then, a different physical mount should flip the sign here
# (or invert the motor in the CubeMars app). The driver applies the
# sign to every commanded position (one-shot + live) and to the
# arrival check.
CUBEMARS_MOTOR_DIRECTIONS = (-1, 1, 1, 1, 1, 1, 1)


def _get_cubemars_driver() -> cubemars_driver.CubeMarsDriver:
    """Get or create the module-level CubeMarsDriver.

    The driver (and the CAN bus it opens) is REUSED across Send/Stop /
    telemetry cycles - the adapter allows one open handle at a time, so
    re-opening on every Send is what destabilized the WinUSB device.
    A new driver is created only when the interface/channel/motor-ID
    configuration changed (the old driver's bus is disconnected first).
    """
    p = bpy.context.scene.pickik
    motor_ids = [
        p.cubemars_j1_id, p.cubemars_j2_id, p.cubemars_j3_id,
        p.cubemars_j4_id, p.cubemars_j5_id, p.cubemars_j6_id,
        p.cubemars_j7_id,
    ]
    cur = _state.cubemars
    directions = list(CUBEMARS_MOTOR_DIRECTIONS)
    # A direction change must also rebuild the driver - never stream with
    # a stale direction table. (Today the table is a module constant;
    # when the per-joint invert buttons land (roadmap v1.3.0) they will
    # feed this list from scene properties, and this same check keeps it
    # correct.)
    if cur is not None and (
            cur.is_active
            or (cur._interface == p.cubemars_interface
                and cur._channel == p.cubemars_channel
                and cur._motor_ids == motor_ids
                and cur._directions == directions)):
        return cur
    if cur is not None:
        cur.disconnect()  # config changed: release the adapter
    _state.cubemars = cubemars_driver.CubeMarsDriver(
        interface=p.cubemars_interface,
        channel=p.cubemars_channel,
        motor_ids=motor_ids,
        directions=directions,
    )
    return _state.cubemars


# ---------------------------------------------------------------------------
# Trajectory keyframe capture + export + replay
# ---------------------------------------------------------------------------
# Authoring stays in Blender: the user poses the arm manually (or via the FK
# sliders), hits "Add Keyframe" at each pose on the timeline, and Blender's
# F-curves interpolate between them. Because that interpolation is sampled as
# discrete *frames*, it is not itself jitter-free - so we re-plan the sampled
# curve as a smooth S-curve (trajectory.plan_s_curve) and replay it on the
# motor's own cadence (cubemars_driver.stream_trajectory) with per-sample
# pos/vel. The result is fully time-parameterized: no Blender frame step is
# left in the physical motion.
def _traj_frame_range(p):
    """Frame range for capture/export: scene start..end by default, else the
    p.traj_fps ... uses the playback range. Returns (f_start, f_end)."""
    if hasattr(p, "traj_fps") is False:
        pass
    sc = bpy.context.scene
    fps = getattr(sc.render, "fps", 24)
    frame_start = sc.frame_start
    frame_end = sc.frame_end
    return int(frame_start), int(frame_end), fps


class PICKIK_OT_frame_key(bpy.types.Operator):
    bl_idname = "pickik.frame_key"
    bl_label = "Add Keyframe"
    bl_description = ("Keyframe the arm's current joint angles on the timeline "
                      "at the playback head (J1..J7, radians)")

    def execute(self, context) -> set[str]:
        try:
            rig = _rig_or_die()
            p = context.scene.pickik
            c = context.scene
            frame = int(c.frame_current)
            # Refresh the props from the rig so a manual pose is captured.
            q = arm7_rig.joint_angles(rig)
            for i in range(7):
                setattr(p, f"q_j{i + 1}", q[i])
            bpy.context.view_layer.update()
            for i in range(1, 8):
                try:
                    p.keyframe_insert(data_path=f"q_j{i}", frame=frame)
                except Exception:
                    pass
            p.status = f"keyframed J1..J7 at frame {frame}"
            self.report({'INFO'}, f"Keyframed arm at frame {frame}")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_delete_frame_key(bpy.types.Operator):
    bl_idname = "pickik.delete_frame_key"
    bl_label = "Delete Keyframe"
    bl_description = "Remove the arm's joint keyframes at the playback head"

    def execute(self, context) -> set[str]:
        try:
            rig = _rig_or_die()
            p = context.scene.pickik
            frame = int(context.scene.frame_current)
            import bpy as _b
            for i in range(1, 8):
                try:
                    p.keyframe_delete(data_path=f"q_j{i}", frame=frame)
                except Exception:
                    pass
            p.status = f"removed arm keyframes at frame {frame}"
            self.report({'INFO'}, f"Removed arm keyframes at frame {frame}")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


def _sample_track(context) -> list[tuple[int, list[float]]]:
    """Read the armature's keyed q_j{i} F-curves over the playback range and
    produce [(frame, [q_deg...])]. Falls back to current-pose if no keys."""
    p = context.scene.pickik
    fr = _traj_frame_range(p)
    f_start, f_end, fps = fr[0], fr[1], fr[2]
    pts = []
    data_paths = [f"q_j{i}" for i in range(1, 8)]
    # Evaluate each q_j prop at each integer frame (its evaluated F-curves).
    for f in range(f_start, f_end + 1):
        context.scene.frame_set(f)
        bpy.context.view_layer.update()
        row = [math.degrees(getattr(p, f"q_j{i}")) for i in range(1, 8)]
        pts.append((f, row))
    if len(pts) < 2:
        raise RuntimeError("need at least two keyframed frames")
    return pts


class PICKIK_OT_export_trajectory(bpy.types.Operator):
    bl_idname = "pickik.export_trajectory"
    bl_label = "Export trajectory"
    bl_description = ("Sample the arm's keyed timeline into a smooth S-curve "
                      "and write it to JSON for deterministic replay")

    filepath: bpy.props.StringProperty(subtype="FILE_PATH", default="")
    fps: bpy.props.IntProperty(name="Sample FPS", default=60, min=5, max=500)

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context) -> set[str]:
        try:
            from . import trajectory as _traj
            pts = _sample_track(context)
            # resample the frame grid onto a fine dt (1/fps)
            times = [f[0] / self.fps for f in pts]
            joints = [f[1] for f in pts]
            dt = 1.0 / float(self.fps)
            dense = _traj.resample_curve(times, joints, dt)
            # smooth the (possibly few) captured waypoints into an S-curve
            s = _traj.plan_s_curve([list(d[1]) for d in dense], dt)
            pk = _traj.pack_samples(s)
            path = self.filepath or os.path.join(
                os.path.dirname(context.blend_data.filepath or "."),
                "pickik_trajectory.json")
            import json
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(pk, fh, indent=2)
            context.scene.pickik.status = (
                f"exported {pk['n_samples']} samples @ {self.fps} fps -> {os.path.basename(path)}")
            self.report({'INFO'}, f"Exported {pk['n_samples']} samples")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_play_trajectory(bpy.types.Operator):
    bl_idname = "pickik.play_trajectory"
    bl_label = "Play smooth trajectory"
    bl_description = ("Re-plan the keyed arm motion as a smooth S-curve and "
                      "stream it to the attached motors with per-sample pos/vel")

    def execute(self, context) -> set[str]:
        try:
            from . import trajectory as _traj
            drv = _get_cubemars_driver()
            pts = _sample_track(context)
            dt = 0.01  # 100 Hz output
            s = _traj.plan_s_curve([list(d[1]) for d in pts], dt)
            pk = _traj.pack_samples(s)
            if drv is None or not drv._active_idx:
                raise RuntimeError("No active actuators configured")
            drv.stream_trajectory(pk, send_hz=100.0)
            context.scene.pickik.status = ("playing %d-sample smooth trajectory "
                                           "(%d motors)" % (pk["n_samples"], len(drv._active_idx)))
            self.report({'INFO'}, f"Playing {pk['n_samples']}-sample smooth trajectory")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_send_to_cubemars(bpy.types.Operator):
    bl_idname = "pickik.send_to_cubemars"
    bl_label = "Send positions to actuators"
    bl_description = ("Stream the current J1..J7 joint angles to the connected "
                      "CubeMars actuators via CAN")

    def execute(self, context) -> set[str]:
        try:
            p = context.scene.pickik
            if not p.cubemars_enabled:
                _status("CubeMars is disabled — enable it first")
                self.report({'WARNING'}, "CubeMars is disabled")
                return {'CANCELLED'}

            rig = _rig_or_die()
            bpy.context.view_layer.update()

            # Read current joint angles (radians -> degrees).
            q_rad = arm7_rig.joint_angles(rig)
            targets_deg = [math.degrees(q) for q in q_rad]

            driver = _get_cubemars_driver()
            if driver.is_live and driver.is_active:
                # Live update is running: retarget the live stream
                # instead of starting a second (burst) stream.
                driver.update_live_targets(targets_deg)
                _register_cubemars_timer()
                p.cubemars_status = "live update: target updated"
                p.cubemars_detail = ""
                print("[PickIK] CubeMars: Send routed to the live stream")
                return {'FINISHED'}
            driver.stream_to_targets(
                targets_deg=targets_deg,
                speed_erpm=p.cubemars_speed_erpm,
                accel_erpm_s2=p.cubemars_accel_erpm_s2,
            )
            _register_cubemars_timer()
            p.cubemars_status = "sending..."
            p.cubemars_detail = ""  # clear the previous task detail
            qdeg = ", ".join(f"{v:.1f}" for v in targets_deg[:2])
            _status(f"CubeMars: streaming J1={targets_deg[0]:.1f}°, "
                    f"J2={targets_deg[1]:.1f}° to actuators")
            return {'FINISHED'}
        except RuntimeError as e:
            _status(f"CubeMars: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        except Exception as e:
            _status(f"CubeMars error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_stop_cubemars(bpy.types.Operator):
    bl_idname = "pickik.stop_cubemars"
    bl_label = "Stop"
    bl_description = "Stop streaming to the CubeMars actuators and disable motors"

    def execute(self, context) -> set[str]:
        try:
            drv = _state.cubemars
            if drv is None or not drv.is_active:
                _status("CubeMars: not streaming")
                return {'CANCELLED'}
            # stop() now also joins the streaming thread (usually < 0.6 s)
            # and disables the motors; the CAN bus itself stays open
            # (persistent bus - see the driver docstring).
            was_live = drv.is_live
            drv.stop()
            p = bpy.context.scene.pickik
            p.cubemars_status = drv.status or "stopped"
            if was_live:
                p.cubemars_live = False  # -> _cubemars_live_stop (idempotent)
            _register_cubemars_timer()
            print("[PickIK] CubeMars stop: " + drv.status)
            return {'FINISHED'}
        except Exception as e:
            _status(f"CubeMars stop error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_cubemars_install_deps(bpy.types.Operator):
    bl_idname = "pickik.cubemars_install_deps"
    bl_label = "Install deps"
    bl_description = ("Install python-can + gs_usb into this Blender's Python "
                      "(background pip; no restart needed on success)")

    def execute(self, context) -> set[str]:
        try:
            if _cubemars_task_poll()[0]:
                print("[PickIK] CubeMars: task already running, install "
                      "request ignored")
                self.report({'WARNING'},
                            "A CubeMars task is already running")
                return {'CANCELLED'}
            progress = ("installing python-can + gs_usb "
                        "(pip, can take a minute)...")
            _state.cubemars_deps = cubemars_driver.check_dependencies()
            _cubemars_task_start(progress)
            context.scene.pickik.cubemars_status = progress  # instant UI
            _register_cubemars_timer()
            print("[PickIK] CubeMars: pip install python-can gs_usb into "
                  f"{cubemars_driver._DEP_DIR} via "
                  f"{cubemars_driver.sys.executable} (background thread)")

            def _run() -> None:
                print("[PickIK] CubeMars: pip install started")
                try:
                    msg = cubemars_driver.install_dependencies()
                    # Re-probe so the panel's dep line reflects the install.
                    _state.cubemars_deps = cubemars_driver.check_dependencies()
                except Exception:
                    import traceback
                    traceback.print_exc()
                    _cubemars_task_finish("ERROR: install thread crashed "
                                          "(see console)")
                    return
                _cubemars_task_finish(msg)
                print("[PickIK] CubeMars install result:\n" + msg)

            threading.Thread(target=_run, daemon=True).start()
            return {'FINISHED'}
        except Exception as e:
            _status(f"CubeMars install error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_cubemars_check_driver(bpy.types.Operator):
    bl_idname = "pickik.cubemars_check_driver"
    bl_label = "Check driver"
    bl_description = ("Open the CAN adapter once to verify the OS driver + "
                      "USB connection (safe: sends no motor commands)")

    def execute(self, context) -> set[str]:
        try:
            if _cubemars_task_poll()[0]:
                print("[PickIK] CubeMars: task already running, driver "
                      "check ignored")
                self.report({'WARNING'},
                            "A CubeMars task is already running")
                return {'CANCELLED'}
            progress = "checking CAN driver (opening the adapter once)..."
            _state.cubemars_deps = cubemars_driver.check_dependencies()
            drv = _get_cubemars_driver()  # main thread: reads bpy context
            _cubemars_task_start(progress)
            context.scene.pickik.cubemars_status = progress  # instant UI
            _register_cubemars_timer()
            print("[PickIK] CubeMars: opening "
                  f"{drv._interface} ch{drv._channel} @ {drv._bitrate} bit/s "
                  "to verify the driver (no motor commands)")

            def _run() -> None:
                try:
                    _ok, msg, detail = drv.check_driver()
                except Exception:
                    import traceback
                    traceback.print_exc()
                    _cubemars_task_finish("ERROR: driver-check thread "
                                          "crashed (see console)")
                    return
                _cubemars_task_finish(msg, detail)
                print("[PickIK] CubeMars driver check result: " + msg)
                if detail:
                    print("[PickIK] CubeMars driver check detail:\n"
                          + detail)

            threading.Thread(target=_run, daemon=True).start()
            return {'FINISHED'}
        except Exception as e:
            _status(f"CubeMars driver-check error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_cubemars_read_telemetry(bpy.types.Operator):
    bl_idname = "pickik.cubemars_read_telemetry"
    bl_label = "Read telemetry"
    bl_description = ("Listen to the CAN bus for ~2 s and show the motors' "
                      "live periodic feedback (position / speed / current "
                      "/ temperature / error). Sends no motor commands - "
                      "safe with or without the actuators attached.")

    def execute(self, context) -> set[str]:
        try:
            if _cubemars_task_poll()[0]:
                print("[PickIK] CubeMars: task already running, telemetry "
                      "request ignored")
                self.report({'WARNING'},
                             "A CubeMars task is already running")
                return {'CANCELLED'}
            progress = "reading motor telemetry (~2 s) ..."
            _state.cubemars_deps = cubemars_driver.check_dependencies()
            drv = _get_cubemars_driver()
            _cubemars_task_start(progress)
            context.scene.pickik.cubemars_status = progress
            context.scene.pickik.cubemars_detail = ""
            _register_cubemars_timer()
            print("[PickIK] CubeMars: reading telemetry for 2 s "
                  "(no motor commands)")

            def _run() -> None:
                try:
                    res = drv.read_telemetry(seconds=2.0)
                except Exception:
                    import traceback
                    traceback.print_exc()
                    _cubemars_task_finish("ERROR: telemetry thread crashed "
                                          "(see console)")
                    return
                _cubemars_task_finish(res["text"],
                                      "\n".join(res["lines"]))
                print("[PickIK] CubeMars telemetry: " + res["text"])
                for line in res["lines"]:
                    print("[PickIK]   " + line)

            threading.Thread(target=_run, daemon=True).start()
            return {'FINISHED'}
        except Exception as e:
            _status(f"CubeMars telemetry error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_cubemars_set_zero(bpy.types.Operator):
    bl_idname = "pickik.cubemars_set_zero"
    bl_label = "Set zero position"
    bl_description = (
        "Sends the CubeMars 'Set Origin' command (mode 5): each active "
        "motor treats its CURRENT position as 0 deg. The arm must be at "
        "the physical zero pose - press Stop first, move the arm by "
        "hand to the physical limit, then click. Nothing moves: the "
        "command only re-references the encoder, and the panel confirms "
        "the result per motor. The zero is NOT stored in the motors - "
        "re-run after every power-up.")

    def execute(self, context) -> set[str]:
        try:
            if _cubemars_task_poll()[0]:
                print("[PickIK] CubeMars: task already running, set-zero "
                      "request ignored")
                self.report({'WARNING'},
                             "A CubeMars task is already running")
                return {'CANCELLED'}
            progress = ("setting zero position (feedback sample + "
                       "'Set Origin' frame) ...")
            _state.cubemars_deps = cubemars_driver.check_dependencies()
            drv = _get_cubemars_driver()
            _cubemars_task_start(progress)
            context.scene.pickik.cubemars_status = progress
            context.scene.pickik.cubemars_detail = ""
            _register_cubemars_timer()
            print("[PickIK] CubeMars: setting the zero position "
                  "(mode 5 'Set Origin', 8 zero bytes, per active motor)")

            def _run() -> None:
                try:
                    res = drv.set_origin()
                except Exception:
                    import traceback
                    traceback.print_exc()
                    _cubemars_task_finish("ERROR: set-zero thread crashed "
                                          "(see console)")
                    return
                _cubemars_task_finish(res["text"], "\n".join(res["lines"]))
                print("[PickIK] CubeMars set origin: " + res["text"])
                for line in res["lines"]:
                    print("[PickIK]   " + line)

            threading.Thread(target=_run, daemon=True).start()
            return {'FINISHED'}
        except Exception as e:
            _status(f"CubeMars set-zero error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_cubemars_disconnect(bpy.types.Operator):
    bl_idname = "pickik.cubemars_disconnect"
    bl_label = "Disconnect adapter"
    bl_description = ("Stop streaming and close the CAN bus, releasing the "
                      "adapter (recover a stuck adapter, or do this before "
                      "changing interface/channel). Send re-opens on demand.")

    def execute(self, context) -> set[str]:
        try:
            if _cubemars_task_poll()[0]:
                print("[PickIK] CubeMars: task already running, disconnect "
                      "request ignored")
                self.report({'WARNING'},
                             "A CubeMars task is already running")
                return {'CANCELLED'}
            drv = _state.cubemars
            if drv is None:
                _status("CubeMars: nothing to disconnect")
                return {'FINISHED'}
            _cubemars_task_start("disconnecting adapter ...")
            context.scene.pickik.cubemars_status = "disconnecting adapter ..."
            context.scene.pickik.cubemars_detail = ""
            _register_cubemars_timer()
            print("[PickIK] CubeMars: disconnecting the CAN adapter")

            def _run() -> None:
                try:
                    drv.disconnect()
                except Exception:
                    import traceback
                    traceback.print_exc()
                    _cubemars_task_finish("ERROR: disconnect thread crashed "
                                          "(see console)")
                    return
                _cubemars_task_finish(
                    "Adapter disconnected - bus closed, handle released")
                print("[PickIK] CubeMars: adapter disconnected")

            threading.Thread(target=_run, daemon=True).start()
            return {'FINISHED'}
        except Exception as e:
            _status(f"CubeMars disconnect error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}


class PICKIK_OT_save_urdf(bpy.types.Operator):
    bl_idname = "pickik.save_urdf"
    bl_label = "Save URDF"
    bl_description = ("Export the rig as a URDF + meter-unit STL meshes "
                      "(viewer-correct scale and placement)")

    directory: bpy.props.StringProperty(subtype="DIR_PATH",
                                          default="",
                                          description="Folder to save the URDF and STL files into")

    def execute(self, context) -> set[str]:
        try:
            rig = _rig_or_die()
            if rig is None:
                self.report({'ERROR'}, "Build the rig first")
                return {'CANCELLED'}
            bpy.context.view_layer.update()

            meshes_dir = _resolve_meshes_dir(context.scene.pickik.meshes_path)

            # Output directory (DIR_PATH: the user picks a folder).
            out_dir = self.directory.strip()
            if not out_dir:
                out_dir = os.path.dirname(meshes_dir)
            os.makedirs(out_dir, exist_ok=True)
            stl_dir = os.path.join(out_dir, "meshes")
            os.makedirs(stl_dir, exist_ok=True)

            # A viewer-correct description needs two things:
            #  1. meter-unit meshes — viewers read STL coordinates as
            #     meters, but the source files on disk are often mm
            #     (Fusion CAD exports); each file is re-written in meters;
            #  2. an exact <origin> from the STL file frame to the URDF
            #     link frame — the link's +/-90deg roll and the STL's own
            #     origin matter (a bbox-center/rpy=0 approximation does
            #     not place the geometry where the rig shows it).
            # The link->mesh transform is pose-independent (the mesh is
            # rigidly attached to its link: mo.matrix_world is always
            # parent.matrix_world @ C for the constant C), so the export
            # never touches the user's scene pose.
            mesh_map = {}
            for mo in rig.mesh_objects:
                parent = mo.parent.name if mo.parent else ""
                link_idx = None
                if parent == "Arm7_Base":
                    link_idx = 0
                elif parent.startswith("Arm7_J"):
                    link_idx = int(parent[6:])
                elif parent == "Arm7_Tool0":
                    link_idx = 8
                if link_idx is None:
                    continue
                stl_name = mo.name.replace("Arm7_Mesh_", "") + ".stl"
                src = os.path.join(meshes_dir, stl_name)
                if not os.path.isfile(src):
                    continue
                arm7_rig.export_stl_as_meters(src, os.path.join(stl_dir, stl_name))
                # STL file frame -> URDF link frame. The parent empty's
                # world matrix IS the link frame (arm7_rig FK convention:
                # child = parent * T(origin) * R(roll) * Rz(q)), and the
                # mesh object's world matrix maps its STL frame into the
                # world — the difference is exactly the rigid transform
                # the URDF <origin> must carry.
                t = (mo.parent.matrix_world.inverted()
                     @ mo.matrix_world)
                mesh_map[link_idx] = {
                    "stl": stl_name,
                    "xyz": t.to_translation(),
                    "quat": t.to_quaternion(),
                }

            lines = []
            lines.append('<?xml version="1.0"?>')
            lines.append('<robot name="arm7">')
            lines.append('')
            lines.append('  <!-- URDF exported from blender_ik_addon (Save URDF). -->')
            lines.append('  <!-- Joints match the solver kinematic chain; meshes are -->')
            lines.append('  <!-- in meters in meshes/ next to this file, each -->')
            lines.append('  <!-- <visual><origin> placing the file frame in the link. -->')
            lines.append('  <!-- FK convention (identical to the solver and the rig): -->')
            lines.append('  <!--   child = parent * T(origin.xyz) * R(roll about X) * Rz(q) -->')
            lines.append('  <!-- i.e. every revolute joint rotates about its local Z. The -->')
            lines.append('  <!-- +/-90deg roll on joint2/4/6 puts the horizontal pitch axis -->')
            lines.append('  <!-- onto local Z, so link2/4/6 are intentionally NOT Z-up at -->')
            lines.append('  <!-- rest (their +Z is the pitch axis, pointing world +Y); all -->')
            lines.append('  <!-- other links are Z-up. Standard URDF pattern for pitch -->')
            lines.append('  <!-- joints (cf. the xArm7 robot description). -->')
            lines.append('')

            link_names = [f"link{i}" for i in range(1, 8)]
            link_names.insert(0, "base_link")
            link_names.append("tool_link")

            # Joint table — single source of truth in arm7_rig.
            joint_origins = [(o.x, o.y, o.z) for o, _ in arm7_rig.JOINTS]
            joint_rolls = [r for _, r in arm7_rig.JOINTS]

            def _visual(m):
                x = m["xyz"]
                # Write rpy, not quaternion: R = Rz(yaw) Ry(pitch) Rx(roll)
                # is the exact order of mathutils' 'XYZ' Euler, and rpy is
                # the attribute every minimal URDF parser implements — a
                # parser that ignores quaternion (e.g. ik-service's web
                # viewer) would otherwise drop the mesh orientation and
                # render the part at its translation with identity rotation.
                r, p, y = m["quat"].to_euler("XYZ")
                return [
                    '    <visual>',
                    f'      <origin xyz="{x[0]:.6f} {x[1]:.6f} {x[2]:.6f}" '
                    f'rpy="{r:.6f} {p:.6f} {y:.6f}"/>',
                    '      <geometry>',
                    f'        <mesh filename="meshes/{m["stl"]}" scale="1 1 1"/>',
                    '      </geometry>',
                    '    </visual>',
                ]

            lines.append('  <link name="base_link">')
            if 0 in mesh_map:
                lines.extend(_visual(mesh_map[0]))
            lines.append('  </link>')
            lines.append('')

            for i in range(7):
                roll = joint_rolls[i]
                ox, oy, oz = joint_origins[i]
                lines.append(f'  <joint name="joint{i + 1}" type="revolute">')
                lines.append(f'    <parent link="{link_names[i]}"/>')
                lines.append(f'    <child link="{link_names[i + 1]}"/>')
                lines.append(f'    <origin xyz="{ox:.6f} {oy:.6f} {oz:.6f}" rpy="{roll:.6f} 0 0"/>')
                lines.append(f'    <axis xyz="0 0 1"/>')
                lines.append(f'    <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1"/>')
                lines.append(f'  </joint>')
                lines.append('')
                lines.append(f'  <link name="{link_names[i + 1]}">')
                if i + 1 in mesh_map:
                    lines.extend(_visual(mesh_map[i + 1]))
                lines.append(f'  </link>')
                lines.append('')

            lines.append('  <joint name="tool_offset" type="fixed">')
            lines.append('    <parent link="link7"/>')
            lines.append('    <child link="tool_link"/>')
            lines.append(f'    <origin xyz="0 0 {arm7_rig.TOOL_OFFSET:.6f}" rpy="0 0 0"/>')
            lines.append('  </joint>')
            lines.append('')
            lines.append('  <link name="tool_link">')
            if 8 in mesh_map:
                lines.extend(_visual(mesh_map[8]))
            lines.append('  </link>')
            lines.append('')
            lines.append('</robot>')

            filepath = os.path.join(out_dir, "arm7.urdf")
            with open(filepath, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))

            _status(f"URDF saved to {filepath}")
            return {'FINISHED'}
        except Exception as e:
            _status(f"error: {e}")
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


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

        # -- CubeMars Actuators section (motor control = main feature,
        # so it leads the panel) -------------------------------------------
        # Note: label(icon=...) in N-panels only accepts icon ENUMS, not
        # image paths (verified on 3.4.1: passing cube_mars_logo.png
        # raised TypeError mid-draw), so the banner uses the DRIVER enum
        # icon. The bundled cube_mars_logo.png stays in the repo for a
        # future custom-drawn banner / asset use.
        box = layout.box()
        box.label(text="CubeMars — AK-series actuators", icon='DRIVER')
        box.prop(p, "cubemars_enabled")
        deps = _cubemars_deps()
        if deps["ready"]:
            box.label(text=(f"CAN deps found: python-can {deps['can_version']}"
                            " + gs_usb"), icon='CHECKMARK')
        else:
            missing = [n for n, k in (("python-can", "can"),
                                      ("gs_usb", "gs_usb")) if not deps[k]]
            box.label(text=("CAN deps missing: " + ", ".join(missing)
                            + " — click 'Install deps'"), icon='ERROR')
        row = box.row()
        row.operator("pickik.cubemars_install_deps",
                     text="Install deps", icon='SCRIPT')
        row.operator("pickik.cubemars_check_driver",
                     text="Check driver", icon='DRIVER')
        row = box.row()
        row.operator("pickik.cubemars_read_telemetry",
                     text="Read telemetry", icon='INFO')
        row.operator("pickik.cubemars_disconnect",
                     text="Disconnect adapter", icon='UNLINKED')
        if p.cubemars_enabled:
            row = box.row()
            row.prop(p, "cubemars_interface", text="Interface")
            row.prop(p, "cubemars_channel", text="Ch")
            row = box.row()
            row.prop(p, "cubemars_j1_id", text="J1 ID")
            row.prop(p, "cubemars_j2_id", text="J2 ID")
            row = box.row()
            row.prop(p, "cubemars_speed_erpm", text="Speed")
            row.prop(p, "cubemars_accel_erpm_s2", text="Accel")
            row = box.row()
            row.prop(p, "cubemars_live", text="Live update (arm pose -> motors)")
            row.prop(p, "cubemars_slew_deg_per_tick", text="Follow")
            row = box.row()
            row.operator("pickik.cubemars_set_zero",
                         text="Set zero position", icon='DRIVER')
            box.label(text="(arm at the physical zero pose + stream "
                           "Stopped; re-run after each power-up)")
            row = box.row()
            row.operator("pickik.send_to_cubemars",
                         text="Send positions to actuators",
                         icon='MESH_DATA')
            row.operator("pickik.stop_cubemars",
                         text="Stop", icon='X')
        # Task feedback (install deps / check driver / streaming) must be
        # visible even when the section is OFF — that is exactly when the
        # user clicks the buttons to fix the setup.
        if p.cubemars_status:
            box.label(text=p.cubemars_status, icon='INFO')
        # Multi-line detail (telemetry readout / driver-check diagnostics)
        # persists on the panel after the task finishes.
        for _line in (p.cubemars_detail or "").splitlines():
            if _line:
                box.label(text=_line)

        box = layout.box()
        box.prop(p, "dll_path", text="")
        box.prop(p, "meshes_path", text="")
        row = box.row()
        row.operator("pickik.build_rig", text="Build rig")
        row.operator("pickik.save_urdf", text="Save URDF")
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

        # -- Trajectory keyframe capture + smooth replay ---------------------
        # Author a camera move by keyframing the arm on the timeline, then
        # export/play it as a smooth S-curve (no Blender-frame jitter in the
        # physical motion).
        box = layout.box()
        box.label(text="Trajectory (smooth camera move)", icon='TIME')
        row = box.row()
        row.operator("pickik.frame_key", text="Add Keyframe", icon='KEYINGSET')
        row.operator("pickik.delete_frame_key", text="Delete Keyframe", icon='X')
        row = box.row()
        row.label(text="Sample FPS")
        row.prop(p, "traj_fps", text="")
        row = box.row()
        row.operator("pickik.export_trajectory", text="Export trajectory", icon='FILE_TICK')
        row.operator("pickik.play_trajectory", text="Play smooth", icon='PLAY')

        # -- MCP bridge: what an agent may reach over the wire, right now (plan §8). Read-out only:
        # the token is never drawn, never logged, never copied into a field that could be saved.
        box = layout.box()
        box.label(text="MCP bridge (agent over the wire)", icon='SCRIPT')
        srv = mcp_bridge.get()
        if srv is None:
            box.label(text="not running — an agent cannot reach this session", icon='X')
            if _MCP_START_STATE["why"]:
                # The reason the last Start did not start. Before this line a failed start was
                # entirely silent here: the message went to scene.pickik.status, a field shared with
                # the solver and the continuous drive and therefore not displayable in this box, and
                # this arm drew neither that nor the server's own last_error (which lives in the else
                # arm, unreachable precisely when nothing ever started). So the button did nothing,
                # visibly, and the port stayed shut with no reason printed anywhere on screen.
                box.label(text=_MCP_START_STATE["why"][:88], icon='ERROR')
            row = box.row()
            row.operator("pickik.mcp_start", text="Start", icon='CHECKMARK')
        else:
            st = srv.status_dict()
            box.label(text=(f"listening on {st['host']}:{st['port']} · client "
                           f"{'yes' if st['client'] else 'no'} · queue {st['queued']}"),
                      icon='CHECKMARK')
            box.label(text=("auth: INSECURE, no token asked" if st["insecure_no_auth"]
                            else "auth: token required (generated, not shown here)"))
            box.label(text=f"runtime file {srv.runtime_path or '(not written)'}")
            if st["last_error"]:
                box.label(text=f"last error: {st['last_error'][:72]}", icon='ERROR')
            row = box.row()
            row.operator("pickik.mcp_stop", text="Stop", icon='X')
        pr = _mcp_prefs(context)
        hw = _mcp_settings(context)["hardware_enabled"]
        if pr is not None:
            box.prop(pr, "enable_mcp_bridge")
            if pr.enable_mcp_bridge:
                row = box.row()
                row.prop(pr, "mcp_port", text="Port")
                row.prop(pr, "mcp_start_on_load", text="Start on load")
                box.prop(pr, "mcp_insecure_no_auth", text="INSECURE: no auth")
                box.prop(pr, "mcp_hardware_enabled", text="Hardware commands")
                box.prop(pr, "mcp_multi_client", text="Multi-client (unsafe: concurrent agents)")
        else:
            box.label(text="preferences unavailable in this session: the bridge runs on defaults")
        box.label(text=("hardware UNLOCKED — the agent can move the physical arm" if hw
                        else "hardware locked — the agent cannot move the arm"),
                  icon='ERROR' if hw else 'INFO')


# ---------------------------------------------------------------------------
# The MCP bridge: preferences, start/stop, panel section
# (MCP_INTEGRATION_PLAN.md §8; the wire itself is mcp_bridge.py)
# ---------------------------------------------------------------------------

MCP_DEFAULTS = {"host": "127.0.0.1", "port": 9876, "token": "",
                "runtime_file": "~/.pickik/bridge.json", "export_root": "~/pickik/export",
                "insecure_no_auth": False, "hardware_enabled": False, "multi_client": False}


class PICKIK_PG_preferences(bpy.types.AddonPreferences):
    """Add-ons, PickIK arm7 (native C ABI) — how the agent may reach Blender.

    A port, a token, and the permission to move the physical arm belong to the person at the
    machine, not to the open .blend file: they must not travel inside a scene, and must not be
    silently absent when a colleague opens one that has them."""

    # The id Blender uses to bind an AddonPreferences subclass to its add-on block.
    #
    # Measured on both builds this add-on ships for (3.4.1 and 4.5.3), by re-registering this one
    # class under each candidate and reading back what the block hands out:
    #   "blender_ik_addon"                 -> .preferences is a PICKIK_PG_preferences  -- BINDS
    #   "USERPREF_BLENDER_IK_ADDON"        -> .preferences is NoneType                -- dead
    #   "USERPREF_addon_blender_ik_addon"  -> .preferences is NoneType                -- dead
    # A dead id is not cosmetic: with .preferences None, `_mcp_prefs` returns None, the panel's
    # `if pr is not None:` branch never runs, and the MCP box draws its fallback line with no tick,
    # no Port and no hardware gate in it. bool_tool, which works, ships `bl_idname = __package__`.
    bl_idname = __package__ or __name__
    bl_label = "PickIK arm7 (native C ABI)"
    bl_category = "PREFERENCES"

    enable_mcp_bridge: BoolProperty(
        name="MCP bridge", default=False,
        description="Permit the bridge to be started at all")
    mcp_start_on_load: BoolProperty(
        name="Start on load", default=False,
        description="Bind the socket when the add-on registers. Never in a background instance — "
                    "registering the add-on does not open a port by itself")
    mcp_port: IntProperty(
        name="Port", default=MCP_DEFAULTS["port"], min=0, max=65535,
        description="0 lets the operating system choose a free port; whichever port was bound is "
                    "the one written to the runtime file")
    mcp_auth_token: StringProperty(
        name="Auth token", default="", subtype='NONE',
        description="Leave empty and a strong token is generated for you; it is published in the "
                    "runtime file and never written to a log or the console")
    mcp_runtime_file: StringProperty(
        name="Runtime file", default=MCP_DEFAULTS["runtime_file"], subtype='FILE_PATH',
        description="Where the bound port and the token are published so the MCP server can find "
                    "the bridge")
    mcp_export_root: StringProperty(
        name="Export root", default=MCP_DEFAULTS["export_root"], subtype='DIR_PATH',
        description="export_urdf may write only inside this tree: an agent cannot be handed an "
                    "arbitrary file-system path")
    mcp_hardware_enabled: BoolProperty(
        name="Hardware commands", default=False,
        description="Unlock the hw_* group, which moves the physical arm. Every such call still "
                    "requires the confirm phrase, and the e-stop is never gated")
    mcp_insecure_no_auth: BoolProperty(
        name="INSECURE: no auth", default=False,
        description="Debug only: accept connections without a token. Cannot be combined with the "
                    "hardware group, and must never face a machine that is powered")
    mcp_multi_client: BoolProperty(
        name="Multi-client", default=False,
        description="Inject for tests, bench and multi-agent handoffs: do not refuse a second "
                    "concurrent agent, and lift the single operational seat. Safety-critical "
                    "default, and never on while the physical panel is being watched")

    def draw(self, context):
        """The MCP knobs in `Edit > Preferences > Add-ons`, as well as in the 3D-view box.

        Until this method existed the class had none, so the preferences page drew nothing at all
        for this add-on and the eight properties below were reachable only through the N-panel. The
        drawing mirrors `PICKIK_PT_main` rather than inventing a second arrangement: the permission
        first, and everything that depends on it dimmed until it is given."""
        layout = self.layout
        col = layout.column(align=True)
        col.prop(self, "enable_mcp_bridge")

        sub = col.column(align=True)
        sub.active = bool(self.enable_mcp_bridge)
        sub.prop(self, "mcp_port")
        sub.prop(self, "mcp_start_on_load")
        sub.prop(self, "mcp_runtime_file")
        sub.prop(self, "mcp_export_root")

        gate = col.column(align=True)
        gate.active = bool(self.enable_mcp_bridge)
        gate.label(text="Security and the physical arm", icon='LOCKED')
        gate.prop(self, "mcp_auth_token")
        gate.prop(self, "mcp_insecure_no_auth", text="INSECURE: no auth")
        gate.prop(self, "mcp_hardware_enabled", text="Hardware commands")
        gate.prop(self, "mcp_multi_client", text="Multi-client (unsafe: concurrent agents)")


def _mcp_headless() -> bool:
    """True in a background instance (`blender --background`, the CI and test case)."""
    return bool(getattr(getattr(bpy, "app", None), "background", False))


def _mcp_prefs(context):
    """This add-on's own preference block, or None when this build has none to hand out.

    Measured rather than assumed: under `--factory-startup` — which is how the test suite and any
    hand-driven `register()` arrive — `preferences.addons` does not know this add-on exists, so no
    call site may treat the block as certain. None is a normal answer, not a failure."""
    root = getattr(getattr(context, "preferences", None), "addons", None)
    if root is None:
        return None
    home = __package__ or __name__
    for key in (home, home + ".py"):
        try:
            block = root[key]
        except (KeyError, TypeError):
            continue
        prefs = getattr(block, "preferences", None)
        if prefs is not None and getattr(prefs, "mcp_port", None) is not None:
            return prefs
    return None


def _mcp_settings(context) -> dict:
    """What the bridge will really run on: the preferences where they exist, the defaults where they
    do not. The token is returned as typed (usually empty) — the bridge generates one and the panel
    must never print it, so nothing here echoes the value back."""
    got = dict(MCP_DEFAULTS)
    pr = _mcp_prefs(context)
    if pr is not None:
        got.update(port = int(pr.mcp_port), token = (pr.mcp_auth_token or "").strip(),
                    runtime_file = (pr.mcp_runtime_file or "").strip() or MCP_DEFAULTS["runtime_file"],
                    export_root = (pr.mcp_export_root or "").strip() or MCP_DEFAULTS["export_root"],
                    hardware_enabled = bool(pr.mcp_hardware_enabled),
                    insecure_no_auth = bool(pr.mcp_insecure_no_auth),
                    multi_client = bool(pr.mcp_multi_client))
    return got


class PICKIK_OT_mcp_start(bpy.types.Operator):
    """Open the loopback socket and let one agent drive this session."""
    bl_idname = "pickik.mcp_start"
    bl_label = "Start the MCP bridge"
    bl_description = ("Listen on loopback for MCP requests. Starting it moves nothing: the hardware "
                      "group stays locked until it is unlocked on purpose")
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context) -> bool:
        pr = _mcp_prefs(context)
        return mcp_bridge.get() is None and (pr is None or pr.enable_mcp_bridge)

    def execute(self, context) -> set:
        srv = _mcp_begin_start(context)
        if srv is None:
            return {'CANCELLED'}
        p = context.scene.pickik
        p.status = (f"MCP bridge on {srv.host}:{srv.bound_port} · auth "
                    f"{'NONE (insecure)' if srv.insecure_no_auth else 'required'} · "
                    f"runtime {srv.runtime_path or 'not written'}")
        return {'FINISHED'}


class PICKIK_OT_mcp_stop(bpy.types.Operator):
    """Close the socket and drop the agent's session."""
    bl_idname = "pickik.mcp_stop"
    bl_label = "Stop the MCP bridge"
    bl_description = "Stop answering MCP requests and release the port; nothing is left listening"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context) -> bool:
        return mcp_bridge.get() is not None

    def execute(self, context) -> set:
        mcp_bridge.stop()
        context.scene.pickik.status = "MCP bridge stopped; the port is released"
        return {'FINISHED'}


#: Why the last Start attempt did not start, for the panel's "not running" arm to show. Kept apart
#: from scene.pickik.status on purpose: that field is the shared line the solver, the continuous
#: drive and the loader all write to, so displaying it here would put "no solution (pos err 41 mm)"
#: inside the security box. Written only by _mcp_begin_start below.
_MCP_START_STATE = {"why": ""}


def _mcp_begin_start(context):
    """Attempt to start the bridge, keeping the reason here if the attempt fails.

    One recording site rather than three, so that nothing in `_mcp_start_from_prefs` has to be
    disturbed to make the failure visible: that function already reports every reason it knows into
    scene.pickik.status, and this copies an MCP-shaped answer out of it at the one point that also
    knows whether the attempt succeeded. A fresh attempt clears the last verdict first, which is
    why a stale complaint cannot sit under the box after a Stop that went through fine.
    """
    _MCP_START_STATE["why"] = ""
    srv = _mcp_start_from_prefs(context)
    if srv is None:
        why = ""
        try:
            why = context.scene.pickik.status or ""
        except AttributeError:                                # no scene: nothing was reported
            why = ""
        if why.startswith("MCP bridge refused") or why.startswith("MCP bridge failed"):
            _MCP_START_STATE["why"] = why
    return srv


def _mcp_start_from_prefs(context):
    """Start the bridge from the preferences. Returns the server, or None after reporting why not.

    The pair `insecure_no_auth` + hardware is refused here as well as inside the bridge: an
    unauthenticated socket is acceptable only while nothing that can move an arm is reachable through
    it, and a preference that was set on another day must not quietly re-arm it (§9)."""
    srv = mcp_bridge.get()
    if srv is not None:
        return srv
    st = _mcp_settings(context)
    if st["insecure_no_auth"] and st["hardware_enabled"]:
        try:
            context.window_manager.prompt(
                "The MCP bridge refuses to run unauthenticated while the hardware group is enabled")
        except (AttributeError, TypeError):                 # headless: the status line is the report
            pass
        context.scene.pickik.status = ("MCP bridge refused to start: no-auth cannot be combined "
                                        "with hardware commands")
        return None
    if st["hardware_enabled"] and not any(k.startswith("hw_") for k in mcp_handlers_obs.HANDLERS):
        # Honest about the build, not about the intent: the gate exists, the handlers are Phase 3.
        print("[pickik-mcp] hardware commands requested but this build registers no hw_* handler "
              "yet (Phase 3); the agent will be told the commands do not exist")
    try:
        return mcp_bridge.start(
            host = st["host"], port = st["port"], token = st["token"],
            insecure_no_auth = st["insecure_no_auth"], export_root = st["export_root"],
            multi_client = st["multi_client"],
            runtime_file = os.path.expanduser(st["runtime_file"]),
            handlers = dict(mcp_handlers_obs.HANDLERS))
    except BaseException as exc:
        # A taken port is reported, never papered over by silently sliding to another one (§4.1).
        context.scene.pickik.status = f"MCP bridge failed to start: {exc}"
        return None


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

CLASSES = (PickIKProps, PICKIK_OT_build_rig, PICKIK_OT_solve,
           PICKIK_OT_toggle_continuous, PICKIK_OT_apply_fk, PICKIK_OT_sync_fk,
           PICKIK_OT_frame_key, PICKIK_OT_delete_frame_key,
           PICKIK_OT_export_trajectory, PICKIK_OT_play_trajectory,
           PICKIK_OT_send_to_cubemars, PICKIK_OT_stop_cubemars,
           PICKIK_OT_cubemars_read_telemetry,
           PICKIK_OT_cubemars_set_zero,
           PICKIK_OT_cubemars_disconnect,
           PICKIK_OT_cubemars_install_deps, PICKIK_OT_cubemars_check_driver,
           PICKIK_OT_save_urdf, PICKIK_PT_main,
           PICKIK_PG_preferences, PICKIK_OT_mcp_start, PICKIK_OT_mcp_stop)


def register() -> None:
    global _state
    bpy.utils.register_class(PickIKProps)
    bpy.types.Scene.pickik = bpy.props.PointerProperty(type=PickIKProps)
    for cls in CLASSES[1:]:
        bpy.utils.register_class(cls)
    _state = _CoreState()  # fresh state per register
    # §8: registering the add-on must never open a socket by itself. It is opened when the operator
    # presses Start, or here when the operator asked for exactly that, on a real session, and the
    # bridge is explicitly enabled. A background instance never binds (§2.3).
    if not _mcp_headless():
        pr = _mcp_prefs(bpy.context)
        if pr is not None and pr.enable_mcp_bridge and pr.mcp_start_on_load \
                and bpy.context.scene is not None:
            _mcp_start_from_prefs(bpy.context)


def unregister() -> None:
    # §8 teardown, first and unconditionally: a receiver thread, a worker thread or a listening
    # socket that outlives the add-on is a leftover, and the next register() would meet a port
    # already taken by its own previous life. stop() closes the socket, sets the abort flag and
    # joins every thread it started.
    mcp_bridge.stop()
    global _cubemars_timer_registered, _cubemars_live_timer_registered
    _unregister_continuous()
    # Stop and close the CAN driver if active (live or one-shot stream).
    if _state.cubemars is not None:
        _cubemars_live_stop()
        _state.cubemars.close()
        _state.cubemars = None
    if _cubemars_live_timer_registered:
        try:
            bpy.app.timers.unregister(_cubemars_live_tick)
        except Exception:
            pass
        _cubemars_live_timer_registered = False
    if _cubemars_timer_registered:
        try:
            bpy.app.timers.unregister(_cubemars_status_tick)
        except Exception:
            pass
        _cubemars_timer_registered = False
    for cls in reversed(CLASSES[1:]):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.pickik
    bpy.utils.unregister_class(PickIKProps)


if __name__ == "__main__":
    register()
