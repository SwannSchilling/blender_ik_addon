"""Headless acceptance test for the PickIK arm7 add-on (roadmap section 3.1).

Run with Blender 4.x:

    blender --background --python test_acceptance.py

Gates (the section 3.0c protocol, through the add-on's own code paths):
  1. the spec section-5 anchor poses, checked both through the add-on's
     Blender rig FK (empty hierarchy) and through the C ABI FK — the two
     must agree with the spec within a micron;
  2. the Design B cross-check targets: B (300/150/300 mm) with the
     gradient solver and A (200/100/300 mm) with the memetic solver,
     both < 1 mm;
  3. the out-of-workspace case reports a clean no-solution;
  4. the synchronous solve path (the UI-thread path) stays under the 4 ms
     main-thread stall budget, measured end to end (Python + ctypes +
     solve);
  5. the operators work end to end through bpy.ops (catches
     version-incompatible return sets and stale-matrix target reads);
  6. manual FK: the joint sliders pose the arm without a solver and the FK
     values are exposed (empty local Z rotation, scene properties, tool0);
  7. the auto-found DLL path is pre-selected in the panel field (and
     round-trips through find_dll's explicit branch);
  8. robustness: rig objects deleted in the viewport (partial or full)
     cannot crash the operators with ReferenceError — the add-on
     self-heals (re-adopt/rebuild) and keeps working;
  9. panel draw uses only this Blender's icon enums ('BLANK' is 4.x-only
     — it used to TypeError mid-draw right after Solve, when the status
     line became two lines, collapsing the panel); the whitelist is
     cross-checked against the running Blender's live icon enum);
 10. target authority: the empty is the source of truth (Solve moves the
     arm, fields mirror the empty, no snap-back);
 11. field->target direction: typing a mm field moves the empty live;
 12. URDF export: Save URDF writes meter-unit STLs (the source files are
     mm CAD exports; viewers read STLs as meters) and exact link-frame
     <origin> transforms — checked by redoing the viewer's chain math
     against the rig's verified world placements. The written chain is
     also animated through the three pitch anchors with a viewer's own
     math (T(origin) @ R(rpy) @ R(axis, q)) and must match the C ABI FK:
     that is the proof the +/-90deg rolls on joint2/4/6 put the pitch
     axes where the physics wants them (a Z-up link2 would rotate the
     shoulder about a vertical axis instead). The rolls are the standard
     URDF convention (joint axis = local Z, cf. the xArm7 description);
 13. CubeMars CAN plumbing, headless: the python-can/gs_usb dependency
     probe returns a well-formed dict, the Install-deps and Check-driver
     operators are registered, and the Check-driver operator runs end to
     end on its background thread (failing gracefully with a diagnostic
     when deps/hardware are absent, completing in either case), and the
     panel draws its full CubeMars section (dep line + buttons) on this
     Blender's icon enum.

Continuous mode is UI-thread timer driven and cannot be exercised
headlessly; its stall budget is exactly gate 4 (the synchronous solvers)
plus the background-thread path, which gate 2 exercises on a real thread.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
PARENT = os.path.dirname(HERE)

# --- boot the add-on --------------------------------------------------------
import bpy  # noqa: E402  (provided by blender --background)

sys.path.insert(0, PARENT)
import blender_ik_addon as addon  # noqa: E402
from blender_ik_addon import arm7_rig, ik_core  # noqa: E402

try:
    addon.register()
except ValueError:
    pass  # already registered (e.g. an installed copy in the addons dir)

RESULTS: list[tuple[str, bool, str]] = []


def gate(name: str, ok: bool, detail: str) -> None:
    RESULTS.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def main() -> int:
    dll = ik_core.find_dll(os.path.join(PARENT, "libpick-ik-core", "build",
                                        "Release", "pick_ik_c.dll"))
    print(f"DLL: {dll}")
    core = ik_core.Core(dll)
    rig = arm7_rig.build(target_m=(0.300, 0.150, 0.300))

    # 1) FK anchors: spec values vs rig FK vs C ABI FK.
    anchors = (
        ("zero pose", [0.0] * 7, (0.0, 0.0, 0.675)),
        ("J1 +90", [1.5707963267948966] + [0.0] * 6, (0.0, 0.0, 0.675)),
        ("J2 +90", [0.0, 1.5707963267948966] + [0.0] * 5, (0.495, 0.0, 0.180)),
        ("J2 -90", [0.0, -1.5707963267948966] + [0.0] * 5, (-0.495, 0.0, 0.180)),
        ("J4 +90", [0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0],
         (0.280, 0.0, 0.395)),
        ("J6 +90", [0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963267948966, 0.0],
         (0.065, 0.0, 0.610)),
    )
    worst = 0.0
    for name, q, expected in anchors:
        arm7_rig.apply_q(rig, q)
        bpy.context.view_layer.update()  # world matrices derive from locals
        rig_t = tuple(arm7_rig.tool0_translation(rig))
        abi_t, _ = core.fk_tool0(q)
        rig_err = max(abs(rig_t[i] - expected[i]) for i in range(3))
        abi_err = max(abs(abi_t[i] - expected[i]) for i in range(3))
        worst = max(worst, rig_err, abi_err)
        ok = rig_err < 1e-6 and abi_err < 1e-6
        if not ok:
            gate(f"anchor {name}", False,
                 f"rig err {rig_err:.2e} m, ABI err {abi_err:.2e} m, "
                 f"expected {expected}")
    gate("FK anchors (spec section 5, rig + C ABI)", worst < 1e-6,
         f"worst deviation {worst:.2e} m over {len(anchors)} anchors")

    # 2a) target B with the gradient solver (the UI-thread path), timed.
    rig.target.location = (0.300, 0.150, 0.300)
    t0 = time.perf_counter()
    res_b = core.solve("gradient", (0.300, 0.150, 0.300), [0.0] * 7)
    wall_b_ms = (time.perf_counter() - t0) * 1000.0
    ok = res_b["success"] and res_b["position_error"] < 1e-3
    arm7_rig.apply_q(rig, res_b["q"])
    bpy.context.view_layer.update()
    rig_t = tuple(arm7_rig.tool0_translation(rig))
    abi_t, _ = core.fk_tool0(res_b["q"])
    rig_abi_diff = max(abs(rig_t[i] - abi_t[i]) for i in range(3))
    # 1e-6 m (a micron): the rig composes the same joint table through
    # Blender's mathutils, the ABI through Eigen/libm — the last ulps differ.
    agree = rig_abi_diff < 1e-6
    gate("target B (300/150/300) gradient",
         ok and agree,
         f"success={res_b['success']} pos err {res_b['position_error']*1e3:.4f} mm, "
         f"rig-vs-ABI {rig_abi_diff:.2e} m, solver {res_b['time_ms']:.1f} ms, "
         f"end-to-end {wall_b_ms:.1f} ms")

    # 2b) target A with the memetic solver, on a real background thread.
    out: dict = {}

    def _memetic_thread():
        try:
            out["res"] = core.solve("memetic", (0.200, 0.100, 0.300), [0.0] * 7)
        except ik_core.CoreError as e:
            out["error"] = str(e)

    th = threading.Thread(target=_memetic_thread, daemon=True)
    th.start()
    th.join(timeout=60.0)
    if "error" in out:
        gate("target A (200/100/300) memetic (bg thread)", False, out["error"])
    else:
        res_a = out["res"]
        ok = res_a["success"] and res_a["position_error"] < 1e-3
        gate("target A (200/100/300) memetic (bg thread)", ok,
             f"success={res_a['success']} pos err "
             f"{res_a['position_error']*1e3:.4f} mm, {res_a['time_ms']:.0f} ms")

    # 3) out-of-workspace: clean no-solution.
    res_ow = core.solve("gradient", (0.050, 0.0, 0.180), [0.0] * 7)
    gate("out-of-workspace (50,0,180) reports no solution",
         not res_ow["success"] and res_ow["position_error"] > 0.1,
         f"success={res_ow['success']} pos err {res_ow['position_error']*1e3:.0f} mm")

    # 4) UI-thread stall budget for the synchronous solvers, measured in
    # steady state: a one-time cold-start cost (first allocations, lazy CRT
    # paths) is paid on the first "Solve" click of a session and is not a
    # per-frame stall. Warm up, then time.
    for _ in range(3):  # warm-up (cold start)
        core.solve("gradient", (0.300, 0.150, 0.300), [0.0] * 7)
        core.solve("ccd", (0.300, 0.150, 0.300), [0.0] * 7)
    times = []
    for _ in range(10):
        t0 = time.perf_counter()
        core.solve("gradient", (0.300, 0.150, 0.300), [0.0] * 7)
        times.append((time.perf_counter() - t0) * 1000.0)
        t0 = time.perf_counter()
        core.solve("ccd", (0.300, 0.150, 0.300), [0.0] * 7)
        times.append((time.perf_counter() - t0) * 1000.0)
    times.sort()
    p90_ms = times[int(0.9 * len(times)) - 1]
    worst_ms = times[-1]
    # The 4 ms budget is a p90 property (measured p90 ~3.8 ms for CCD at
    # 100 passes on a loaded desktop); a single worst-case blip gets a 6 ms
    # OS-jitter tolerance.
    gate("main-thread stall budget < 4 ms p90 (synchronous solvers, steady state)",
         p90_ms < 4.0 and worst_ms < 6.0,
         f"p90 {p90_ms:.2f} ms, worst {worst_ms:.2f} ms over {len(times)} warm calls "
         f"(gradient + ccd alternated)")

    # 5) operator smoke, end to end through bpy.ops: the operators' return
    # sets must be valid on the running Blender version ('RUNNING_EXECUTABLE'
    # only exists in 4.x and raises a RuntimeError after execute() on 3.x —
    # the operators return {'FINISHED'}), and the target must read from
    # fresh matrices when Build is followed immediately by Solve.
    try:
        bpy.ops.pickik.build_rig()  # rebuilds via the panel fields (mm)
        bpy.ops.pickik.solve()
        ok_op = "OK" in bpy.context.scene.pickik.status
        op_msg = bpy.context.scene.pickik.status.split("\n")[0][:80]
    except Exception as ex:
        ok_op, op_msg = False, str(ex)[:80]
    gate("operators end-to-end on this Blender version (bpy.ops build+solve)",
         ok_op, op_msg)

    # 6) Manual FK + value exposure: the J2 slider poses the arm without a
    # solver, and the FK values are readable where they are needed — the
    # joint empty's local Z rotation, the scene properties, and tool0's
    # world position (spec section-5 anchor: J2 +90deg -> (0.495, 0, 0.180)).
    scene = bpy.context.scene
    rig_op = addon._state.rig  # the rig the gate-5 operators built
    scene.pickik.fk_j2 = 90.0
    bpy.ops.pickik.apply_fk()
    bpy.context.view_layer.update()
    t6 = rig_op.tool.matrix_world.to_translation()
    anchor6 = (0.495, 0.0, 0.180)
    err6 = max(abs(t6[i] - anchor6[i]) for i in range(3))
    # Blender RNA/ID properties store at ~float32 precision, so read-back
    # values carry a ~1e-8 artifact; 1e-6 is the honest tolerance here.
    angle_ok = abs(rig_op.joint_empties[1].rotation_euler.z - math.pi / 2) < 1e-6
    qprop_ok = abs(scene.pickik.q_j2 - math.pi / 2) < 1e-6
    tool0_ok = abs(scene.pickik.tool0_x_mm - 495.0) < 0.1
    gate("manual FK: sliders pose the arm + FK values exposed",
         err6 < 1e-6 and angle_ok and qprop_ok and tool0_ok,
         f"tool0 err {err6:.1e} m | J2 empty {rig_op.joint_empties[1].rotation_euler.z:.4f} rad "
         f"| scene q_j2 {scene.pickik.q_j2:.4f} | tool0_x {scene.pickik.tool0_x_mm:.1f} mm")

    # 7) DLL path pre-select: the operators loaded the DLL with an empty
    # field (the factory-startup default), so the auto-found location must
    # now be preset in the panel field, and feeding that path back through
    # find_dll's explicit branch must resolve to the same file (a typed
    # path is what gets reused next time, and is never overwritten).
    pref = scene.pickik.dll_path
    pref_ok = (bool(pref) and os.path.isfile(pref)
               and addon.ik_core.find_dll(pref) == os.path.abspath(pref))
    gate("DLL path pre-select: auto-found path preset in the panel field",
         pref_ok, f"field = {pref or '(still empty)'}")

    # 8) Robustness: the user deleted rig empties in the viewport, so the
    # module's Rig holds stale StructRNA references. Operators must
    # self-heal (rebuild from the panel fields) instead of raising
    # ReferenceError out of execute() — that raw traceback is what
    # collapses the panel in the GUI.
    scene.pickik.target_x_mm = 300.0
    scene.pickik.target_y_mm = 150.0
    scene.pickik.target_z_mm = 300.0
    rig_del = addon._state.rig
    bpy.data.objects.remove(rig_del.target, do_unlink=True)  # partial delete
    ok8a, detail8a = True, ""
    try:
        bpy.ops.pickik.solve()
        ok8a = "OK" in scene.pickik.status
        detail8a = scene.pickik.status.split("\n")[0][:80]
    except Exception as ex:
        ok8a, detail8a = False, str(ex)[:80]
    gate("target empty deleted in viewport: Solve self-heals (no ReferenceError)",
         ok8a, detail8a)

    rig_now = addon._state.rig
    for obj in ([rig_now.base, rig_now.tool, rig_now.target]
                + rig_now.joint_empties):
        bpy.data.objects.remove(obj, do_unlink=True)  # full delete
    ok8b, detail8b = True, ""
    try:
        bpy.ops.pickik.build_rig()
        bpy.ops.pickik.solve()
        dupes = [o.name for o in bpy.data.objects
                 if o.name.startswith("Arm7_J1.")]
        ok8b = ("OK" in scene.pickik.status) and not dupes
        detail8b = scene.pickik.status.split("\n")[0][:80]
        if dupes:
            ok8b = False
            detail8b = f"duplicate rig objects: {dupes}"
    except Exception as ex:
        ok8b, detail8b = False, str(ex)[:80]
    gate("entire rig deleted: Build rig + Solve recover, no duplicate objects",
         ok8b, detail8b)

    # 9) Panel draw with a strict Blender 3.4 icon whitelist: every icon
    # draw() requests must exist in this Blender's enum. 'BLANK' (4.x-only)
    # used to raise TypeError mid-draw exactly when the status line became
    # two lines — i.e. right after Solve — collapsing the panel. The
    # whitelist is cross-checked against the running Blender's live icon
    # enum, so a stale entry fails here instead of mid-draw in the UI.
    class _StrictLayout:
        KNOWN = {'NONE', 'INFO', 'ERROR', 'MESH_DATA', 'BLANK1', 'X',
                 'CHECKMARK', 'SCRIPT', 'DRIVER', 'UNLINKED'}
        IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tga", ".ico"}

        @staticmethod
        def _is_icon_image_path(icon) -> bool:
            # label()/operator() also accept a PATH to an image file as
            # 'icon' (the CubeMars banner uses cube_mars_logo.png) - only
            # bare enum-style names are validated against the 3.4 set.
            return (isinstance(icon, str)
                    and os.path.splitext(icon)[1].lower() in _StrictLayout.IMG_EXTS)

        def label(self, text="", icon=None):
            if (icon is not None and icon not in _StrictLayout.KNOWN
                    and not self._is_icon_image_path(icon)):
                raise TypeError(f"icon {icon!r} not in the 3.4 enum")

        def box(self):
            return type(self)()  # same class -> subclass hooks still apply

        def row(self):
            return type(self)()

        def prop(self, data, name, text=None, **kw):
            pass

        def operator(self, idname, text=None, icon=None):
            if icon is not None and icon not in _StrictLayout.KNOWN:
                raise TypeError(f"icon {icon!r} not in the 3.4 enum")

    # Cross-check the whitelist against THIS Blender's live icon enum
    # (3.4 has 835 icons, 'BLANK' is not among them — the original bug).
    _icon_fn = bpy.types.UILayout.bl_rna.functions.get("operator")
    live_icons = (frozenset(e.identifier for e in
                            _icon_fn.parameters["icon"].enum_items)
                  if _icon_fn else None)
    stale_icons = sorted(_StrictLayout.KNOWN - live_icons) if live_icons else []

    scene.pickik.status = "gradient OK  pos err 0.68 mm\nq[deg] = [0, 0, 0, 0, 0, 0, 0]"
    class _Self:
        layout = _StrictLayout()
    try:
        # _draw directly: the public draw() wraps it in try/except and logs
        # to ~/pickik_addon_draw_errors.log, which would swallow the very
        # TypeError this gate exists to catch (making the gate a no-op).
        addon.PICKIK_PT_main._draw(_Self(), bpy.context)
        ok9, detail9 = True, "all icons valid"
    except TypeError as ex:
        ok9, detail9 = False, str(ex)[:80]
    if stale_icons:
        ok9 = False
        detail9 = f"whitelist entries missing on this Blender: {stale_icons}"
    elif live_icons is not None:
        detail9 = f"all icons valid ({len(live_icons)}-icon live enum)"
    gate("panel draw uses only this Blender's icon enums (no mid-draw TypeError)",
         ok9, detail9)

    # 10) Target authority: the empty is the single source of truth.
    # Dragging the target (simulated by setting its location) must not be
    # overridden on Solve; the arm solves to where the empty is, and the
    # fields mirror it. Typing a field moves the empty live.
    rig10 = addon._state.rig
    rig10.target.location = Vector((0.250, -0.100, 0.350))
    bpy.context.view_layer.update()
    bpy.ops.pickik.solve()
    t10 = rig10.tool.matrix_world.to_translation()
    err10 = (Vector(t10) - Vector((0.250, -0.100, 0.350))).length
    fields10 = (scene.pickik.target_x_mm, scene.pickik.target_y_mm,
                scene.pickik.target_z_mm)
    fields_match_target = (abs(fields10[0] - 250.0) < 1e-3
                           and abs(fields10[1] - -100.0) < 1e-3
                           and abs(fields10[2] - 350.0) < 1e-3)
    target_stayed = abs(rig10.target.location.x - 0.250) < 1e-9
    target_ok = err10 < 1e-3 and fields_match_target and target_stayed
    gate("target empty is the authority: Solve moves the arm, fields mirror the empty",
         target_ok and "OK" in scene.pickik.status,
         f"tool0 err {err10*1e3:.4f} mm | fields {fields10} | target.x {rig10.target.location.x:.4f}")

    # 11) Field->target direction: setting the field moves the empty live.
    scene.pickik.target_x_mm = 400.0
    bpy.context.view_layer.update()
    fx = scene.pickik.target_x_mm
    tx = rig10.target.location.x
    field_ok = abs(fx - 400.0) < 0.001
    # Blender Object.location stores float32 (~6e-9 artifact on 0.4).
    target_ok = abs(tx - 0.400) < 1e-6
    gate("typing a mm field moves the target empty live (no snap-back)",
         field_ok and target_ok,
         f"field X = {fx:.1f}, target.x = {tx:.4f}, diff = {abs(tx-0.400):.2e}")

    # 12) URDF export: Save URDF must produce a viewer-correct description.
    # The source STLs are mm CAD exports; viewers read STL coordinates as
    # meters, so the exported files must be meter-scaled, and each
    # <origin> must place the STL file frame in the URDF link frame
    # exactly (the link's +/-90deg roll and the STL's own origin matter —
    # a bbox-center/rpy=0 origin does not place the geometry where the rig
    # shows it). Checked by redoing the viewer's math: the link chain
    # rebuilt from the written URDF, @ the written origin, @ the exported
    # file's bbox center — against the rig object's world bbox center;
    # plus file dimensions vs world dimensions (a 1000x regression shows
    # up in both).
    import tempfile
    import xml.etree.ElementTree as ET
    from mathutils import Matrix as _M
    from mathutils import Quaternion as _Q
    out12 = tempfile.mkdtemp(prefix="pickik_urdf_")
    q12 = list(arm7_rig.joint_angles(rig10))
    ok12, detail12 = True, ""
    try:
        bpy.ops.pickik.save_urdf(directory=out12)
        root = ET.parse(os.path.join(out12, "arm7.urdf")).getroot()
        links12 = {l.get("name"): l for l in root.findall("link")}
        joints12 = {j.get("name"): j for j in root.findall("joint")}
        # Rebuild the link frames from the written URDF itself.
        L12 = {"base_link": _M.Identity(4)}
        for k in range(1, 8):
            jn = joints12[f"joint{k}"]
            ox, oy, oz = (float(s) for s in jn.find("origin").get("xyz").split())
            r12, _, _ = (float(s) for s in jn.find("origin").get("rpy").split())
            L12[jn.find("child").get("link")] = (
                L12[jn.find("parent").get("link")]
                @ _M.Translation((ox, oy, oz))
                @ _M.Rotation(r12, 4, "X"))
        jo12 = joints12["tool_offset"]
        tx, ty, tz = (float(s) for s in jo12.find("origin").get("xyz").split())
        L12[jo12.find("child").get("link")] = (
            L12[jo12.find("parent").get("link")]
            @ _M.Translation((tx, ty, tz)))

        def _parent_empty(lname):
            if lname == "base_link":
                return "Arm7_Base"
            if lname == "tool_link":
                return "Arm7_Tool0"
            return f"Arm7_J{lname[4:]}"

        # The URDF describes the rest pose (q = 0); a viewer rendering it
        # uses exactly the chain above. Read the reference placements from
        # the rig at the rest pose, then restore the user's pose.
        arm7_rig.apply_q(rig10, [0.0] * arm7_rig.N_JOINTS)
        bpy.context.view_layer.update()
        worst12 = 0.0
        n12 = 0
        for lname, link in links12.items():
            for vis in link.findall("visual"):
                o = vis.find("origin")
                m = vis.find("geometry/mesh")
                fname = m.get("filename")
                xyz12 = tuple(float(s) for s in o.get("xyz").split())
                # Reconstruct the origin rotation exactly as a viewer does:
                # R = Rz(yaw) Ry(pitch) Rx(roll) from the written rpy.
                rv, pv, yv = (float(s) for s in o.get("rpy").split())
                R12v = (_M.Rotation(yv, 3, "Z")
                        @ _M.Rotation(pv, 3, "Y")
                        @ _M.Rotation(rv, 3, "X"))
                t_base = (L12[lname]
                          @ _M.Translation(xyz12)
                          @ R12v.to_4x4())
                stl12 = os.path.join(out12, fname)
                v12, _f12, _mm12 = arm7_rig._load_stl(stl12)
                mn12 = [min(pt[i] for pt in v12) for i in range(3)]
                mx12 = [max(pt[i] for pt in v12) for i in range(3)]
                c12 = [(mn12[i] + mx12[i]) / 2 for i in range(3)]
                base = os.path.basename(os.path.splitext(fname)[0])
                mo12 = bpy.data.objects["Arm7_Mesh_" + base]
                bb = [mo12.matrix_world @ Vector(b) for b in mo12.bound_box]
                c_bl = [sum(p[i] for p in bb) / 8 for i in range(3)]
                worst12 = max(worst12,
                              (t_base @ Vector(c12) - Vector(c_bl)).length)
                # Scale check, both sides in the same (base) frame: the
                # file's 8 bbox corners transformed by the written origin
                # must give the same AABB extents as the object's local
                # bbox corners transformed by its world matrix (a 1000x
                # regression shows up as a ~999x difference here; plain
                # file-frame vs world-frame AABBs would differ by design
                # for any mesh whose frame is rotated against the world).
                corners = [Vector((mx12[0] if a else mn12[0],
                                   mx12[1] if b else mn12[1],
                                   mx12[2] if c else mn12[2]))
                           for a in (0, 1) for b in (0, 1) for c in (0, 1)]
                c_ws = [t_base @ c for c in corners]
                d_file = [max(p[i] for p in c_ws) - min(p[i] for p in c_ws)
                          for i in range(3)]
                d_world = [max(p[i] for p in bb) - min(p[i] for p in bb)
                           for i in range(3)]
                for i in range(3):
                    worst12 = max(worst12, abs(d_file[i] - d_world[i]))
                n12 += 1
        # The exported chain must also ANIMATE like the solver: rebuild
        # the link frames with a viewer's own math (T(origin) @ R(rpy) @
        # R(axis, q)) at the three pitch anchors and compare tool0 with
        # the C ABI FK. This is what proves the +/-90deg rolls on
        # joint2/4/6 are not a defect: they put each pitch joint's local
        # Z (the URDF rotation axis) onto the horizontal pitch axis.
        def _chain12(qc):
            F = {"base_link": _M.Identity(4)}
            for k in range(1, 8):
                jn = joints12[f"joint{k}"]
                o = jn.find("origin")
                ox, oy, oz = (float(s) for s in o.get("xyz").split())
                r, p, y = (float(s) for s in o.get("rpy").split())
                ax = Vector((float(s) for s in jn.find("axis").get("xyz").split()))
                R_o = (_M.Rotation(y, 3, "Z") @ _M.Rotation(p, 3, "Y")
                       @ _M.Rotation(r, 3, "X"))
                th = qc[k - 1]
                R_q = _Q((math.cos(th / 2), ax.x * math.sin(th / 2),
                          ax.y * math.sin(th / 2),
                          ax.z * math.sin(th / 2))).to_matrix().to_4x4()
                F[jn.find("child").get("link")] = (
                    F[jn.find("parent").get("link")]
                    @ _M.Translation((ox, oy, oz)) @ _M(R_o).to_4x4() @ R_q)
            jo = joints12["tool_offset"]
            tx, ty, tz = (float(s) for s in jo.find("origin").get("xyz").split())
            F[jo.find("child").get("link")] = (
                F[jo.find("parent").get("link")]
                @ _M.Translation((tx, ty, tz)))
            return F["tool_link"].to_translation()

        worst12_anim = 0.0
        for qc in ([0.0, math.pi / 2] + [0.0] * 5,
                   [0.0, 0.0, 0.0, math.pi / 2, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0]):
            t12 = _chain12(qc)
            abi12, _ = core.fk_tool0(qc)
            worst12_anim = max(worst12_anim,
                               (Vector(t12) - Vector(abi12)).length)
        arm7_rig.apply_q(rig10, q12)
        bpy.context.view_layer.update()
        ok12 = (n12 == 8 and worst12 < 1e-5 and worst12_anim < 1e-4
                and "URDF saved" in scene.pickik.status)
        detail12 = (f"{n12}/8 meshes, worst placement+scale err "
                    f"{worst12*1e3:.4f} mm, worst animated-chain err "
                    f"{worst12_anim*1e3:.4f} mm (budget 0.1 mm), "
                    f"status {scene.pickik.status[:40]!r}")
    except Exception as ex:
        ok12 = False
        detail12 = f"exception: {ex}"
    gate("Save URDF: meter-unit STLs + exact link-frame origins (viewer-correct)",
         ok12, detail12)

    # 13) CubeMars CAN plumbing, headless: the dependency probe is
    # well-formed, the operators are registered, the Check-driver
    # operator runs end to end on its background thread
    # (deps/hardware absent in CI -> it must still complete with a
    # diagnostic, never crash or hang), the panel draws its full
    # CubeMars section (logo banner + dep line + install/check buttons
    # + id rows) on this Blender's live icon enum, the set-origin
    # (zero position) command is well-formed (mode-5 frame + 8 zero
    # bytes, the no-motors guard, and - when a live stream is
    # available - the refuse-while-streaming guard; no frames sent for
    # the guard checks), the per-joint direction table is intact (J1
    # hardcoded inverted for this install, applied to rig-space
    # targets), and the bundled logo PNG is present + resolved by the
    # banner (label(icon=<image path>) must not raise mid-draw).
    from blender_ik_addon import cubemars_driver as _cm
    deps13 = _cm.check_dependencies()
    deps_ok = (isinstance(deps13, dict)
               and set(deps13) == {"can", "can_version", "gs_usb", "ready"}
               and isinstance(deps13["can"], bool)
               and isinstance(deps13["gs_usb"], bool)
               and deps13["ready"] is (bool(deps13["can"])
                                       and bool(deps13["gs_usb"])))
    scene.pickik.cubemars_enabled = True
    ok13_ops, task_text, task_done = False, "(not run)", False
    telemetry_text, disconnect_text, reopen_text, live_text = ("(not run)",
                                                               "(not run)",
                                                               "(not run)",
                                                               "(not run)")
    zero_guard = None
    zframe_ok = znoid_ok = False
    dir_ok = False
    logo_ok = False

    def _poll13(timeout: float = 90.0) -> tuple:
        t0 = time.time()
        while addon._cubemars_task_poll()[0] and time.time() - t0 < timeout:
            time.sleep(0.1)
        return addon._cubemars_task_poll()

    try:
        _ = bpy.ops.pickik.cubemars_install_deps      # registered?
        bpy.ops.pickik.cubemars_check_driver()        # run end to end
        _active, task_text, _d = _poll13()
        task_done = not _active
        # Telemetry: must complete with a well-formed readout. With the
        # motors unplugged, "no motor feedback" / an open failure are the
        # expected outcomes - this gate checks the plumbing, not the
        # motors.
        bpy.ops.pickik.cubemars_read_telemetry()
        _active, telemetry_text, _detail13 = _poll13()
        tel_ok = (not _active and bool(telemetry_text)
                  and telemetry_text.startswith(("Telemetry", "ERROR")))
        # Disconnect: must complete and report the adapter release.
        bpy.ops.pickik.cubemars_disconnect()
        _active, disconnect_text, _d = _poll13()
        disc_ok = (not _active
                  and "disconnected" in disconnect_text.lower())
        # Live update: toggling ON must start the pose-following stream
        # and toggling OFF must stop it cleanly (or, without hardware,
        # the stream must die on its own with a clear error - either way
        # no live flag/thread may be left behind).
        try:
            scene.pickik.cubemars_live = True
            t0 = time.time()
            live_running = False
            while time.time() - t0 < 15:
                drv13 = addon._state.cubemars
                if drv13 is not None and drv13.is_live and drv13.is_active:
                    live_running = True
                    break
                if (drv13 is not None and not drv13.is_active
                        and not drv13.is_live):
                    break  # stream started up and died (e.g. open failed)
                time.sleep(0.05)
            drv13 = addon._state.cubemars
            if live_running:
                live_text = f"on: {drv13.status[:40]!r}"
                # Set-origin guard: it must be REFUSED while the live
                # stream owns the bus (a re-reference mid-stream would
                # move the arm). The guard returns before touching the
                # bus, so this sends no frames.
                zlive = drv13.set_origin()
                zero_guard = (zlive.get("ok") is False
                              and "Stop first" in zlive.get("text", ""))
            else:
                zero_guard = None
                if drv13 is not None:
                    live_text = f"on(dead): {drv13.status[:40]!r}"
                else:
                    live_text = "on: no driver"
            scene.pickik.cubemars_live = False
            t0 = time.time()
            while time.time() - t0 < 10:
                drv13 = addon._state.cubemars
                if drv13 is None or not drv13.is_active:
                    break
                time.sleep(0.05)
            drv13 = addon._state.cubemars
            if drv13 is not None and (drv13.is_active or drv13.is_live):
                live_text += " | off: STILL LIVE"
            else:
                live_text += " | off OK"
        except Exception as ex:
            live_text = f"raised: {ex}"
        live_ok = live_text.endswith("off OK")
        # Set origin (zero position): the protocol frame must be
        # well-formed (mode 5, 8 zero bytes), and the no-motors guard
        # must answer without touching the bus - both hardware-free.
        zframe_ok = (_cm.make_can_id(_cm.MODE_SET_ORIGIN, 0x68)
                     == ((5 << 8) | 0x68)
                     and list(_cm.pack_set_origin()) == [0] * 8)
        z_noid = _cm.CubeMarsDriver(motor_ids=[0] * 7).set_origin()
        znoid_ok = (z_noid.get("ok") is False
                    and "no motor IDs" in z_noid.get("text", ""))
        # Per-joint direction: this install's J1 motor is mounted
        # opposite to the rig convention, so the addon hardcodes J1
        # inverted (CUBEMARS_MOTOR_DIRECTIONS; per-joint toggles are
        # roadmap v1.3.0) and the driver must apply the sign to rig-
        # space targets (motor-space arrival check relies on it).
        drv_dir = addon._get_cubemars_driver()
        dir_ok = (list(addon.CUBEMARS_MOTOR_DIRECTIONS)
                  == [-1, 1, 1, 1, 1, 1, 1]
                  and list(drv_dir._directions)
                  == list(addon.CUBEMARS_MOTOR_DIRECTIONS)
                  and drv_dir._apply_directions(
                      [10.0, 20.0, 30.0, 0.0, 0.0, 0.0, 0.0])[:3]
                  == [-10.0, 20.0, 30.0])
        # Banner: the bundled logo PNG must exist next to the addon and
        # the section header must resolve to it (the panel-draw checks
        # below prove label(icon=<image path>) survives this Blender;
        # _StrictLayout only validates enum-style names).
        logo13 = os.path.join(HERE, "cube_mars_logo.png")
        logo_ok = (os.path.isfile(logo13)
                   and addon._cubemars_logo_icon() == logo13)
        # Disconnect again so the re-open check below is a genuine
        # fresh open (the live stream may have (re)opened the bus).
        bpy.ops.pickik.cubemars_disconnect()
        _poll13()
        # Re-open cycle: with hardware attached, the check right after
        # disconnect() must succeed - a genuine fresh open (the WinUSB
        # handle must have been released; old python-can shutdown +
        # pyusb GC-timing leaked the handle, making in-process re-opens
        # fail with 'Access denied').
        if task_done and task_text.startswith("OK"):
            bpy.ops.pickik.cubemars_check_driver()
            _active, reopen_text, _d = _poll13()
        zero_ok = (zframe_ok and znoid_ok
                   and (zero_guard is None or zero_guard))
        ok13_ops = (task_done and bool(task_text)
                    and ("FAIL" in task_text or "OK" in task_text)
                    and tel_ok and disc_ok and live_ok
                    and zero_ok and dir_ok and logo_ok
                    and (not task_text.startswith("OK")
                         or reopen_text.startswith("OK")))
    except Exception as ex:
        task_text = f"operator raised: {ex}"
    _zero_txt = ("OK" if (zframe_ok and znoid_ok
                          and (zero_guard is None or zero_guard))
                 else "FAIL")
    detail13 = (f"can={deps13['can']} gs_usb={deps13['gs_usb']} "
                f"(v{deps13['can_version'] or '?'}) | check: "
                f"{task_text[:48]!r} | telemetry: "
                f"{telemetry_text[:36]!r} | disconnect: "
                f"{disconnect_text[:30]!r} | live: {live_text[:64]!r} "
                f"| zero: {_zero_txt}"
                f"{' (guard skipped)' if zero_guard is None else ''} "
                f"| dir: {'OK' if dir_ok else 'FAIL'}"
                f"| logo: {'OK' if logo_ok else 'FAIL'}"
                f"| re-open: {reopen_text[:36]!r} | ")
    try:
        addon.PICKIK_PT_main._draw(_Self(), bpy.context)
        detail13 += "panel draw OK | "
    except TypeError as ex:
        detail13 += f"panel draw raised: {ex} | "
    # The feedback line must render even when the section is OFF: the user
    # clicks Install deps / Check driver before enabling anything, and the
    # task result has nowhere else to go (this is what made the buttons
    # look dead on first release).
    scene.pickik.cubemars_enabled = False
    scene.pickik.cubemars_status = task_text  # what the timer would write
    labels13: list = []

    class _Rec13(_StrictLayout):
        def label(self, text="", icon=None):
            super().label(text, icon)
            labels13.append(text)

    class _Self13:
        layout = _Rec13()

    label_ok = False
    try:
        addon.PICKIK_PT_main._draw(_Self13(), bpy.context)
        label_ok = any(task_text in s for s in labels13)
        detail13 += "status line while disabled: " + ("OK" if label_ok
                                                      else "MISSING")
    except TypeError as ex:
        detail13 += f"disabled draw raised: {ex}"
    gate("CubeMars: dep probe + install/check operators + panel draw (headless)",
         deps_ok and ok13_ops and label_ok, detail13)

    # -- summary -------------------------------------------------------------
    failed = [r for r in RESULTS if not r[1]]
    print(f"\n=== {len(RESULTS) - len(failed)}/{len(RESULTS)} gates passed ===")
    if failed:
        for name, _, detail in failed:
            print(f"  FAILED: {name}: {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
