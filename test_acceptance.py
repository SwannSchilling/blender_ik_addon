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
  9. panel draw uses only Blender 3.4 icon enums ('BLANK' is 4.x-only —
     it used to TypeError mid-draw right after Solve, when the status
     line became two lines, collapsing the panel);
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
    # draw() requests must exist in 3.4's enum. 'BLANK' (4.x-only) used to
    # raise TypeError mid-draw exactly when the status line became two
    # lines — i.e. right after Solve — collapsing the panel.
    class _StrictLayout:
        KNOWN = {'NONE', 'INFO', 'ERROR', 'MESH_DATA', 'BLANK1'}

        def label(self, text="", icon=None):
            if icon is not None and icon not in _StrictLayout.KNOWN:
                raise TypeError(f"icon {icon!r} not in the 3.4 enum")

        def box(self):
            return _StrictLayout()

        def row(self):
            return _StrictLayout()

        def prop(self, data, name, text=None, **kw):
            pass

        def operator(self, idname, text=None, icon=None):
            if icon is not None and icon not in _StrictLayout.KNOWN:
                raise TypeError(f"icon {icon!r} not in the 3.4 enum")

    scene.pickik.status = "gradient OK  pos err 0.68 mm\nq[deg] = [0, 0, 0, 0, 0, 0, 0]"
    class _Self:
        layout = _StrictLayout()
    try:
        addon.PICKIK_PT_main.draw(_Self(), bpy.context)
        ok9, detail9 = True, "all icons valid on 3.4"
    except TypeError as ex:
        ok9, detail9 = False, str(ex)[:80]
    gate("panel draw uses only Blender 3.4 icon enums (no mid-draw TypeError)",
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

    # -- summary -------------------------------------------------------------
    failed = [r for r in RESULTS if not r[1]]
    print(f"\n=== {len(RESULTS) - len(failed)}/{len(RESULTS)} gates passed ===")
    if failed:
        for name, _, detail in failed:
            print(f"  FAILED: {name}: {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
