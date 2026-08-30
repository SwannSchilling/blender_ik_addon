# PickIK arm7 — Blender add-on (native C ABI)

Blender 4.x add-on for interactive inverse kinematics of the 7-DOF `arm7`
desktop arm (Design B: 0.675 m chain). It loads `pick_ik_c.dll` (the C ABI
shared library from the sibling `libpick-ik-core` repo) via **ctypes** and
uses the arm7 model compiled into the DLL — no Python FK callback, so FK/IK
are millisecond-class and never touch the GIL pump.

- Part of the `URDF_BIO_IK` project suite:
  - C ABI shared library + PickIK core: [`SwannSchilling/libpick-ik-core`](https://github.com/SwannSchilling/libpick-ik-core)
  - FastAPI IK service + p5 web demo: [`SwannSchilling/ik-service`](https://github.com/SwannSchilling/ik-service)
- Spec/model source of truth: `libpick-ik-core/docs/arm7-kinematic-spec.md`
  (Design B, locked 2026-08-28). The joint table in `arm7_rig.py` and the
  C++ model in `libpick-ik-core/examples/arm7/arm7.hpp` are ports of it.
- C ABI contract: `libpick-ik-core/include/pick_ik_c/pickik_c.h`.

## Install (Blender 3.4+; verified on 3.4.1 and 4.5.3)

1. Build the DLL:
   ```sh
   cd libpick-ik-core
   cmake -S . -B build -G "Visual Studio 17 2022" -A x64
   cmake --build build --config Release --target pick_ik_c
   ```
   (This machine: `cmake --build libpick-ik-core/build --config Release`.)
   Or `git clone https://github.com/SwannSchilling/blender_ik_addon` and
   use the checkout as the add-on folder.
2. In Blender: *Preferences > Add-ons > Install...* → select the
   `blender_ik_addon` folder → enable **PickIK arm7 (native C ABI)**.
   Or zip the folder and install the zip.
3. The add-on finds the DLL in this order: the panel's *DLL path* field >
   `$PICKIK_C_DLL` > `blender_ik_addon/pick_ik_c.dll` >
   `libpick-ik-core/build/Release/pick_ik_c.dll` (sibling-repo layout).
   On other machines, set the path in the panel.

## Use

1. **Build rig** (sidebar > PickIK): creates the empty hierarchy
   `Arm7_Base > Arm7_J1..J7 > Arm7_Tool0` plus the movable `Arm7_IK_Target`
   empty. Each empty's world matrix equals the FK frame (the acceptance
   test verifies this against the C ABI's FK, worst deviation 1e-7 m).
2. Move the target empty — or drag the **X/Y/Z (mm) sliders** (the same
   ranges as the ik-service web demo: x/y −550…550, z 0…650) — and press
   **Solve**.
   - **gradient** (default): deterministic, ~1 ms end-to-end here.
   - **ccd**: local/fast, ~2.2 ms; good when the arm is already near the
     goal (continuous mode). One-shot solves from far seeds can stall in a
     CCD local minimum — that is what gradient/memetic are for.
   - **memetic**: global/recover solve, ~55 ms, runs on a background thread
     (ctypes releases the GIL for the whole C call) and snaps the result in
     when it finishes.
3. **Continuous** toggles a 50 ms timer that re-solves whenever the target
   position or the options change (busy-guarded). Use with the synchronous
   solvers; with memetic it kicks background solves and applies results as
   they land.

The status line shows success, position error (mm), solve time, and the
solved `q` in degrees. `q` of the current pose is the seed for the next
solve.

### FK / manual mode (no solver)

The **J1…J7 degree sliders** pose the arm directly (forward kinematics):
set angles, press **Apply FK**; **Sync from arm** copies the arm's current
angles back into the sliders (useful after dragging empties by hand). The
joint sliders are clamped to the Design B joint limits (±180°, pitch joints
±119.7°). In continuous mode a manual FK pose holds until the target moves
again.

### FK values exposed in the scene

After every pose change (IK solve or manual FK):

- each joint angle = `Arm7_Ji.rotation_euler.z` (radians; item panel,
  drivers, keyframes, scripts — note the empties use the `ZYX` euler order
  so that local rotation = roll about X then joint angle about Z, matching
  the FK convention),
- `bpy.context.scene.pickik.q_j1 … q_j7` (radians),
- `bpy.context.scene.pickik.tool0_x_mm / tool0_y_mm / tool0_z_mm`
  (end-effector position),
- custom property `ik_q_deg` on every joint empty.

All FK frames remain available as the empties' `matrix_world` (and the C
ABI's `pickik_arm7_link_fk`), so nothing needs to be wired up by hand —
script, driver, or other add-ons (e.g. Phobos custom properties) can read
the same values.


## Performance notes (measured, Blender 4.5.3 / this machine, 2026-08-30)

- gradient: p50 ~1.1 ms, p90 ~2.1 ms end-to-end (Python + ctypes + solve).
- ccd (100 passes): p50 ~2.5 ms, p90 ~3.8 ms. The C-ABI default of 600
  passes costs ~15 ms — the add-on deliberately uses 100; CCD is a local
  method and 100 passes is plenty from a nearby seed.
- memetic (nt=4): ~55 ms on a worker thread.
- Main-thread stall budget: p90 < 4 ms for the synchronous solvers in
  steady state (see `test_acceptance.py` gate 4; the cold first call of a
  session pays a one-time ~15 ms warm-up cost, not a per-frame stall).

## Why the C ABI (and why `pick_ik_c` is built with a static CRT)

Blender ships its own C++ runtime — `blender.crt/msvcp140.dll` (14.29 in
4.5) — and pins it for the whole process via app-local manifest. Its
`std::condition_variable`/`this_thread` internals do not match the layout a
modern MSVC (14.4x) build assumes, so any code that calls into the host's
MSVCP140 for thread waiting dereferences null (crash in the memetic
multi-threaded harvest). `libpick-ik-core` therefore builds the plugin-ABI
chain (`pick_ik_core_plugin` + `pick_ik_c`) with the **static MSVC runtime**
(`/MT`), making every std:: thread/sync primitive self-contained inside the
DLL. This is also the shape Unity and any future host want: one self-
contained DLL, no redistributable, no host-CRT version coupling.

## Acceptance

Headless acceptance (roadmap §3.0c protocol through the add-on's own code
paths — spec §5 anchors via rig + C ABI FK, Design B targets A/B, the
out-of-workspace case, stall budget):

```sh
"C:\Program Files\Blender Foundation\Blender 4.5\blender.exe" \
    --background --factory-startup --python test_acceptance.py
```

(`--factory-startup` keeps installed add-ons out of the run — e.g. a
broken Phobos on 3.4 or a stale installed copy of this add-on.)

Exit code 0 = all 6 gates pass. (Continuous mode is timer-driven on the UI
thread and cannot be exercised headlessly; its stall budget is gate 4 plus
the background-thread path exercised by gate 2.)

## Status / next

- v1 (this folder): build rig, target gizmo + sliders, solver dropdown,
  Solve, continuous mode, minimal-displacement weight, FK/manual sliders,
  exposed FK values (scene properties + empty local rotations).
- v1.1: per-joint angle targets + look-at point/axis in the panel (the C
  ABI already plumbs both — `pickik_options.joint_target_*` / `has_look_at`),
  optional STL mesh display parented to the joint empties.
