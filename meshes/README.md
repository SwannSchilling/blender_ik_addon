# Design B arm visual meshes

These dummy STL files are generated from the URDF spec and sized exactly to
the kinematic chain dimensions. Replace any file with your CAD-exported STL
(meters, binary or ASCII) to update that segment's visual.

| File              | Parents to      | Local offset (m) | Shape source       |
|-------------------|----------------|-------------------|--------------------|
| `base.stl`        | Arm7_Base      | (0, 0, 0.09)     | cylinder Ø0.2×0.18 |
| `column.stl`      | Arm7_J1        | (0, 0, 0.09)     | box 0.08×0.08×0.18 |
| `upper_arm.stl`   | Arm7_J2        | (0, -0.1075, 0)  | box 0.06×0.215×0.06|
| `upper_arm_pivot.stl`| Arm7_J2     | (0, 0, 0)        | sphere Ø0.11       |
| `forearm.stl`     | Arm7_J4        | (0, -0.1075, 0)  | box 0.06×0.215×0.06|
| `forearm_pivot.stl`  | Arm7_J4     | (0, 0, 0)        | sphere Ø0.096      |
| `wrist.stl`       | Arm7_J6        | (0, -0.0325, 0)  | box 0.06×0.065×0.06|
| `wrist_pivot.stl` | Arm7_J6        | (0, 0, 0)        | sphere Ø0.076      |
| `tool_grip.stl`   | Arm7_Tool0     | (0, 0, 0.065)    | sphere Ø0.044 + cross |

## Replacing with CAD STLs

1. Export from CAD as binary STL, **meters** (or adjust scale).
2. Place the file in this directory, overwriting the dummy.
3. In Blender, press **Build arm7 rig** again (rebuild replaces mesh objects).

## Regenerating dummies

```bash
python generate_dummy_stls.py
```