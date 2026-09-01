# Design B arm visual meshes

These dummy STL files are generated from the URDF spec and sized exactly to
the kinematic chain dimensions. Replace any file with your CAD-exported STL
(meters, binary or ASCII) to update that segment's visual.

| File                     | Parents to  | Local offset (m) | Description                        |
|--------------------------|-------------|-------------------|------------------------------------|
| `J1_baseyaw_Z.stl`      | Arm7_Base   | (0, 0, 0)         | AK10-9, Ø98×38.5mm @ world z=0     |
| `column.stl`            | Arm7_J1     | (0, 0, 0)         | your CAD column STL                 |
| `J2_shoulderpitch_X.stl`| Arm7_J2     | (0, 0, 0)         | AK10-9, Ø98×61.7mm @ world z=180   |
| `J3_shoulderroll_Z.stl` | Arm7_J3     | (0, 0, 0.0501)    | AK10-9, Ø98×38.5mm @ world z=230.1 |
| `J4_elbowpitch_X.stl`   | Arm7_J4     | (0, 0, 0)         | AK70-10, Ø89×50.25mm @ world z=395 |
| `J5_wristroll_Z.stl`    | Arm7_J5     | (0, 0, 0.172)     | Ø79×43mm @ world z=567             |
| `J6_wristpitch_X.stl`   | Arm7_J6     | (0, 0, 0)         | Ø79×43mm @ world z=610             |
| `J7_toolroll_Z.stl`     | Arm7_J7     | (0, 0, 0.043)     | Ø79×43mm @ world z=653             |
| `tool_grip.stl`         | Arm7_Tool0  | (0, 0, 0)         | sphere Ø44mm + cross bars @ tool0  |

## Replacing with CAD STLs

1. Export from CAD as binary STL, **meters** (or adjust scale).
2. Place the file in this directory, overwriting the dummy.
3. In Blender, press **Build arm7 rig** again (rebuild replaces mesh objects).

## Regenerating dummies

```bash
python generate_dummy_stls.py
```