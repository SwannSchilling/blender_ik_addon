"""Check world orientation of the mesh objects after counter-rotation fix."""
import bpy, sys
from mathutils import Vector

sys.path.insert(0, r"F:/GithubProjects/URDF_BIO_IK/blender_ik_addon")

# Manually define the required property group
from bpy.props import (
    StringProperty, FloatProperty, BoolProperty, EnumProperty, PointerProperty,
)
class DummyProps(bpy.types.PropertyGroup):
    meshes_path: StringProperty(default="")

bpy.utils.register_class(DummyProps)
bpy.types.Scene.pickik = PointerProperty(type=DummyProps)
bpy.context.scene.pickik.meshes_path = ""  # will use default

import arm7_rig
rig = arm7_rig.build()
bpy.context.view_layer.update()

ok = True
for mo in rig.mesh_objects:
    bb = [mo.matrix_world @ Vector(v) for v in mo.bound_box]
    ws = [max(v[i] for v in bb) - min(v[i] for v in bb) for i in range(3)]
    dominant = "X" if ws[0] > ws[1] and ws[0] > ws[2] else ("Y" if ws[1] > ws[2] else "Z")
    loc = tuple(round(v,4) for v in mo.location)
    rot = tuple(round(v,4) for v in mo.rotation_euler)
    print(f"{mo.name}  world_size(X={ws[0]*1000:.1f} Y={ws[1]*1000:.1f} Z={ws[2]*1000:.1f}) dominant={dominant}  loc={loc}  rot={rot}")
    if dominant != "Z":
        print(f"  WARNING: {mo.name} dominant axis is {dominant}, expected Z")
        ok = False

print(f"\n{'ALL ORIENTED CORRECTLY' if ok else 'SOME NEED FIXING'}")
bpy.utils.unregister_class(DummyProps)
del bpy.types.Scene.pickik
