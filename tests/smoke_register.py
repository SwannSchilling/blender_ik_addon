"""Headless registration smoke: register, touch every operator property, unregister.

Run from anywhere:
  blender --background --factory-startup --python-exit-code 1 --python tests/smoke_register.py

Catches the class of fault where a declared property never reaches RNA at
all (for example an invalid `options` identifier), which otherwise only
surfaces later as "operator object has no attribute ..." at execute() time.
Deliberately never calls execute() on any operator: the host may have the
real arm attached.
"""
import os
import sys

_ADDON = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.dirname(_ADDON)
for _p in (_ROOT, _ADDON):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import bpy                                                  # noqa: E402
import blender_ik_addon as A                                # noqa: E402

FAILS: list = []


def check(name: str, cond: bool, detail: str = "") -> None:
    print(("PASS  " if cond else "FAIL  ") + name
          + (("  |  " + detail) if (detail and not cond) else ""))
    if not cond:
        FAILS.append(name)


def touch(prop) -> tuple:
    """Read the metadata a property definition exposes; raises on a broken one."""
    try:
        default = prop.default
    except Exception:
        default = tuple(prop.default_array)
    return (prop.identifier, prop.name, prop.type, prop.subtype,
            prop.description, prop.is_readonly, default)


def live(cls) -> bool:
    """Registered classes report is_registered; operators/panels also land in bpy.types."""
    return bool(getattr(cls, "is_registered", False)) or hasattr(bpy.types, cls.__name__)


A.register()

for cls in A.CLASSES:
    check("registered: %s" % cls.__name__, live(cls))

touched = 0
for cls in A.CLASSES:
    if not (isinstance(cls, type) and issubclass(cls, bpy.types.Operator)):
        continue
    mod, _, opname = cls.bl_idname.partition(".")
    try:
        rna = getattr(getattr(bpy.ops, mod), opname).get_rna_type()
    except Exception as e:
        check("%s: rna reachable" % cls.__name__, False,
              "%s: %s" % (type(e).__name__, e))
        continue
    props = rna.properties
    for name in sorted(getattr(cls, "__annotations__", {})):
        check("%s.%s: registered in RNA" % (cls.__name__, name), name in props,
              "rna properties: %s" % sorted(p.identifier for p in props))
        if name not in props:
            continue
        try:
            touch(props[name])
            check("%s.%s: metadata touched" % (cls.__name__, name), True)
            touched += 1
        except Exception as e:
            check("%s.%s: metadata touched" % (cls.__name__, name), False,
                  "%s: %s" % (type(e).__name__, e))

# --- the SPM rig registers alongside arm7 and builds, without disturbing arm7 -------------------------
# The registration smoke's job is to catch faults that only show at execute() time. For a rig registered
# through the Phase-1 registry, the equivalent faults are: the descriptor never landed, its property names
# are not the real shared ones, its hooks are not callable, or build leaves a half-made hierarchy. Checked
# headless and without touching any hardware, mirroring how the arm7 path above is exercised.
from blender_ik_addon import rig_types as RT                       # noqa: E402

check("the registry carries both the arm7 and the SPM rig",
      RT.get_rig_type("arm7") is not None and RT.get_rig_type("spm") is not None,
      "ids: %s" % (sorted(RT.RIGS),))
spm = RT.get_rig_type("spm")
if spm is None:
    check("SPM descriptor present", False, "get_rig_type('spm') returned None")
else:
    check("SPM is a 3-crank, FK-only rig reusing the shared sliders",
          spm.n_joints == 3 and len(spm.joint_names) == 3
          and spm.fk_prop_names == ("fk_j1", "fk_j2", "fk_j3")
          and spm.q_prop_names == ("q_j1", "q_j2", "q_j3")
          and "manual_fk" in spm.capabilities and not spm.solvers,
          "%r" % (spm,))
    check("SPM hooks are all callable",
          all(callable(getattr(spm, h, None)) for h in
              ("build", "apply_q", "read_q", "object_names", "find", "target_position")))
    names = list(spm.object_names())
    check("SPM object names all carry the rig_id prefix", bool(names) and all(n.startswith("SPM") for n in names),
          "prefix-violating: %s" % [n for n in names if not n.startswith("SPM")])
    try:
        rig = spm.build(target_m=(0.0, 0.0, 0.2))
        bpy.context.view_layer.update()
        check("SPM builds an FK hierarchy through the descriptor", rig is not None)
        check("SPM exposes three crank empties the shared paths can read",
              rig is not None and len(rig.joint_empties) == 3)
        check("SPM owns its platform and crank objects in the scene",
              rig is not None and bpy.data.objects.get("SPM_Platform") is not None
              and all(bpy.data.objects.get("SPM_Crank%d" % i) is not None for i in (1, 2, 3)))
        check("building the SPM left arm7 still registered (inert)",
              RT.get_rig_type("arm7") is not None)
        if rig is not None:
            rig.unlink_all()
    except Exception as e:
        check("SPM builds an FK hierarchy through the descriptor", False,
              "%s: %s" % (type(e).__name__, e))

A.unregister()
for cls in A.CLASSES:
    check("unregistered: %s" % cls.__name__, not live(cls))

print("touched %d operator properties" % touched)
print("RESULT:", "OK" if not FAILS else "FAILED %s" % (FAILS,))
sys.exit(1 if FAILS else 0)
