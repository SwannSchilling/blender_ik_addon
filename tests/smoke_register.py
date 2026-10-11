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

A.unregister()
for cls in A.CLASSES:
    check("unregistered: %s" % cls.__name__, not live(cls))

print("touched %d operator properties" % touched)
print("RESULT:", "OK" if not FAILS else "FAILED %s" % (FAILS,))
sys.exit(1 if FAILS else 0)
