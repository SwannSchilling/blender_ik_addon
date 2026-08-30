"""arm7 model data + the Blender rig (empty-object hierarchy).

The joint table mirrors the C++ shared model (libpick-ik-core
`examples/arm7/arm7.hpp`, Design B desktop dimensions) and the URDF in
ik-service/robot_description. FK convention (URDF standard, identical
across all ports):

    child = parent * T(origin.xyz) * R(roll) * Rz(q)

(all joints rotate about their local Z; the origin "rpy" has roll only).

The rig is a nested empty hierarchy: base -> J1 -> ... -> J7 -> tool0.
Every empty's matrix_world equals the corresponding FK frame, which the
headless acceptance test checks against the C ABI's FK (that check also
validates this table against the DLL's compiled-in model).
"""

from __future__ import annotations

import math
from typing import Sequence

import bpy
from mathutils import Matrix, Vector

N_JOINTS = 7
TOOL_OFFSET = 0.065

# (origin xyz in parent frame, roll offset about parent Z) — Design B.
JOINTS: tuple[tuple[Vector, float], ...] = (
    (Vector((0.0, 0.0, 0.000)), 0.0),          # J1 base yaw
    (Vector((0.0, 0.0, 0.180)), -math.pi / 2),  # J2 shoulder pitch
    (Vector((0.0, 0.0, 0.000)), math.pi / 2),   # J3 shoulder roll
    (Vector((0.0, 0.0, 0.215)), -math.pi / 2),  # J4 elbow pitch
    (Vector((0.0, 0.0, 0.000)), math.pi / 2),   # J5 forearm roll
    (Vector((0.0, 0.0, 0.215)), -math.pi / 2),  # J6 wrist pitch
    (Vector((0.0, 0.0, 0.000)), math.pi / 2),   # J7 tool roll
)

BASE_NAME = "Arm7_Base"
TARGET_NAME = "Arm7_IK_Target"


class Rig:
    """One arm7 empty hierarchy + its target empty."""

    def __init__(self, joint_empties: list, base: bpy.types.Object,
                 tool: bpy.types.Object, target: bpy.types.Object,
                 last_q: list[float]):
        self.joint_empties = joint_empties
        self.base = base
        self.tool = tool
        self.target = target
        self.last_q = last_q

    # -- scene lookup / construction --------------------------------------

    @staticmethod
    def find() -> "Rig | None":
        if not bpy.data.objects.get(BASE_NAME):
            return None
        joints = [bpy.data.objects.get(f"Arm7_J{i + 1}") for i in range(N_JOINTS)]
        tool = bpy.data.objects.get("Arm7_Tool0")
        target = bpy.data.objects.get(TARGET_NAME)
        if None in (joints + [tool, target]):
            return None
        return Rig(joints, bpy.data.objects[BASE_NAME], tool, target, [0.0] * N_JOINTS)

    def unlink_all(self) -> None:
        """Remove the rig objects from the scene (keeps the datablocks minimal)."""
        for obj in [self.base] + self.joint_empties + [self.tool, self.target]:
            bpy.data.objects.remove(obj, do_unlink=True)


def _make_empty(name: str, parent: bpy.types.Object | None, collection) -> bpy.types.Object:
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_size = 0.04
    if collection is not None:
        collection.objects.link(empty)  # every object must be linked to a collection
    if parent is not None:
        empty.parent = parent
        empty.matrix_parent_inverse = Matrix.Identity(4)
    return empty


def build(q: Sequence[float] | None = None,
          target_m: tuple[float, float, float] = (0.300, 0.150, 0.300)) -> Rig:
    """Create (or replace) the rig. q defaults to all-zero."""
    q = list(q) if q is not None else [0.0] * N_JOINTS
    assert len(q) == N_JOINTS

    old = Rig.find()
    if old is not None:
        old.unlink_all()

    scene = bpy.context.scene
    collection = scene.collection

    base = _make_empty(BASE_NAME, None, collection)
    base.empty_display_size = 0.08
    # Build sequentially: each joint's parent is the previous empty.
    parent = base
    joints = []
    for i in range(N_JOINTS):
        j = _make_empty(f"Arm7_J{i + 1}", parent, collection)
        joints.append(j)
        parent = j
    tool = _make_empty("Arm7_Tool0", parent, None)
    tool.empty_display_size = 0.03
    target = _make_empty(TARGET_NAME, None, collection)
    target.empty_display_type = 'PLAIN_AXES'
    target.empty_display_size = 0.15
    target.location = Vector(target_m)

    rig = Rig(joints, base, tool, target, list(q))
    apply_q(rig, q)
    return rig


def apply_q(rig: Rig, q: Sequence[float]) -> None:
    """Pose the rig: every joint empty's world matrix becomes the FK frame."""
    world = rig.base.matrix_world.copy()  # base is at the origin/identity
    for i, em in enumerate(rig.joint_empties):
        origin, roll = JOINTS[i]
        m = (Matrix.Translation(origin)
             @ Matrix.Rotation(roll, 4, 'X')
             @ Matrix.Rotation(q[i], 4, 'Z'))
        em.matrix_world = world @ m
        world = em.matrix_world.copy()
    rig.tool.matrix_world = world @ Matrix.Translation(Vector((0.0, 0.0, TOOL_OFFSET)))
    rig.last_q = list(q)


def tool0_translation(rig: Rig) -> Vector:
    """tool0 empty's world translation (the rig's own FK)."""
    return rig.tool.matrix_world.to_translation()


def target_position(rig: Rig) -> tuple[float, float, float]:
    return tuple(rig.target.matrix_world.to_translation())
