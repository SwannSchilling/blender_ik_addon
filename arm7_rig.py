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
import os
import struct
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
TOOL_NAME = "Arm7_Tool0"
TARGET_NAME = "Arm7_IK_Target"

# -- Visual meshes --------------------------------------------------------
# Each entry: (stl_filename, parent_empty_name, local_xyz_offset)
# Offsets match the URDF <origin xyz> for that link's visual in the
# link frame (which is the empty's frame).

MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")

# Each entry: (stl_filename, parent_empty_name, local_xyz_offset, euler_zyx)
# Offsets position the actuator center in the parent empty's frame.
# euler_zyx counter-rotates the parent's roll so the mesh stays aligned
# to world axes (necessary because your CAD STLs are designed in world space).
# Joint rolls: J2=-pi/2, J3=+pi/2, J4=-pi/2, J5=+pi/2, J6=-pi/2, J7=+pi/2
# Cancelling roll: negate the parent's roll in ZYX order (roll, 0, 0)
_J2_ROLL = -math.pi / 2
_J3_ROLL = math.pi / 2
_J4_ROLL = -math.pi / 2
_J5_ROLL = math.pi / 2
_J6_ROLL = -math.pi / 2
_J7_ROLL = math.pi / 2
MESH_SPEC: tuple[tuple[str, str, tuple[float, float, float]], ...] = (
    ("J1_baseyaw_Z.stl",      "Arm7_J1",      (0.0, 0.0, 0.0)),
    ("J2_shoulderpitch_X.stl","Arm7_J2",       (0.0, 0.0, 0.0)),
    ("J3_shoulderroll_Z.stl", "Arm7_J3",       (0.0, 0.0, 0.0501)),
    ("J4_elbowpitch_X.stl",   "Arm7_J4",       (0.0, 0.0, 0.0)),
    ("J5_wristroll_Z.stl",    "Arm7_J5",       (0.0, 0.0, 0.172)),
    ("J6_wristpitch_X.stl",   "Arm7_J6",       (0.0, 0.0, 0.0)),
    ("J7_toolroll_Z.stl",     "Arm7_J7",       (0.0, 0.0, 0.043)),
    ("tool_grip.stl",         TOOL_NAME,       (0.0, 0.0, 0.0)),
)

MESH_PREFIX = "Arm7_Mesh_"


def _ensure_meshes() -> None:
    """Apply the scene's meshes_path to MESH_DIR — the explicit path or the
    add-on's own meshes/ folder. Safe to call before the add-on properties
    are registered (the acceptance test calls build() directly)."""
    try:
        import bpy
        _ = bpy.context.scene.pickik.meshes_path  # may raise if not registered
    except (AttributeError, RuntimeError):
        return  # props not registered; use default MESH_DIR
    if bpy.context.scene.pickik.meshes_path.strip():
        global MESH_DIR
        MESH_DIR = bpy.context.scene.pickik.meshes_path.strip()


def _load_stl(path: str) -> tuple[list, list, bool]:
    """Read binary or ASCII STL, return (vertices, faces, was_mm).

    - Auto-detects binary vs ASCII format.
    - If values > 1.0, assumes mm and scales to meters (returns was_mm=True).
    - Vertices stay at the STL's inherent origin.
    """
    with open(path, "rb") as f:
        raw = f.read()
    if raw[:5] == b"solid":
        _verts, _faces = _read_stl_ascii(raw)
    else:
        _verts, _faces = _read_stl_binary(raw)
    if not _verts:
        return [], [], False
    # Convert units: if max magnitude > 1.0, assume mm -> scale to m
    max_mag = max(max(abs(v) for v in pt) for pt in _verts)
    was_mm = max_mag > 1.0
    if was_mm:
        _verts = [(v[0] * 0.001, v[1] * 0.001, v[2] * 0.001) for v in _verts]
    return _verts, _faces, was_mm


def _read_stl_binary(data: bytes) -> tuple[list, list]:
    """Parse binary STL from raw bytes."""
    n = struct.unpack("<I", data[80:84])[0]
    verts: list = []
    faces: list = []
    offset = 84
    for _ in range(n):
        chunk = data[offset:offset + 50]
        if len(chunk) < 50:
            break
        v = struct.unpack("<12fH", chunk)
        i = len(verts)
        verts.extend([(v[3], v[4], v[5]),
                      (v[6], v[7], v[8]),
                      (v[9], v[10], v[11])])
        faces.append((i, i + 1, i + 2))
        offset += 50
    return verts, faces


def _read_stl_ascii(data: bytes) -> tuple[list, list]:
    """Parse ASCII STL from raw bytes."""
    text = data.decode("ascii", errors="replace")
    verts: list = []
    faces: list = []
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("vertex"):
            parts = s.split()
            if len(parts) >= 4:
                verts.append((float(parts[1]), float(parts[2]),
                              float(parts[3])))
    # Group every 3 vertices into a face
    faces = [(i, i + 1, i + 2) for i in range(0, len(verts), 3)]
    return verts, faces


def _mesh_object_name(stl_name: str) -> str:
    """Stable object name from the STL filename."""
    base, _ = os.path.splitext(stl_name)
    return MESH_PREFIX + base


def _build_mesh(rig: "Rig", stl_name: str, parent_name: str,
                 offset: tuple[float, float, float]) -> bpy.types.Object | None:
    """Load one STL, create the mesh object, parent it, return the object.

    Uses standard Blender parenting: keep the mesh at its world-space
    position while making it a child of the joint empty. The STL vertices
    are in world coordinates (Fusion origin at 0,0,0). After parenting
    with matrix_parent_inverse = parent_world.inverted(), the mesh
    maintains its world position and inherits the parent's rotation
    (which is correct for FK — the mesh rotates with the joint).

    This is the automated equivalent of:
      import STL → select both mesh and parent empty → Ctrl+P (Keep Transform)
    """
    path = os.path.join(MESH_DIR, stl_name)
    if not os.path.isfile(path):
        return None
    obj_name = _mesh_object_name(stl_name)
    existing = bpy.data.objects.get(obj_name)
    if existing is not None:
        bpy.data.objects.remove(existing, do_unlink=True)
    verts, faces, was_mm = _load_stl(path)
    if not verts:
        return None

    mesh = bpy.data.meshes.new(obj_name)
    mesh.from_pydata(verts, [], faces)
    mesh.validate(verbose=False)
    for f in mesh.polygons:
        f.use_smooth = True
    obj = bpy.data.objects.new(obj_name, mesh)
    bpy.context.scene.collection.objects.link(obj)

    parent_obj = bpy.data.objects.get(parent_name)
    if parent_obj is not None:
        if was_mm:
            # CAD: vertices in world space. Keep Transform so mesh stays
            # at world origin — parent transform is compensated.
            obj.parent = parent_obj
            obj.matrix_parent_inverse = parent_obj.matrix_world.inverted()
            obj.location = Vector((0, 0, 0))
        else:
            # Dummy: vertices at local origin. Simple parent + offset.
            obj.parent = parent_obj
            obj.matrix_parent_inverse = Matrix.Identity(4)
            obj.location = Vector(offset)

    # 90° Z rotation to align Fusion's coordinate system with Blender
    obj.rotation_mode = 'XYZ'
    obj.rotation_euler.z = math.pi / 2

    obj.display_type = 'SOLID'
    obj.hide_render = True
    return obj


def rig_object_names() -> list[str]:
    """Every object name the rig owns (for cleanup + stale-reference checks)."""
    names = [BASE_NAME, TOOL_NAME, TARGET_NAME]
    names += [f"Arm7_J{i + 1}" for i in range(N_JOINTS)]
    # Mesh objects
    for spec in MESH_SPEC:
        names.append(_mesh_object_name(spec[0]))
    return names


class Rig:
    """One arm7 empty hierarchy + its target empty + optional mesh objects."""

    def __init__(self, joint_empties: list, base: bpy.types.Object,
                 tool: bpy.types.Object, target: bpy.types.Object,
                 last_q: list[float],
                 mesh_objects: list[bpy.types.Object] | None = None):
        self.joint_empties = joint_empties
        self.base = base
        self.tool = tool
        self.target = target
        self.last_q = last_q
        self.mesh_objects = mesh_objects if mesh_objects is not None else []

    # -- scene lookup / construction --------------------------------------

    @staticmethod
    def find() -> "Rig | None":
        if not bpy.data.objects.get(BASE_NAME):
            return None
        joints = [bpy.data.objects.get(f"Arm7_J{i + 1}") for i in range(N_JOINTS)]
        tool = bpy.data.objects.get(TOOL_NAME)
        target = bpy.data.objects.get(TARGET_NAME)
        if None in (joints + [tool, target]):
            return None
        return Rig(joints, bpy.data.objects[BASE_NAME], tool, target, [0.0] * N_JOINTS)

    def unlink_all(self) -> None:
        """Remove the rig objects from the scene (keeps the datablocks minimal)."""
        for obj in [self.base] + self.joint_empties + [self.tool, self.target,
                                                       *self.mesh_objects]:
            try:
                bpy.data.objects.remove(obj, do_unlink=True)
            except (ReferenceError, RuntimeError):
                pass

    def alive(self) -> bool:
        """True while every rig object still exists in the scene.

        A user can delete rig empties in the viewport (X key); the cached
        StructRNA references stay around in Python but raise ReferenceError
        on attribute access. Every add-on code path that touches the rig
        checks this first.
        """
        try:
            for obj in ([self.base, self.tool, self.target]
                        + self.joint_empties + self.mesh_objects):
                obj.name  # raises ReferenceError if the object was removed
            return True
        except (ReferenceError, RuntimeError):
            return False


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
    _ensure_meshes()
    q = list(q) if q is not None else [0.0] * N_JOINTS
    assert len(q) == N_JOINTS

    # Replace semantics: remove any surviving rig objects. This must cover a
    # PARTIALLY deleted rig (the user deleted some empties in the viewport):
    # Rig.find() returns None then, and re-creating the same names would make
    # Blender auto-rename to *.001, leaving a second half-arm in the scene.
    for name in rig_object_names():
        obj = bpy.data.objects.get(name)
        if obj is not None:
            bpy.data.objects.remove(obj, do_unlink=True)

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
    # Linked like the joints: an object that is only parented (not linked to
    # a collection) never enters the view layer, so its matrix_world would
    # never be computed from the parent chain.
    tool = _make_empty(TOOL_NAME, parent, collection)
    tool.empty_display_size = 0.03
    target = _make_empty(TARGET_NAME, None, collection)
    target.empty_display_type = 'PLAIN_AXES'
    target.empty_display_size = 0.15
    target.location = Vector(target_m)

    rig = Rig(joints, base, tool, target, list(q))
    apply_q(rig, q)
    # Fresh transforms: the parent empties' matrix_world must be valid
    # before _build_mesh transforms vertices into the local frame.
    bpy.context.view_layer.update()
    # Build meshes and attach to the empties
    mesh_objects = []
    for stl_name, parent_name, offset in MESH_SPEC:
        mo = _build_mesh(rig, stl_name, parent_name, offset)
        if mo is not None:
            mesh_objects.append(mo)
            # Make the driver slim: hide mesh in viewport wireframe overlay
            mo.show_wire = False
    rig.mesh_objects = mesh_objects
    # Fresh objects: their matrix_world is not valid until the view layer
    # updates once — an immediate read (e.g. Solve right after Build) would
    # see the identity matrices.
    bpy.context.view_layer.update()
    return rig


def apply_q(rig: Rig, q: Sequence[float]) -> None:
    """Pose the rig: each joint empty's *local* transform becomes one FK step
    (location = link origin in the parent frame, rotation = roll about X then
    the joint angle about Z — Euler XYZ gives Rx(roll) @ Rz(q)).

    The joint angle is therefore `empty.rotation_euler.z`: readable in the
    item panel, scriptable, keyframeable, and usable as a driver target.
    After a view-layer update, every empty's `matrix_world` equals the
    corresponding FK frame (tool0 = end-effector position).

    Note on euler order: Blender's 'XYZ' computes Rz @ Ry @ Rx (X applied
    first in object space, so it lands rightmost in the matrix). The FK
    step needs Rx(roll) @ Rz(q) — that is the 'ZYX' order with components
    (roll, 0, q). Verified by matrix probe on 3.4.1 and 4.5.3.
    """
    for i, em in enumerate(rig.joint_empties):
        origin, roll = JOINTS[i]
        em.location = origin
        em.rotation_mode = 'ZYX'
        em.rotation_euler = (roll, 0.0, q[i])
    rig.tool.location = Vector((0.0, 0.0, TOOL_OFFSET))
    rig.tool.rotation_euler = (0.0, 0.0, 0.0)
    rig.last_q = list(q)


def joint_angles(rig: Rig) -> list[float]:
    """Current joint angles in radians (the empties' local Z rotations).
    Note: reads need a fresh view layer for a just-written pose."""
    return [em.rotation_euler.z for em in rig.joint_empties]


def tool0_translation(rig: Rig) -> Vector:
    """tool0 empty's world translation (the rig's own FK)."""
    return rig.tool.matrix_world.to_translation()


def target_position(rig: Rig) -> tuple[float, float, float]:
    return tuple(rig.target.matrix_world.to_translation())
