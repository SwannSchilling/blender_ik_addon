# SPDX-License-Identifier: BSD-3-Clause
"""The standalone SPM rig in Blender: a closed Stewart platform as an empty hierarchy.

    from blender_ik_addon import spm_rig          # inside the add-on; needs bpy (Blender only)

Slice 3's deliverable is deliberately narrow: the rig BUILDS, and forward kinematics reproduces the
golden poses. There is no IK target and no panel polish yet - those are the next slice.

What the rig is. The arm7 rig is a serial chain, so its end-effector empty's matrix_world is just the
product of the joint steps. The SPM is a closed loop: three crank angles DO NOT compose into a platform
orientation by chaining - the platform orientation is the root of three rod-closure constraints, which is
exactly what spm_kinematics.fk_to_orientation solves. So this rig keeps two kinds of empty:

    SPM_Base            the root, identity
    SPM_Crank1..3       the passively-carried ROD of one actuator. Its empty's local Z rotation IS the
                        commanded crank angle (theta), readable/keyframeable/drivable the way Arm7_Ji's is,
                        and it carries the rod around the vertical motor axis through the pivot; the rod's
                        MESH additionally takes the tool's PASSIVE closed-loop roll about the rod's own axis
                        (the visual helper that makes it read like the real model), derived from the platform.
    SPM_Disk1..3        the MOTOR disc each actuator drives: it turns by theta about the vertical axis. This
                        is the "rotation" a viewer sees driving the mechanism; the rod/crank is only carried
                        by it plus its own passive roll - the two are easy to conflate, which is why they are
                        named separately here.
    SPM_Platform        THE output. Its matrix_world rotation is fk_to_orientation(theta), turning about the
                        platform's own centre (CALIB.pivot), seeded from the previous frame so a live tick
                        does not jump to a mirror assembly mode
    SPM_Target          a handle for the next slice's IK; unused by FK

The joint table is "3 cranks": three revolute-about-Z steps at the base, one per actuator. The mesh table
is the tool's own (Platform_v2_1, AxisConnector_v3_{1,2,3}, Disk_{1,2,3}). Object names carry the rig_id
as their prefix ("SPM_"), so a clone can hold arm7 and SPM in one scene and every rig-scoped path still
addresses one of them by rig_id.

Frame contract (FC-1, and the tool's own convention). Joint math is metres and radians only. The STL
vertices are millimetres from Fusion, so SCALE=0.001 is folded in at the MESH, never in a joint; and a
mesh hangs off its body through C = translate(0,0,C_Z) x Rx(90deg), the global CAD-to-model correction the
tool draws with. Neither SCALE nor C ever touches the platform orientation the golden replay checks - the
platform empty's rotation is the mechanism-frame R, nothing else.

Runtime imports are package-relative (from . import ...) because this module only ever loads inside the
add-on under Blender. The offline tests that exercise the same numbers import spm_kinematics standalone
and never touch this file.
"""
from __future__ import annotations

import math
import os
import struct

import bpy
from mathutils import Matrix, Quaternion, Vector

from . import spm_kinematics as K

RIG_ID = "spm"
PREFIX = "SPM"                                     # the rig_id is the object-name prefix
N_JOINTS = 3                                        # three cranks; the add-on must read this, not hardcode

#: the closed-loop mechanism the port solves. One instance per rig build; shared, stateless.
_MECH = K.mech_from()

# -- mesh-attachment frame. These three are the tool's geometry constants (SPMGEOM), and they belong to
#    the mesh, never to a joint: SCALE scales STL vertices mm->m, C is the CAD-to-model correction.
SCALE = 0.001
C_Z = 0.2
RX90_Q = (0.707107, 0.707107, 0.0, 0.0)           # [w,x,y,z]; Rx(90deg) as the tool states it

# the platform's spherical centre - CALIB.pivot in the tool, THE point its "body rotates about" (line ~602)
# and the one the joint axis is aimed at exactly (line ~652). Like C_Z and RX90_Q this is a METRES
# mechanism-frame display constant taken from the tool (SPMGEOM/CALIB): presentation only, never joint
# math (FC-1). The platform empty turns about THIS point, not the world origin, so the platform spins on
# its own centre instead of orbiting the world centre.
PLATFORM_PIVOT = (0.000283, -0.000274, 0.333433)

# the crank bodies' kinematic origins = BODY_Q0.crank_i.p in the tool, evaluated by the tool's OWN chainT
# over its SPMGEOM MJCF chains (see _scratch/_crank_centers.mjs). Presentation geometry only, like the
# pivot: it fixes each crank's pivot point - the axis through this body origin toward the sphere centre
# about which the crank's PASSIVE spin acts. It never enters the closure or the platform's R.
CRANK_CENTERS = (
    (-0.015556, -0.008981, 0.2352),
    (0.015556, -0.008981, 0.2352),
    (0.0, 0.017963, 0.2352),
)

BASE_NAME = f"{PREFIX}_Base"
PLATFORM_NAME = f"{PREFIX}_Platform"
TARGET_NAME = f"{PREFIX}_Target"
CRANK_NAMES = [f"{PREFIX}_Crank{i + 1}" for i in range(N_JOINTS)]
DISK_NAMES = [f"{PREFIX}_Disk{i + 1}" for i in range(N_JOINTS)]
MESH_PREFIX = f"{PREFIX}_Mesh_"

# the joint table: three revolute-about-Z crank steps at the base. One actuator each; the fixed roll of a
# serial link has no meaning in a closed loop, so every crank leans 0 and turns about local Z by theta.
JOINTS = ((Vector((0.0, 0.0, 0.0)), 0.0),) * N_JOINTS

#: the tool's mesh table: which STL hangs off which body. Eight files - the whole mechanism including the
#: static base (the bench) so the complete model shows on build; only the translucent GhostOverlay stays out
#: (a display choice, not a kinematic one). Every part shares the universal SCALE + C frame correction and
#: rides its body's pose; the base rides the base empty, which is never re-posed, so it stays put like the
#: name says while still being a member of the assembly.
MESH_SPEC = (
    ("StaticBase.stl", BASE_NAME),                                  # static assembly part on the never-moved base empty
    ("Platform_v2_1.stl", PLATFORM_NAME),
    ("AxisConnector_v3_1.stl", CRANK_NAMES[0]), ("AxisConnector_v3_2.stl", CRANK_NAMES[1]),
    ("AxisConnector_v3_3.stl", CRANK_NAMES[2]),
    ("Disk_1.stl", DISK_NAMES[0]), ("Disk_2.stl", DISK_NAMES[1]), ("Disk_3.stl", DISK_NAMES[2]),
)

MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes", "spm")


# --------------------------------------------------------------------------- mesh folders --
def _resolve_mesh_dir() -> str:
    """Where the SPM STLs are. An explicit env or scene path wins (the headless gate points this at the
    tool's SPM/meshes before the binaries are committed); else the package's own meshes/spm. Resolving a
    real folder beats trusting a loader's literal - and lets the rig build whether or not the STLs are in
    the package yet."""
    env = os.environ.get("SPM_MESH_DIR", "").strip()
    if env and os.path.isdir(env):
        return env
    try:
        scene_path = bpy.context.scene.pickik.meshes_path.strip()
    except (AttributeError, RuntimeError):
        scene_path = ""
    if scene_path and os.path.isdir(os.path.join(scene_path, "spm")):
        return os.path.join(scene_path, "spm")
    if scene_path and os.path.isdir(scene_path) and os.path.isfile(os.path.join(scene_path, "Platform_v2_1.stl")):
        return scene_path
    return MESH_DIR


# --------------------------------------------------------------------------- STL, and the version shim --
def _read_stl(path):
    """Parse a binary or ASCII STL to triangle vertex triples, WITHOUT any unit assumption. Scaling is
    the caller's (the frame contract: SCALE on vertices, always, because these are mm from Fusion)."""
    with open(path, "rb") as fh:
        raw = fh.read()
    if raw[:5] == b"solid":
        verts = []
        for line in raw.decode("ascii", errors="replace").splitlines():
            s = line.strip()
            if s.startswith("vertex"):
                parts = s.split()
                if len(parts) >= 4:
                    verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
        faces = [(i, i + 1, i + 2) for i in range(0, len(verts), 3)]
        return verts, faces
    n = struct.unpack("<I", raw[80:84])[0]
    verts, faces, off = [], [], 84
    for _ in range(n):
        chunk = raw[off:off + 50]
        if len(chunk) < 50:
            break
        v = struct.unpack("<12fH", chunk)
        i = len(verts)
        verts += [(v[3], v[4], v[5]), (v[6], v[7], v[8]), (v[9], v[10], v[11])]
        faces.append((i, i + 1, i + 2))
        off += 50
    return verts, faces


def _native_stl_import_op():
    """The STL import operator this Blender actually offers, or None. 3.4 names it import_mesh.stl,
    4.x names it wm.stl_import; the shim tries both so one build works across the version boundary the
    rig must register on. Returned as a callable so the caller decides whether to trust it."""
    ops = (("wm", "stl_import"), ("import_mesh", "stl"))
    for mod, name in ops:
        group = getattr(bpy.ops, mod, None)
        call = getattr(group, name, None) if group is not None else None
        if call is not None:
            return call, f"{mod}.{name}"
    return None, None


def _new_mesh_datablock(name, verts, faces):
    """A mesh straight from parsed triangles; the fallback that makes the rig build even where no STL
    operator is enabled (and the deterministic path the golden replay leans on)."""
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata([(v[0], v[1], v[2]) for v in verts], [], faces)
    mesh.validate(verbose=False)
    return mesh


def _build_mesh(rig, stl_name, parent_name):
    """Load one SPM STL and hang it off its body through C, with SCALE applied to the vertices. Returns
    (object, how) - how says which import path was taken, so the gate can report the shim resolved."""
    path = os.path.join(_resolve_mesh_dir(), stl_name)
    if not os.path.isfile(path):
        return None, "missing"
    obj_name = MESH_PREFIX + os.path.splitext(stl_name)[0]
    stale = bpy.data.objects.get(obj_name)
    if stale is not None:
        bpy.data.objects.remove(stale, do_unlink=True)

    call, idname = _native_stl_import_op()
    built, how = None, None
    if call is not None:
        try:                                                       # try the real importer first
            before = set(bpy.data.objects)
            call(filepath=path, files=[os.path.basename(path)], directory=os.path.dirname(path))
            fresh = [o for o in bpy.data.objects if o not in before and o.type == 'MESH']
            if fresh:
                src = fresh[0]
                built = src.data
                bpy.data.objects.remove(src, do_unlink=True)       # keep the data-block, drop its object
                how = idname
        except Exception:
            built = None
    if built is None:                                              # deterministic fallback
        verts, faces = _read_stl(path)
        if not verts:
            return None, "empty"
        scaled = [(v[0] * SCALE, v[1] * SCALE, v[2] * SCALE) for v in verts]   # mm -> m, vertices only
        built = _new_mesh_datablock(obj_name, scaled, faces)
        how = "bundled-parser"

    obj = bpy.data.objects.new(obj_name, built)
    bpy.context.scene.collection.objects.link(obj)
    parent = bpy.data.objects.get(parent_name) if parent_name else None   # parent None = a world-anchored visual dummy
    obj.parent = parent
    obj.matrix_parent_inverse = Matrix.Identity(4)
    obj.matrix_basis = _c_local()                                  # mesh_world = body_world x C
    obj.display_type = 'SOLID'
    obj.hide_render = True
    return obj, how


def _c_local():
    """C = translate(0,0,C_Z) x Rx(90deg) as a local 4x4, from the tool's Rx90 quaternion. Written as
    [R | p; 0 1] directly (that product IS the homogeneous form) so it needs no matrix multiply - and so
    a mesh's matrix_basis reads back with R in the top-left block and C_Z in the third of the last column."""
    r3 = K.nested_r(K.quat_to_rot(list(RX90_Q)))                  # tool quaternion -> nested 3x3
    return Matrix((
        (r3[0][0], r3[0][1], r3[0][2], 0.0),
        (r3[1][0], r3[1][1], r3[1][2], 0.0),
        (r3[2][0], r3[2][1], r3[2][2], C_Z),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _basis_from_R(R):
    """The platform's local transform straight from the mechanism-frame rotation matrix R - the object the
    golden replay measures. Set from the matrix rather from a quaternion on purpose: R is the very matrix
    the differential and the golden check prove equals the tool's, so the platform's matrix_world IS that
    rotation up to Blender's single-precision storage, and there is no quaternion convention between us
    and Blender's to trust for the thing the whole gate rests on."""
    return Matrix((
        (R[0][0], R[0][1], R[0][2], 0.0),
        (R[1][0], R[1][1], R[1][2], 0.0),
        (R[2][0], R[2][1], R[2][2], 0.0),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _basis_about_pivot(R, pivot):
    """The platform's local transform from the mechanism-frame rotation R, but turning about the FIXED
    world point `pivot` (the platform's spherical centre) rather than about the world origin. It is the
    homogeneous form of Translate(pivot) x Rot(R) x Translate(-pivot): a local point c maps to R.c +
    (pivot - R.pivot), so the local point `pivot` stays pinned at the world pivot for every R. The 3x3
    rotation block is exactly R, so the golden replay - which reads matrix_world's rotation block only -
    is untouched; what changes is that the platform and everything parented to it (meshes included) orbit
    their own centre instead of the world origin. Presentation only: it never touches the R the closure
    produced."""
    p = (float(pivot[0]), float(pivot[1]), float(pivot[2]))
    t = [p[r] - (R[r][0] * p[0] + R[r][1] * p[1] + R[r][2] * p[2]) for r in range(3)]
    return Matrix((
        (R[0][0], R[0][1], R[0][2], t[0]),
        (R[1][0], R[1][1], R[1][2], t[1]),
        (R[2][0], R[2][1], R[2][2], t[2]),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _pin_azimuth(w, v):
    """Azimuth of a direction v about an axis w (right-handed about w), measured from the projection of
    world Z onto the plane normal to w. Verbatim port of the tool's pinAzimuth (SPMMATH lane); it is the
    one number the passive crank roll is read from."""
    z = (0.0, 0.0, 1.0)
    zw = K.dot(z, w)
    vw = K.dot(v, w)
    zp = K._norm([z[k] - w[k] * zw for k in range(3)])
    vp = K._norm([v[k] - w[k] * vw for k in range(3)])
    return math.atan2(K.dot(w, K.cross(zp, vp)), K.dot(zp, vp))


def _wrap_pi(a):
    """Wrap to (-pi, pi] - the tool's wrapPi, so a full turn of the pin never reads as a jump."""
    return math.atan2(math.sin(a), math.cos(a))


def crank_passive_spin(theta_i, R_plat, w0_i, v0_i):
    """The PASSIVE closed-loop roll of a crank about its own axis: how far the platform pin direction v has
    turned about the crank axis w relative to home. Verbatim port of the tool's cmAuto block (the viewer's
    axis points -w, hence the minus). This is a visual-only helper that makes the crank read like the real
    model - it is NOT a driven motor angle and it never feeds the closure."""
    w_cur = K.mat_vec(K.rodrigues((0.0, 0.0, 1.0), theta_i), list(w0_i))
    v_cur = K.mat_vec(R_plat, list(v0_i))
    roll = _wrap_pi(_pin_azimuth(w_cur, v_cur) - _pin_azimuth(list(w0_i), list(v0_i)))
    return -roll                                              # tool: spin = tune.flip ? roll : -roll (flip off by default)


def _crank_local_spin(theta_i, R_plat, w0_i, v0_i, center, spin_rad=None, axis=None, pivot_pt=None):
    """The crank rod's LOCAL transform: a roll `spin_rad` about the rod's real hinge line - the tip -> sphere
    centre line - applied on top of the shared C. That line's direction is -w0 (the tool's applyCrankDefaults
    moves the rotation centre onto the tip, so the hinge is tip->pivot, whose direction is -w0), and it is
    expressed in the HOME frame: correct, because the crank EMPTY already carries it by Rz(theta) about the
    pivot, so the carried frame's tip/pivot line is this home line rotated with the disc. When `spin_rad` is
    None it uses the passive closed-loop roll (the tool's cmAuto default). A roll about this hinge cannot move
    the LOWER contact (the tip sits ON the axis) and it is what brings the UPPER contact onto the platform pin;
    a roll about any line NOT through the tip slides the lower contact off the disc pin (that is the whole bug
    this guards against). `center` (the body origin) is no longer the spin's axis - kept only for signature.
    Presentation only: it is carried by the platform's pose and never feeds the closure."""
    if axis is None:
        axis = K._norm([-c for c in w0_i])                                     # tip -> sphere centre = -w0, the real hinge
    else:
        axis = K._norm([float(c) for c in axis])                              # an explicit override direction
    if spin_rad is None:
        spin_rad = crank_passive_spin(theta_i, R_plat, w0_i, v0_i)
    pivot_pt = list(PLATFORM_PIVOT) if pivot_pt is None else [float(c) for c in pivot_pt]   # a point on the hinge (the sphere centre)
    R_spin = K.rodrigues(axis, spin_rad)
    return _basis_about_pivot(R_spin, pivot_pt) @ _c_local()


def _crank_mesh(rig, i):
    """The mesh body hanging off crank i (its AxisConnector rod), or None before the meshes are built."""
    em = rig.crank_empties[i]
    for o in getattr(rig, "mesh_objects", None) or []:
        if o.parent is em:
            return o
    return None


# --------------------------------------------------------------------------- empties and the rig --
def _make_empty(name, parent, collection):
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_size = 0.04
    if collection is not None:
        collection.objects.link(empty)
    if parent is not None:
        empty.parent = parent
        empty.matrix_parent_inverse = Matrix.Identity(4)
    return empty


class Rig:
    """The SPM empty hierarchy. joint_empties holds the three crank empties so the shared add-on paths
    (which index joint_empties by the descriptor's n_joints) treat the SPM like any other rig; the
    platform empty is the FK output and the thing the golden replay measures."""

    def __init__(self, crank_empties, disk_empties, base, platform, target,
                 last_q, last_R=None, mesh_objects=None):
        self.joint_empties = crank_empties          # the shared paths read this; it is the cranks
        self.crank_empties = crank_empties
        self.disk_empties = disk_empties
        self.base = base
        self.platform = platform
        self.target = target
        self.tool = platform                        # the add-on's generic 'tool' is the output link
        self.last_q = last_q
        self.last_R = last_R                        # the platform orientation the last FK returned
        self.mesh_objects = mesh_objects if mesh_objects is not None else []
        self.mech = _MECH

    @staticmethod
    def find():
        if bpy.data.objects.get(BASE_NAME) is None:
            return None
        want = [BASE_NAME, PLATFORM_NAME, TARGET_NAME, *CRANK_NAMES, *DISK_NAMES]
        objs = {n: bpy.data.objects.get(n) for n in want}
        if any(v is None for v in objs.values()):
            return None
        return Rig([objs[n] for n in CRANK_NAMES], [objs[n] for n in DISK_NAMES],
                   objs[BASE_NAME], objs[PLATFORM_NAME], objs[TARGET_NAME], [0.0] * N_JOINTS)

    def unlink_all(self):
        for obj in ([self.base, self.platform, self.target] + self.crank_empties
                    + self.disk_empties + list(self.mesh_objects)):
            try:
                bpy.data.objects.remove(obj, do_unlink=True)
            except (ReferenceError, RuntimeError):
                pass

    def alive(self):
        try:
            for obj in ([self.base, self.platform, self.target] + self.crank_empties
                        + self.disk_empties + self.mesh_objects):
                obj.name
            return True
        except (ReferenceError, RuntimeError):
            return False


def rig_object_names():
    """Every object the rig owns, all prefixed with the rig_id, for cleanup and stale checks."""
    return [BASE_NAME, PLATFORM_NAME, TARGET_NAME, *CRANK_NAMES, *DISK_NAMES,
            *(MESH_PREFIX + os.path.splitext(s[0])[0] for s in MESH_SPEC)]


def build(q=None, target_m=(0.0, 0.0, 0.2)):
    """Build (or replace) the rig. q = the three crank angles, radians; default zero. Mirrors arm7.build:
    replace semantics first so a half-deleted rig cannot leave a second platform in the scene."""
    q = list(q) if q is not None else [0.0] * N_JOINTS
    assert len(q) == N_JOINTS
    for name in rig_object_names():
        obj = bpy.data.objects.get(name)
        if obj is not None:
            bpy.data.objects.remove(obj, do_unlink=True)

    collection = bpy.context.scene.collection
    base = _make_empty(BASE_NAME, None, collection)
    base.empty_display_size = 0.10
    cranks = [_make_empty(CRANK_NAMES[i], base, collection) for i in range(N_JOINTS)]
    disks = [_make_empty(DISK_NAMES[i], base, collection) for i in range(N_JOINTS)]
    platform = _make_empty(PLATFORM_NAME, base, collection)
    platform.empty_display_type = 'SINGLE_ARROW'
    platform.empty_display_size = 0.08
    target = _make_empty(TARGET_NAME, None, collection)
    target.empty_display_type = 'PLAIN_AXES'
    target.empty_display_size = 0.12
    target.location = Vector(target_m)

    rig = Rig(cranks, disks, base, platform, target, list(q))
    bpy.context.view_layer.update()                # let the empties land so the meshes can parent to them

    mesh_objects, how = [], {}
    for stl_name, parent_name in MESH_SPEC:
        mo, tag = _build_mesh(rig, stl_name, parent_name)
        if mo is not None:
            mesh_objects.append(mo)
            how[stl_name] = tag
    rig.mesh_objects = mesh_objects
    rig.mesh_import = how

    apply_q(rig, q, seed_from_prev=False)          # pose platform, discs and the crank rods' passive spin once every part exists
    bpy.context.view_layer.update()
    return rig


def apply_q(rig, q, prev_R=None, seed_from_prev=True):
    """Pose the SPM from three crank angles, by the port's numerical FK.

    seed_from_prev is the live path: the FK is seeded from the previous frame's platform orientation so a
    tick lands back in the SAME assembly mode (no mirror jump between frames). A fresh build, or the
    golden replay of independent poses, passes seed_from_prev=False so each pose is solved cold and the
    answer does not depend on what was posed before it. Either way the root is chosen by the property
    that defines the physical one (it re-solves back to these angles), so seeding can bias discovery but
    can never pick a root that is not the physical one."""
    q = list(q)
    assert len(q) == N_JOINTS
    finite = all(math.isfinite(t) for t in q)
    R_plat = None

    # The DISKS are the motor-driven members: each turns about the vertical motor axis through the pivot (the
    # sphere centre) by its own channel's angle, so the discs and the cranks keep turning about one common centre
    # and stay coupled - Crank i drives Rod i, the physical truth; there is no operator re-assignment. Their
    # meshes hang off via C.
    for i, disk in enumerate(rig.disk_empties):
        disk.matrix_basis = _basis_about_pivot(K.rodrigues((0.0, 0.0, 1.0), q[i]), PLATFORM_PIVOT)

    if finite:
        seed = (rig.last_R if (prev_R is None and seed_from_prev) else prev_R)
        res = K.fk_to_orientation(q, rig.mech, seed)
        if res is None or not res["converged"]:
            res = K.fk_to_orientation(q, rig.mech, None)   # cold net: independent of any prior frame
        if res is not None and res["converged"]:
            rig.platform.matrix_basis = _basis_about_pivot(res["R"], PLATFORM_PIVOT)   # rotation block = the tool's R, but it turns about the platform's own centre (CALIB.pivot), not the world origin
            rig.last_R = res["R"]
            R_plat = res["R"]
        else:
            rig.last_R = None

    # The CRANKS are passively-carried rods, not second motors. The empty keeps the disc carry about the
    # vertical through the pivot (a pure Rz, so it still reads its own angle on the joint-angle path); the
    # MESH then rolls the rod about its real hinge - the tip -> sphere centre line (direction -w0) - by the tool's
    # closed-loop passive amount. Because that hinge passes THROUGH the tip, the roll never slides the lower
    # (disc-pin) contact while it brings the upper contact onto the platform pin; the spin is expressed in the
    # home frame because the empty carries it. Pure presentation, carried by the platform's pose and never fed
    # back to the closure, so the platform's R is byte-identical to the solver's answer.
    m = rig.mech
    for i, em in enumerate(rig.crank_empties):
        a = q[i]
        em.rotation_mode = 'XYZ'
        em.matrix_basis = _basis_about_pivot(K.rodrigues((0.0, 0.0, 1.0), a), PLATFORM_PIVOT)
        mo = _crank_mesh(rig, i)
        if mo is not None:
            w0i, v0i = m["w0"][i], m["v0"][i]
            if R_plat is None:
                mo.matrix_basis = _c_local()          # this leg has not closed the loop yet: pure carry, no passive available
                continue
            # the rod's only real spin axis is the tip -> pivot hinge; roll it about that by the passive closed-loop amount
            mo.matrix_basis = _crank_local_spin(a, R_plat, w0i, v0i, CRANK_CENTERS[i],
                                                spin_rad=crank_passive_spin(a, R_plat, w0i, v0i))
    rig.last_q = list(q)


def joint_angles(rig):
    """The three crank angles now, radians - the empties' local Z, the scriptable readout."""
    return [em.rotation_euler.z for em in rig.crank_empties]


def platform_orientation(rig):
    """The FK output as a nested 3x3, read back off the platform empty - what the golden replay checks."""
    m = rig.platform.matrix_world
    return [[m[r][c] for c in range(3)] for r in range(3)]


def target_position(rig):
    return tuple(rig.target.matrix_world.to_translation())
