# SPDX-License-Identifier: BSD-3-Clause
"""Pure-Python SPM leg kinematics for the standalone rig. Slice 2.

    import spm_kinematics as K           # from blender_ik_addon/ on sys.path; no Blender needed

What this is. The closed-loop SPM leg math, written again in Python so a Blender driver can call it
without a browser and without the C solver. It is a MIRROR of the HTML tool, not a second opinion
about it: the mechanism model, the closed-form leg solve, the conditioning and the numerical FK below
are the tool's own formulas, transcribed from its source, and
tests/test_spm_kinematics.py asserts this module against the committed golden fixture row by row. If
this file disagrees with the tool on any of the 27 poses it is wrong, full stop - the fixture, whose
numbers are the tool's own output, is the authority.

What this is NOT. No bpy, no ctypes, no C ABI, no import of the tool or of any test. It is orientation
in, joint angles out; the operator's motor signs, the gear ratio and the hardware offsets live in
spm_driver.py, never here. An orientation here is a matrix or the order-tested quaternion; the tilt/
pan/roll triple is turned into one by orientation_from_tilt_pan_roll below - the tool's IK construction,
pure geometry, which is why it sits here and not in the driver - and the FK readout's decomposition is
never consulted, so nothing in this module can mistake that decomposition for an inverse.

Packaging. This is a RUNTIME module, not dev tooling: pure Python, no bpy, so it is import-safe outside
Blender. When the SPM rig ships it belongs on the package's explicit module list beside arm7_rig.py and
ik_core.py - unlike tools/spm and tools/gates, which are deliberately kept off that list.

Conventions, matching the tool exactly:
  * a quaternion is [w, x, y, z];  quat_to_rot reads q[0] as w and returns a FLAT nine;
  * rot_vec / rot_mul / rot_t / flat_r take and return a FLAT nine (row-major), as the tool does;
  * rodrigues / exp_so3 / mat_vec / mat_mul / solve_fk use a NESTED 3x3, as the tool does;
  * solve_ik returns the WRAPPED-NOT-UNWRAPPED joint angles of the tool's solveIK (the +acos branch),
    which is what the fixture records in theta_rad; mapIK's wrapPi is a separate concern.
Units are metres and radians throughout; millimetres appear only where a name says mm.
"""
from __future__ import annotations

import math

PI = math.pi

# ---- the mechanism model ------------------------------------------------------------------------
# The geometry constants, the tool's CALIB/GOLDEN_INPUTS. A driver may pass its own to mech_from;
# these defaults are the calibrated arm so the kinematics stands alone for the offline suite. The
# motor signs, gear ratio and per-leg offsets are NOT here - spm_driver.py owns those control values.
R1_MM = 45.0
R2_MM = 26.8
H_MM = 31.8
LA_DEG = -90.0
B_DEG = 90.0
DEFAULT_HOME_AZIMUTH = (0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0)
Z3 = (0.0, 0.0, 1.0)


def mech_from(r1_mm=R1_MM, h_mm=H_MM, la_deg=LA_DEG, b_deg=B_DEG, home_azimuth=DEFAULT_HOME_AZIMUTH):
    """The leg frames {w0, v0, cosA2}, the tool's mechFrom transcribed line for line.

    It builds the three crank axes, the three pin directions and the three constraint cosines the way
    updateKinematics does, so a copy of this arithmetic in a caller could not drift from the one the
    screen shows. r2 and the rod length L are returned too because the closure checks need them and
    deriving them anywhere else would be a second definition of the same geometry.
    """
    r1 = r1_mm / 1000.0
    h = h_mm / 1000.0
    r2 = R2_MM / 1000.0
    la_rad = la_deg * PI / 180.0
    b_rad = b_deg * PI / 180.0
    d_r = math.sqrt(max(0.0, r1 * r1 - h * h))
    w0, v0, cos_a2 = [], [], []
    for i in range(3):
        az = home_azimuth[i]
        w0.append(_norm([d_r * math.cos(az), d_r * math.sin(az), -h]))
        a2 = az + la_rad
        v0.append([math.sin(b_rad) * math.cos(a2), math.sin(b_rad) * math.sin(a2), math.cos(b_rad)])
        cos_a2.append(dot(w0[i], v0[i]))
    return {"w0": w0, "v0": v0, "cosA2": cos_a2, "r1": r1, "r2": r2, "h": h,
            "diskR": math.sqrt(max(0.0, r1 * r1 - h * h)),
            "L": math.sqrt(r1 * r1 + r2 * r2 - 2.0 * r1 * r2 * cos_a2[0])}


# ---- vector helpers ------------------------------------------------------------------------------
def _norm(v):
    n = math.hypot(v[0], v[1], v[2]) or 1.0
    return [v[0] / n, v[1] / n, v[2] / n]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


# ---- flat-nine rotation ops (rot_vec/rot_mul/rot_t read R[0..8]) ----------------------------------
def rot_vec(R, v):
    return [R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
            R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
            R[6] * v[0] + R[7] * v[1] + R[8] * v[2]]


def rot_mul(A, B):
    return [A[0] * B[0] + A[1] * B[3] + A[2] * B[6], A[0] * B[1] + A[1] * B[4] + A[2] * B[7], A[0] * B[2] + A[1] * B[5] + A[2] * B[8],
            A[3] * B[0] + A[4] * B[3] + A[5] * B[6], A[3] * B[1] + A[4] * B[4] + A[5] * B[7], A[3] * B[2] + A[4] * B[5] + A[5] * B[8],
            A[6] * B[0] + A[7] * B[3] + A[8] * B[6], A[6] * B[1] + A[7] * B[4] + A[8] * B[7], A[6] * B[2] + A[7] * B[5] + A[8] * B[8]]


def rot_t(R):
    return [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]]


def flat_r(M):
    """Nested 3x3 -> flat nine, row-major. The tool's flatR; sigma_min wants its R in this shape."""
    return [M[0][0], M[0][1], M[0][2], M[1][0], M[1][1], M[1][2], M[2][0], M[2][1], M[2][2]]


def nested_r(R):
    """Flat nine -> nested 3x3, the inverse of flat_r."""
    return [[R[0], R[1], R[2]], [R[3], R[4], R[5]], [R[6], R[7], R[8]]]


# ---- nested 3x3 ops (rodrigues/exp_so3/mat_vec/mat_mul/solve_fk use these) -------------------------
def mat_vec(M, v):
    return [M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
            M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
            M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]]


def mat_mul(A, B):
    R = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    for i in range(3):
        for j in range(3):
            s = 0.0
            for k in range(3):
                s += A[i][k] * B[k][j]
            R[i][j] = s
    return R


def transpose(A):
    return [[A[0][0], A[1][0], A[2][0]], [A[0][1], A[1][1], A[2][1]], [A[0][2], A[1][2], A[2][2]]]


def rodrigues(axis, angle):
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def exp_so3(xi):
    th = math.hypot(xi[0], xi[1], xi[2])
    if th < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return rodrigues([xi[0] / th, xi[1] / th, xi[2] / th], th)


def so3_log(R):
    """The inverse of exp_so3: a nested 3x3 to its axis-angle vector. Used only to seed the FK."""
    tr = R[0][0] + R[1][1] + R[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    th = math.acos(c)
    if th < 1e-12:
        return [0.0, 0.0, 0.0]
    if abs(th - PI) < 1e-9:                                    # near pi: the skew part vanishes, take the
        best, axis = -1.0, [0.0, 0.0, 1.0]                       # dominant column of R+I (symmetric rank-1
        for j in range(3):                                      # there) as the axis instead.
            col = [R[i][j] + (1.0 if i == j else 0.0) for i in range(3)]
            n = math.hypot(col[0], col[1], col[2])
            if n > best:
                best, axis = n, _norm(col)
        return [axis[0] * th, axis[1] * th, axis[2] * th]
    k = 1.0 / (2.0 * math.sin(th))
    axis = [k * (R[2][1] - R[1][2]), k * (R[0][2] - R[2][0]), k * (R[1][0] - R[0][1])]
    return [axis[0] * th, axis[1] * th, axis[2] * th]


def solve_3x3(A, b):
    det = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
           - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
           + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
    if abs(det) < 1e-14:
        return None

    def det_with(col):
        M = [list(A[0]), list(A[1]), list(A[2])]
        for r in range(3):
            M[r][col] = b[r]
        return (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))

    return [det_with(0) / det, det_with(1) / det, det_with(2) / det]


def min_eig_sym3(M):
    """Smallest eigenvalue of a symmetric 3x3, closed form - the tool's minEigSym3 verbatim."""
    m00, m11, m22 = M[0][0], M[1][1], M[2][2]
    m01, m02, m12 = M[0][1], M[0][2], M[1][2]
    q = (m00 + m11 + m22) / 3.0
    p2 = ((m00 - q) ** 2 + (m11 - q) ** 2 + (m22 - q) ** 2 + 2.0 * (m01 * m01 + m02 * m02 + m12 * m12))
    p = math.sqrt(p2 / 6.0)
    if p < 1e-12:
        return q
    b00, b11, b22 = (m00 - q) / p, (m11 - q) / p, (m22 - q) / p
    b01, b02, b12 = m01 / p, m02 / p, m12 / p
    det_b = b00 * (b11 * b22 - b12 * b12) - b01 * (b01 * b22 - b12 * b02) + b02 * (b01 * b12 - b11 * b02)
    phi = math.acos(max(-1.0, min(1.0, det_b / 2.0))) / 3.0
    return q + 2.0 * p * math.cos(phi + 2.0 * PI / 3.0)


# ---- the quaternion helper, order-tested -----------------------------------------------------------
def quat_to_rot(q):
    """[w,x,y,z] -> flat nine. The tool's quatToRot verbatim: it reads q[0] as w. See quat_order_probe."""
    w, x, y, z = q
    n = (w * w + x * x + y * y + z * z) or 1.0
    s = 2.0 / n
    return [1.0 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y),
            s * (x * y + w * z), 1.0 - s * (x * x + z * z), s * (y * z - w * x),
            s * (x * z - w * y), s * (y * z + w * x), 1.0 - s * (x * x + y * y)]


def rot_to_quat(R):
    """Nested 3x3 -> [w,x,y,z]. The canonical four-branch extraction (the tool's spmQuatOf), the one
    that fixed the earlier order bug, sign-normalised so w >= 0 so the pair is unique up to nothing."""
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        w, x, y, z = (R[2][1] - R[1][2]) / s, 0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        w, x, y, z = (R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        w, x, y, z = (R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s
    return [-w, -x, -y, -z] if w < 0 else [w, x, y, z]


def as_orientation(orientation, quat_order="wxyz"):
    """Accept an orientation as a nested 3x3, a flat nine, or a quaternion, and return a NESTED 3x3.

    The quaternion path is the order-tested one: quat_order says whether the four numbers are
    [w,x,y,z] (the tool's convention, the default) or [x,y,z,w]. Feeding the wrong order is not
    silently tolerated - quat_order_probe below shows the two orders give different matrices, so a
    caller that flips it gets a different pose and the suite catches it, rather than a quiet wrong solve.
    """
    seq = list(orientation)
    if len(seq) == 3 and all(isinstance(r, (list, tuple)) for r in seq):
        return [[float(seq[i][j]) for j in range(3)] for i in range(3)]
    if len(seq) == 9:
        return nested_r([float(x) for x in seq])
    if len(seq) == 4:
        q = [float(x) for x in seq]
        if quat_order not in ("wxyz", "xyzw"):
            raise ValueError("quat_order must be 'wxyz' or 'xyzw', not %r" % (quat_order,))
        if quat_order == "xyzw":
            q = [q[3], q[0], q[1], q[2]]
        return nested_r(quat_to_rot(q))
    raise ValueError("an orientation is a 3x3, a flat nine or a length-4 quaternion, got length %d" % len(seq))


def quat_order_probe():
    """Proof the quaternion helper is order-sensitive, so 'wxyz' is a real claim and not a no-op.

    The same four numbers of one non-symmetric rotation are read twice, once as [w,x,y,z] and once as
    [x,y,z,w]. Returns (read_as_wxyz_is_R, read_as_xyzw_is_R): the first True, the second False, which
    together say the helper puts slot 0 as w and would mis-solve an [x,y,z,w] handoff - the exact order
    mistake the canonical four-branch extraction exists to make loud instead of quiet.
    """
    R = mat_mul(rodrigues([1.0, 0.0, 0.0], 0.7), rodrigues([0.0, 1.0, 0.0], -0.4))
    q = rot_to_quat(R)                                              # [w,x,y,z] of R
    right = angle_between(as_orientation(q, quat_order="wxyz"), R) < 1e-9
    misread = angle_between(as_orientation(q, quat_order="xyzw"), R) < 1e-9   # same four numbers, wrong order
    return right, misread


def angle_between(A, B):
    """The rotation angle taking A to B: |log(A^T B)|. The transpose convention the golden test uses."""
    M = mat_mul(transpose(A), B)
    tr = M[0][0] + M[1][1] + M[2][2]
    return math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))


# ---- the closed-form leg solve, the tool's solveIK verbatim --------------------------------------
def solve_ik(w0, v0, cos_a2, R):
    """Orientation -> per-leg joint angles. One root per leg, the +acos branch the tool's solveIK
    takes (atan2(Q, P) + acos(arg)) - that single branch IS the per-leg root choice this port must
    reproduce; mapIK's wrapPi of the same expression is the wrapped variant the map panel uses, not
    this one. A leg whose arg leaves [-1,1] has no root: valid[i] false, theta[i] NaN, never a raise.
    """
    u = [0.0, 0.0, 1.0]
    theta, valid = [0.0, 0.0, 0.0], [False, False, False]
    for i in range(3):
        vtgt = _norm(mat_vec(R, v0[i]))
        P = dot(w0[i], vtgt) - dot(w0[i], u) * dot(u, vtgt)
        Q = dot(cross(u, w0[i]), vtgt)
        rho = cos_a2[i] - dot(w0[i], u) * dot(u, vtgt)
        denom = math.hypot(P, Q)
        arg = rho / denom if denom > 1e-9 else 2.0
        if abs(arg) <= 1.0 + 1e-9:
            valid[i] = True
            theta[i] = math.atan2(Q, P) + math.acos(max(-1.0, min(1.0, arg)))
        else:
            valid[i] = False
            theta[i] = float("nan")
    return {"theta": theta, "valid": valid}


def sigma_min_j(m, R_flat, th):
    """The smallest singular value of the motor-rate-to-platform-angular-velocity Jacobean, the
    tool's sigmaMinJ verbatim: J = -A^-1 diag(b), sigma_min = sqrt(min eig of J^T J). NaN at a parallel
    singularity (|det A| < 1e-9). R_flat is a FLAT nine, exactly as the on-screen readout calls it."""
    a, bd = [], []
    for i in range(3):
        c, s = math.cos(th[i]), math.sin(th[i])
        u = m["w0"][i]
        w = [c * u[0] - s * u[1], s * u[0] + c * u[1], u[2]]
        v = rot_vec(R_flat, m["v0"][i])
        a.append(cross(v, w))
        bd.append(dot(cross(Z3, w), v))
    det = dot(a[0], cross(a[1], a[2]))
    if abs(det) < 1e-9:
        return float("nan")
    cols = [cross(a[1], a[2]), cross(a[2], a[0]), cross(a[0], a[1])]           # columns of adj(A)
    J = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    for k in range(3):
        for j in range(3):
            J[k][j] = -cols[j][k] * bd[j] / det
    M = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    for p in range(3):
        for q in range(3):
            s = 0.0
            for k in range(3):
                s += J[k][p] * J[k][q]
            M[p][q] = s
    return math.sqrt(max(0.0, min_eig_sym3(M)))


def solve_from(orientation, mech=None, quat_order="wxyz"):
    """The public in->out: an orientation (matrix or quaternion) to theta[3], valid[3], sigma_min.

    sigma_min is computed only when every theta is finite - the on-screen readout's `fin` guard, so an
    unreachable pose reports an absent conditioning rather than a NaN the UI would have to special-case.
    """
    m = mech or mech_from()
    R = as_orientation(orientation, quat_order)
    ik = solve_ik(m["w0"], m["v0"], m["cosA2"], R)
    all_fin = all(math.isfinite(t) for t in ik["theta"])
    R_flat = flat_r(R)
    sig = sigma_min_j(m, R_flat, ik["theta"]) if all_fin else float("nan")
    return {"theta": ik["theta"], "valid": ik["valid"], "sigma_min": sig,
            "R": R, "R_flat": R_flat, "fit_ok": bool(ik["valid"][0] and ik["valid"][1] and ik["valid"][2])}


# ---- the IK construction: tilt/pan/roll -> orientation (the tool's spmPoseR) -------------------------
def orientation_from_tilt_pan_roll(tilt_deg, pan_deg, roll_deg):
    """The commanded tilt/pan/roll (degrees) to an orientation, NESTED 3x3 - the tool's spmPoseR, the
    IK branch's own construction: lean +z toward azimuth pan by tilt, then twist by roll about the
    platform z. It is geometry, not calibration, so it lives with the kinematics and holds no motor
    sign, gear ratio or offset; spm_driver.py keeps those. The angles it consumes are the ones the
    operator commanded and so own the pose - which is exactly why the tilt/pan/roll shown back to the
    operator must come from THIS construction's convention and never from the FK readout's decomposition."""
    axis_tp = [math.cos(pan_deg * PI / 180.0), math.sin(pan_deg * PI / 180.0), 0.0]
    return mat_mul(rodrigues(axis_tp, tilt_deg * PI / 180.0),
                   rodrigues([0.0, 0.0, 1.0], roll_deg * PI / 180.0))


# ---- the closure geometry (what the fixture prints and the band measures) --------------------------
def tips_and_pins(m, theta, R_flat):
    """The rod's two ends: tip = pivot + r1 (Rz(theta_i) w0_i), pin = pivot + r2 (R . v0_i). Kept here
    because the FK round trip and the closure checks both need them and neither may re-derive them."""
    pivot = [0.000283, -0.000274, 0.333433]                       # CALIB.pivot; cancels in every diff, carried for the shape
    tips, pins = [], []
    for i in range(3):
        tips.append(add(pivot, scale(mat_vec(rodrigues([0.0, 0.0, 1.0], theta[i]), m["w0"][i]), m["r1"])))
        pins.append(add(pivot, scale(rot_vec(R_flat, m["v0"][i]), m["r2"])))
    return {"crankTips": tips, "platPts": pins}


def pin_errors_mm(tips, pins):
    return [math.hypot(*(sub(pins[i], tips[i]))) * 1000.0 for i in range(3)]


def closure_residual_mm(pin_to_pin_mm, L_m):
    """|tip->pin - L| per leg: the quantity the 0.5 mm band belongs to (on the length itself a band of
    0.5 mm is one nothing can enter, which is why the readout colours this, not the raw distance)."""
    return [abs(d - L_m * 1000.0) for d in pin_to_pin_mm]


# ---- the numerical forward kinematics, theta -> orientation ----------------------------------------
def residual_fk(xi, w_arr, v0, cos_a2):
    R = exp_so3(xi)
    return [dot(w_arr[i], mat_vec(R, v0[i])) - cos_a2[i] for i in range(3)]


def solve_fk(w_arr, v0, cos_a2, xi0):
    """Gauss-Newton on exp_so3, the tool's solveFK verbatim: drive the three closure residuals
    (Rz(theta_i) w0_i) . (R v0_i) = cosA2_i to zero and return the axis-angle xi of the R that does."""
    xi = list(xi0)
    lam = 1e-3
    for _ in range(40):
        r0 = residual_fk(xi, w_arr, v0, cos_a2)
        norm0 = math.hypot(*r0)
        if norm0 < 1e-11:
            break
        eps = 1e-6
        J = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        for c in range(3):
            xip = list(xi)
            xip[c] += eps
            rp = residual_fk(xip, w_arr, v0, cos_a2)
            for row in range(3):
                J[row][c] = (rp[row] - r0[row]) / eps
        JtJ = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        Jtr = [0.0, 0.0, 0.0]
        for aa in range(3):
            for bb in range(3):
                s = 0.0
                for k in range(3):
                    s += J[k][aa] * J[k][bb]
                JtJ[aa][bb] = s
            s2 = 0.0
            for k in range(3):
                s2 += J[k][aa] * r0[k]
            Jtr[aa] = s2
        for aa in range(3):
            JtJ[aa][aa] += lam
        dx = solve_3x3(JtJ, [-v for v in Jtr])
        if dx is None:
            break
        xi_try = [xi[0] + dx[0], xi[1] + dx[1], xi[2] + dx[2]]
        r_try = residual_fk(xi_try, w_arr, v0, cos_a2)
        if math.hypot(*r_try) < norm0:
            xi = xi_try
            lam = max(lam * 0.7, 1e-8)
        else:
            lam *= 3.0
            if lam > 1e6:
                break
    return xi


def fk_to_orientation(theta, mech=None, prev_R=None):
    """Joint angles -> orientation, the numerical FK. Returns None when a leg has no angle (an
    unreachable pose has no forward orientation to recover, and a caller reports that, never crashes).

    Live use passes prev_R, the orientation the previous tick's FK returned (the panel caches it). A root
    is first required to be a real one on its own terms - residuals vanish AND feeding it back through
    solve_ik returns these very angles modulo a turn - but that test alone cannot separate the physical
    assembly from its crossed-assembly twin: the 3-rod closure folds, and a twin that is a finite angle
    away can close AND re-solve to the same theta (the differential shows such twins a full pi apart on
    some poses). The one thing that does separate them is history, which is the tool's own mechanism for
    staying on one branch. So when prev_R is given, every re-solving root the seeds reach is ranked by how
    near it is to the previous frame and the nearest is returned - one Gauss-Newton pass from prev_R, a tick
    moves the platform only a little, lands back in the SAME assembly mode, and the crossed twin (far from
    prev_R) loses the ranking. That single pass from prev_R - closing, re-solving, and staying near prev_R
    at once - is the ordinary live case and is accepted on its own, one run rather than the sweep, so a live
    tick never pays for the full net. With prev_R absent (a cold solve) the deterministic seed net is the fallback.
    'seed' says which path produced the answer.
    """
    if not all(math.isfinite(t) for t in theta):
        return None
    m = mech or mech_from()
    w_arr = [mat_vec(rodrigues([0.0, 0.0, 1.0], theta[i]), m["w0"][i]) for i in range(3)]
    prev_R_m = as_orientation(prev_R) if prev_R is not None else None

    def _hit(R, resid, rmax, tag):
        return {"R": R, "R_flat": flat_r(R), "residual": resid, "residual_max": rmax,
                "converged": True, "seed": tag}

    def _resid_best(R, resid, rmax):
        return {"R": R, "R_flat": flat_r(R), "residual": resid, "residual_max": rmax,
                "converged": False, "seed": "none"}

    # Fast path, and the common one: seed the Gauss-Newton from the previous frame. A live tick moves the
    # platform only a little, so if that single pass BOTH closes the loop (residual ~0) AND re-solves back to
    # these very angles AND lands near prev_R, it is the physical continuation - and the crossed twin is a
    # finite angle away, so nothing else can sit nearer to prev_R. Accepting on those three facts costs ONE
    # GN run, not the whole seed net, which is what keeps a tick far inside the budget. If it does not qualify
    # (cold, or prev_R itself sat on the twin, or it did not converge), sweep the net and choose among the
    # re-solving roots by proximity to prev_R - dropping the twin - or, cold, take the first (net order fixed).
    if prev_R_m is not None:
        xi = solve_fk(w_arr, m["v0"], m["cosA2"], so3_log(prev_R_m))
        R = exp_so3(xi)
        resid = residual_fk(xi, w_arr, m["v0"], m["cosA2"])
        rmax = max(abs(x) for x in resid)
        if (rmax < 1e-9 and angle_between(R, prev_R_m) < 0.5
                and _theta_matches(solve_ik(m["w0"], m["v0"], m["cosA2"], R)["theta"], theta)):
            return _hit(R, resid, rmax, "prev")

    resolving = []
    best = None
    for seed in _fk_seed_net(w_arr, m["v0"]):
        xi = solve_fk(w_arr, m["v0"], m["cosA2"], seed)
        R = exp_so3(xi)
        resid = residual_fk(xi, w_arr, m["v0"], m["cosA2"])
        rmax = max(abs(x) for x in resid)
        if rmax < 1e-9 and _theta_matches(solve_ik(m["w0"], m["v0"], m["cosA2"], R)["theta"], theta):
            if prev_R_m is None:
                return _hit(R, resid, rmax, "net")
            resolving.append((angle_between(R, prev_R_m), R, resid, rmax))
        if best is None or rmax < best["residual_max"]:
            best = _resid_best(R, resid, rmax)
    if resolving:                                                # seeded: nearest re-solving root to prev is the physical one
        dist, R, resid, rmax = min(resolving, key=lambda t: t[0])
        return _hit(R, resid, rmax, "net")
    return best


def _fk_seed(w_arr, v0):
    """The unique linear map R_lin with R_lin . v0_i = w_arr_i, one step of the polar decomposition to
    bring it near SO(3), then its log: a starting rotation from the angles and geometry only."""
    V = [[v0[c][r] for c in range(3)] for r in range(3)]           # columns are v0_i
    W = [[w_arr[c][r] for c in range(3)] for r in range(3)]        # columns are w_arr_i
    cols = []
    for j in range(3):                                             # R_lin = W V^-1, column by column
        e = [1.0 if k == j else 0.0 for k in range(3)]
        c = solve_3x3(V, e)
        if c is None:
            return [0.0, 0.0, 0.0]
        cols.append(c)
    Vinv = [[cols[j][i] for j in range(3)] for i in range(3)]
    R_lin = mat_mul(W, Vinv)
    R_inv_t = transpose(R_lin)
    polar = [[0.5 * (R_lin[i][j] + R_inv_t[i][j]) for j in range(3)] for i in range(3)]
    return so3_log(polar)


def _wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def _theta_matches(th_a, th_b, tol=1e-6):
    """Whether two joint-angle triples describe the same configuration, each compared modulo a full
    turn. The joint angle is periodic, so the +acos root near a 2pi seam reads 4.147 one moment and
    -2.136 the next off a 1e-16 nudge; an unwrapped subtraction would call that a miss when it is not."""
    if len(th_a) != len(th_b):
        return False
    for a, b in zip(th_a, th_b):
        if not (math.isfinite(a) and math.isfinite(b)):
            return False
        if abs(_wrap_pi(a - b)) > tol:
            return False
    return True


_FK_AXES = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
            (0.7071, 0.7071, 0.0), (0.0, 0.7071, 0.7071), (0.7071, 0.0, 0.7071),
            (0.5774, 0.5774, 0.5774))
_FK_ANGS = (0.0, PI / 4.0, PI / 2.0, 3.0 * PI / 4.0, PI, -PI / 4.0, -PI / 2.0, -3.0 * PI / 4.0)


def _fk_seed_net(w_arr, v0):
    """The starting rotations the FK is attempted from: the flat pose, a spread of axis-angle probes,
    and the pin-to-tip map. Enough basins to capture the physical root even when the flat pose and the
    pin-to-tip map both fall into a mirror mode of the same three rod constraints."""
    seeds = [[0.0, 0.0, 0.0]]
    for a in _FK_AXES:
        for t in _FK_ANGS:
            seeds.append([a[0] * t, a[1] * t, a[2] * t])
    seeds.append(_fk_seed(w_arr, v0))
    return seeds
