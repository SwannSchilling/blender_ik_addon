"""ctypes wrapper around the pick_ik_c C ABI (libpick_ik_core).

This module is the only place the add-on touches the shared library. The
DLL is built from the sibling repo `libpick-ik-core` (target `pick_ik_c`);
the C ABI contract is documented in `include/pick_ik_c/pickik_c.h` of that
repo:

- meters / radians everywhere;
- poses are 16 doubles, ROW-major 4x4 in the standard homogeneous layout
  `[R | t; 0 0 0 1]` (translation in column 3);
- position-only goal: `orientation_threshold < 0` and the default
  `rotation_scale = 0.0`.

The arm7 model is compiled into the DLL (`pickik_arm7_robot_create` /
`pickik_arm7_link_fk`), so no Python FK callback is needed: solves are
millisecond-class and never touch the GIL pump.
"""

from __future__ import annotations

import ctypes
import os
from typing import Sequence

# --- result codes (mirror pickik_c.h) --------------------------------------
OK = 0
E_BADARG = -1
E_NOMEM = -2
E_FK = -3
E_INTERNAL = -4

N_JOINTS = 7
N_FRAMES = N_JOINTS + 1  # 7 pivots + tool0

# --- ctypes structures (mirror the C PODs) ----------------------------------


class _Options(ctypes.Structure):
    _fields_ = [
        ("position_threshold", ctypes.c_double),
        ("orientation_threshold", ctypes.c_double),  # < 0 => position-only
        ("cost_threshold", ctypes.c_double),
        ("position_scale", ctypes.c_double),
        ("rotation_scale", ctypes.c_double),
        ("minimal_displacement_weight", ctypes.c_double),
        ("joint_target_weight", ctypes.c_double),
        ("joint_target_values", ctypes.POINTER(ctypes.c_double)),
        ("joint_target_has", ctypes.POINTER(ctypes.c_int)),
        ("has_look_at", ctypes.c_int),
        ("look_at_point", ctypes.c_double * 3),
        ("look_at_axis", ctypes.c_double * 3),
        ("look_at_weight", ctypes.c_double),
    ]


class _Result(ctypes.Structure):
    _fields_ = [
        ("success", ctypes.c_int),
        ("position_error", ctypes.c_double),
        ("orientation_error", ctypes.c_double),
        ("time_ms", ctypes.c_double),
    ]


# C link-FK callback: int (void* user, int n, const double* q, double* frames)
_FkFn = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_int,
                         ctypes.POINTER(ctypes.c_double),
                         ctypes.POINTER(ctypes.c_double))


class CoreError(RuntimeError):
    pass


class Core:
    """One loaded pick_ik_c.dll (robot + solver handles live here)."""

    def __init__(self, dll_path: str):
        self.dll_path = dll_path
        self.lib = ctypes.CDLL(dll_path)
        self._bind_signatures()
        self._arm7_link_fk_c = _FkFn(self.lib.pickik_arm7_link_fk)
        self._arm7_link_fk_keepalive = self._arm7_link_fk_c  # prevent GC
        self._robot = ctypes.c_void_p()
        self.lib.pickik_arm7_robot_create(ctypes.byref(self._robot))
        if not self._robot:
            raise CoreError("pickik_arm7_robot_create failed")
        # Keep the CFUNCTYPE instance alive for the process lifetime — the
        # solver calls it on every FK evaluation.
        self._solvers: dict[str, ctypes.c_void_p] = {}

    def _bind_signatures(self) -> None:
        L = self.lib
        L.pickik_options_default.argtypes = [ctypes.POINTER(_Options)]
        L.pickik_options_default.restype = None
        L.pickik_arm7_robot_create.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        L.pickik_arm7_robot_create.restype = ctypes.c_int
        L.pickik_arm7_link_fk.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                          ctypes.POINTER(ctypes.c_double),
                                          ctypes.POINTER(ctypes.c_double)]
        L.pickik_arm7_link_fk.restype = ctypes.c_int
        L.pickik_ccd_create.argtypes = [ctypes.c_int, ctypes.c_double,
                                        ctypes.c_double,
                                        ctypes.POINTER(ctypes.c_void_p)]
        L.pickik_ccd_create.restype = ctypes.c_int
        L.pickik_gradient_create.argtypes = [ctypes.c_double, ctypes.c_double,
                                             ctypes.c_double, ctypes.c_int,
                                             ctypes.c_int,
                                             ctypes.POINTER(ctypes.c_void_p)]
        L.pickik_gradient_create.restype = ctypes.c_int
        L.pickik_memetic_create.argtypes = [ctypes.c_int, ctypes.c_int,
                                            ctypes.c_double, ctypes.c_int,
                                            ctypes.c_double, ctypes.c_int,
                                            ctypes.c_int, ctypes.c_int,
                                            ctypes.POINTER(ctypes.c_void_p)]
        L.pickik_memetic_create.restype = ctypes.c_int
        L.pickik_solver_free.argtypes = [ctypes.c_void_p]
        L.pickik_solver_free.restype = ctypes.c_int
        L.pickik_robot_is_valid.argtypes = [ctypes.c_void_p,
                                            ctypes.POINTER(ctypes.c_double),
                                            ctypes.POINTER(ctypes.c_int)]
        L.pickik_robot_is_valid.restype = ctypes.c_int
        L.pickik_solve.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                   _FkFn, ctypes.c_void_p,
                                   ctypes.POINTER(ctypes.c_double),
                                   ctypes.POINTER(ctypes.c_double),
                                   ctypes.POINTER(_Options),
                                   ctypes.POINTER(ctypes.c_double),
                                   ctypes.POINTER(_Result)]
        L.pickik_solve.restype = ctypes.c_int

    def solver(self, kind: str, **params) -> ctypes.c_void_p:
        """Create (and cache) a solver handle. See pickik_c.h for parameters."""
        if kind not in self._solvers:
            L = self.lib
            p = ctypes.c_void_p()
            if kind == "ccd":
                # 100 passes: CCD is a LOCAL method — from the previous pose
                # (continuous mode) it converges in a fraction of these. 600
                # (the C default) costs ~15 ms/call and only buys margin on
                # unlucky seeds; one-shot solves from far seeds belong to
                # gradient/memetic.
                L.pickik_ccd_create(int(params.get("max_passes", 100)),
                                    float(params.get("damping", 0.1)),
                                    float(params.get("epsilon", 1e-8)),
                                    ctypes.byref(p))
            elif kind == "gradient":
                L.pickik_gradient_create(float(params.get("step_size", 1e-4)),
                                         float(params.get("min_cost_delta", 1e-12)),
                                         float(params.get("max_time", 2.0)),
                                         int(params.get("max_iterations", 2000)),
                                         1, ctypes.byref(p))
            elif kind == "memetic":
                L.pickik_memetic_create(int(params.get("elite_size", 2)),
                                        int(params.get("population_size", 16)),
                                        float(params.get("wipeout_fitness_tol", 1e-5)),
                                        int(params.get("max_generations", 100)),
                                        float(params.get("max_time", 4.0)),
                                        int(params.get("num_threads", 4)),
                                        1, 1, ctypes.byref(p))
            else:
                raise CoreError(f"unknown solver {kind!r}")
            if not p:
                raise CoreError(f"{kind} solver create failed")
            self._solvers[kind] = p
        return self._solvers[kind]

    def fk_tool0(self, q: Sequence[float]) -> tuple[tuple[float, float, float],
                                                    list[float]]:
        """arm7 FK through the C ABI. Returns (tool0 translation, 16 row-major)."""
        frames = (ctypes.c_double * (N_FRAMES * 16))()
        qarr = (ctypes.c_double * N_JOINTS)(*q)
        rc = self.lib.pickik_arm7_link_fk(None, N_JOINTS, qarr, frames)
        if rc != OK:
            raise CoreError(f"arm7 link FK failed: rc={rc}")
        t0 = tuple(frames[7 * 16 + 4 * r + 3] for r in range(3))
        return t0, [frames[i] for i in range(16 * N_FRAMES)]

    def solve(self, kind: str, target_xyz_m: Sequence[float],
              seed: Sequence[float],
              position_only: bool = True,
              orientation_quat: tuple[float, float, float, float] | None = None,
              md_weight: float = 0.0,
              joint_targets: tuple[float | None, ...] | None = None,
              jt_weight: float = 0.0,
              look_at_point_m: tuple[float, float, float] | None = None,
              look_at_axis: tuple[float, float, float] = (1.0, 0.0, 0.0),
              la_weight: float = 0.0) -> dict:
        """One IK solve. Returns {success, q, position_error, orientation_error, time_ms}."""
        solver = self.solver(kind)

        target = (ctypes.c_double * 16)()
        # Standard homogeneous row-major: identity rotation + column-3 t.
        for i in (0, 5, 10, 15):
            target[i] = 1.0
        target[3] = target_xyz_m[0]
        target[7] = target_xyz_m[1]
        target[11] = target_xyz_m[2]

        options = _Options()
        self.lib.pickik_options_default(ctypes.byref(options))
        options.position_threshold = 1e-3
        options.cost_threshold = 1e-3
        options.position_scale = 1.0
        options.minimal_displacement_weight = md_weight
        if position_only or orientation_quat is None:
            options.orientation_threshold = -1.0
            options.rotation_scale = 0.0
        else:
            options.orientation_threshold = 1e-3
            options.rotation_scale = 0.5
            self._quat_into_target(target, orientation_quat)
        if jt_weight > 0.0 and joint_targets:
            values = (ctypes.c_double * N_JOINTS)(*[0.0 if v is None else v
                                                    for v in joint_targets])
            has = (ctypes.c_int * N_JOINTS)(* [1 if v is not None else 0
                                               for v in joint_targets])
            options.joint_target_values = values
            options.joint_target_has = has
            options.joint_target_weight = jt_weight
            # keepalive (pointers must outlive the call)
            options._keepalive = (values, has)
        if la_weight > 0.0 and look_at_point_m:
            options.has_look_at = 1
            for i, v in enumerate(look_at_point_m):
                options.look_at_point[i] = v
            for i, v in enumerate(look_at_axis):
                options.look_at_axis[i] = v
            options.look_at_weight = la_weight

        seedarr = (ctypes.c_double * N_JOINTS)(*seed)
        qout = (ctypes.c_double * N_JOINTS)()
        res = _Result()
        rc = self.lib.pickik_solve(solver, self._robot, self._arm7_link_fk_c, None,
                                   seedarr, target, ctypes.byref(options), qout,
                                   ctypes.byref(res))
        if rc != OK:
            raise CoreError(f"pickik_solve failed: rc={rc} "
                            f"({rc: #x})" if rc in (E_BADARG, E_NOMEM, E_FK, E_INTERNAL)
                            else f"pickik_solve failed: rc={rc}")
        return {
            "success": bool(res.success),
            "q": [qout[i] for i in range(N_JOINTS)],
            "position_error": res.position_error,
            "orientation_error": res.orientation_error,
            "time_ms": res.time_ms,
        }

    @staticmethod
    def _quat_into_target(target, quat: tuple[float, float, float, float]) -> None:
        """Write the [x,y,z,w] quaternion as the rotation part (column-major C++
        side re-normalizes; keep it unit length here)."""
        import math
        x, y, z, w = quat
        n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        x, y, z, w = x / n, y / n, z / n, w / n
        m = (
            1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
            2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
            2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y),
        )
        for r in range(3):
            for c in range(3):
                target[4 * r + c] = m[3 * r + c]


def find_dll(explicit: str = "") -> str:
    """Locate pick_ik_c.dll. Order: explicit path > $PICKIK_C_DLL > next to
    the add-on > the sibling repo's build tree."""
    candidates = []
    if explicit:
        candidates.append(explicit)
    if os.environ.get("PICKIK_C_DLL"):
        candidates.append(os.environ["PICKIK_C_DLL"])
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, "pick_ik_c.dll"))
    candidates.append(os.path.join(here, os.pardir, "libpick-ik-core", "build",
                                   "Release", "pick_ik_c.dll"))
    for c in candidates:
        c = os.path.abspath(c)
        if os.path.isfile(c):
            return c
    raise CoreError(
        "pick_ik_c.dll not found. Looked in: " + "; ".join(candidates)
        + ". Build it with: cmake --build libpick-ik-core/build --config Release "
        "--target pick_ik_c — or set the DLL path in the panel.")
