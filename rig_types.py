# SPDX-License-Identifier: BSD-3-Clause
"""rig_types — the rig registry: one declarative descriptor per rigged body.

Phase 1 introduces the seam the arm7_SPM roadmap (PHASE0_RIG_AUDIT.md §6) needs:
every rig-scoped code path (operators, panel, drivers, MCP handlers) addresses a
rig by ``rig_id`` through this registry instead of importing ``arm7_rig`` and
hardcoding ``7``. ``arm7`` is the first and only entry; nothing here changes its
behaviour — the descriptor names the same module-level functions the add-on
already calls (build / apply_q / joint_angles / rig_object_names / Rig.find).

A rig is *data* plus six hooks. The hooks are plain module functions so a rig
implementation never needs to import this file, and this file never needs to
import a rig implementation: :func:`install_builtin_rigs` wires the shipped
descriptor from the module handed to it, called once by ``__init__.py`` after it
has imported ``arm7_rig``.

Convention contract (frame contract FC-1, PHASE0_RIG_AUDIT.md §5): joints are
URDF-style local-Z revolute steps, the fixed roll lives in the joint table,
angles in radians, lengths in metres, every property name is a *string* on the
scene's ``pickik`` PropertyGroup (``fk_j*``/``q_j*``) so a new rig registers
under its own names instead of the add-on hardcoding arm7's.
"""
from __future__ import annotations

from typing import Callable

#: The rig every rig-scoped call addresses when the caller does not name one —
#: the panel's selector (`pickik.rig`) and every server-side command default to
#: it. Phase 1: exactly "arm7", which is what "zero behavior change" means here.
DEFAULT_RIG_ID = "arm7"

#: Driver families a rig's actuators speak to (`hw_family`). Phase 1 binds the
#: CubeMars CAN driver to arm7 only; a rig that says "none" gets the solver,
#: the panel and the FK paths, but no actuator binding (the STS3215 serial
#: driver is the Phase 2 drop-in behind its own family tag).
HW_NONE = "none"
HW_CUBEMARS = "cubemars"


class RigType:
    """One rig's whole identity, as data (PHASE0_RIG_AUDIT.md §6).

    Frozen-by-convention (``__setattr__`` refused): the registry hands this
    object to operators, timers and handlers, and a mutated descriptor would
    desync two call paths from one another while both stayed "correct". Build
    new rigs by registering a new RigType, never by editing one live.

    Fields
        rig_id           stable key: "arm7". Names scene objects' prefix and the
                         MCP ``rig`` parameter's accepted value.
        title            one honest line for the panel and for get_robot_info.
        n_joints         joint count — the number the add-on must never hardcode
                         again (every ``range(7)``/``(None,)*7`` reads this).
        joint_names      ("J1"...) display names, len == n_joints.
        fk_prop_names    scene PropertyGroup names of the manual-FK degree
                         sliders ("fk_j1"...), len == n_joints.
        q_prop_names     scene PropertyGroup names of the radian readouts
                         ("q_j1"...), len == n_joints.
        solvers          the solver names this rig accepts; handlers and the
                         panel offer exactly these (arm7: the three shared ones).
        hw_family        driver family (HW_* above); "none" = no actuator path.
        capabilities     what this rig's implementation actually supports;
                         operators refuse the rest with an honest error
                         instead of half-running. Known: "manual_fk", "urdf",
                         "hw_stream", "continuous".
        build            callable(target_m=(x,y,z)) -> a live rig instance.
        apply_q          callable(rig, q) -> poses the rig by software FK.
        read_q           callable(rig) -> Sequence[float] radians, current pose.
        object_names     callable() -> every object name the rig owns.
        find             callable() -> adopt a matching hierarchy already in
                         the scene, or None.
        target_position  callable(rig) -> (x, y, z) of the rig's target in m.
    """

    __slots__ = ("rig_id", "title", "n_joints", "joint_names", "fk_prop_names",
                 "q_prop_names", "solvers", "hw_family", "capabilities",
                 "build", "apply_q", "read_q", "object_names", "find",
                 "target_position")

    def __init__(self, *, rig_id: str, title: str, n_joints: int,
                 joint_names: tuple[str, ...], fk_prop_names: tuple[str, ...],
                 q_prop_names: tuple[str, ...],
                 solvers: tuple[str, ...] = ("ccd", "gradient", "memetic"),
                 hw_family: str = HW_NONE,
                 capabilities: frozenset[str] = frozenset(),
                 build: Callable[..., object], apply_q: Callable, read_q: Callable,
                 object_names: Callable[[], list[str]], find: Callable[[], object],
                 target_position: Callable) -> None:
        for name, value in (
                ("rig_id", rig_id), ("title", title), ("n_joints", n_joints),
                ("joint_names", joint_names), ("fk_prop_names", fk_prop_names),
                ("q_prop_names", q_prop_names), ("solvers", solvers),
                ("hw_family", hw_family), ("capabilities", frozenset(capabilities)),
                ("build", build), ("apply_q", apply_q), ("read_q", read_q),
                ("object_names", object_names), ("find", find),
                ("target_position", target_position)):
            object.__setattr__(self, name, value)
        self._check()

    def __setattr__(self, *_):                        # frozen — see docstring
        raise AttributeError("RigType is frozen; register a new one instead")

    def _check(self) -> None:
        """Validate the descriptor at registration: a malformed rig must not
        survive the import that declares it (same rule as the command
        catalogue's import-time self-check in mcp_protocol)."""
        assert isinstance(self.rig_id, str) and self.rig_id, "rig_id must be a non-empty str"
        assert "[" not in self.rig_id and "]" not in self.rig_id and "," not in self.rig_id \
            and '"' not in self.rig_id and "'" not in self.rig_id, \
            f"rig_id {self.rig_id!r} cannot double as a safe object-name prefix"
        assert isinstance(self.n_joints, int) and self.n_joints > 0, "n_joints must be positive"
        for name, table in (("joint_names", self.joint_names),
                            ("fk_prop_names", self.fk_prop_names),
                            ("q_prop_names", self.q_prop_names)):
            assert len(table) == self.n_joints, \
                f"{name} has {len(table)} entries, expected {self.n_joints}"
            assert all(isinstance(s, str) and s for s in table), f"{name} must be non-empty str"
        assert self.hw_family in (HW_NONE, HW_CUBEMARS), f"unknown hw_family {self.hw_family!r}"
        for hook in ("build", "apply_q", "read_q", "object_names", "find", "target_position"):
            assert callable(getattr(self, hook)), f"hook {hook} must be callable"

    def __repr__(self) -> str:                          # pragma: no-cover
        return (f"<RigType {self.rig_id!r} n={self.n_joints} hw={self.hw_family} "
                f"caps={sorted(self.capabilities)}>")


#: The registry. Populated by :func:`install_builtin_rigs` at add-on import;
#: the stdio server never imports this module (it AST-reads the handlers and
#: renders tools from the catalogue — a new rig costs zero server code).
RIGS: dict[str, RigType] = {}


def register_rig(rig_type: RigType) -> RigType:
    """Add a rig. Double-registering the same rig_id is a bug, not an update:
    two descriptors under one key would mean two truths about one rig."""
    assert rig_type.rig_id not in RIGS, \
        f"rig {rig_type.rig_id!r} is already registered ({RIGS[rig_type.rig_id]!r})"
    RIGS[rig_type.rig_id] = rig_type
    return rig_type


def rig_ids() -> tuple[str, ...]:
    return tuple(sorted(RIGS))


def get_rig_type(rig_id: str) -> RigType | None:
    """Lookup without opinion: None says 'not registered' (handlers turn that
    into E_INVAL naming the known ids; operators turn it into a reported
    CANCELLED). Never raises, so no call path can 500 on a typo."""
    return RIGS.get(rig_id)


def resolve_rig_id(rig_id: str, scene_props) -> str:
    """The one rule for an empty/omitted rig: the caller's name, else the
    panel's selector, else the default rig. '' from a server caller therefore
    means 'whatever the user selected' — which until Phase 2 is 'arm7', so
    every existing caller keeps addressing exactly the rig it addresses today."""
    given = (rig_id or "").strip()
    if given:
        return given
    try:
        selected = str(getattr(scene_props, "rig", "") or "").strip()
    except (ReferenceError, RuntimeError):
        selected = ""
    return selected or DEFAULT_RIG_ID


def descriptor_for(rig: object) -> RigType:
    """The descriptor a live rig instance belongs to. ``_rig_or_die`` stamps
    ``rig_type`` on every rig it builds or adopts; the stamp is optional to
    construct (test harnesses build rigs directly through the rig module), so
    an unstamped instance falls back through its ``rig_id`` and then to the
    default — an honest lookup for the single-rig world, never a guess of a
    second one's data."""
    desc = getattr(rig, "rig_type", None)
    if desc is not None:
        return desc
    rid = getattr(rig, "rig_id", None) or DEFAULT_RIG_ID
    return RIGS[rid]


def install_builtin_rigs(arm7_module) -> None:
    """Register the shipped rig from the module handed in (``blender_ik_addon.
    arm7_rig``, imported by ``__init__.py`` — this module deliberately does not
    import it, so the registry stays importable without touching bpy).

    Pure delegation: every hook is the very function the add-on called
    directly before the registry existed."""
    register_rig(RigType(
        rig_id="arm7",
        title="arm7 — 7-DOF desktop arm (CubeMars CAN)",
        n_joints=arm7_module.N_JOINTS,
        joint_names=tuple(f"J{i}" for i in range(1, arm7_module.N_JOINTS + 1)),
        fk_prop_names=tuple(f"fk_j{i}" for i in range(1, arm7_module.N_JOINTS + 1)),
        q_prop_names=tuple(f"q_j{i}" for i in range(1, arm7_module.N_JOINTS + 1)),
        solvers=("ccd", "gradient", "memetic"),
        hw_family=HW_CUBEMARS,
        capabilities=frozenset(("manual_fk", "urdf", "hw_stream", "continuous")),
        build=lambda target_m: arm7_module.build(target_m=target_m),
        apply_q=arm7_module.apply_q,
        read_q=arm7_module.joint_angles,
        object_names=arm7_module.rig_object_names,
        find=arm7_module.Rig.find,
        target_position=arm7_module.target_position,
    ))
