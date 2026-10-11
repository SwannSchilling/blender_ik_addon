# SPDX-License-Identifier: BSD-3-Clause
"""Standalone tests for the PURE half of mcp_handlers_obs — plain CPython, no Blender.

    python test_mcp_handlers_pure.py     # from blender_ik_addon/

`mcp_handlers_obs` runs inside Blender, so normally only test_mcp_bridge.py exercises
it — over the wire, with a live tick. But its rig-addressing decisions answer BEFORE
anything touches bpy (`_rig_id`, `_desc_for`, `_nums`, `_in_bounds` are pure), and
those are the decisions that decide whether an invented `rig` id gets a clean
E_INVAL or a half-executed command. This file loads the real module against a stub
`bpy` and a stub add-on package whose `rig_types` is the real registry module, and
checks those decisions directly — what the bridge suite can only reach by asking the
whole machine a question, this asks the function itself.

A test is (name, callable); it PASSES iff the callable returns. Same tiny harness
as test_mcp_protocol.py.
"""
from __future__ import annotations

import os
import sys
import types
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))         # blender_ik_addon/tests
_ADDON = os.path.dirname(_HERE)                             # blender_ik_addon
for _p in (_ADDON, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# --- the stubs. bpy is imported by the module and never used by the functions under test.
_BPY = types.ModuleType("bpy")
sys.modules.setdefault("bpy", _BPY)
# The handlers resolve the add-on through sys.modules["blender_ik_addon"] (_addon()); hand
# them a stub package whose rig_types is the REAL registry module, and import the handlers
# bare so their `from . import mcp_protocol` falls back to the top-level import.
if "blender_ik_addon" not in sys.modules:
    _PKG = types.ModuleType("blender_ik_addon")
    _PKG.__path__ = [_ADDON]
    sys.modules["blender_ik_addon"] = _PKG
else:
    _PKG = sys.modules["blender_ik_addon"]

import mcp_protocol as P                                            # noqa: E402
import rig_types as R                                               # noqa: E402
import mcp_handlers_obs as H                                        # noqa: E402

PASS, FAIL = [], []


def test(fn):                                                      # tiny, no deps
    try:
        fn()
        PASS.append(fn.__name__)
    except BaseException:
        lines = traceback.format_exception_only()                  # the stdlib spelling of this helper
        FAIL.append((fn.__name__, lines[-1].strip()))
        print(f"FAIL {fn.__name__}")
        print("".join(lines))
    return fn


class _FakeRig:                                                    # the rig_types install input
    N_JOINTS = 7

    class Rig:
        @staticmethod
        def find():
            return None

    @staticmethod
    def build(target_m = None):
        return "built"

    @staticmethod
    def apply_q(rig, q):
        return None

    @staticmethod
    def joint_angles(rig):
        return (0.0,) * 7

    @staticmethod
    def rig_object_names():
        return ["Arm7_Base"] + [f"Arm7_J{i}" for i in range(1, 8)]

    @staticmethod
    def target_position(rig):
        return (0.0, 0.0, 0.0)


if R.get_rig_type("arm7") is None:                                 # as __init__.py would have
    R.install_builtin_rigs(_FakeRig)
_PKG.rig_types = R                                                 # what _addon() hands out offline


def inval(fn, *a, **kw):
    """Run a pure gate, insist it answers E_INVAL, and hand back its message."""
    try:
        fn(*a, **kw)
    except P.McpError as e:
        assert e.code == P.ERR.INVAL, f"{e.code} where E_INVAL was due"
        return e.message
    raise AssertionError("expected an E_INVAL, got a plain return")


# -- the dispatch table: the invariant the bridge lives on ------------------------------------
@test
def handlers_cover_exactly_the_commands_this_build_answers():
    assert frozenset(H.HANDLERS) <= frozenset(P.COMMANDS), \
        f"handler with no command: {frozenset(H.HANDLERS) - frozenset(P.COMMANDS)}"
    assert len(H.HANDLERS) == 14, f"the MVP answers 14 commands, now {len(H.HANDLERS)}"
    for cmd in ("status", "get_state", "get_robot_info", "validate_pose", "build_rig",
                "delete_rig", "export_urdf", "solve_ik", "set_target", "get_target",
                "set_joint_angles", "set_solver", "set_solver_config", "set_continuous"):
        assert cmd in H.HANDLERS, cmd
    for name in H.HANDLERS.values():
        assert callable(name)


# -- the rig-addressing decisions (pure: they run on the worker lane, off the tick) -------------
@test
def an_omitted_rig_is_the_default_rig_named_not_a_panel_read():
    assert H._rig_id({}) == "arm7"
    assert H._rig_id({"rig": None}) == "arm7"
    assert H._rig_id({"rig": ""}) == "arm7"
    assert H._rig_id({"rig": "  "}) == "arm7"                      # the wire's default must not reach bpy
    assert H._rig_id({"rig": R.DEFAULT_RIG_ID}) == "arm7"


@test
def a_named_rig_is_trimmed_and_a_scalar_is_not_secretly_accepted():
    assert H._rig_id({"rig": "  arm7  "}) == "arm7"
    # a JSON scalar for a string param is coerced to its text and then refused by the lookup,
    # never silently mapped onto the default rig:
    assert H._rig_id({"rig": 7}) == "7"
    inval(lambda: H._desc_for({"rig": 7}))


@test
def an_invented_rig_is_refused_naming_the_registered_ones():
    assert H._desc_for({}) is R.get_rig_type("arm7")               # omitted = the default, resolved
    msg = inval(lambda: H._desc_for({"rig": "arm8"}))
    assert "arm8" in msg and "arm7" in msg, msg                    # the caller is told what exists


@test
def a_second_registered_rig_needs_no_handler_change_to_address():
    spur = R.RigType(rig_id="spur", title="Spur", n_joints=4,
                     joint_names=("J1", "J2", "J3", "J4"),
                     fk_prop_names=tuple(f"fk_p{i}" for i in range(1, 5)),
                     q_prop_names=tuple(f"q_p{i}" for i in range(1, 5)),
                     hw_family=R.HW_NONE, capabilities=("manual_fk",),
                     build=lambda **kw: None, apply_q=lambda rig, q: None,
                     read_q=lambda rig: (), object_names=lambda: [],
                     find=lambda: None, target_position=lambda rig: (0.0, 0.0, 0.0))
    R.register_rig(spur)
    try:
        assert H._desc_for({"rig": "spur"}) is spur
        assert H._rig_id({"rig": "spur"}) == "spur"
    finally:
        del R.RIGS["spur"]                                          # no unregister API; leave as found


# -- the numeric-shape gate every array argument goes through ---------------------------------
@test
def nums_accepts_a_clean_list_of_the_right_count():
    assert H._nums([0.0, 1.0, 2.0], 3, "q_rad") == [0.0, 1.0, 2.0]
    assert H._nums((1, 2), 2, "q_rad") == [1.0, 2.0]              # ints and tuples are fine


@test
def nums_refuses_every_wrong_shape_with_one_code_and_words():
    assert inval(lambda: H._nums([1.0] * 6, 7, "seed_q")) == "seed_q needs 7 values, got 6"
    assert inval(lambda: H._nums("[1,2,3]", 3, "q_rad")).startswith("q_rad must be a list of 3 numbers")
    assert inval(lambda: H._nums(b"[1,2,3]", 3, "q_rad")).startswith("q_rad must be a list of 3 numbers")
    assert inval(lambda: H._nums(1.0, 3, "q_rad")).startswith("q_rad must be a list of 3 numbers")
    assert inval(lambda: H._nums({"a": 1}, 3, "q_rad")).startswith("q_rad must be a list of 3 numbers")
    msg = inval(lambda: H._nums(["1", "x"], 2, "q_rad"))
    assert "entries must be numbers" in msg and "'x'" in msg, msg


@test
def the_seed_width_the_handler_will_ask_for_is_the_rigs_own_count():
    # the R3 pair to this file's header promise: what h_solve_ik asks _nums for is
    # rt.n_joints values, so a 6-wide seed against a 7-joint rig is a refusal this
    # gate answers (the bridge suite pins the wire-level E_INVAL; this pins the word
    # it names and that the count comes from the descriptor, not a literal).
    rt = H._desc_for({"rig": "arm7"})
    assert inval(lambda: H._nums([0.0] * 6, rt.n_joints, "seed_q")) == "seed_q needs 7 values, got 6"
    assert H._nums([0.0] * rt.n_joints, rt.n_joints, "seed_q") == [0.0] * 7


@test
def in_bounds_reports_every_violating_joint_by_index():
    lower, upper = [-3.0] * 7, [3.0] * 7
    assert H._in_bounds([0.0] * 7, lower, upper) == []
    bad = H._in_bounds([0.0, 4.0, -4.0] + [0.0] * 4, lower, upper)
    assert [b["index"] for b in bad] == [1, 2], bad                # h_validate_pose reports these as
    assert [b["value"] for b in bad] == [4.0, -4.0], bad           #   "violations", h_get_state as "valid"
    assert all(b["lower"] == -3.0 and b["upper"] == 3.0 for b in bad), bad
    # the tolerance the gate actually carries: a hair over the bound is inside, a
    # thousandth is not (the q_* clamps and this must not disagree about that edge):
    assert H._in_bounds([3.0 + 1e-10] + [0.0] * 6, lower, upper) == []
    assert len(H._in_bounds([3.0 + 1e-6] + [0.0] * 6, lower, upper)) == 1


# -- summary ------------------------------------------------------------------------------------
print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("failing:", ", ".join(n for n, _ in FAIL))
sys.exit(1 if FAIL else 0)
