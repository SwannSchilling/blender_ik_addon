# SPDX-License-Identifier: BSD-3-Clause
"""Standalone tests for rig_types — run with plain CPython, no Blender needed.

    python test_rig_types.py             # from blender_ik_addon/

`rig_types` is the seam Phase 1 cut (PHASE0_RIG_AUDIT.md §6): it imports only
`typing`, never bpy and never a rig implementation, so everything the registry
promises must be checkable here, outside Blender — the in-Blender suites can
only see one rig being used, never the contract itself. The arm7 entry is
installed here from a FAKE module handed to `install_builtin_rigs` (exactly
what `__init__.py` does with the real `arm7_rig` after importing it), so what
these tests pin is the delegation itself: the descriptor must name the very
functions the module passed in, with per-joint tables built from its N_JOINTS.

A test is (name, callable); it PASSES iff the callable returns. Same tiny
harness as test_mcp_protocol.py.
"""
from __future__ import annotations

import os
import sys
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))         # blender_ik_addon/tests
_ADDON = os.path.dirname(_HERE)                             # blender_ik_addon  (where the module is)
for _p in (_ADDON, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import rig_types as R                                              # noqa: E402

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


class _FakeRig:
    """Just enough of an arm7_rig module to be installed: the same names
    __init__.py hands over from the real one."""
    N_JOINTS = 7

    class Rig:
        @staticmethod
        def find():
            return "found-hierarchy"

    @staticmethod
    def build(target_m = None):
        return ("built", target_m)

    @staticmethod
    def apply_q(rig, q):
        return ("applied", rig, tuple(q))

    @staticmethod
    def joint_angles(rig):
        return ("read", rig)

    @staticmethod
    def rig_object_names():
        return ["Arm7_Base"] + [f"Arm7_J{i}" for i in range(1, 8)]

    @staticmethod
    def target_position(rig):
        return (0.1, 0.2, 0.3)


def _mk(**over):
    """A valid RigType with one field replaced (for the _check rejections)."""
    kw = dict(rig_id="spur", title="Spur", n_joints=4,
              joint_names=("J1", "J2", "J3", "J4"),
              fk_prop_names=("fk_s1", "fk_s2", "fk_s3", "fk_s4"),
              q_prop_names=("q_s1", "q_s2", "q_s3", "q_s4"),
              hw_family=R.HW_NONE, capabilities=("manual_fk",),
              build=lambda **kw: None, apply_q=lambda rig, q: None,
              read_q=lambda rig: (), object_names=lambda: [],
              find=lambda: None, target_position=lambda rig: (0.0, 0.0, 0.0))
    kw.update(over)
    return R.RigType(**kw)


# install the shipped entry from the fake — exactly the call __init__.py makes
R.install_builtin_rigs(_FakeRig)
ARM = R.RIGS[R.DEFAULT_RIG_ID]


# -- the shipped registration ------------------------------------------------------
@test
def default_rig_is_arm7_and_is_registered():
    assert R.DEFAULT_RIG_ID == "arm7"
    assert R.rig_ids() == ("arm7",)
    assert R.get_rig_type("arm7") is ARM
    assert R.get_rig_type("nope") is None                        # lookup never raises on a typo


@test
def descriptor_is_built_from_the_modules_own_joint_count():
    assert ARM.n_joints == _FakeRig.N_JOINTS == 7
    assert ARM.joint_names == tuple(f"J{i}" for i in range(1, 8))
    assert ARM.fk_prop_names == tuple(f"fk_j{i}" for i in range(1, 8))
    assert ARM.q_prop_names == tuple(f"q_j{i}" for i in range(1, 8))
    assert ARM.solvers == ("ccd", "gradient", "memetic")
    assert ARM.hw_family == R.HW_CUBEMARS
    assert sorted(ARM.capabilities) == ["continuous", "hw_stream", "manual_fk", "urdf"]


@test
def hooks_delegate_to_the_very_functions_the_module_handed_over():
    # The registry must add no behavior of its own: every hook is the module's
    # function (build wrapped only to take the keyword), so the pre-registry
    # call and the registry call are the same call.
    assert ARM.build(target_m=(1.0, 2.0, 3.0)) == ("built", (1.0, 2.0, 3.0))
    assert ARM.apply_q("rig", (0.1,) * 7) == ("applied", "rig", (0.1,) * 7)
    assert ARM.read_q("rig") == ("read", "rig")
    assert ARM.object_names() == _FakeRig.rig_object_names()
    assert ARM.find() == "found-hierarchy"
    assert ARM.target_position("rig") == (0.1, 0.2, 0.3)


@test
def descriptor_is_frozen():
    # Two call paths holding one descriptor must not be able to desync it.
    try:
        ARM.n_joints = 8
    except AttributeError:
        pass
    else:
        raise AssertionError("RigType must refuse mutation after registration")


@test
def repr_names_the_rig_its_tables_and_its_family():
    t = repr(ARM)
    assert "arm7" in t and "n=7" in t and R.HW_CUBEMARS in t


# -- the import-time self-check (_check): a malformed rig must not survive it ------
@test
def check_rejects_malformed_descriptors():
    broken = {
        "empty_id": dict(rig_id=""),
        "id_with_a_list_bracket": dict(rig_id="arm[7]"),
        "id_with_a_comma": dict(rig_id="arm,7"),
        "id_with_quotes": dict(rig_id='a"b'),
        "zero_joints": dict(n_joints=0),
        "short_joint_names": dict(joint_names=("J1", "J2", "J3")),
        "short_fk_table": dict(fk_prop_names=("fk_s1",) * 3),
        "short_q_table": dict(q_prop_names=("q_s1", "q_s2", "q_s3", "q_s4", "q_s5")),
        "empty_property_name": dict(fk_prop_names=("fk_s1", "", "fk_s3", "fk_s4")),
        "unknown_hw_family": dict(hw_family="sts3215"),
        "hook_not_callable": dict(build="not-a-function"),
    }
    for why, over in broken.items():
        try:
            _mk(**over)
        except AssertionError:
            continue                                              # the rejection we want
        raise AssertionError(f"_check let a malformed descriptor through: {why}")


# -- registration -----------------------------------------------------------------
@test
def registering_the_same_id_twice_is_a_bug_not_an_update():
    try:
        R.register_rig(ARM)
    except AssertionError:
        pass
    else:
        raise AssertionError("two descriptors under one key would be two truths about one rig")


@test
def a_second_rig_resolves_without_anything_knowing_it():
    spur = _mk()                                                  # a 4-joint, driver-less rig
    R.register_rig(spur)
    try:
        assert R.rig_ids() == ("arm7", "spur")
        assert R.get_rig_type("spur") is spur
        assert spur.n_joints == 4 and spur.hw_family == R.HW_NONE
        # nothing in the registry hardcoded the one rig it shipped with:
        assert R.resolve_rig_id("spur", None) == "spur"
    finally:
        del R.RIGS["spur"]                                        # there is no unregister; leave RIGS as found


# -- the one rule for an empty/omitted rig -----------------------------------------
@test
def a_named_rig_wins_and_whitespace_is_trimmed():
    assert R.resolve_rig_id("arm7", None) == "arm7"
    assert R.resolve_rig_id("  arm7  ", None) == "arm7"


@test
def an_empty_id_falls_to_the_panel_selector_then_the_default():
    class Props:
        rig = "arm7"
    assert R.resolve_rig_id("", Props()) == "arm7"
    assert R.resolve_rig_id(None, Props()) == "arm7"

    class Unselected:
        rig = ""                                                  # a selector nobody ever touched
    assert R.resolve_rig_id("", Unselected()) == R.DEFAULT_RIG_ID

    class NoneProps:                                              # no scene at all (early calls)
        pass
    assert R.resolve_rig_id("", NoneProps()) == R.DEFAULT_RIG_ID

    class DeadRNA:                                                # a StructRNA whose owner died: any
        @property                                                   # attribute read raises
        def rig(self):
            raise RuntimeError("owner removed from the file")
    assert R.resolve_rig_id("", DeadRNA()) == R.DEFAULT_RIG_ID   # ...answers the default, not a crash


# -- which descriptor a live instance belongs to -----------------------------------
@test
def a_stamped_instance_answers_with_its_stamp():
    class Inst:
        rig_type = ARM
    assert R.descriptor_for(Inst()) is ARM


@test
def an_unstamped_instance_falls_back_through_its_id_then_the_default():
    class WithId:
        rig_id = "arm7"
    class Bare:
        pass
    assert R.descriptor_for(WithId()) is ARM
    assert R.descriptor_for(Bare()) is ARM                        # the single-rig world: the default, not a guess
    assert R.descriptor_for(None) is ARM


# -- summary ------------------------------------------------------------------------
print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("failing:", ", ".join(n for n, _ in FAIL))
sys.exit(1 if FAIL else 0)
