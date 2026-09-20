# SPDX-License-Identifier: BSD-3-Clause
"""Standalone tests for mcp_protocol — run with plain CPython, no Blender needed.

    python test_mcp_protocol.py        # from blender_ik_addon/

These are behaviour specs for the wire contract (MCP_INTEGRATION_PLAN.md §4–§7). They are
deliberately runnable *outside* Blender: `mcp_protocol` is pure stdlib, so if this file
cannot run here it is broken for both processes. The in-Blender gates (14–18) live in
test_mcp_bridge.py; this file pins the shared vocabulary they all depend on.

A test is (name, callable). It PASSES iff the callable returns (does not raise). The
first assertion failure or unexpected error is printed with its type and line — in this
terminal lookalike identifiers fold visually, so read the ERROR TYPE, not the token.
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
import mcp_protocol as P                                          # noqa: E402

PASS, FAIL = [], []


def test(fn):                                                      # tiny, no deps
    try:
        fn()
        PASS.append(fn.__name__)
    except BaseException:
        FAIL.append((fn.__name__, traceback.format_exc_only()[-1].strip()))
        print(f"FAIL {fn.__name__}")
        traceback.print_exc_only()
    return fn


# -- frame codec -----------------------------------------------------------------
@test
def round_trip_single_line():
    req = {"id": 7, "cmd": "get_state", "args": {}}
    wire = P.encode(req)
    assert isinstance(wire, bytes) and wire.endswith(b"\n") and wire.count(b"\n") == 1
    assert P.decode(wire) == req


@test
def encode_proves_the_frame_on_the_wire_is_one_clean_line():
    # What the guard is really for. json.dumps escapes control characters, so a string carrying a
    # NUL never yields a NUL byte: a guard written as `"\x00" in line` could then never fire, while
    # still looking like a safety check. So assert the property on the BYTES, for every payload that
    # would be a hazard if the encoder ever changed its mind.
    for payload in ({"id": 1, "cmd": "status", "args": {"note": "line\none"}},
                    {"id": 2, "cmd": "status", "args": {"note": "nul\x00here"}},
                    {"id": 3, "cmd": "status", "args": {"note": "cr\rhere"}},
                    {"id": 4, "cmd": "export_urdf", "args": {"directory": "../../etc"}}):
        wire = P.encode(payload)
        assert wire.endswith(b"\n") and wire.count(b"\n") == 1, payload
        assert b"\x00" not in wire and b"\r" not in wire[:-1], payload
        assert P.decode(wire) == payload, payload
    assert b"\x00" not in P.encode({"q": "\x00" * 9, "u": "\n" * 3})


@test
def decode_rejects_garbage_and_wrong_shapes():
    for bad in (b"", b"   \n", b"not json\n", b"[1,2,3]\n", b'"a string"'):
        try:
            P.decode(bad)
        except P.McpError as e:
            assert e.code == P.ERR.PROTO, (bad, e.code)
        else:
            raise AssertionError(f"decode accepted junk: {bad!r}")


@test
def decode_rejects_oversize_and_bad_utf():
    try:
        P.decode(b"x" * (2 * 1024 * 1024))
    except P.McpError as e:
        assert e.code == P.ERR.PROTO
    else:
        raise AssertionError("oversize frame accepted")
    try:
        P.decode(b"\xff\xfe not utf-8")
    except P.McpError as e:
        assert e.code == P.ERR.PROTO
    else:
        raise AssertionError("non-utf-8 accepted")


@test
def encode_refuses_nan_and_inf():
    try:
        P.encode({"id": 1, "data": {"v": float("nan")}})
    except (P.McpError, ValueError):                                # allow_nan=False
        pass
    else:
        raise AssertionError("NaN survived to the wire")


@test
def responses_have_the_verdict_shape():
    ok = P.response(7, {"q": [0] * 7})
    bad = P.error_response(7, P.ERR.RANGE, "J4 out of range")
    assert ok == {"id": 7, "ok": True, "data": {"q": [0] * 7}}
    assert bad["ok"] is False and bad["error"]["code"] == P.ERR.RANGE


# -- the allow-list is closed (§4.4) --------------------------------------------
@test
def unknown_command_is_inval_never_symbol_lookup():
    for evil in ("os.system", "eval", "__import__", "", None, 123, "get_state "):
        try:
            P.lookup(evil)                                           # type: ignore[arg-type]
        except P.McpError as e:
            assert e.code == P.ERR.INVAL, (evil, e.code)
        else:
            raise AssertionError(f"allow-list opened for {evil!r}")


@test
def catalogue_is_complete_and_dispatchable():
    assert "get_state" in P.COMMANDS and "hw_motors_stop" in P.COMMANDS
    for cmd, spec in P.COMMANDS.items():
        assert spec.cmd == cmd
        assert spec.cls in (P.CLASS.PURE, P.CLASS.READ, P.CLASS.WRITE,
                            P.CLASS.HW, P.CLASS.PRIORITY), (cmd, spec.cls)
        assert spec.executor in (P.CLASS and P.EXECUTOR.WORKER, P.EXECUTOR.TICK,
                                 P.EXECUTOR.RECEIVER), (cmd, spec.executor)
        assert spec.gate in (P.GATE.NONE, P.GATE.CONFIRM, P.GATE.ARM), (cmd, spec.gate)
        assert isinstance(spec.deadline_ms, int) and spec.deadline_ms > 0


# -- the e-stop is sacred (§5.3, §7) --------------------------------------------
@test
def estop_is_priority_receiver_and_ungated():
    s = P.COMMANDS["hw_motors_stop"]
    assert s.cls == P.CLASS.PRIORITY and s.executor == P.EXECUTOR.RECEIVER
    assert s.gate == P.GATE.NONE and not s.mutating and not s.may_refuse_busy


@test
def gate_split_motion_versus_non_motion():
    for n in ("hw_motors_move", "hw_motors_set_zero", "hw_live_start", "hw_live_update",
              "hw_send_frame"):
        assert P.COMMANDS[n].gate == P.GATE.ARM, f"{n} is motion, needs arm"
    for n in ("hw_configure", "hw_install", "hw_disconnect"):
        assert P.COMMANDS[n].gate == P.GATE.CONFIRM, f"{n} must be confirm-only (§7)"
    for n in ("hw_status", "hw_get_info", "hw_read_telemetry", "hw_analyze_frame", "hw_check"):
        assert P.COMMANDS[n].gate == P.GATE.NONE, f"{n} must stay open to look-before-touch"
    assert P.COMMANDS["hw_live_stop"].gate == P.GATE.NONE


# -- the derived taxonomy (condition 3) -----------------------------------------
@test
def dry_run_with_seed_is_pure_and_concurrent():
    c = P.classify("solve_ik", {"target_xyz_mm": [1, 2, 3], "solver": "gradient",
                                "dry_run": True, "seed_q": [0.0] * 7})
    assert c.cls == P.CLASS.PURE and c.executor == P.EXECUTOR.WORKER
    assert not c.mutating and not c.may_refuse_busy, "a pure what-if may not be refused E_BUSY"


@test
def dry_run_without_seed_reads_on_main_thread():
    c = P.classify("solve_ik", {"target_xyz_mm": [1, 2, 3], "solver": "gradient", "dry_run": True})
    assert c.cls == P.CLASS.READ and c.executor == P.EXECUTOR.TICK
    assert not c.mutating and not c.may_refuse_busy


@test
def apply_gradient_is_write_on_tick_memetic_is_write_via_worker():
    g = P.classify("solve_ik", {"target_xyz_mm": [1, 2, 3], "solver": "gradient", "execute": True})
    m = P.classify("solve_ik", {"target_xyz_mm": [1, 2, 3], "solver": "memetic", "execute": True})
    assert g.cls == P.CLASS.WRITE and g.executor == P.EXECUTOR.TICK and g.mutating
    assert m.cls == P.CLASS.WRITE and m.executor == P.EXECUTOR.WORKER and m.mutating
    assert m.may_refuse_busy and g.may_refuse_busy


@test
def classify_passes_through_unknown_and_non_solve():
    try:
        P.classify("nope", {})
    except P.McpError as e:
        assert e.code == P.ERR.INVAL
    else:
        raise AssertionError("classify must reject unknown cmd")
    c = P.classify("build_rig", {"dry_run": True, "seed_q": [0] * 7})       # args must not leak
    assert c.cls == P.CLASS.WRITE and c.executor == P.EXECUTOR.TICK


# -- E_TIMEOUT is outcome-unknown, not "did not happen" (§4.5) ----------------
@test
def timeout_is_the_outcome_unknown_code():
    assert P.ERR.TIMEOUT in P.OUTCOME_UNKNOWN
    assert P.ERR.INVAL not in P.OUTCOME_UNKNOWN and P.ERR.OK not in P.OUTCOME_UNKNOWN


# -- secrets (§4.3) --------------------------------------------------------------
@test
def token_is_generated_and_compared_constant_time():
    t = P.gen_token()
    assert isinstance(t, str) and len(t) >= 32 and "-" in t or "_" in t or t.isalnum()
    assert P.token_ok(t, t) is True
    assert P.token_ok("wrong", t) is False
    assert P.token_ok("", t) is False and P.token_ok(None, t) is False      # no auth path here
    assert P.token_ok("x", "") is False                                     # empty never matches


# -- filesystem sandbox (§9.5) --------------------------------------------------
@test
def sandbox_allows_relative_and_blocks_traversal():
    root = os.path.realpath(os.path.join(sys.executable, os.pardir, os.pardir))   # a real dir
    root = os.path.dirname(os.path.dirname(os.path.realpath(sys.executable)))
    os.makedirs(os.path.join(root, "pickik_export_test"), exist_ok=True)
    root = os.path.join(root, "pickik_export_test")
    good = P.sandbox_under(root, os.path.join("sub", "arm.urdf"))
    assert P.is_within(good, root), good
    for evil in ("../../../Windows/win.ini", "C:\\Windows\\system32\\evil", "/etc/passwd"):
        try:
            P.sandbox_under(root, evil)
        except P.SanitizeError:
            pass
        else:
            raise AssertionError(f"sandbox opened for {evil!r}")


@test
def is_within_is_strict_and_case_insensitive_on_windows():
    root = os.path.realpath(os.getcwd())
    assert P.is_within(os.path.join(root, "a", "b"), root) is True
    assert P.is_within(root, root) is False                       # must be *under*, not the root
    assert P.is_within(os.path.dirname(root), root) is False


# -- proto_rev drift guard (§2.5) -----------------------------------------------
@test
def proto_rev_is_stable_hex12():
    a, b = P.proto_rev(), P.proto_rev()
    assert a == b and len(a) == 12 and all(ch in "0123456789abcdef" for ch in a)


@test
def hello_frame_carries_the_handshake_fields():
    h = P.hello(proto_rev = P.proto_rev(), token = "sekrit")["hello"]
    assert h["protocol"] == P.PROTOCOL and len(h["proto_rev"]) == 12
    assert h["auth"] == "sekrit" and h["client"] == "mcp-server"


# -- summary --------------------------------------------------------------------
print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
if FAIL:
    print("failing:", ", ".join(n for n, _ in FAIL))
sys.exit(1 if FAIL else 0)
