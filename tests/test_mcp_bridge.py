# SPDX-License-Identifier: BSD-3-Clause
"""Headless gates 14-16 for the in-Blender MCP bridge (MCP_INTEGRATION_PLAN.md §11).

    "C:\\Program Files\\Blender Foundation\\Blender 4.5\\blender.exe" \
        --background --factory-startup --python blender_ik_addon/tests/test_mcp_bridge.py

Run this against a SEPARATE background instance, never against a live GUI session: it owns the
scene (build_rig / delete_rig / apply_fk) and it owns the tick.

**Why the client pumps.** A background instance has no window, so `bpy.app.timers` never fires and
the bridge's normal lane would never drain (§2.3). The client below therefore lives on the thread
that started the bridge and calls `pump_once()` while it waits for a reply — the very pump an
interactive session's timer drives. That is what makes the scheduler observable, which is gate 16.

  gate 14  round-trip: the handshake (token, proto_rev drift guard, one client, rate cap), the
           runtime-file discovery record, loopback-only bind, fail-loud on a taken port, the nine
           spec-§5 anchors through `validate_pose` cross-checked against the C-ABI FK, and the three
           argument-derived shapes of `solve_ik` on targets A (200/100/300, memetic) and B
           (300/150/300, gradient) — agreement with the model within a micron, error_pos < 1 mm.
  gate 15  allow-list / RCE: hostile `cmd` frames are refused (E_INVAL), never name-resolved, never
           eval'd or exec'd; malformed frames are refused (E_PROTO); the gate is enforced by the
           bridge BEFORE a handler is looked up; `export_urdf` is sandboxed to `export_root` (§9.5).
  gate 16  drain-to-budget: the tick is timed from entry to exit and its job count is bounded by
           the budget (not one job per tick, and not unbounded either); the heavy solve is measured
           OFF the main thread while its apply crosses TO it; `E_BUSY` is data, reserved for the
           mutating commands; `E_TIMEOUT` means "outcome unknown", not "did not happen"; the e-stop
           rides the priority lane and is answered without pumping at all.

A check PASSES iff it does not raise. The first failure in a group is reported with the numbers it
observed, because in this terminal lookalike identifiers fold visually — read the ERROR TYPE and the
measured value, not the token.
"""
from __future__ import annotations

import contextlib
import json
import math
import os
import select
import socket
import sys
import tempfile
import threading
import time
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))            # .../blender_ik_addon/tests
_ADDON = os.path.dirname(_HERE)                               # .../blender_ik_addon
_ROOT = os.path.dirname(_ADDON)                               # the repository root
for _p in (_ROOT, _ADDON):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import bpy                                                     # noqa: E402  (blender --background)
import blender_ik_addon as A                                    # noqa: E402
from blender_ik_addon import mcp_protocol as P                 # noqa: E402
from blender_ik_addon import mcp_bridge as B                   # noqa: E402
from blender_ik_addon import mcp_handlers_obs as H             # noqa: E402

_BASE_MODULES = frozenset(sys.modules)                         # what was already loaded at boot

TOKEN = "pickik-gates-token-not-a-real-secret"
PI_2 = 1.5707963267948966                                      # π/2, spelled as in test_acceptance.py
TARGET_A = (200.0, 100.0, 300.0)                              # mm — the memetic what-if
TARGET_B = (300.0, 150.0, 300.0)                              # mm — the gradient what-if
ZERO_Q = [0.0] * 7

#: The nine anchor poses of the kinematic spec, §5 — the tool0 table, in metres.
NINE_ANCHORS = (
    ("zero pose",       (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.675)),
    ("J1 yaw",          (PI_2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.675)),
    ("J2 shoulder fwd",  (0.0, PI_2, 0.0, 0.0, 0.0, 0.0, 0.0), (0.495, 0.0, 0.180)),
    ("J2 shoulder back", (0.0, -PI_2, 0.0, 0.0, 0.0, 0.0, 0.0), (-0.495, 0.0, 0.180)),
    ("J3 shoulder roll", (0.0, 0.0, PI_2, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.675)),
    ("J4 elbow fwd",     (0.0, 0.0, 0.0, PI_2, 0.0, 0.0, 0.0), (0.280, 0.0, 0.395)),
    ("J5 forearm roll",  (0.0, 0.0, 0.0, 0.0, PI_2, 0.0, 0.0), (0.0, 0.0, 0.675)),
    ("J6 wrist fwd",     (0.0, 0.0, 0.0, 0.0, 0.0, PI_2, 0.0), (0.065, 0.0, 0.610)),
    ("J7 tool roll",     (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, PI_2), (0.0, 0.0, 0.675)),
)

RESULTS: list[tuple[str, bool, str]] = []


class GateFailed(Exception):
    """Raised by check() so the FIRST failure in a group is visible with its own numbers."""


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}\n        {detail}")
    if not ok:
        raise GateFailed(f"{name}: {detail}")


# --------------------------------------------------------------------------- pin the working tree --
def verify_the_pin() -> None:
    """Never test a stale copy. Both the add-on and `mcp_protocol` MUST resolve inside the working
    tree: an installed copy under scripts/addons otherwise shadows `import blender_ik_addon` (it was,
    at the time this gate was written), and a top-level `import mcp_protocol` can load a SECOND copy
    of the catalogue — two COMMANDS tables means proto_rev no longer guards against drift."""
    want = os.path.normcase(os.path.realpath(_ADDON)) + os.sep
    for name, mod in (("blender_ik_addon", A), ("mcp_protocol", P), ("mcp_bridge", B),
                      ("mcp_handlers_obs", H)):
        got = os.path.normcase(os.path.realpath(getattr(mod, "__file__", "") or ""))
        if not got.startswith(want):
            raise RuntimeError(f"{name} resolved to {got!r}, outside the working tree {want!r} — a copy "
                               f"in Blender's scripts/addons is shadowing it. Uninstall it, then retry.")
    if P is not B.P or P is not H.P:
        raise RuntimeError("mcp_protocol was loaded twice: the bridge and the handlers disagree about "
                           "the catalogue, so proto_rev is not a drift guard any more")


# ------------------------------------------------------------------ the instrumented bridge & client --
class Instrumented(B.BridgeServer):
    """A bridge with the measurement holes drilled in, for gate 16.

    `probe_calls` records one entry per handler EXECUTION (which command, which thread, how long),
    `crossings` one per `call_on_main` (who called it, on which thread it ran), `drains` one per
    `pump_once` (entry to exit, and how many handler executions it may be credited with)."""

    def __init__(self, **kw) -> None:
        super().__init__(**kw)
        self.probe_calls: list[dict] = []
        self.crossings: list[dict] = []
        self.drains: list[dict] = []

    def install_probe(self) -> None:
        for cmd, fn in list(self.handlers.items()):
            self.handlers[cmd] = self._wrap(cmd, fn)

    def _wrap(self, cmd, fn):
        def probed(args, bridge):
            t0 = time.perf_counter()
            try:
                return fn(args, bridge)
            finally:
                self.probe_calls.append({"cmd": cmd, "on_main": bridge.is_main(),
                                         "ident": threading.get_ident(),
                                         "wall_ms": (time.perf_counter() - t0) * 1e3})
        return probed

    def call_on_main(self, fn, timeout = None):
        caller_main = self.is_main()
        seen: dict = {}

        def instrumented():
            seen["exec_main"] = self.is_main()
            return fn()

        out = super().call_on_main(instrumented, timeout)
        self.crossings.append({"caller_main": caller_main, "exec_main": seen.get("exec_main")})
        return out

    def pump_once(self, budget_ms = None) -> int:
        before = len(self.probe_calls)
        t0 = time.perf_counter()
        ran = super().pump_once(budget_ms)
        self.drains.append({"wall_ms": (time.perf_counter() - t0) * 1e3, "ran": ran,
                            "jobs": len(self.probe_calls) - before})
        return ran


def _spin_for(seconds: float) -> None:
    """Busy-wait. `time.sleep` has ~15 ms granularity on Windows, which would blur every measurement."""
    until = time.perf_counter() + seconds
    while time.perf_counter() < until:
        pass


@contextlib.contextmanager
def running(**kw):
    """Build, start and later stop one bridge, with the harness defaults below.

    port 0 → ephemeral, discovered from the bound socket (the operator never hard-codes 9876);
    request_timeout_ms is generous, because a memetic solve legitimately takes a while."""
    params = dict(host = "127.0.0.1", port = 0, token = TOKEN, tick_interval = 0.05,
                  tick_budget_ms = 4.0, request_timeout_ms = 20000, export_root = None,
                  max_handshake_attempts = 64, insecure_no_auth = False)
    params.update(kw)
    instrument = params.pop("instrument", True)
    runtime_file = params.pop("runtime_file", None)
    params.pop("allow_headless", None)
    b = Instrumented(handlers = dict(H.HANDLERS), **params)
    if instrument:
        b.install_probe()
    for attempt in range(4):                                # a loopback bind can be refused outright
        try:                                                # by the platform: an antivirus filter
            b.start(runtime_file = runtime_file, allow_headless = True)   # §2.3: here the harness pumps
            break
        except OSError as e:                                # or an OS port exclusion, at random ports
            if attempt == 3:
                raise
            print(f"    (bind attempt {attempt + 1} refused: {e}; asking for another port)")
    try:
        yield b
    finally:
        b.stop()


class Client:
    """A wire client on the thread that owns the tick: it pumps the lane while it waits."""

    def __init__(self, bridge, *, token = None, proto_rev = None, client = "test-harness",
                 pump = True, auto_handshake = True, timeout = 15.0) -> None:
        self.b, self.pump_enabled, self.timeout = bridge, pump, float(timeout)
        self.buf, self.pending, self.seq, self.hello = b"", {}, 0, None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((bridge.host, bridge.bound_port))
        if auto_handshake:
            self.hello = self.handshake(TOKEN if token is None else token, proto_rev, client)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # -- transmit -------------------------------------------------------------
    def send_frame(self, obj) -> None:
        self.sock.sendall(P.encode(obj))

    def send_raw(self, payload: bytes) -> None:
        self.sock.sendall(payload)

    # -- handshake ------------------------------------------------------------
    def handshake(self, token, proto_rev, client) -> dict:
        self.send_frame(P.hello(proto_rev = P.proto_rev() if proto_rev is None else proto_rev,
                                token = token, client = client))
        return self.reply(None, pump = self.pump_enabled)

    # -- request / reply ------------------------------------------------------
    def request(self, cmd, args = None, *, timeout = None, pump = None) -> dict:
        self.seq += 1
        self.send_frame({"id": self.seq, "cmd": cmd, "args": dict(args or {})})
        return self.reply(self.seq, timeout = timeout,
                           pump = self.pump_enabled if pump is None else pump)

    def request_raw(self, frame, *, timeout = None, pump = None) -> dict:
        self.send_frame(frame)
        return self.reply(frame.get("id"), timeout = timeout,
                           pump = self.pump_enabled if pump is None else pump)

    def reply(self, want_id = None, *, timeout = None, pump = True) -> dict:
        if want_id is not None and want_id in self.pending:
            return self.pending.pop(want_id)
        until = time.monotonic() + (self.timeout if timeout is None else float(timeout))
        while True:
            cut = self.buf.find(b"\n")
            if cut >= 0:
                frame = P.decode(self.buf[:cut])
                self.buf = self.buf[cut + 1:]
                if want_id is None or frame.get("id") == want_id:
                    return frame
                self.pending[frame.get("id")] = frame            # not for us: keep it, keep reading
                continue
            if pump:
                self.b.pump_once()                                # the pump, driving the normal lane
            readable, _, _ = select.select([self.sock], [], [], 0.002)
            if readable:
                try:
                    chunk = self.sock.recv(65536)
                except OSError:
                    chunk = b""
                if not chunk:
                    raise ConnectionError("the bridge closed the connection")
                self.buf += chunk
            if time.monotonic() > until:
                raise TimeoutError(f"no reply to id={want_id} in time; bridge={self.b.status_dict()} "
                                   f"— nothing pumped, or the lane is stuck")


def code_of(frame: dict) -> str:
    return "EXECUTED" if frame.get("ok") is True else str((frame.get("error") or {}).get("code"))


def msg_of(frame: dict) -> str:
    return str((frame.get("error") or {}).get("message", ""))


def rig_objects_present() -> list[str]:
    """Ask the datablock, not the handler: which of the rig's objects actually exist."""
    return [n for n in A.arm7_rig.rig_object_names() if bpy.data.objects.get(n) is not None]


# ------------------------------------------------------------------------------ gate 14 -----------
def gate14_round_trip() -> None:
    """14 bridge round-trip: the handshake, the discovery record, and the model behind the wire."""
    with running() as b:
        with Client(b) as c:
            check("14 the handshake completes against the add-on's own mcp_protocol",
                   "hello" in c.hello and c.hello["hello"]["protocol"] == P.PROTOCOL
                   and c.hello["hello"]["proto_rev"] == P.proto_rev()
                   and c.hello["hello"]["server"] == "pickik-blender",
                   f"{P.PROTOCOL} rev {P.proto_rev()} from {os.path.relpath(P.__file__, _ROOT)}, "
                   f"served by {c.hello['hello'].get('blender')}")

            core = A._core_or_die()                              # load the one C-ABI, on the main thread
            built = c.request("build_rig", {"rebuild": False})
            check("14 build_rig brings the rig up over the wire",
                   built.get("ok") is True and built["data"]["rig_present"] is True
                   and len(built["data"]["objects"]) > 0, json.dumps(built["data"], sort_keys = True))

            st = c.request("status", {})
            check("14 status answers over the wire (one round trip, pumped)",
                   st.get("ok") is True and st["data"]["dll_loaded"] is True
                   and st["data"]["rig_present"] is True and st["data"]["bridge"]["running"] is True
                   and st["data"]["bridge"]["client"] is True,
                   json.dumps(st["data"], sort_keys = True, default = str)[:220])

            ri = c.request("get_robot_info", {})["data"]
            low, up = ri["limits"]["lower"], ri["limits"]["upper"]
            check("14 get_robot_info reports the one model, not a second one",
                   ri["n_joints"] == 7 and ri["solvers"] == ["ccd", "gradient", "memetic"]
                   and len(low) == len(up) == 7 and all(lo < 0.0 < hi and abs(hi) <= 7.0
                                                        for lo, hi in zip(low, up)),
                   f"upper[0:2] = {up[:2]} read from the registered PickIKProps, not restated here")

            # --- the nine anchors, over the wire, against the C-ABI --------------------------------
            worst, out_of_bounds = 0.0, []
            for name, q, expected in NINE_ANCHORS:
                qq = [float(v) for v in q]
                d = c.request("validate_pose", {"q_rad": qq})["data"]
                abi, _ = core.fk_tool0(qq)                         # the model itself, called in-process
                got = [d["tool0_xyz_mm"][i] / 1e3 for i in range(3)]
                for i in range(3):
                    worst = max(worst, abs(got[i] - expected[i]), abs(abi[i] - expected[i]),
                                abs(got[i] - abi[i]))
                if not d["in_bounds"]:
                    out_of_bounds.append(name)
            check("14 the nine spec-§5 anchors agree, through validate_pose, with the C-ABI FK",
                   worst < 1e-6 and not out_of_bounds,
                   f"worst deviation {worst:.3e} m over {len(NINE_ANCHORS)} anchors; "
                   f"out-of-bounds: {out_of_bounds or 'none'}")

            bad = [0.0] * 7
            bad[1] = math.radians(150.0)                          # J2's hard limit is ±119.75°
            d = c.request("validate_pose", {"q_rad": bad})["data"]
            v = d["violations"]
            check("14 an out-of-range pose is refused, named by the limits the model itself reports",
                   d["in_bounds"] is False and len(v) == 1 and v[0]["index"] == 1
                   and v[0]["lower"] == low[1] and v[0]["upper"] == up[1],
                   f"violations = {v}")

            dg = c.request("validate_pose", {"q_deg": [0.0, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0]})
            check("14 validate_pose takes q_deg as well as q_rad",
                   dg.get("ok") is True and dg["data"]["in_bounds"] is True, msg_of(dg))

            # --- the three argument-derived shapes of solve_ik --------------------------------------
            a = c.request("solve_ik", {"target_xyz_mm": list(TARGET_A), "solver": "memetic",
                                       "dry_run": True, "seed_q": list(ZERO_Q)})
            check("14 solve_ik dry+seeded is a pure what-if: it solves, and it does not touch the scene",
                   a.get("ok") is True and a["data"]["success"] is True and a["data"]["applied"] is False
                   and a["data"]["dry_run"] is True and len(a["data"]["q"]) == 7
                   and a["data"]["error_pos_mm"] < 1.0,
                   f"target A {TARGET_A} memetic: error_pos {a['data']['error_pos_mm']:.4f} mm in "
                   f"{a['data']['time_ms']:.1f} ms")

            bfr = c.request("solve_ik", {"target_xyz_mm": list(TARGET_B), "solver": "gradient",
                                          "dry_run": True})
            check("14 solve_ik dry and unseeded reads the rig on the main thread, and still does not apply",
                   bfr.get("ok") is True and bfr["data"]["success"] is True
                   and bfr["data"]["applied"] is False and bfr["data"]["error_pos_mm"] < 1.0,
                   f"target B {TARGET_B} gradient: error_pos {bfr['data']['error_pos_mm']:.4f} mm in "
                   f"{bfr['data']['time_ms']:.1f} ms")

            abi_a, _ = core.fk_tool0(list(a["data"]["q"]))
            err = max(abs(abi_a[i] * 1e3 - TARGET_A[i]) for i in range(3))
            check("14 the solution the wire returned reproduces the target under the C-ABI FK",
                   err <= a["data"]["error_pos_mm"] + 1e-3 and err < 1.0,
                   f"reported {a['data']['error_pos_mm']:.4f} mm against FK-truth {err:.4f} mm")

            # two what-ifs submitted together, with the pump switched OFF: pure needs no main thread
            i1, i2 = c.seq + 1, c.seq + 2
            c.send_frame({"id": i1, "cmd": "validate_pose", "args": {"q_rad": list(ZERO_Q)}})
            c.send_frame({"id": i2, "cmd": "solve_ik", "args": {"target_xyz_mm": list(TARGET_A),
                                                                  "solver": "ccd", "dry_run": True,
                                                                  "seed_q": [PI_2] + [0.0] * 6}})
            f1 = c.reply(i1, timeout = 5.0, pump = False)
            f2 = c.reply(i2, timeout = 5.0, pump = False)
            check("14 two pure what-ifs complete with the pump switched off (genuinely concurrent)",
                   f1.get("ok") is True and f2.get("ok") is True,
                   f"validate_pose {code_of(f1)}, solve_ik {code_of(f2)} — never once touched the lane")

            gs = c.request("get_state", {})
            d = gs["data"]
            check("14 get_state is the authoritative path for live state",
                   gs.get("ok") is True and len(d["q_rad"]) == 7 and len(d["q_deg"]) == 7
                   and len(d["target_xyz_mm"]) == 3 and len(d["tool0_xyz_mm"]) == 3
                   and isinstance(d["valid"], bool) and d["fk_source"] == "c-abi",
                   f"tool0 {['%.3f' % v for v in d['tool0_xyz_mm']]} mm, valid={d['valid']}")

            # --- the handshake, refused: one client only, and no request before the hello ---------
            with Client(b) as second:
                check("14 one client only: a second is refused while the first holds the socket",
                       code_of(second.hello) == P.ERR.ACCES and "already connected" in msg_of(second.hello),
                       msg_of(second.hello))
            with Client(b, auto_handshake = False) as raw:
                raw.send_frame({"id": 1, "cmd": "status"})                   # a request before the hello
                early = raw.reply(None, timeout = 5.0)
                check("14 a request before the hello is a protocol violation, and is never dispatched",
                       code_of(early) == P.ERR.PROTO and "hello" in msg_of(early), msg_of(early))

    # --- the handshakes that must never be answered, observed on a bridge nobody is connected to.
    # They need one of their own: `_serve_client` checks the single-client rule FIRST, before the
    # token — refusing on capacity before refusing on auth, and "a client is already connected" is
    # not a secret. Tested with a client connected, they could only ever report the capacity refusal,
    # which is a different check and would hide the auth one.
    with running() as b:
        with Client(b, token = "not-the-token") as intruder:
            check("14 a wrong token is refused with E_ACCES, and the token is never echoed back",
                   "error" in intruder.hello and code_of(intruder.hello) == P.ERR.ACCES
                   and "not-the-token" not in json.dumps(intruder.hello), msg_of(intruder.hello))
        with Client(b, proto_rev = "0123456789ab") as drifted:
            check("14 a proto_rev mismatch is refused at connect — the drift guard works",
                   "error" in drifted.hello and code_of(drifted.hello) == P.ERR.PROTO,
                   msg_of(drifted.hello))
        with Client(b, token = TOKEN) as admitted:
            check("14 an authenticated client is admitted once, and only once",
                   "hello" in admitted.hello and b.status_dict()["client"] is True,
                   json.dumps(admitted.hello.get("hello", {}), sort_keys = True))

    with running(max_handshake_attempts = 3) as b:
        seen = []
        for _ in range(4):
            with Client(b, token = "nope") as intruder:
                seen.append((code_of(intruder.hello), msg_of(intruder.hello)))
        check("14 the handshake is rate-capped, so a brute-forcer cannot knock for ever",
               [s[0] for s in seen[:3]] == [P.ERR.ACCES] * 3 and seen[3][0] == P.ERR.ACCES
               and "rate limit" in seen[3][1], f"{seen}")

    # --- the discovery record, the bind policy, and the fail-loud rule ------------------------------
    rf = os.path.join(tempfile.mkdtemp("pickik-runtime-"), "bridge.json")
    with running(runtime_file = rf) as b:
        with open(rf, encoding = "utf-8") as fh:
            rec = json.load(fh)
        check("14 the runtime file publishes what the server needs in order to find the bridge",
               rec["host"] == "127.0.0.1" and rec["port"] == b.bound_port
               and rec["proto_rev"] == P.proto_rev() and rec["token"] == TOKEN
               and rec["pid"] == os.getpid() and "blender" in rec and "started_at" in rec,
               json.dumps({k: ("***" if k == "token" else v) for k, v in rec.items()}, sort_keys = True))
        mode = os.stat(rf).st_mode & 0o777
        home = os.path.normcase(os.path.realpath(os.path.expanduser("~")))
        here = os.path.normcase(os.path.realpath(rf))
        private = mode & 0o077 == 0
        inside = here.startswith(home + os.sep)
        check("14 the file that holds the token is private: 0600 where the platform honours the mode "
               "bits, inside the operator's own home where it does not",
               private or inside,
               f"mode 0{mode:o} ({'the mode bits took' if private else 'advisory on this platform'}), "
               f"under the home directory={inside}")
    with running(insecure_no_auth = True, token = "", runtime_file = rf + ".noauth") as b:
        with open(rf + ".noauth", encoding = "utf-8") as fh:
            rec = json.load(fh)
        check("14 insecure_no_auth publishes an empty token, so no secret is ever written twice",
               rec["token"] == "" and b.token == "" and b.insecure_no_auth is True,
               json.dumps(rec, sort_keys = True))

    refused_bind = ""
    try:
        B.BridgeServer(host = "0.0.0.0", port = 0)
    except ValueError as e:
        refused_bind = str(e)
    check("14 the bridge refuses to bind a routable interface (loopback only, §9)",
           "routable" in refused_bind, refused_bind or "no ValueError raised at all")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as squatter:
        squatter.bind(("127.0.0.1", 0))
        squatter.listen(1)
        taken_port = squatter.getsockname()[1]
        message = ""
        try:
            b = B.BridgeServer(host = "127.0.0.1", port = taken_port, token = TOKEN)
            b.start(allow_headless = True)
            b.stop()
        except RuntimeError as e:
            message = str(e)
        check("14 a configured port already in use fails loud; it never falls back silently",
               "could not bind" in message and str(taken_port) in message,
               message or "it bound somewhere else, which would be the silent fallback")


# ------------------------------------------------------------------------------ gate 15 -----------
def gate15_allow_list() -> None:
    """15 allow-list / remote code execution: nothing off the wire is ever eval'd, exec'd or
    name-resolved; a malformed frame is a protocol violation, not a crash; the sandbox holds."""
    sentinel = os.path.join(tempfile.gettempdir(), "pickik-rce-sentinel")
    for p in (sentinel, sentinel + ".urdf"):
        if os.path.exists(p):
            os.remove(p)
    export_root = os.path.join(tempfile.gettempdir(), "pickik-export-root")
    os.makedirs(export_root, exist_ok = True)

    with running(export_root = export_root) as b:
        with Client(b) as c:
            check("15 build_rig over the wire, before the storm",
                   c.request("build_rig", {"rebuild": False}).get("ok") is True, "rig up")

            # --- hostile commands ---------------------------------------------------------------
            refused = []
            for cmd in ("eval", "exec", "__import__", "builtins.eval", "os.system", "getattr",
                       "", " ", "STATUS", "get_state ", "get__state", "hw_motors_stop__"):
                fr = c.request(cmd, {"name": "os", "cmd": "eval(__import__('os').system('calc'))",
                                    "payload": f"__import__('os').popen('calc').read()",
                                    "sentinel": sentinel, "__class__": {"bases": {}}})
                refused.append((cmd, code_of(fr)))
            check("15 a cmd off the wire is looked up in the allow-list only, never name-resolved",
                   all(code == P.ERR.INVAL for _, code in refused), json.dumps(refused))

            # --- weird values in the cmd slot ---------------------------------------------------
            weird = []
            for value in (None, 7, 3.5, [], {}, True, ["status"]):
                fr = c.request_raw({"id": 4000 + len(weird), "cmd": value, "args": {}})
                weird.append((type(value).__name__, code_of(fr)))
            check("15 an unhashable or non-string cmd is E_INVAL, not a traceback",
                   all(code == P.ERR.INVAL for _, code in weird), json.dumps(weird))

            wire = P.encode({"id": 1, "cmd": "status", "args": {"note": "nul\x00here\nthere\r!"}})
            check("15 the codec puts one clean line on the wire: no NUL, no early newline, one terminator",
                   wire.endswith(b"\n") and wire.count(b"\n") == 1 and b"\x00" not in wire
                   and b"\r" not in wire[:-1] and P.decode(wire)["args"]["note"] == "nul\x00here\nthere\r!",
                   repr(wire))

            # --- the argument slot is inert -----------------------------------------------------
            inert = c.request("validate_pose", {"q_rad": list(ZERO_Q),
                                                 "__import__": {"name": "os"},
                                                 "exec": f"open({sentinel!r}, 'w').write('pwned')"})
            loaded = [m for m in ("turtle", "venv", "subprocess", "pty", "shelve", "cgi")
                      if m in sys.modules and m not in _BASE_MODULES]
            check("15 an argument payload is data: nothing was executed, no module was imported",
                   inert.get("ok") is True and not os.path.exists(sentinel) and not loaded,
                   f"sentinel exists={os.path.exists(sentinel)}, newly loadable modules={loaded or 'none'}")

            # (the malformed-frame probes live at the end of this gate, on a bridge of their own:
            #  only one client is admitted at a time, so with `c` holding the slot they could only
            #  ever be told "a client is already connected", which is a different check entirely)

            # --- the gate is bridge policy, checked before the handler is even looked up ----------
            armed = c.request("hw_motors_move", {"positions_deg": [0.0] * 7, "arm": True,
                                                 "confirm": P.MOTION_CONFIRM_PHRASE})
            check("15 an allow-listed command with no handler in this build reports E_STATE "
                   "(Phase 1 has no hardware at all)", code_of(armed) == P.ERR.STATE, msg_of(armed))
            hungry = c.request("hw_motors_move", {"positions_deg": [0.0] * 7})
            check("15 a motion without arm=true is refused BEFORE the handler is looked up",
                   code_of(hungry) == P.ERR.ACCES and "arm=true" in msg_of(hungry), msg_of(hungry))
            half = c.request("hw_motors_move", {"positions_deg": [0.0] * 7, "arm": True,
                                                "confirm": "i think so"})
            check("15 arm=true alone is not enough: the confirm phrase is the second key (§7)",
                   code_of(half) == P.ERR.ACCES and P.MOTION_CONFIRM_PHRASE in msg_of(half), msg_of(half))
            sneaky = c.request("hw_motors_move", {"positions_deg": [0.0] * 7, "arm": "true",
                                                  "confirm": P.MOTION_CONFIRM_PHRASE.lower()})
            check("15 arm must be the boolean true; a truthy string does not open the gate",
                   code_of(sneaky) == P.ERR.ACCES and "arm=true" in msg_of(sneaky), msg_of(sneaky))
            casual = c.request("delete_rig", {"confirm": "yes"})
            check("15 a non-motion privileged command needs the single confirm key, and it is a boolean",
                   code_of(casual) == P.ERR.ACCES, msg_of(casual))

            # --- delete_rig: the one confirm-gated command in this build -------------------------
            udel = c.request("delete_rig", {})
            before = rig_objects_present()
            check("15 delete_rig without confirm is refused, and every rig object survives",
                   code_of(udel) == P.ERR.ACCES and len(before) == len(A.arm7_rig.rig_object_names()),
                   f"{len(before)}/{len(A.arm7_rig.rig_object_names())} objects still there; {msg_of(udel)}")
            removed = c.request("delete_rig", {"confirm": True})
            after_objs = rig_objects_present()
            check("15 delete_rig with confirm=true removes the rig objects it names",
                   removed.get("ok") is True and len(removed["data"]["removed"]) > 0 and not after_objs,
                   json.dumps(removed["data"], sort_keys = True))
            restored = c.request("build_rig", {"rebuild": False})
            check("15 build_rig self-heals a deleted rig (the agent's way back)",
                   restored.get("ok") is True and len(rig_objects_present()) > 0,
                   json.dumps(restored["data"], sort_keys = True))

            # --- export_urdf: not an arbitrary-file-write primitive (§9.5) -------------------------
            good = c.request("export_urdf", {"directory": "gate15"})
            wrote = os.path.isfile(os.path.join(export_root, "gate15", "arm7.urdf"))
            escapes = [(evil, code_of(c.request("export_urdf", {"directory": evil})))
                      for evil in ("../../etc/pwned", "..\\..\\windows\\pwned",
                                   "C:\\Windows\\System32\\pwned", "/etc/pwned",
                                   os.path.join("..", "..", "boot"), "x/../../../outside")]
            leaked = [e for e, code in escapes if code != P.ERR.ACCES]
            check("15 export_urdf writes inside export_root and refuses every path that climbs out of it",
                   good.get("ok") is True and wrote and not leaked,
                   f"in-root arm7.urdf written={wrote}; refused {len(escapes) - len(leaked)}/{len(escapes)}; "
                   f"leaked={leaked or 'none'}")

            check("15 the handler table is a subset of the catalogue, and the hardware is simply absent",
                   set(H.HANDLERS) <= set(P.COMMANDS) and not any(k.startswith("hw_") for k in H.HANDLERS),
                   f"{len(H.HANDLERS)} handlers for {len(P.COMMANDS)} commands, no hw group")

    # --- malformed frames, on a bridge of their own, one client at a time ---------------------------
    # The single-client rule is checked before anything else in `_serve_client`, so a second socket
    # here would only ever be told "a client is already connected" and the frame would never reach
    # the dispatcher. Each probe below is therefore the only client its bridge has ever had.
    with running() as b:
        with Client(b) as boot:
            check("15 the rig is up before the storm", boot.request("build_rig", {"rebuild": False}).get("ok") is True,
                  "ready")
        for payload, why in ((b"\xff\xfe\xfd\n", "is not valid UTF-8"),
                             (b"not json at all\n", "is not JSON"),
                             (b"[1,2,3]\n", "is not a JSON object"),
                             (b"   \n", "is empty"),
                             (b'{"id":9,"cmd":"status","a":"\x00"}\n', "carries a raw NUL byte")):
            with Client(b) as wired:                           # the hello is answered first, ...
                wired.send_raw(payload)                        # ... and then, whatever the bytes are
                fr = wired.reply(None, timeout = 5.0)
                check(f"15 a frame that {why} is refused with E_PROTO, and the refusal is delivered",
                       code_of(fr) == P.ERR.PROTO and "hello" not in fr,
                       f"{payload!r} -> {code_of(fr)}: {msg_of(fr)}")

        with Client(b, auto_handshake = False) as pipelined:     # no round trip in between, as the
            pipelined.send_frame(P.hello(proto_rev = P.proto_rev(), token = TOKEN, client = "pipelined"))
            pipelined.send_raw(b"not json at all\n")              # wire format permits: both at once
            first = pipelined.reply(None, timeout = 8.0)
            second = pipelined.reply(None, timeout = 8.0)
            check("15 a request pipelined behind the hello is answered too: both frames arrive, in order",
                   "hello" in first and code_of(second) == P.ERR.PROTO,
                   f"first={'the hello' if 'hello' in first else code_of(first)}, "
                   f"second={code_of(second)}: {msg_of(second)}")

        with Client(b) as survivor:
            survivor.send_raw(b"[1,2,3]\n")
            junk = survivor.reply(None, timeout = 5.0)
            after = survivor.request("get_state", {})
            check("15 the bridge is still alive and the allow-list still holds after the storm",
                   code_of(junk) == P.ERR.PROTO and after.get("ok") is True,
                   f"{msg_of(junk)}; then get_state -> {code_of(after)}")

            survivor.send_raw(b"x" * (B._FRAME_MAX + 1) + b"\n")
            over = survivor.reply(None, timeout = 10.0)
            check("15 an oversized frame is refused before it can exhaust the buffer",
                   code_of(over) == P.ERR.PROTO and "too large" in msg_of(over), msg_of(over))


# ------------------------------------------------------------------------------ gate 16 -----------
def gate16_budget() -> None:
    """16 the main-thread budget: the drain callback is timed from entry to exit, and the lane is
    drained to the budget — not one job per tick, and not unbounded either."""
    SLOW_MS, BUDGET_MS, QUEUED = 1.0, 4.0, 12
    with running(tick_budget_ms = BUDGET_MS, instrument = False) as b:
        original = b.handlers["status"]                    # one deliberately slow read-only handler, so

        def slow(args, bridge):                            # that one job per tick would show as a bug
            _spin_for(SLOW_MS / 1e3)
            return original(args, bridge)

        b.handlers["status"] = slow
        b.install_probe()
        with Client(b) as c:
            for cmd, args in (("build_rig", {"rebuild": False}), ("status", {})):
                check(f"15 {cmd} over the wire, warming the caches on the main thread",
                       c.request(cmd, args).get("ok") is True, "ready")
            b.probe_calls.clear(); b.drains.clear()

            first, last = c.seq + 1, c.seq + QUEUED
            for i in range(first, last + 1):
                c.send_frame({"id": i, "cmd": "status", "args": {}})
            _spin_for(0.08)                                # let the receiver put them all in the lane
            got = [c.reply(i, timeout = 20.0) for i in range(first, last + 1)]
            drains = list(b.drains)
            jobs = [d["jobs"] for d in drains if d["jobs"] > 0]
            walls = [d["wall_ms"] for d in drains] or [0.0]
            ceiling = math.ceil(BUDGET_MS / SLOW_MS) + 1    # +1: the budget is checked before a job starts
            check("16 the lane drains to the budget: more than one job per tick, fewer than the whole "
                   "queue, and never more than the budget allows",
                   all(g.get("ok") is True for g in got) and bool(jobs) and max(jobs) >= 2
                   and max(jobs) <= ceiling and max(jobs) < QUEUED,
                   f"{QUEUED} queued -> {len(jobs)} busy drains, jobs per drain {jobs} "
                   f"(budget {BUDGET_MS} ms / handler {SLOW_MS} ms, ceiling {ceiling})")
            check("16 the drain callback itself stays inside the budget (entry to exit, not round trip)",
                   max(walls) < 8.0 * BUDGET_MS,
                   f"slowest drain {max(walls):.2f} ms, sum {sum(walls):.2f} ms over {len(drains)} ticks")

            # --- E_BUSY is data: reserved for the mutating commands (condition 1) ----------------
            b.probe_calls.clear(); b.drains.clear()
            i_write, i_pure, i_second = c.seq + 1, c.seq + 2, c.seq + 3
            c.send_frame({"id": i_write, "cmd": "set_joint_angles", "args": {"angles_deg": [0.0] * 7}})
            c.send_frame({"id": i_pure, "cmd": "validate_pose", "args": {"q_rad": list(ZERO_Q)}})
            c.send_frame({"id": i_second, "cmd": "set_solver", "args": {"solver": "ccd"}})
            _spin_for(0.08)                                # dispatched by the receiver thread, unpumped
            f_pure = c.reply(i_pure, timeout = 6.0, pump = False)
            f_busy = c.reply(i_second, timeout = 6.0, pump = False)
            snap = b.status_dict()                         # BEFORE pumping: catch the lane as it is
            f_write = c.reply(i_write, timeout = 20.0)
            check("16 a mutating command in flight refuses the next mutating command with E_BUSY",
                   code_of(f_busy) == P.ERR.BUSY, msg_of(f_busy))
            check("16 ... but a pure what-if is never busy-refused, and completed with the pump off",
                   f_pure.get("ok") is True, code_of(f_pure))
            check("16 the single-flight slot is held by exactly one parked write",
                   snap["queued"] == 1 and snap["pending_mutating"] == 1 and f_write.get("ok") is True,
                   f"queued={snap['queued']} pending_mutating={snap['pending_mutating']} "
                   f"then {code_of(f_write)}")

            # --- the e-stop rides the priority lane (§5.3): answered without pumping at all -------
            fired: list[bool] = []

            def emergency_stop(args, bridge):
                bridge.abort_flag.set()
                fired.append(bridge.is_main())
                return {"stopped": True, "abort_flag": True}

            b.handlers["hw_motors_stop"] = emergency_stop
            i_motion, i_stop = c.seq + 1, c.seq + 2
            c.send_frame({"id": i_motion, "cmd": "set_joint_angles", "args": {"angles_deg": [1.0] * 7}})
            _spin_for(0.06)
            c.send_frame({"id": i_stop, "cmd": "hw_motors_stop", "args": {}})
            f_stop = c.reply(i_stop, timeout = 6.0, pump = False)      # no pump: the lane is inline
            mid = b.status_dict()
            f_motion = c.reply(i_motion, timeout = 20.0)               # now pump: the write goes through
            check("16 the e-stop is answered without pumping, off the main thread, ahead of the parked write",
                   f_stop.get("ok") is True and f_stop["data"]["stopped"] is True and fired == [False]
                   and mid["pending_mutating"] == 1 and mid["abort"] is True,
                   f"answered inline on the receiver thread (is_main={fired[0] if fired else '?'}), "
                   f"pending_mutating={mid['pending_mutating']} abort={mid['abort']}")
            check("16 the e-stop does not cancel the queue: the parked write runs afterwards",
                   f_motion.get("ok") is True, code_of(f_motion))

            # --- the heavy solve is measured off the main thread; the apply crosses to it ----------
            b.probe_calls.clear(); b.drains.clear(); b.crossings.clear()
            mem = c.request("solve_ik", {"target_xyz_mm": list(TARGET_A), "solver": "memetic",
                                         "execute": True}, timeout = 40.0)
            solves = [p for p in b.probe_calls if p["cmd"] == "solve_ik"]
            on_main = [p for p in solves if p["on_main"]]
            crossings = [x for x in b.crossings if not x["caller_main"]]
            drain_walls = [d["wall_ms"] for d in b.drains] or [0.0]
            check("16 a memetic solve is measured off the main thread, and never run by a drain",
                   mem.get("ok") is True and len(solves) == 1 and not on_main,
                   f"solve {solves[0]['wall_ms']:.1f} ms on thread {solves[0]['ident']:#x}, "
                   f"executions attributed to the tick: {len(on_main)}")
            check("16 its one scene mutation crosses to the main thread, where the pump runs it",
                   mem.get("ok") is True and mem["data"]["applied"] is True and bool(crossings)
                   and all(x["caller_main"] is False and x["exec_main"] is True for x in crossings),
                   f"{len(crossings)} crossing(s): caller_main="
                   f"{ {x['caller_main'] for x in crossings} }, exec_main="
                   f"{ {x['exec_main'] for x in crossings} }")
            check("16 the tick stays responsive during a heavy solve: no drain ever lasted as long as it",
                   max(drain_walls) < 250.0 and max(drain_walls) < max(solves[0]["wall_ms"], 1.0),
                   f"slowest drain {max(drain_walls):.2f} ms vs solve {solves[0]['wall_ms']:.1f} ms "
                   f"(budget {BUDGET_MS} ms, {len(drain_walls)} ticks during the flight)")
            now = c.request("get_state", {})
            solved_q = mem["data"]["q"]
            drift = max(abs(solved_q[i] - now["data"]["q_rad"][i]) for i in range(7))
            check("16 the pose that was applied is the pose that was solved (the scene is the transcript)",
                   drift < 1e-9 and now.get("ok") is True,
                   f"max |q_applied - q_solved| = {drift:.2e} rad, error_pos "
                   f"{mem['data']['error_pos_mm']:.4f} mm")

            # --- E_TIMEOUT means "outcome unknown", never "did not happen" (condition 2) ----------
            class FakeConn:
                def __init__(self) -> None:
                    self.frames: list[bytes] = []

                def sendall(self, payload: bytes) -> None:
                    self.frames.append(payload)

            stale = B._Job(999, "status", P.lookup("status"), {}, time.monotonic() - 1.0, FakeConn())
            b._run_job(stale)
            late = P.decode(stale.conn.frames[0])
            check("16 a job whose deadline passed is answered E_TIMEOUT, with the reconcile wording",
                   late["id"] == 999 and code_of(late) == P.ERR.TIMEOUT
                   and "reconcile" in msg_of(late), msg_of(late))

            box: dict = {}

            def off_thread():
                try:
                    box["value"] = b.call_on_main(lambda: "never pumped", timeout = 0.3)
                except P.McpError as e:
                    box["error"] = (e.code, e.message)

            th = threading.Thread(target = off_thread, name = "pickik-off-thread")
            th.start()
            th.join(6.0)                                   # the main thread does NOT pump while it waits
            err = box.get("error") or (None, "")
            check("16 call_on_main with the pump stopped is E_TIMEOUT, and says the outcome is unknown",
                   err[0] == P.ERR.TIMEOUT and "reconcile" in err[1] and "value" not in box,
                   f"{err[0]}: {err[1]}")
            check("16 the contract classifies E_TIMEOUT alone as outcome-unknown",
                   P.OUTCOME_UNKNOWN == frozenset({P.ERR.TIMEOUT}), f"OUTCOME_UNKNOWN={sorted(P.OUTCOME_UNKNOWN)}")

            # --- a cold pure handler crosses to the main thread once, then stays off it ------------
            H._CORE, H._LIMITS = None, None
            b.crossings.clear()
            cold = c.request("validate_pose", {"q_rad": list(ZERO_Q)}, timeout = 20.0)
            cold_cross = [x for x in b.crossings if not x["caller_main"]]
            check("16 a cold pure handler posts to the main thread and waits for the pump",
                   cold.get("ok") is True and bool(cold_cross)
                   and all(x["caller_main"] is False and x["exec_main"] is True for x in cold_cross)
                   and H._CORE is not None and len(H._LIMITS[0]) == 7,
                   f"{len(cold_cross)} cold crossing(s)")
            b.crossings.clear()
            warm = c.request("validate_pose", {"q_rad": [PI_2] + [0.0] * 6}, timeout = 10.0, pump = False)
            check("16 ... and once cached, validate_pose answers with the pump switched off",
                   warm.get("ok") is True and b.crossings == [], f"{code_of(warm)} crossings={b.crossings}")

            # --- an idle tick is a spin, not a stall ------------------------------------------------
            b.drains.clear()
            for _ in range(50):
                b.pump_once()
            idle = [d["wall_ms"] for d in b.drains] or [0.0]
            check("16 an empty lane returns at once: an idle tick is a spin, not a stall",
                   max(idle) < 4.0, f"slowest idle tick {max(idle):.3f} ms over {len(idle)} ticks")


# ------------------------------------------------------------------------------ the driver -------

# --------------------------------------------------------------------------- gate 19 ----
# --------------------------------------------------------------------------- gate 19
# Gate 19: the section-8 contract — teardown leaves nothing behind, and the panel's MCP section
# draws in every branch without asking for an icon this Blender does not have.
#

def _live_icons() -> frozenset:
    """The icon enums this very build offers, obtained the way acceptance gate 9 obtains them.
    Gate 9 exists because a 4.x-only name (BLANK) raised TypeError *mid-draw* and collapsed the
    panel after Solve; a new panel section must be checked against the live enum, not against a list
    somebody remembered."""
    for attr in ("bl_rna", "bl_rna_type"):
        rna = getattr(bpy.types.UILayout, attr, None)
        fn = rna.functions.get("operator") if rna is not None else None
        if fn is not None:
            try:
                return frozenset(e.identifier for e in fn.parameters["icon"].enum_items)
            except (AttributeError, TypeError):
                continue
    return frozenset()                       # cannot validate; the draw must then still not raise


class _StrictLayout:
    """A layout that refuses what the real N-panel refuses: an icon outside the live enum, and an
    icon that is not even an enum string. Records what was drawn so a branch can be proven drawn."""

    def __init__(self, sink, depth = 0) -> None:
        self.sink, self.depth = sink, depth

    def _icon(self, icon) -> None:
        if icon is None:
            return
        if not isinstance(icon, str):
            raise TypeError(f"icon {icon!r} is not an enum name; N-panel label/operator take enums")
        if icon not in self.sink["icons"]:
            raise TypeError(f"icon {icon!r} is not in this Blender's live icon enum")
        self.sink["used"].add(icon)

    def label(self, text = "", icon = None) -> None:
        self._icon(icon)
        self.sink["text"].append(str(text))

    def operator(self, idname, text = None, icon = None) -> None:
        self._icon(icon)
        self.sink["ops"].append(idname)

    def prop(self, ptr, prop_name, text = None) -> None:
        self.sink["props"].append(str(prop_name))

    def box(self) -> "_StrictLayout":
        return type(self)(self.sink, self.depth + 1)

    def row(self) -> "_StrictLayout":
        return type(self)(self.sink, self.depth + 1)


class _PanelShim:
    """Enough of a Panel to drive the add-on's real draw: `draw` needs `self.layout` and calls
    `self._draw`, which is an ordinary method of the panel class and so can be borrowed as-is."""
    _draw = A.PICKIK_PT_main._draw


def _draw_mcp_section(bridge_or_none, prefs) -> dict:
    """Drive the add-on's real panel draw and hand back what the MCP section asked for."""
    sink = {"text": [], "ops": [], "props": [], "used": set(), "icons": _live_icons()}
    shim = _PanelShim()
    shim.layout = _StrictLayout(sink)
    ctx = bpy.context
    real_prefs, real_get = A._mcp_prefs, B.get
    try:
        A._mcp_prefs = lambda _c: prefs
        B.get = lambda: bridge_or_none
        A.PICKIK_PT_main.draw(shim, ctx)
    finally:
        A._mcp_prefs, B.get = real_prefs, real_get
    drawn = " | ".join(sink["text"])
    assert "draw error" not in drawn, "the panel hid an exception in its draw-error line: " + drawn[-400:]
    return sink


class _FakePrefs:
    """Stands in for the AddonPreferences block, which a factory-startup session does not hand out.
    Without it the branch the operator actually lives in is unreachable headless."""
    enable_mcp_bridge = True
    mcp_start_on_load = False
    mcp_port = 9876
    mcp_auth_token = ""
    mcp_runtime_file = "~/.pickik/bridge.json"
    mcp_export_root = "~/pickik/export"
    mcp_hardware_enabled = False
    mcp_insecure_no_auth = False


def gate19_teardown_and_panel() -> None:
    sink = _draw_mcp_section(None, None)
    joined = " | ".join(sink["text"])
    check("19 with no bridge and no preferences the section still draws, and offers Start",
          "not running" in joined and "pickik.mcp_start" in sink["ops"]
          and "pickik.mcp_stop" not in sink["ops"],
          f"start offered, stop absent; {len(sink['ops'])} operators drawn by the whole panel, "
          f"icons={sorted(sink['used']) or 'none'}")

    with running() as b:
        # The harness starts its own instrumented instance rather than going through
        # mcp_bridge.start(), so the module singleton stays empty on purpose: read the server off
        # what the context manager handed back, not off B.get().
        srv = b
        check("19 the harness bridge is the running one the panel must describe",
              srv.is_running() and srv.bound_port > 0, f"bound port {srv.bound_port}")
        sink = _draw_mcp_section(srv, _FakePrefs())
        joined = " | ".join(sink["text"])
        check("19 with the bridge running the section reads out the bound port and offers Stop",
               f"{srv.host}:{srv.bound_port}" in joined and "pickik.mcp_stop" in sink["ops"]
               and "pickik.mcp_start" not in sink["ops"],
               f"port {srv.bound_port} drawn, stop offered, start withdrawn")
        check("19 the section draws the preference controls it is meant to expose",
               set(sink["props"]) >= {"enable_mcp_bridge", "mcp_port", "mcp_hardware_enabled"},
               f"props={sorted(set(sink['props']))}")
        check("19 every icon the section asked for exists in this Blender's live enum",
               sink["used"] <= _live_icons() and bool(sink["used"]),
               f"used={sorted(sink['used'])} of {len(_live_icons())} live")

        unlocked = _FakePrefs()
        unlocked.mcp_hardware_enabled = True
        sink = _draw_mcp_section(srv, unlocked)
        joined = " | ".join(sink["text"])
        check("19 the hardware gate is stated in the unlocked state, with the warning it owes",
               "UNLOCKED" in joined and "ERROR" in sink["used"],
               f"stated={'UNLOCKED' in joined}, icons={sorted(sink['used'])}")
        check("19 the token value is never drawn, whatever the preferences hold",
              not any(srv.token and srv.token in t for t in sink["text"]),
              f"{len(sink['text'])} lines drawn, none carrying the token")

    B.stop()
    before = frozenset(t.name for t in threading.enumerate())
    A.unregister()
    after = frozenset(t.name for t in threading.enumerate())
    check("19 unregister() leaves no bridge singleton behind (the teardown clause)",
          B.get() is None and B._SERVER is None, f"_SERVER={B._SERVER!r}")
    left = sorted(n for n in (after - before) if "pickik" in n.lower() or "mcp" in n.lower())
    alive = sorted(t.name for t in threading.enumerate() if t.is_alive()
                   and ("pickik" in t.name.lower() or "mcp" in t.name.lower()))
    check("19 unregister() leaves no thread of the bridge behind either",
          not left and not alive, f"new={left or 'none'}, still alive={alive or 'none'}")

    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.connect(("127.0.0.1", srv.bound_port))
        answered = True
    except OSError:
        answered = False
    finally:
        probe.close()
    check("19 the port the bridge held is released, so a later register() can bind it again",
          not answered, f"port {srv.bound_port} answered a connect after unregister: {answered}")
    A.register()
    check("19 and register() works again after that, so a reload cycle is clean",
          len(A.CLASSES) >= 18 and A._state is not None, f"{len(A.CLASSES)} classes")


# --------------------------------------------------------------------------- gate 20 -----------
def gate20_docs_agree_with_the_catalogue() -> None:
    """The docs repeat numbers that live in the catalogue. A hand-copied table can be wrong in a way
    no reader can see — one was: it said twelve tools where there are fourteen and invented a
    `confirm` gate for `export_urdf`, which has none. So the copy is checked against the authority it
    came from, here, rather than left to a reader who happens to already know the answer."""
    from blender_ik_addon import mcp_docs
    bad = mcp_docs.check_docs(verbose = True)
    check("20 the docs' catalogue tables agree with the catalogue they were copied from",
          not bad, bad[0] if bad else f"proto_rev {P.proto_rev()}, no disagreement")


# ------------------------------------------------------------------------------ run -----------
def main() -> int:
    print("== pickik mcp bridge: headless gates 14-16, 19-20 ==")
    try:
        verify_the_pin()
    except BaseException as exc:
        print(f"[FATAL] {exc}")
        return 2
    print(f"   blender  {bpy.app.version_string}   python {sys.version.split()[0]}   "
          f"{P.PROTOCOL} rev {P.proto_rev()}")
    print(f"   addon    {os.path.relpath(A.__file__, _ROOT)}")
    print(f"   bridge   {os.path.relpath(B.__file__, _ROOT)}")
    print(f"   handlers {len(H.HANDLERS)} of {len(P.COMMANDS)} commands (no hw group in Phase 1)")
    A.register()
    for gate in (gate14_round_trip, gate15_allow_list, gate16_budget,
                 gate19_teardown_and_panel, gate20_docs_agree_with_the_catalogue):
        try:
            gate()
        except GateFailed as exc:
            print(f"   FAIL {exc}")
        except BaseException:
            traceback.print_exc()
            print(f"   FAIL {gate.__name__}: unexpected exception, see the traceback above")
    failed = sum(1 for _n, ok, _d in RESULTS if not ok)
    print(f"\n== {len(RESULTS) - failed} passed, {failed} failed, {len(RESULTS)} checks ==")
    for name, ok, detail in RESULTS:
        if not ok:
            print(f"   FAIL {name}: {detail}")
    try:
        A.unregister()
    except BaseException:
        traceback.print_exc()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())