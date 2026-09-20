# SPDX-License-Identifier: BSD-3-Clause
"""mcp_protocol — the shared wire vocabulary of the PickIK MCP bridge.

Pure stdlib. Imports **no** `bpy`, so this exact module is imported unchanged by the
in-Blender bridge (Blender's bundled Python 3.11) *and* is the source of the copy vendored
into the sibling `pick_ik_mcp` server (system Python 3.13). If it ever needs `bpy`, it is
in the wrong file.

It owns three things the two processes must never disagree about:
  * the frame codec (NDJSON, one JSON object per line),
  * the error-code set (a stable, machine-readable taxonomy),
  * the command catalogue — the closed allow-list (§4.4) whose `class`/`executor`/`gate`
    fields ARE the §5.2 concurrency taxonomy and the §7 gate split, expressed as data.

The whole point of putting the taxonomy here is that correctness becomes checkable: an
unknown `cmd` cannot execute, `hw_motors_stop`'s priority-lane facts are asserted at import,
`E_BUSY` refusal is a data field not folklore, and `solve_ik`'s class is *derived from its
arguments* (condition 3) so the pure/read/write split is one implementation, not a copy.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import secrets as _secrets

__all__ = [
    "PROTOCOL", "MOTION_CONFIRM_PHRASE", "ERR", "Command", "CLASS", "EXECUTOR", "GATE", "COMMANDS",
    "lookup", "classify", "encode", "decode", "hello", "response", "error_response",
    "gen_token", "token_ok", "proto_rev", "is_within", "sandbox_under",
    "McpError", "SanitizeError",
]

PROTOCOL = "pickik-bridge/1"          # bump on any wire change; handshake cross-checks it

#: The literal an agent must echo (case-insensitive) alongside arm=true to unlock a motion. Two
#: keys — one structural (arm), one semantic (this) — so `arm` cannot become routine paperwork (§7).
MOTION_CONFIRM_PHRASE = "I UNDERSTAND THIS MOVES THE PHYSICAL ARM"


# ---------------------------------------------------------------------------
# Error codes (contract §4.5). Stable strings; the agent switches on these.
# ---------------------------------------------------------------------------
class ERR:
    OK = "E_OK"
    INVAL = "E_INVAL"          # malformed frame / bad argument
    RANGE = "E_RANGE"          # value out of joint range
    BUSY = "E_BUSY"            # a *mutating* command is running (never for priority/pure)
    AGAIN = "E_AGAIN"          # temporary; retry (e.g. CAN read with no frames yet)
    NOMEM = "E_NOMEM"          # allocation / solver create failed
    ACCES = "E_ACCES"          # not permitted: gate closed / auth failed / second client / path escape
    HW = "E_HW"                # hardware / bus error
    STATE = "E_STATE"          # object state invalid (rig missing, stale plan_id)
    TIMEOUT = "E_TIMEOUT"      # OUTCOME UNKNOWN — see is_outcome_unknown()
    PROTO = "E_PROTO"          # protocol violation / proto_rev mismatch
    INTERNAL = "E_INTERNAL"    # unexpected

#: The codes that mean "the result is undetermined", NOT "it did not happen" (§4.5 note).
#: A timed-out mutating command may still have landed; the agent's recovery is get_state.
OUTCOME_UNKNOWN = frozenset({ERR.TIMEOUT})


class McpError(Exception):
    """Raised by handlers to report a classified failure. Maps to {ok:false,error:{…}}."""
    def __init__(self, code: str, message: str, *, data: dict | None = None) -> None:
        super().__init__(f"{code}: {message}")
        self.code, self.message, self.data = code, message, (data or {})


class SanitizeError(McpError):
    def __init__(self, message: str) -> None:
        super().__init__(ERR.ACCES, message)


# ---------------------------------------------------------------------------
# Command taxonomy (contract §5.2 / §6). These constants are the ONLY legal values.
# ---------------------------------------------------------------------------
class CLASS:
    PURE = "pure"          # no bpy at all -> worker, concurrent, never busy-refused
    READ = "read"          # reads bpy -> main thread, in the tick, no busy guard
    WRITE = "write"        # mutates bpy -> main thread, in the tick, busy-guarded
    HW = "hw"              # cubemars over a worker under the task lock, polls abort_flag
    PRIORITY = "priority"  # e-stop: dispatched on the receiver thread, never queued

class EXECUTOR:
    WORKER = "worker"      # off the tick
    TICK = "tick"          # main-thread timer drain
    RECEIVER = "receiver"  # the socket thread (legal here ONLY for non-bpy work, §5.1)

class GATE:
    NONE = "none"          # open
    CONFIRM = "confirm"    # non-motion privileged: one key (§7.2) — moves nothing
    ARM = "arm"            # motion: two keys, arm+confirm (§7.2) — the arm is sacred


class Command:
    """One allow-listed command. Frozen; validated at import so a bad row cannot ship."""
    __slots__ = ("cmd", "cls", "executor", "gate", "mutating", "may_refuse_busy",
                 "deadline_ms", "handler", "note")
    def __init__(self, cmd, cls, executor, gate, *, mutating, may_refuse_busy,
                 deadline_ms, handler, note = "") -> None:
        object.__setattr__(self, "cmd", cmd); object.__setattr__(self, "cls", cls)
        object.__setattr__(self, "executor", executor); object.__setattr__(self, "gate", gate)
        object.__setattr__(self, "mutating", mutating); object.__setattr__(self, "may_refuse_busy", may_refuse_busy)
        object.__setattr__(self, "deadline_ms", deadline_ms); object.__setattr__(self, "handler", handler)
        object.__setattr__(self, "note", note)
    def __setattr__(self, *_):                       # frozen
        raise AttributeError("CommandSpec is frozen")
    def __repr__(self) -> str:                        # pragma: no-cover
        return f"<Command {self.cmd}:{self.cls}/{self.executor} gate={self.gate}>"


def _c(cmd, cls, executor, gate, *, deadline_ms = 5000, handler = None, note = "") -> Command:
    return Command(cmd, cls, executor, gate,
                   mutating=(cls in (CLASS.WRITE, CLASS.HW)),   # hw motion is a mutation
                   may_refuse_busy=(cls in (CLASS.WRITE, CLASS.HW)),  # §5.3: only these get E_BUSY
                   deadline_ms=deadline_ms, handler=handler or ("h_" + cmd), note=note)


# The catalogue. Names mirror the add-on's operators & driver surface 1:1 (§6).
COMMANDS: dict[str, Command] = {c.cmd: c for c in [
    # observe (§6.1) — validate_pose is pure (Core.fk_tool0 on the ctypes handle): concurrent.
    _c("status",           CLASS.READ,     EXECUTOR.TICK,    GATE.NONE, deadline_ms = 1000),
    _c("get_state",        CLASS.READ,     EXECUTOR.TICK,    GATE.NONE, deadline_ms = 1000),
    _c("get_robot_info",   CLASS.READ,     EXECUTOR.TICK,    GATE.NONE, deadline_ms = 1000),
    _c("validate_pose",    CLASS.PURE,     EXECUTOR.WORKER,  GATE.NONE, deadline_ms = 2000),
    # rig & urdf (§6.2) — export_urdf is path-sandboxed to export_root (§9.5)
    _c("build_rig",        CLASS.WRITE,    EXECUTOR.TICK,    GATE.NONE, deadline_ms = 5000),
    _c("delete_rig",       CLASS.WRITE,    EXECUTOR.TICK,    GATE.CONFIRM),
    _c("export_urdf",      CLASS.WRITE,    EXECUTOR.TICK,    GATE.NONE, deadline_ms = 10000),
    # IK / FK (§6.3). Registered as WRITE-apply; classify() narrows per the args (condition 3).
    _c("solve_ik",         CLASS.WRITE,    EXECUTOR.TICK,    GATE.NONE, deadline_ms = 7000,
       note = "class is DERIVED from args by classify(): dry+seed=pure, dry=only=read, execute=write"),
    _c("set_target",       CLASS.WRITE,    EXECUTOR.TICK,    GATE.NONE, deadline_ms = 1000),
    _c("get_target",       CLASS.READ,     EXECUTOR.TICK,    GATE.NONE, deadline_ms = 1000),
    _c("set_joint_angles", CLASS.WRITE,    EXECUTOR.TICK,    GATE.NONE, deadline_ms = 2000),
    _c("set_solver",       CLASS.WRITE,    EXECUTOR.TICK,    GATE.NONE, deadline_ms = 1000),
    _c("set_solver_config",CLASS.WRITE,    EXECUTOR.TICK,    GATE.NONE, deadline_ms = 1000),
    _c("set_continuous",   CLASS.WRITE,    EXECUTOR.TICK,    GATE.NONE, deadline_ms = 1000),
    # hardware (§6.4) — gate split: arm is MOTION ONLY; non-motion privileged is confirm-only.
    _c("hw_status",         CLASS.READ,    EXECUTOR.TICK,    GATE.NONE,    deadline_ms = 1000),
    _c("hw_get_info",       CLASS.READ,    EXECUTOR.TICK,    GATE.NONE,    deadline_ms = 2000),
    _c("hw_analyze_frame",  CLASS.PURE,    EXECUTOR.WORKER,  GATE.NONE,    deadline_ms = 1000),
    _c("hw_configure",      CLASS.WRITE,   EXECUTOR.TICK,    GATE.CONFIRM, note = "moves nothing"),
    _c("hw_check",          CLASS.HW,      EXECUTOR.WORKER,  GATE.NONE,    deadline_ms = 30000),
    _c("hw_install",        CLASS.HW,      EXECUTOR.WORKER,  GATE.CONFIRM, deadline_ms = 600000,
       note = "pip→network→exec inside Blender's interpreter; privileged sink, confirm NOT arm (§9.5)"),
    _c("hw_read_telemetry", CLASS.HW,      EXECUTOR.WORKER,  GATE.NONE,    deadline_ms = 15000),
    _c("hw_send_frame",     CLASS.HW,      EXECUTOR.WORKER,  GATE.ARM,     note = "raw CAN-TX, advanced"),
    _c("hw_motors_move",    CLASS.HW,      EXECUTOR.WORKER,  GATE.ARM),
    _c("hw_motors_set_zero",CLASS.HW,      EXECUTOR.WORKER,  GATE.ARM),
    _c("hw_live_start",     CLASS.HW,      EXECUTOR.WORKER,  GATE.ARM),
    _c("hw_live_update",    CLASS.HW,      EXECUTOR.WORKER,  GATE.ARM),
    _c("hw_live_stop",      CLASS.HW,      EXECUTOR.WORKER,  GATE.NONE,    note = "stops motion: open to all"),
    _c("hw_motors_stop",    CLASS.PRIORITY,EXECUTOR.RECEIVER,GATE.NONE,    deadline_ms = 2000,
       note = "e-stop: never queued, never E_BUSY, sets abort_flag, sends disable frames (§5.3)"),
    _c("hw_disconnect",     CLASS.HW,      EXECUTOR.WORKER,  GATE.CONFIRM, note = "closes bus, moves nothing"),
]}

# Import-time self-check: the safety-critical rows are data, so assert them as facts.
_STOP = COMMANDS["hw_motors_stop"]
assert _STOP.cls == CLASS.PRIORITY and _STOP.executor == EXECUTOR.RECEIVER, "e-stop left the priority lane"
assert not _STOP.mutating and not _STOP.may_refuse_busy and _STOP.gate == GATE.NONE, "e-stop became gate-able"
for _n in ("hw_configure", "hw_install", "hw_disconnect"):
    assert COMMANDS[_n].gate == GATE.CONFIRM, f"{_n} must be confirm-only, never arm (§7)"
for _n in ("hw_motors_move", "hw_motors_set_zero", "hw_live_start", "hw_live_update"):
    assert COMMANDS[_n].gate == GATE.ARM, f"{_n} is motion and must require arm (§7)"
for _n in ("status", "get_state", "validate_pose", "hw_status", "hw_analyze_frame",
           "hw_read_telemetry", "hw_get_info"):
    assert COMMANDS[_n].gate == GATE.NONE, f"{_n} must stay open so the agent can look before it touches"


# ---------------------------------------------------------------------------
# Dispatch & the derived taxonomy (condition 3)
# ---------------------------------------------------------------------------
def lookup(cmd: str) -> Command:
    """Resolve a command for dispatch. Unknown is E_INVAL, never a symbol lookup (§4.4)."""
    try:
        return COMMANDS[cmd]
    except (KeyError, TypeError):                    # incl. unhashable cmd (dict/list off the wire)
        raise McpError(ERR.INVAL, f"unknown command {cmd!r}") from None


def classify(cmd: str, args: dict) -> Command:
    """Return the Command with class/executor/mutating/may_refuse_busy resolved for THESE args.

    solve_ik is the one whose class is derived, not declared (condition 3):
      dry_run + seed_q  -> PURE / WORKER   (touches nothing: Core.solve on a copy; concurrent,
                                             never busy-refused, does not set _state.busy)
      dry_run, no seed  -> READ  / TICK    (must read rig.last_q on the main thread)
      execute/apply     -> WRITE / TICK    (ccd/gradient inline) — memetic applies after a worker solve
    """
    spec = lookup(cmd)
    if cmd != "solve_ik" or not isinstance(args, dict):
        return spec
    dry = bool(args.get("dry_run")); seeded = isinstance(args.get("seed_q"), (list, tuple)) \
        and len(args["seed_q"]) == 7
    if dry and seeded:
        return Command(cmd, CLASS.PURE, EXECUTOR.WORKER, spec.gate, mutating = False,
                       may_refuse_busy = False, deadline_ms = spec.deadline_ms,
                       handler = spec.handler, note = "dry+seed: pure what-if, off the tick")
    if dry:
        return Command(cmd, CLASS.READ, EXECUTOR.TICK, spec.gate, mutating = False,
                       may_refuse_busy = False, deadline_ms = spec.deadline_ms,
                       handler = spec.handler, note = "dry: reads last_q on the main thread")
    if str(args.get("solver", "")).lower() == "memetic":
        return Command(cmd, CLASS.WRITE, EXECUTOR.WORKER, spec.gate, mutating = True,
                       may_refuse_busy = True, deadline_ms = spec.deadline_ms,
                       handler = spec.handler, note = "memetic: worker solve, main-thread apply")
    return spec


# ---------------------------------------------------------------------------
# Frame codec (NDJSON). Server-side heavy validation lives with the server; here we only
# guarantee well-formed, single-line, JSON-safe frames and never echo a raw NUL.
# ---------------------------------------------------------------------------
def encode(obj: dict) -> bytes:
    """Serialise one frame to a single \\n-terminated line.

    The guard checks the BYTES that go on the wire, not the JSON text. Testing the text would be a
    tautology and would look like a safety check while being one: json.dumps escapes control
    characters, so a payload string containing a NUL comes back as the six characters of an escape
    sequence and the test `\\x00 in line` can never fire (measured). What could actually break the
    framing is a raw NUL or an early newline surviving the encoder, so that, and only that, is what
    is refused here."""
    line = json.dumps(obj, separators = (",", ":"), ensure_ascii = True,
                      allow_nan = False, sort_keys = True)
    payload = (line + "\n").encode("utf-8")
    body = payload[:len(payload) - 1]                     # strip the one terminator we just added
    if b"\n" in body or b"\r" in body or b"\x00" in body:
        raise McpError(ERR.PROTO, "frame would break the NDJSON framing")
    return payload


def decode(line: bytes | str) -> dict:
    """Parse one wire line into a request/response dict. Malformed is E_PROTO (§4.5)."""
    if isinstance(line, bytes):
        if len(line) > 1_048_576:                      # 1 MiB ceiling: bound the read, no OOM
            raise McpError(ERR.PROTO, "frame too large")
        try:
            line = line.decode("utf-8", errors = "strict")
        except UnicodeDecodeError as e:
            raise McpError(ERR.PROTO, f"frame is not valid UTF-8: {e}") from None
    line = line.strip()
    if not line:
        raise McpError(ERR.PROTO, "empty frame")
    try:
        obj = json.loads(line)
    except (ValueError, UnicodeDecodeError) as e:
        raise McpError(ERR.PROTO, f"frame is not valid JSON: {e}") from None
    if not isinstance(obj, dict):
        raise McpError(ERR.PROTO, "frame is not a JSON object")
    return obj


def hello(*, proto_rev: str, token: str, client: str = "mcp-server", info: dict | None = None) -> dict:
    return {"hello": {"protocol": PROTOCOL, "proto_rev": proto_rev, "auth": token,
                      "client": client, "info": info or {}}}


def response(req_id, data: dict) -> dict:
    return {"id": req_id, "ok": True, "data": data}


def error_response(req_id, code: str, message: str, data: dict | None = None) -> dict:
    out = {"id": req_id, "ok": False, "error": {"code": code, "message": message}}
    if data:
        out["error"]["data"] = data
    return out


# ---------------------------------------------------------------------------
# Secrets (contract §4.3) — generated, not invented by the operator.
# ---------------------------------------------------------------------------
def gen_token(nbytes: int = 32) -> str:
    return _secrets.token_urlsafe(nbytes)             # URL-safe, 0600 runtime file holds it


def token_ok(candidate: str, expected: str) -> bool:
    """Constant-time token compare; empty/None never matches (there is no no-auth path here —
    that lives behind auth.insecure_no_auth, which is mutually exclusive with hardware.enabled)."""
    if not isinstance(candidate, str) or not isinstance(expected, str) or not expected:
        return False
    return hmac.compare_digest(candidate.encode("utf-8"), expected.encode("utf-8"))


# ---------------------------------------------------------------------------
# proto_rev — the drift guard for the vendored copy (§2.5, decision 5).
# ---------------------------------------------------------------------------
def proto_rev() -> str:
    """Deterministic hash of the catalogue + wire constants. Both ends embed it; a mismatch is
    E_PROTO at connect — the earliest point you want to learn the two disagree."""
    rows = [
        [c.cmd, c.cls, c.executor, c.gate, c.mutating, c.may_refuse_busy, c.deadline_ms, c.handler]
        for _, c in sorted(COMMANDS.items())
    ]
    payload = json.dumps({"protocol": PROTOCOL, "err": sorted(vars(ERR).keys() and
                         [v for v in vars(ERR).values() if isinstance(v, str)]),
                          "commands": rows}, sort_keys = True, separators = (",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:12]


# ---------------------------------------------------------------------------
# Filesystem sandbox (contract §9.5) — an untrusted agent gets no arbitrary-write sink.
# ---------------------------------------------------------------------------
def is_within(resolved: str, root: str) -> bool:
    """Pure containment test on already-resolved paths; case-insensitive on Windows."""
    r = os.path.normcase(os.path.realpath(root))
    p = os.path.normcase(os.path.realpath(resolved))
    if r and not p.startswith(r):                      # fast reject, then authoritative check
        pass
    try:
        return os.path.commonpath((p, r)) == r and p != r               # strict: a file under root
    except (ValueError, TypeError):
        return False


def sandbox_under(root: str, candidate: str) -> str:
    """Resolve `candidate` under `root`; REJECT `..` traversal, absolute escapes, and symlink
    escape (os.path.realpath first, then a containment check). Out-of-root ⇒ E_ACCES (§9.5)."""
    if not isinstance(candidate, str) or not candidate:
        raise SanitizeError("empty path")
    if os.path.isabs(candidate) or candidate.startswith(("/", "\\")) or re_match_drive(candidate):
        raise SanitizeError("absolute paths are not permitted")        # must be relative to export_root
    root_abs = os.path.realpath(os.path.expanduser(root))
    joined = os.path.join(root_abs, *candidate.replace("\\", "/").split("/"))
    resolved = os.path.realpath(joined)                               # follow symlinks, then judge
    if not is_within(resolved, root_abs):
        raise SanitizeError(f"path escapes export_root: {candidate!r}")
    return resolved


def _re_match_drive(p: str) -> bool:
    return bool(len(p) >= 2 and p[1] == ":" and p[0].isalpha())      # "C:…" on Windows
re_match_drive = _re_match_drive
