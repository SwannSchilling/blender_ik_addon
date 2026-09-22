# SPDX-License-Identifier: BSD-3-Clause
"""mcp_bridge — the in-Blender half of the PickIK MCP integration.

Loads INSIDE Blender (bundled 3.11). Imports `bpy`; therefore must never be imported by the
server process. It is the only new module that both (a) opens a socket and (b) touches the UI
thread, and it keeps those two apart exactly as the add-on's own `_bg_solve`/`_drain_pending`
already do (MCP_INTEGRATION_PLAN.md §5).

Two rules carry the whole design:
  * the RECEIVER thread never touches bpy — it parses, classifies, and either handles the
    e-stop inline or hands the job to a lane;
  * anything that reads/writes the scene runs on the MAIN thread, drained to the per-tick budget.

Dispatch goes only through `mcp_protocol.classify()` — a closed allow-list. This file must never
`eval`/`exec`/name-resolve anything that arrived on the wire. Commands are routed by
`spec.executor` (NOT by class): a `pure` command runs off-thread, a `read`/`write` command runs on
the tick, and a *mutating* `WORKER` command (memetic, and all Phase-3 hardware) runs off-thread and
posts its scene-applying step back to the main thread via `post()`.
"""
from __future__ import annotations

import collections
import errno
import json
import os
import queue
import socket
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import bpy                                             # this module is Blender-only by design

try:                                                   # loaded as part of the add-on package — the normal
    from . import mcp_protocol as P                    # path, exactly like `from . import ik_core`
except ImportError:                                    # bare top-level import: a standalone probe or the
    import mcp_protocol as P                           # headless harness put the add-on dir on sys.path

_FRAME_MAX = 1_048_576                                 # 1 MiB wire-frame ceiling (§4.2)

#: The only clock the bridge uses to measure an INTERVAL (the tick budget, a request deadline, the
#: handshake window) — `time.perf_counter`, and that is not a style preference. On this Windows
#: build `time.monotonic` is GetTickCount-based with a ~16 ms granularity: it reads 11 ms of busy
#: work as 0 ms, so a 4 ms budget measured on it never expires and the lane drains unbounded —
#: precisely the failure mode §5.3 exists to prevent (measured: 12 jobs in one tick on monotonic,
#: 4 jobs in one tick on perf_counter, same 1 ms handlers). Only deltas are ever taken from this
#: clock, which is all `perf_counter` promises; wall-clock stamps still use `time.time`.
_now = time.perf_counter


def _in_headless() -> bool:
    """The single headless detector (§2.3). `bpy.app.background` is the real attribute on 4.5 LTS;
    `in_background`/`headless` do NOT exist. Funnelling it here keeps the two call sites from
    drifting and makes the guard testable."""
    try:
        bg = bool(bpy.app.background)
    except AttributeError:                              # defensive: an unexpected bpy build
        bg = False
    return bg or ("-b" in sys.argv) or ("--background" in sys.argv)


#: How to half close each direction, resolved defensively: every CPython build in use here exports
#: these two names, but the values (SD_SEND / SD_RECEIVE on Windows) are what actually matter.
_SHUT_WR = getattr(socket, "SHUT_WR", 1)
_SHUT_RD = getattr(socket, "SHUT_RD", 0)


def _bye(conn) -> None:
    """End a connection so the peer still receives the answer that was last written to it.

    A bare `close()` is not enough on Windows, for two separate reasons, both measured here:

      * unsent data: a socket closed while its send buffer still holds the refusal we just wrote can
        be reset instead of flushed, so a wrong token or a malformed frame goes missing under the
        client's nose — it sees a dropped connection where an E_PROTO/E_ACCES was earned;
      * unread data: a socket closed while the *receive* queue still holds bytes the peer pipelined
        (an agent that sends its first request immediately behind the hello, which the wire format
        permits) makes the stack answer with an RST, which discards the reply as well.

    So: half close the write side to flush the reply and send a FIN, then half close the read side to
    drop whatever is unread, and only then release the socket (§4.1)."""
    if conn is None:
        return
    for how in (_SHUT_WR, _SHUT_RD):                        # flush the reply, then clear the inbox
        try:
            conn.shutdown(how)
        except OSError:
            pass                                            # already gone: nothing left to flush
    try:
        conn.close()
    except OSError:
        pass


class _Job:
    __slots__ = ("req_id", "cmd", "spec", "args", "deadline", "conn", "owns_mutation")
    def __init__(self, req_id, cmd, spec, args, deadline, conn, owns_mutation = False) -> None:
        self.req_id, self.cmd, self.spec, self.args = req_id, cmd, spec, args
        self.deadline, self.conn, self.owns_mutation = deadline, conn, owns_mutation


class BridgeServer:
    """One authenticated client, one in-flight *mutating* command, normal lane drained to budget."""
    def __init__(self, *, host = "127.0.0.1", port = 9876, token = "", insecure_no_auth = False,
                tick_interval = 0.05, tick_budget_ms = 4.0, request_timeout_ms = 5000,
                export_root = None, handlers = None, max_handshake_attempts = 5) -> None:
        if host not in ("127.0.0.1", "localhost", "::1"):
            raise ValueError(f"refusing to bind a routable interface: {host!r}")   # §9 loopback only
        self.host, self.requested_port = host, port
        self.token, self.insecure_no_auth = token, insecure_no_auth
        self.tick_interval, self.tick_budget_ms = tick_interval, float(tick_budget_ms)
        self.request_timeout_ms = int(request_timeout_ms)
        self.export_root = export_root
        self.handlers = dict(handlers or {})
        self.max_handshake_attempts = max_handshake_attempts

        self._listen = None
        self._accept_thread = None
        self._lane = queue.Queue()                       # normal lane (read/write) — main thread drains
        self._post = collections.deque()                 # main-thread closures a worker posts (apply/fail-safe)
        self._post_lock = threading.Lock()
        self._pool = None                                # off-tick pool, created on start
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._client = None                              # (conn, addr); single client (§4.1)
        self._pending_mutating = 0                       # single-flight guard for mutating commands
        self._handshakes = []                            # timestamps, rate-capped (§4.3)
        self._running = threading.Event()
        self.abort_flag = threading.Event()              # polled by hw lanes (§5.3); exposed for API
        self._tick_fn = None                             # bound callback we registered with bpy.app.timers
        self._main_ident = None                          # identity of the thread that called start()
        self.bound_port = None
        self.runtime_path = None
        self.last_error = ""

    # -- lifecycle -------------------------------------------------------------
    def start(self, *, runtime_file = None, allow_headless = False) -> None:
        """Bind, publish the runtime file, start the accept thread; in UI mode also arm the tick.
        In headless the caller pumps `pump_once()` itself (the timer never fires without an event loop)."""
        if self._running.is_set():
            return
        if not self.token and not self.insecure_no_auth:
            self.token = P.gen_token()                    # §4.3 — generated, never demanded of the operator
        if self.insecure_no_auth:                        # §4.3 — loud, and never co-exists with hardware
            print("[pickik-mcp] WARNING: insecure_no_auth is ON — anyone on loopback can drive the "
                  "bridge. It is mutually exclusive with hardware.enabled and must never face a live "
                  "CAN bus.", file=sys.stderr)
        self._main_ident = threading.main_thread().ident   # the UI/main thread for this process
        lst = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lst.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            lst.bind((self.host, self.requested_port))   # port 0 => ephemeral, discovered below
        except OSError as e:                              # §4.1 never fall back silently
            lst.close()
            raise RuntimeError(f"bridge could not bind {self.host}:{self.requested_port}: {e}") from None
        lst.listen(4)
        self.bound_port = lst.getsockname()[1]
        self._listen = lst
        self._pool = ThreadPoolExecutor(max_workers = 4, thread_name_prefix = "pickik-off-")
        self._running.set()
        self._accept_thread = threading.Thread(target = self._accept_loop, daemon = True,
                                              name = "pickik-accept")
        self._accept_thread.start()
        self.runtime_path = self._write_runtime_file(runtime_file)
        if not _in_headless():
            self._arm_tick()
        return None

    def _arm_tick(self) -> None:
        if self._tick_fn is not None and bpy.app.timers.is_registered(self._tick_fn):
            return
        def _tick():                                    # returns the next interval (keeps firing)
            self.pump_once()
            return self.tick_interval
        self._tick_fn = _tick
        bpy.app.timers.register(self._tick_fn, first_interval = self.tick_interval)

    def stop(self) -> None:
        if not self._running.is_set():
            return
        self._running.clear()
        self.abort_flag.set()                             # tell every off-thread lane to wind down
        if self._tick_fn is not None:
            try:
                if bpy.app.timers.is_registered(self._tick_fn):
                    bpy.app.timers.unregister(self._tick_fn)
            except BaseException:
                pass
            self._tick_fn = None
        for sock in (self._listen, *( (self._client[0],) if self._client is not None else () )):
            try:
                _bye(sock)                                  # flush the last frame before the FIN
            except BaseException:
                pass
        self._listen, self._client = None, None
        if self._pool is not None:
            self._pool.shutdown(wait = False, cancel_futures = True)
            self._pool = None
        for th in filter(None, (self._accept_thread,)):
            if th.is_alive():
                th.join(timeout = 1.0)                     # §8 no detached thread outlives teardown
        self._accept_thread = None
        with self._post_lock:
            self._post.clear()
        with self._state_lock:
            self._pending_mutating = 0

    def is_running(self) -> bool:
        return self._running.is_set()

    # -- runtime file (0600) ---------------------------------------------------
    def _write_runtime_file(self, path) -> str | None:
        if not path:
            return None
        path = os.path.expanduser(path)
        try:
            os.makedirs(os.path.dirname(path), exist_ok = True)
        except OSError:
            pass
        payload = {"host": self.host, "port": self.bound_port, "proto_rev": P.proto_rev(),
                  "token": "" if self.insecure_no_auth else self.token, "pid": os.getpid(),
                  "started_at": int(time.time()), "blender": bpy.app.version_string}
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)   # §4.1 0600
        try:
            os.write(fd, json.dumps(payload).encode("utf-8"))
        finally:
            os.close(fd)
        try:
            os.chmod(path, 0o600)                            # the intent; the platform may ignore it
        except OSError:
            pass
        mode = 0o600
        try:
            mode = os.stat(path).st_mode & 0o777
        except OSError:
            pass
        if mode & 0o077:                                     # the bits did not take. On Windows they
            home = os.path.normcase(os.path.realpath(os.path.expanduser("~")))
            here = os.path.normcase(os.path.realpath(path))  # are advisory at best: 0600 comes out 0666
            if not here.startswith(home + os.sep):           # so ask where the file actually lives
                self.last_error = f"runtime file is readable by others: {path} (mode 0{mode:o})"
                print(f"[pickik-mcp] WARNING: the runtime file {path} holds the bridge token, is "
                      f"readable by group or world (mode 0{mode:o} on this platform), and sits outside "
                      f"the operator's home. Move runtime_file somewhere private.", file=sys.stderr)
        return path

    # -- accept / handshake / serve -------------------------------------------
    def _accept_loop(self) -> None:
        while self._running.is_set():
            try:
                conn, addr = self._listen.accept()
            except OSError:
                break                                       # listening socket closed on stop
            threading.Thread(target = self._serve_client, args = (conn, addr),
                           daemon = True, name = "pickik-client").start()

    def _prune_handshakes(self) -> bool:
        # The handshake window is an interval, so it is measured on _now (perf_counter),
        # never on time.time: a wall clock can step backwards and would open the gate.
        cut = _now() - 60.0
        self._handshakes = [t for t in self._handshakes if t > cut]
        if len(self._handshakes) >= self.max_handshake_attempts:
            return False
        self._handshakes.append(_now())
        return True

    def _serve_client(self, conn, addr) -> None:
        conn.settimeout(self.request_timeout_ms / 1000.0)
        buf = b""
        try:
            line, buf, ok = self._read_line(conn, buf)
            if not ok:
                return
            try:
                hs = P.decode(line)
            except P.McpError as e:
                self._send(conn, P.error_response(None, e.code, e.message)); return
            hello = hs.get("hello")
            if not isinstance(hello, dict):
                self._send(conn, P.error_response(None, P.ERR.PROTO, "expected a hello frame")); return
            with self._state_lock:
                if self._client is not None:
                    self._send(conn, P.error_response(None, P.ERR.ACCES, "a client is already connected"))
                    return
                if not self._prune_handshakes():
                    self._send(conn, P.error_response(None, P.ERR.ACCES, "handshake rate limit")); return
                peer_rev = str(hello.get("proto_rev", ""))
                if peer_rev != P.proto_rev():
                    self._send(conn, P.error_response(None, P.ERR.PROTO,
                                f"proto_rev mismatch (bridge {P.proto_rev()} != client {peer_rev!r})"))
                    return
                if not self.insecure_no_auth and not P.token_ok(str(hello.get("auth", "")), self.token):
                    self._send(conn, P.error_response(None, P.ERR.ACCES, "auth failed"))
                    return                                                    # the token is never logged
                self._client = (conn, addr)
                if not self._pending_mutating:
                    #: A fresh session is a fresh intent, and the fail-safe that stopped the world when
                    #: the last one was dropped has done its work. Left set, every idle gap between two
                    #: of a caller's turns -- the bridge drops an idle client at its own read patience,
                    #: and the drop runs this same finally -- would leave the arm refusing every motion
                    #: for the rest of the bridge's life, which is not a fail-safe but a fault. Never
                    #: cleared while work is in flight: a session that dropped mid-motion is not the
                    #: same session, re-admitted.
                    self.abort_flag.clear()
            self._send(conn, {"hello": {"protocol": P.PROTOCOL, "proto_rev": P.proto_rev(),
                        "server": "pickik-blender", "blender": bpy.app.version_string,
                        "hw": {"present": False, "enabled": False}}})
            while self._running.is_set():
                line, buf, ok = self._read_line(conn, buf)
                if not ok:
                    break
                self._dispatch_line(conn, line)
        except socket.timeout:
            self.last_error = "client read timeout"
        except OSError as e:
            if getattr(e, "errno", None) not in (errno.EBADF, errno.ECONNRESET):
                self.last_error = str(e)
        finally:
            with self._state_lock:
                if self._client is not None and self._client[0] is conn:
                    self._client = None
                    self.abort_flag.set()                 # §7.4 fail-safe: any motion must stop on drop
            _bye(conn)                                    # the last frame must reach the peer

    def _read_line(self, conn, buf):
        """Return (line|None, remaining_buffer, ok). ok=False means the connection is done."""
        while b"\n" not in buf:
            try:
                chunk = conn.recv(65536)
            except (ConnectionResetError, OSError):
                return None, buf, False
            if not chunk:
                return None, buf, False
            buf += chunk
            if len(buf) > _FRAME_MAX:
                self._send(conn, P.error_response(None, P.ERR.PROTO, "frame too large"))
                return None, buf, False
        head, _, buf = buf.partition(b"\n")
        return head, buf, True

    # -- dispatch (route by executor, never by class alone) --------------------
    def _dispatch_line(self, conn, line) -> None:
        try:
            frame = P.decode(line)
        except P.McpError as e:
            self._send(conn, P.error_response(None, e.code, e.message)); return
        req_id = frame.get("id")
        cmd = frame.get("cmd")
        args = frame.get("args") if isinstance(frame.get("args"), dict) else {}
        try:
            spec = P.classify(cmd, args)                    # allow-list; unknown => E_INVAL
        except P.McpError as e:
            self._send(conn, P.error_response(req_id, e.code, e.message)); return
        reason = self._gate_reject(spec, args)               # policy here, not duplicated per-handler (§7)
        if reason is not None:
            self._send(conn, P.error_response(req_id, P.ERR.ACCES, reason)); return
        handler = self.handlers.get(cmd)
        if handler is None:
            self._send(conn, P.error_response(req_id, P.ERR.STATE,
                        f"command {cmd!r} has no handler in this build")); return
        timeout = args.get("deadline_ms") or self.request_timeout_ms
        deadline = _now() + max(0.0, float(timeout) / 1000.0)
        job = _Job(req_id, cmd, spec, args, deadline, conn)

        if spec.executor == P.EXECUTOR.RECEIVER:            # the e-stop: inline, never queued (§5.3)
            self._run_job(job); return
        if spec.mutating:                                    # single-flight: at most one mutating cmd
            with self._state_lock:
                if self._pending_mutating >= 1:
                    self._send(conn, P.error_response(req_id, P.ERR.BUSY, "another action is running"))
                    return
                self._pending_mutating += 1
                job.owns_mutation = True
        if spec.executor == P.EXECUTOR.WORKER:               # pure what-ifs AND mutating workers (memetic)
            if self._pool is None:
                self._release_mutation(job)
                self._send(conn, P.error_response(req_id, P.ERR.STATE, "bridge stopping")); return
            self._pool.submit(self._run_job, job)
        else:                                                # TICK: read/write land on the normal lane
            self._lane.put(job)

    def _gate_reject(self, spec, args):
        """Return a human reason string if the command's gate is not satisfied, else None (§7).
        `confirm` = non-motion privileged (one key); `arm` = motion (two keys). The e-stop and all
        observation commands are GATE.NONE and reach here only to be waved through."""
        if spec.gate == P.GATE.NONE:
            return None
        if spec.gate == P.GATE.CONFIRM:
            if args.get("confirm") is not True:
                return f"{spec.cmd} is privileged and needs confirm=true (it moves nothing)"
            return None
        if spec.gate == P.GATE.ARM:                            # motion only — the two-key path
            if args.get("arm") is not True:
                return f"{spec.cmd} moves the physical arm and needs arm=true"
            if str(args.get("confirm", "")).strip().lower() != P.MOTION_CONFIRM_PHRASE.lower():
                return (f"{spec.cmd} also needs the literal confirm phrase "
                        f"{P.MOTION_CONFIRM_PHRASE!r}")
            return None
        return f"{spec.cmd} has an unknown gate {spec.gate!r} — refusing"

    def post(self, closure) -> None:
        """Enqueue a closure to run on the main thread (a worker's scene-apply / a fail-safe). Safe
        to call from any thread; runs under the per-tick budget like any main-lane job."""
        with self._post_lock:
            self._post.append(closure)

    def is_main(self) -> bool:
        """True when the caller is the thread that started the bridge (the UI thread in Blender)."""
        return self._main_ident is not None and threading.get_ident() == self._main_ident

    def call_on_main(self, fn, timeout = None):
        """Run `fn` on the main thread and return its result. A handler reaches `bpy` ONLY through
        this: inline when already on the main thread (read/write jobs), or posted-and-waited when on
        an off-thread worker (memetic's apply, hardware's bus work) — mirroring _bg_solve/_drain_pending."""
        if self.is_main():
            return fn()
        box, ev = {}, threading.Event()
        def _run():
            try:
                box["v"] = fn()
            except BaseException as e:
                box["e"] = e
            finally:
                ev.set()
        self.post(_run)
        budget = (self.request_timeout_ms / 1000.0 + 1.0) if timeout is None else float(timeout)
        if not ev.wait(budget):                            # outcome-unknown, never assume silence is success
            raise P.McpError(P.ERR.TIMEOUT,
                             "main-thread step timed out; call get_state to reconcile before acting")
        if "e" in box:
            raise box["e"]
        return box.get("v")

    # -- main-thread drain (the tick calls this; headless callers pump it) -----
    def pump_once(self, budget_ms = None) -> int:
        """Drain post-closures first (they are cheap and unblock waiters), then the normal lane,
        until the per-tick budget is spent. Never blocks the caller beyond that budget (§5.3)."""
        if not self._running.is_set():
            return 0
        budget = (self.tick_budget_ms if budget_ms is None else float(budget_ms)) / 1000.0
        start = _now()
        done = 0
        while _now() - start < budget:
            ran = False
            with self._post_lock:
                if self._post:
                    closure = self._post.popleft()
                    self._run_closure(closure)
                    ran = True
            if not ran:
                try:
                    job = self._lane.get_nowait()
                except queue.Empty:
                    break
                self._run_job(job)
            done += 1
        return done

    def _run_closure(self, closure) -> None:
        try:
            closure()
        except BaseException as e:                           # a posted closure must never break the loop
            self.last_error = f"post: {e!r}"

    def _release_mutation(self, job) -> None:
        if job.owns_mutation:
            with self._state_lock:
                self._pending_mutating = max(0, self._pending_mutating - 1)
            job.owns_mutation = False

    def _run_job(self, job) -> None:
        if not self._running.is_set():
            self._release_mutation(job); return
        if _now() > job.deadline:                   # §4.5 outcome-unknown, not "did not happen"
            self._release_mutation(job)
            self._send(job.conn, P.error_response(job.req_id, P.ERR.TIMEOUT,
                        "deadline passed before execution; call get_state to reconcile"))
            return
        try:
            data = self.handlers[job.cmd](job.args, self)     # handler contract: (args, bridge) -> dict
            self._send(job.conn, P.response(job.req_id, data if isinstance(data, dict) else {"value": data}))
        except P.McpError as e:
            self._send(job.conn, P.error_response(job.req_id, e.code, e.message, e.data))
        except BaseException as e:                             # §5.4 never propagate into the UI loop
            self.last_error = f"{job.cmd}: {e!r}"
            self._send(job.conn, P.error_response(job.req_id, P.ERR.INTERNAL, f"{type(e).__name__}: {e}"))
        finally:
            # Release the single-flight slot once the job is fully done. `_release_mutation` guards
            # on owns_mutation, so this is idempotent and safe on the timeout-drop path too. A
            # mutating WORKER handler (memetic) blocks until its posted apply has run on the main
            # thread, so by the time we get here the scene mutation is complete.
            self._release_mutation(job)

    # -- transport -------------------------------------------------------------
    def _send(self, conn, obj) -> None:
        try:
            payload = P.encode(obj)
        except P.McpError as e:
            payload = P.encode({"id": None, "ok": False, "error": {"code": e.code, "message": e.message}})
        with self._send_lock:
            try:
                conn.sendall(payload)
            except OSError:
                pass                                          # client gone; the serve loop cleans up

    # -- introspection for the panel -------------------------------------------
    def status_dict(self) -> dict:
        with self._state_lock:
            return {"running": self._running.is_set(), "host": self.host, "port": self.bound_port,
                   "client": bool(self._client), "headless": _in_headless(),
                   "insecure_no_auth": self.insecure_no_auth, "queued": self._lane.qsize(),
                   "pending_mutating": self._pending_mutating, "abort": self.abort_flag.is_set(),
                   "last_error": self.last_error}


# -- module-level singleton that register()/the panel drive ----------------------
_SERVER = None


# Keywords that belong to BridgeServer.start() and not to BridgeServer.__init__. The constructor is
# keyword-only and takes no **kwargs, so a single one of these arriving there is a TypeError. The
# previous filter stripped only "allow_headless", so every Start pressed from the panel -- whose call
# site passes runtime_file through this wrapper -- died on:
#     TypeError: BridgeServer.__init__() got an unexpected keyword argument 'runtime_file'
# which the operator's `except BaseException` then swallowed, leaving the panel on "not running" with
# nothing said anywhere about it. No test caught it because the suite builds the server itself and
# calls the instance's start(), so this module-level wrapper -- the only path the product uses -- was
# never traversed by anything.
_START_ONLY_KW = ("allow_headless", "runtime_file")


def start(**kw) -> BridgeServer:
    global _SERVER
    if _SERVER is not None and _SERVER.is_running():
        _SERVER.stop()
    if _in_headless() and not kw.get("allow_headless", False):        # §2.3 the headless guard
        raise RuntimeError("refusing to auto-bind a socket while headless (pass allow_headless=True "
                          "from the test harness)")
    srv = BridgeServer(**{k: v for k, v in kw.items() if k not in _START_ONLY_KW})
    srv.start(runtime_file = kw.get("runtime_file"), allow_headless = kw.get("allow_headless", False))
    _SERVER = srv
    return srv


def stop() -> None:
    global _SERVER
    if _SERVER is not None:
        _SERVER.stop()
        _SERVER = None


def get() -> BridgeServer | None:
    return _SERVER if (_SERVER is not None and _SERVER.is_running()) else None