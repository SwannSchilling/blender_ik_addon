# PickIK arm7 — MCP integration plan (Model Context Protocol ↔ Blender)

Status: **design approved (v2) — implementation-locked** · Owner: URDF_BIO_IK suite · Rev: 2026-09-06

This document is the contract for Phase 1 of an MCP (Model Context Protocol) integration for
the `blender_ik_addon`. It fixes the architecture, the wire protocol, the tool catalogue, and the
safety model **before** any code is written. Implementation **MUST NOT** diverge from the
requirements here without updating this file first (RFC-2119 keywords *MUST*, *SHOULD*, *MAY* are
normative). v2 folds in the review conditions 1–7 and the five open-question rulings; the
command-class taxonomy in §5 is load-bearing — conditions 1, 3 and 4 all depend on it, so it is
settled here rather than left to the implementer.

### Decisions locked

| # | Decision | Choice |
|---|---|---|
| 1 | Transport | **TCP socket + main-thread timer drain** (mirror of `_bg_solve`/`_drain_pending`) |
| 2 | Server language | **Python + official `mcp` SDK** (`mcp>=1.26`), stdio |
| 3 | Physical CubeMars/CAN | **Yes**, behind a split gate (§7): motion needs `arm`+`confirm`, non-motion privileged needs `confirm` only; `hw_motors_stop` never gated |
| 4 | This step | Plan-first, now **approved**; Phase 1 follows |
| 5 | Home of the server | **Sibling repo `pick_ik_mcp`**; `mcp` must never be pip-installed into Blender |
| 6 | Port | Keep **9876** but **do not hard-code it** (§4.1): bind + publish a `0600` runtime file; `port:0`=ephemeral |
| 7 | Auth | Secret **required but auto-generated**; `insecure_no_auth` is mutually exclusive with `hardware.enabled` (§4.3) |
| 8 | `hw_motors_move_to` | **Dropped from v1**; Phase 3 only via an explicit `plan_id` (§6.4) |

---

## 0. TL;DR

An LLM agent speaks MCP over **stdio** to a thin `mcp_server.py` that runs as a **separate process**
(never inside Blender, in the sibling `pick_ik_mcp` repo). The server opens a **TCP loopback**
connection to a tiny bridge that lives **inside the add-on**. That bridge runs one socket *receiver*
thread that only ever enqueues, and a `bpy.app.timers` tick that drains the queue onto the **main
thread** — the exact primitive the add-on already uses for background memetic solves. Commands
dispatch through a hard **allow-list**; the bridge never `eval`s anything. Every command is one of
three **classes** (`pure` / `read` / `write`) plus a `hw` execution context, which decides whether
it may run off-thread, whether it takes the busy guard, and whether it can be refused `E_BUSY`.

The **e-stop is on its own priority lane**: it is never queued behind the motion it must interrupt,
is never refused `E_BUSY`, is handled on the receiver thread (it touches `python-can`, not `bpy`),
and flips a cooperative `abort_flag` that running motion loops poll.

```
LLM agent (MCP client)
   │  MCP / stdio            JSON-RPC 2.0                         [official mcp SDK]
   ▼
mcp_server.py   (system Python 3.13, OUTSIDE Blender — pick_ik_mcp repo)
   │  TCP 127.0.0.1:9876     newline-delimited JSON, "pickik-bridge/1" + proto_rev
   ▼
mcp_bridge.py   (INSIDE the add-on, Blender's bundled Python 3.11)
   ├─ receiver thread → enqueue ONLY; e-stop handled inline (no bpy) ; abort_flag
   └─ bpy.app.timers tick → main thread → drain-to-budget → dispatch allow-list
                              │
      ┌───────────────────────┼───────────────────────────┐
      ▼                       ▼                           ▼
 bpy.ops.pickik.*       scene.pickik.*            ik_core.Core / CubeMarsDriver
 (build_rig, solve,    (authoritative props)     (pick_ik_c.dll · python-can/gs_usb)
  apply_fk, cubemars_*)
```

---

## 1. Scope

**In scope (v1):** expose the add-on's existing capabilities to an agent — observe the arm's state,
build/clean the rig, solve IK (all three solvers, with a genuinely concurrent dry-run), drive manual
FK, and, under a strict split gate, command the physical CubeMars arm over CAN.

**Out of scope (v1):** the HTTP `ik_service`, the p5 web demo, ROS 2/MoveIt, the offline batch
collision/workspace tool, and the one-shot `hw_motors_move_to` (Phase 3, via `plan_id`). The bridge
talks to the add-on only, not to the service.

**Guiding principle.** The add-on already has a complete, headlessly-tested operator surface
(`test_acceptance.py` drives everything through `bpy.ops.pickik.*`). The MCP layer is a *transport +
policy* wrapper around it — it must not grow solver code, must not re-derive FK, must not invent a
second model. `pickik/*` stays the single source of truth.

---

## 2. Architecture & why this shape

### 2.1 Two processes, not one
`bpy` is only safe on Blender's main thread, inside its event loop. The `mcp` SDK server is
asyncio-based. Running that loop on the main thread stalls the UI; running it on a worker thread
makes it call `bpy` cross-thread. So the **MCP server is a separate process** (no `bpy` import — it
is a dumb proxy that validates, forwards one frame, awaits one response), and the **bridge lives
inside the add-on**, owning only the main-thread hand-off.

### 2.2 The bridge reuses a proven pattern
`__init__.py` already proves the primitive: a `threading.Thread` worker (`_bg_solve`) that writes a
result slot, drained by `bpy.app.timers.register(_drain_pending, 0.05)` on the main thread, guarded
by `_state.busy`. The bridge is that object under a different trigger: a socket instead of a button,
a JSON command instead of a target tuple. No new threading model is introduced — which is why the
risk is low.

### 2.3 Headless behaviour (Blender `-b`)
`bpy.app.timers` does not fire without a running event loop, so a `-b` run must drain on its own.
The bridge **MUST NOT** auto-bind a listening socket when the headless detector `_in_headless()`
is true (a daemonised render must not open a port). The headless test is **`bpy.app.background`** —
verified on 4.5 LTS; `bpy.app.in_background`, `in_edit_mode` and `headless` do **not** exist and
would make this guard dead code. The add-on **MUST** funnel the check through one helper
(`_in_headless() -> bool`, returning `bpy.app.background or "-b" in sys.argv`), so the two call
sites cannot drift. When the bridge is started *explicitly* in background (only from
`test_mcp_bridge.py`), the tick path is replaced by an inline executor that runs the allow-listed
handler on the calling thread — which is what makes the bridge headlessly testable. The **priority
lane is not gated by `-b`** (it never used the tick anyway), so e-stop semantics are identical in
headless and UI runs and can be tested.

### 2.4 Interpreter split (a real constraint, not ceremony)
- `mcp_bridge.py`, `mcp_handlers_obs.py`, `mcp_handlers_hw.py`, `mcp_protocol.py` load **inside
  Blender** and compile against its **bundled Python 3.11** (4.x). They **MUST** be 3.11-compatible
  (the `from __future__ import annotations` style applies) and are the *only* new modules allowed to
  `import bpy`.
- `mcp_server.py` runs under **system Python 3.13** and may use the modern `mcp` SDK. It **MUST
  NOT** `import bpy`, `ik_core`, `cubemars_driver`, or `arm7_rig`.

### 2.5 Layering (separation of concerns)
The wire contract is defined once and shared, so the two runtimes cannot drift:

| Layer | Module | Runs in | May import |
|---|---|---|---|
| Wire / codec | `mcp_protocol.py` | **both** | stdlib only (no `bpy`) |
| Bridge / transport | `mcp_bridge.py` | Blender | `bpy`, `mcp_protocol` |
| Handlers (observe/IK) | `mcp_handlers_obs.py` | Blender | `bpy`, `ik_core`, `arm7_rig`, `mcp_protocol` |
| Handlers (hardware) | `mcp_handlers_hw.py` | Blender | `bpy`, `cubemars_driver`, `mcp_protocol` |
| MCP server / stdio | `mcp_server.py` | system | `mcp`, `mcp_protocol`, stdlib |

`mcp_protocol.py` **MUST** be pure stdlib and import-clean in both interpreters (shared vocabulary).

**Vendoring + drift guard (decision 5).** The server lives in the sibling `pick_ik_mcp` repo, so
`mcp_protocol.py` straddles the boundary: the add-on copy is authoritative; `pick_ik_mcp` vendors a
copy. Both embed a `proto_rev` (hash of the command catalogue + schemas). The handshake (✓ §4.3)
exchanges `proto_rev` alongside `protocol: "pickik-bridge/1"`; a mismatch ⇒ `E_PROTO` **at connect**
— the earliest point you want to learn the two ends disagree. `test_mcp_bridge.py` pins the import
path to the add-on copy so gate 14 cannot silently test a stale vendored file.

---

## 3. Files to add / change

```
blender_ik_addon/                         (this repo — loads INSIDE Blender)
├── __init__.py            CHANGE  register()/unregister(): start/stop bridge; add prefs & panel section
├── mcp_protocol.py        NEW     frame codec, error codes, command catalogue + proto_rev  (authoritative)
├── mcp_bridge.py          NEW     receiver thread (enqueue-only) + priority lane + abort_flag + main-thread drain
├── mcp_handlers_obs.py    NEW     observe / rig / IK / FK command implementations
├── mcp_handlers_hw.py     NEW     CubeMars CAN command implementations (split gate, §7)
├── mcp_config.example.json NEW    copy to mcp_config.json — auth, port, export_root, hardware policy
├── requirements.txt                (unchanged — ctypes-only; `mcp` is NOT a Blender-side dep)
├── test_mcp_bridge.py     NEW     headless gate 14+ (imports the add-on's mcp_protocol)
└── MCP_INTEGRATION_PLAN.md         this file

pick_ik_mcp/                            (NEW SIBLING REPO — runs OUTSIDE Blender, uvx-able)
├── mcp_server.py          NEW     stdio MCP server, thin proxy to the bridge
├── vendored/mcp_protocol.py        copy of the add-on's (drift-guarded by proto_rev)
├── mcp_client.py         NEW     reads the 0600 runtime file, opens the TCP connection
├── requirements.txt      NEW     mcp>=1.26
└── README.md             NEW     install + agent spawn (uvx pick_ik_mcp)
```

`mcp` is a **server-side dependency only** and **MUST NOT** be pip-installed into Blender's
interpreter — which the separate repo makes structurally true, not just a policy.

---

## 4. Wire protocol — `pickik-bridge/1` (bridge ⇄ server)

The MCP hop (agent ⇄ server) is defined by the `mcp` SDK and is not invented here. This section
defines the second hop only (server ⇄ bridge).

### 4.1 Link
- Transport: TCP over IPv4, **bind `127.0.0.1` only** (never a routable address).
- **Port is never hard-coded (decision 6).** The bridge binds the configured `port`; `port:0` means
  ephemeral. On a successful bind it writes a runtime file (`runtime_file`, default
  `~/.pickik/bridge.json`, mode **`0600`**) carrying `{port, token, proto_rev, started_at, blender}`.
  The server reads that file instead of a default duplicated across two repos. If an explicitly
  configured port is already taken, the bridge **MUST** fail to start with a clear error — it **MUST
  NOT** silently fall back to another port. Keeping 9876 (not the service's 8081 family) is intentional:
  a mis-dialled HTTP service must fail loudly, not half-work.
- **Exactly one authenticated client at a time (§8, decision).** A second `hello` while a session is
  live is rejected with `E_ACCES`. This also makes §7.4's "the connection dropped" unambiguous — it
  has exactly one referent.
- One in-flight *state-mutating* command per connection (RPC, no pipelining in v1); `pure` commands
  are exempt (§5.3). Idle connections **MAY** be dropped after `idle_timeout_s` (default 300).

### 4.2 Framing — newline-delimited JSON (NDJSON), UTF-8
One JSON object per line, `\n`-terminated. No embedded raw NUL; strings JSON-escaped.

Request (→):
```json
{"id": 7, "cmd": "solve_ik", "args": { "...tool arguments...": null }}
```
Response (←):
```json
{"id": 7, "ok": true,  "data": { "...result...": 1 }}
{"id": 7, "ok": false, "error": { "code": "E_RANGE", "message": "J4 130.0deg > 119.75deg" }}
```
- `id` **MUST** be echoed verbatim; the server correlates on it.
- `ok` is the boolean verdict; `data` **MUST** be present iff `ok`, `error` iff `not ok`.
- `error.code` **MUST** be one of the stable codes in §4.5.

### 4.3 Handshake (mandatory, first frame each way)
Any command before a completed handshake **MUST** be rejected with `E_PROTO`.
```json
→ {"hello": {"protocol": "pickik-bridge/1", "proto_rev": "<sha256_12>", "auth": "<token>", "client": "mcp-server", "info": {…}}}
← {"hello": {"protocol": "pickik-bridge/1", "proto_rev": "<sha256_12>", "server": "pickik-blender",
              "blender": "4.5.3", "addon": "1.x", "hw": {"driver": "gs_usb", "present": true, "enabled": false}}}
```
Auth & secrets (decision 7):
- A shared secret is **required**; it is **not** the operator's job to invent one. On start, if no
  `auth.token` is configured, the bridge generates `secrets.token_urlsafe(32)` and writes it into
  the `0600` runtime file (§4.1). Zero friction, still authenticated.
- The token is compared with `hmac.compare_digest` (constant-time). A wrong token ⇒ close with
  `E_ACCES`. Token-comparison failures and the token itself **MUST NOT** be logged.
- **Handshake attempts are rate-capped** (`auth.max_handshake_attempts`, default 5, per minute); over
  cap ⇒ stop accepting new connections until the window rolls.
- `auth.insecure_no_auth: true` is the only no-auth path (loopback dev). It logs a loud warning **and
  is mutually exclusive with `hardware.enabled: true`** — no-auth and a live CAN bus **MUST NEVER**
  coexist; if both are set, the bridge **MUST** refuse to bring the hardware lane up and log why.
- `proto_rev` mismatch ⇒ `E_PROTO` at connect (§2.5).

### 4.4 Command catalogue constants
The command names are the catalogue; dispatch **MUST** be a closed `dict[str, CommandSpec]` keyed by
`cmd`. Unknown `cmd` ⇒ `E_INVAL` ("unknown command"), never a lookup into an arbitrary symbol table.
This allow-list is what turns "execute arbitrary Python in Blender" into "run one of N vetted
commands".

### 4.5 Error codes (stable, machine-readable)
Codes are advisory string constants mirroring the `pick_ik_c`/C-ABI spirit so the agent can tell the
cause apart without parsing `message`. They are *not* the host OS `errno`.

| Code | Meaning | Trigger / note |
|---|---|---|
| `E_OK` | success | — |
| `E_INVAL` | malformed frame / bad argument | bad enum member, wrong arity, non-numeric |
| `E_RANGE` | value out of range | joint angle outside the Design B limits |
| `E_BUSY` | resource busy | a *mutating* command is running (`_state.busy`). **Never returned for the e-stop or for `pure` commands** (§5.3) |
| `E_AGAIN` | temporary, retry | CAN read with no frames yet on the bus |
| `E_NOMEM` | allocation | solver create failed |
| `E_ACCES` | not permitted | gate closed, auth failed, or second-client refused (§4.1) |
| `E_HW` | hardware / bus error | `python-can` raise, no adapter, bus-off |
| `E_STATE` | object state invalid | rig not built, DLL missing, target empty missing, **stale `plan_id`** (§6.4) |
| `E_TIMEOUT` | **outcome unknown** | deadline passed before the result was seen. **Not** "it did not happen" — see the note below |
| `E_PROTO` | protocol violation | pre-handshake command, bad framing, `proto_rev` mismatch |
| `E_INTERNAL` | unexpected | uncaught C++/Python exception (reported, never propagated silently) |

> **`E_TIMEOUT` means "outcome unknown", not "did not happen" (condition 2).** The bridge cannot
> preempt a `write`/`read` handler already running inline on the main thread. So: a deadline is
> enforced **before** execution (drop the queued item and reply, nothing ran), and for `hw`/long work
> it is the solver's/driver's own bound. Once a main-thread handler has started it **MUST** run to
> completion — it is never aborted under it. Therefore a `E_TIMEOUT` reply on a mutating command
> tells the agent *the result is undetermined*, and the agent's documented, mandatory recovery is to
> call **`get_state`** to reconcile its world model before it next actuates anything. This matters
> specifically near a physical arm: the agent must not assume "timed out ⇒ nothing moved".

---

### 4.6 Closing a connection (measured on Windows)
A refusal is only a refusal if the peer actually receives it. A bare `close()` loses frames on Windows
two ways, both measured here rather than theorised: unsent data can be reset instead of flushed; and —
the one that bit twice — closing while the **receive** queue still holds bytes the peer pipelined (an
agent that writes its first request immediately behind the `hello`, which §4.2 permits) makes the stack
answer with an RST, discarding the reply that was already on its way. So ending a connection **MUST**
half-close the write side, then the read side, and only then release the socket (`_bye`). A test **MUST**
assert that a refusal for a malformed frame arrives when the client pipelines it behind the `hello`
without waiting for the answer (gate 15).

---

## 5. Execution & threading model (the safety-critical section)

### 5.1 The one invariant
**No `bpy`/`bmesh`/`object` access ever happens off the main thread.** The receiver thread
**MUST** limit itself to `recv`, auth, enqueue, and the *priority lane* (§5.2) — and the priority
lane is legal there precisely because it touches `python-can`, never `bpy`. The `bpy.app.timers` tick
is the sole executor that calls `bpy`-touching handlers, draining the *normal lane* (§5.2).

### 5.2 Command classes (the taxonomy the whole model rests on)
Every command in the catalogue carries a `class` and an `exec` field. These are **not** labels for
decoration; they are what decides concurrency, the busy guard, `E_BUSY`, and where code may run.

| class | touches bpy? | executor | busy-guard? | may be refused `E_BUSY`? | example cmds |
|---|---|---|---|---|---|
| `pure` | no | worker thread (off the tick) | **no** | **never** (runs concurrently) | `solve_ik(dry_run, seed_q)`, `validate_pose(q)` |
| `read` | read-only | main thread, in the tick | no | no | `get_state`, `pickik_status`, `get_target`, `get_robot_info` |
| `write` | mutates | main thread, in the tick | **yes** (`_state.busy`) | yes, while busy | `build_rig`, `set_*`, `solve_ik(execute)`, `export_urdf` |
| `hw` | no (via `cubemars_driver`) | worker, under `_cubemars_task_lock`, **polls `abort_flag`** | n/a (own lock) | yes, while the lock is held | `hw_motors_move`, `hw_*`, `solve_ik(memetic)` |

Priority lane (outside the classes):
- `hw_motors_stop` — **not** in any lane's queue. The receiver thread dispatches it inline (§5.3); it
  touches `python-can` (send the stop/enable-off frames), never `bpy`, so the main-thread invariant
  is not violated.

`memetic` **MUST** run on a worker thread (`num_threads ≥ 1`, `max_time` bounded), never inline on
the main thread (~25–500 ms). `apply_q` to the rig happens only on the main thread after the worker
posted its result — that apply step is a `write`.

### 5.3 Concurrency, back-pressure, and the priority lane
- **Normal lane — drain to budget, not to count (condition 4).** Each tick pops and runs `read`/`write`
  commands **until the add-on's 4 ms main-thread budget for that tick is spent**, then yields to the UI;
  the rest stay queued for the next 50 ms tick. The same "no UI stall" property the old "one command per
  tick" rule gave, without its ~25 ms latency floor or the 20 cmd/s ceiling on cheap `read`s like
  `get_state`. The receiver thread never blocks the tick.
- **`pure` commands run concurrently, off the tick, and take no busy guard.** A `pure` command (a
  `dry_run` with an explicit `seed_q`, a `validate_pose` with an explicit `q`) touches no `bpy` at all
  — it is just `Core.solve`/`Core.fk_tool0` on the ctypes handle — so it is executed on a worker pool,
  is **never** refused `E_BUSY`, and does not set `_state.busy`. This is what gives the agent a real
  what-if loop that does not stall the viewport.
- **One in-flight *mutating* command per connection.** A second `write`/`hw` while one is running is
  refused `E_BUSY` (mirrors the operator's `_state.busy` guard). `read` and `pure` are exempt.
- **The e-stop is never refused and never queued behind its own motion (condition 1).**
  `hw_motors_stop`:
  - is dispatched **inline on the receiver thread** the moment its frame is parsed — it does not wait
    for a tick and is not placed in the normal-lane queue, so it cannot sit behind the `hw_motors_move`
    it exists to interrupt;
  - is exempt from `_cubemars_task_lock` and from the busy guard, so it can **never** answer `E_BUSY`;
  - sets a cooperative **`abort_flag`** (`threading.Event`) that every `hw` streaming/worker loop polls
    each iteration, so an in-flight motion stops issuing new frames promptly; and
  - then acquires the bus briefly to send the stop / motor-disable frames. Releasing enables stopping —
    but the stop path never blocks waiting on a motion that holds the lock: it takes the lock with a
    bounded timeout and, if it cannot get it within that bound, it has already flipped `abort_flag`, so
    the worker is on its way out and the disable lands. A `hw` worker that observes `abort_flag`
    **MUST** cease issuing and release the lock without sending further motion frames.
- **Deadlines.** Every handler carries a per-command `deadline_ms`; enforced before execution (drop +
  reply `E_TIMEOUT`) or via the solver's/driver's own bound. A started main-thread handler is not
  preempted. See the `E_TIMEOUT` semantics note (§4.5): **outcome unknown ⇒ the agent recovers via
  `get_state`.**

### 5.4 Failure containment
A handler raising **MUST** be caught by the dispatcher, logged to the add-on status (`p.status` / the
log), and reported as `{ok:false, error:{code:"E_INTERNAL",...}}`. It **MUST NOT** propagate into the
UI event loop. Stale RNA pointers (`bpy.types.StructRNA` whose object was deleted in the viewport) are
handled exactly as the operators already do — re-adopt/rebuild (`Rig.find()` / `build()`), never crash.

---

### 5.5 The clock the budget is measured on (measured on Windows)
`time.monotonic()` on Windows is `GetTickCount64`: its granularity was measured at **16.0000 ms** —
coarser than the 4 ms budget it exists to enforce. Twelve queued handlers were drained as one batch that
the budget read as 0 ms elapsed, so the budget was decorative and the tick stalled for the whole batch.
`time.perf_counter()` (QPC) measures **0.0033 ms**. Every interval in the bridge — the drain budget, a
job's deadline, the handshake rate window — **MUST** therefore be measured on `perf_counter`; `time.time()`
survives only as the wall stamp written into the runtime file. The same 16 ms floor is why both the pump
and the test harness busy-spin instead of sleeping.

---

## 6. Tool catalogue (what the agent is allowed to call)

Tool names are MCP-side; `cmd` is the bridge command. Class/exec/gate columns come from §5.2 and §7.
All solvers/limits/constants are the ones already compiled into `pick_ik_c.dll`; the bridge exposes
them, it does not add to them. `gate` column: `—` = open, `confirm` = non-motion privileged (one
key), `arm` = motion (two keys, §7).

### 6.1 Observe (read-only)
| tool | cmd | class | exec | gate | returns |
|---|---|---|---|---|---|
| `pickik_status` | `status` | read | tick | — | addon/blender/dll/core/rig/solver/continuous/busy + `cubemars{status,detail,live,enabled}` |
| `get_state` | `get_state` | read | tick | — | `{q_rad[7], q_deg[7], target_xyz_mm[3], tool0_xyz_mm[3], valid}` |
| `get_robot_info` | `get_robot_info` | read | tick | — | `{n_joints:7, joint_names, limits{lower,upper}[rad], axes, units:"rad,m,mm"}` (read-only from the model) |
| `validate_pose` | `validate_pose` | pure | worker | — | `{in_bounds, violations:[{index,value,lower,upper}], tool0_xyz_mm, fk:"c-abi"}` |

`validate_pose` is the agent's feasibility oracle: it calls `Core.fk_tool0(q)` (the same C-ABI FK the
rig agrees with to ~1e-7 m) and checks `q` against the Design B limits, **without** touching the
scene — so it is `pure`, concurrent, and never `E_BUSY`.

### 6.2 Rig & URDF
| tool | cmd | class | exec | gate | notes |
|---|---|---|---|---|---|
| `build_rig` | `build_rig` | write | tick | — | `bpy.ops.pickik.build_rig` (self-heals a deleted rig) |
| `delete_rig` | `delete_rig` | write | tick | `confirm` | `Rig.unlink_all()` + remove rig objects |
| `export_urdf` | `export_urdf` | write | tick | — | `bpy.ops.pickik.save_urdf` — **path-sandboxed, §9.5** |

### 6.3 IK / FK solving (the flagship)
The `dry_run`/seed combination picks the class; the two behaviours are **different on purpose**
(condition 3), not by accident:

| tool | cmd | class | exec | gate | args (subset) → returns |
|---|---|---|---|---|---|
| `solve_ik` (dry, seeded) | `solve_ik` | **pure** | worker | — | `{dry_run:true, seed_q:[7], …}` → `{success,q,error_pos_mm,error_orient_rad,cost,time_ms,applied:false}` — **no `_state.busy`, genuinely concurrent** |
| `solve_ik` (dry, unseeded) | `solve_ik` | **read** | tick | — | `{dry_run:true}` (seed = `rig.last_q`) → as above; needs a main-thread read of `last_q`, so it takes the tick |
| `solve_ik` (apply) | `solve_ik` | **write** | tick (ccd/gradient) · worker+apply (memetic) | — | `{execute:true}` → additionally `arm7_rig.apply_q` on the main thread; `applied:true` |

Shared arg schema (below) is identical across the three rows; `class`/`exec` are **derived from the
args**, not chosen by the caller — a `dry_run` with an explicit `seed_q` is pure by construction
(touches nothing), a `dry_run` without one must read the rig. `set_target`,`get_target` move the
authoritative `Arm7_IK_Target` empty (no snap-back); `set_joint_angles`→`apply_fk` is manual FK;
`set_solver`/`set_solver_config` write the weights; `set_continuous` toggles tracking.

```json
{"name":"solve_ik","description":"Solve one IK for the arm7 (7-DOF) via pick_ik_c.",
 "inputSchema":{"type":"object","required":["target_xyz_mm","solver"],"additionalProperties":false,
  "properties":{
   "target_xyz_mm":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3},
   "quaternion":   {"type":"array","items":{"type":"number"},"minItems":4,"maxItems":4},
   "solver":       {"enum":["ccd","gradient","memetic"]},
   "seed_q":       {"type":"array","items":{"type":"number"},"minItems":7,"maxItems":7},
   "options":{"type":"object","additionalProperties":false,"properties":{
     "position_threshold":{"type":"number"},"orientation_threshold":{"type":"number"},
     "cost_threshold":{"type":"number"},"position_scale":{"type":"number"},
     "rotation_scale":{"type":"number"},"md_weight":{"type":"number"},
     "jt_weight":{"type":"number"},"joint_targets":{"type":"array"},"la_weight":{"type":"number"},
     "look_at":{"type":"object","properties":{"point":{},"axis":{}}}}},
   "execute":{"type":"boolean"},"dry_run":{"type":"boolean"},
   "deadline_ms":{"type":"number"}}}}
```

**Semantics.** Omitted `quaternion` ⇒ **position-only** goal (`orientation_threshold=-1`,
`rotation_scale=0.0`, the stack-wide convention); supplied ⇒ full-pose (`≈1e-3`, `≈0.5`). The target
empty is the source of truth. A no-solution is **not** an error: an out-of-workspace goal returns
`ok:true, success:false, error_pos_mm=…` (the clean no-solution case the acceptance gates pin). The
apply step (`apply_q`) is the only scene mutation and runs on the main thread.

### 6.4 CubeMars hardware — CAN (split gate, §7)
Maps 1:1 onto `cubemars_driver.py`; the bridge **MUST NOT** invent a second CAN stack. Bus/adapter
config mirrors `p.cubemars_*` (`interface` e.g. `gs_usb`, `channel`/bitrate, `cubemars_j1_id..j7_id`,
`cubemars_speed_erpm`, `cubemars_accel_erpm_s2`, directions). The bridge **MUST** reuse
`pack_position_velocity`, `pack_set_origin`, `decode_feedback`, `error_name`, `make_can_id` rather
than re-pack `bytes`. The CiA 301 / device DS-402 dictionary is the frame authority.

Gate split (condition 7): **`arm` is reserved for motion only** — it means "I intend to move a
machine that pinches." Non-motion privileged commands use the single `confirm` key, so that `arm:true`
never becomes routine paperwork and stops carrying information.

| tool | cmd | class | exec | gate | maps to |
|---|---|---|---|---|---|
| `hw_status` | `hw_status` | read | tick | — | state snapshot (bus, node ids, speed, queue depth) |
| `hw_get_info` | `hw_get_info` | read | tick | — | `check_dependencies()` |
| `hw_analyze_frame` | `hw_analyze_frame` | pure | worker | — | `decode_feedback` (decode a hex frame) |
| `hw_configure` | `hw_configure` | write | tick | **`confirm`** | write `p.cubemars_*` (interface, ids, speed…) — moves nothing |
| `hw_check` | `hw_check` | hw | worker | — | `bpy.ops.pickik.cubemars_check_driver` |
| `hw_install` | `hw_install` | hw | worker | **`confirm`** | `cubemars_install_dependencies` (pip; see §9.5 — privileged, no `arm`) |
| `hw_read_telemetry` | `hw_read_telemetry` | hw | worker | — | `read_telemetry(seconds)` → per-joint pos/vel/current/temp |
| `hw_send_frame` | `hw_send_frame` | hw | worker | **`arm`** | raw CAN-TX — advanced (§9.5) |
| `hw_motors_move` | `hw_motors_move` | hw | worker | **`arm`** | `stream_to_targets` (one-shot) |
| `hw_motors_set_zero` | `hw_motors_set_zero` | hw | worker | **`arm`** | `set_origin` (re-zero encoders) |
| `hw_live_start` | `hw_live_start` | hw | worker | **`arm`** | `start_live_streaming` |
| `hw_live_update` | `hw_live_update` | hw | worker | **`arm`** | `update_live_targets` |
| `hw_live_stop` | `hw_live_stop` | hw | worker | — | stop live streaming (safe, allowed any time) |
| **`hw_motors_stop`** | `hw_motors_stop` | **priority** | **receiver** | **—** | `driver.stop()` — **e-stop, exempt from every gate & lock** (§5.3, §7) |
| `hw_disconnect` | `hw_disconnect` | hw | worker | **`confirm`** | `driver.disconnect()` — closes bus, moves nothing |

`hw_motors_stop` and `hw_live_stop` are deliberately open, for all users, always — the only two that
reverse motion.

**`hw_motors_move_to` is dropped from v1 (decision 8).** It would bundle a plan and an irreversible
physical action in one call, so the `q` that gets commanded never appears in the transcript a human
reviews — and on a redundant 7-DOF arm a valid-but-elbow-flipped solution is a collision. If Phase 3
needs the ergonomic, it returns in a verification-preserving form: `solve_ik(dry_run)` returns a
**`plan_id`** bound to the solved `q` *and* the seed it assumed; `hw_motors_move_to(plan_id)` accepts
only that handle, and the bridge rejects it `E_STATE` if the live `q` no longer matches the assumed
seed. One round trip, plan explicit, stale plans cannot fire.

### 6.5 MCP server surface (`pick_ik_mcp/mcp_server.py`)
`mcp>=1.26`, `Server`/`FastMCP`, **stdio** (the agent spawns it, e.g. `uvx pick_ik_mcp`). A thin proxy:
one persistent socket to the bridge, request→response, reconnect + per-command timeout (memetic ≈
`max_time`+2 s). Exposes **tools** (§6) and **resources** (§10). It **MUST** fail loud and clear: a
refused bridge connection, a closed gate, an `E_RANGE`/`E_TIMEOUT` — each becomes an MCP tool error the
agent can read, never a silent `null`. The server **SHOULD** list `hw_motors_stop` first and the
`plan_trajectory` prompt **SHOULD** instruct reaching for it first on any anomaly.

---

## 7. Safety model for the physical arm (taken seriously)

The bridge can move a 7-DOF desktop arm that will pinch. The agent is untrusted-by-default; motion is
privileged. Defence in depth, four layers + a fail-safe:

1. **Master enable (config).** `hardware.enabled` — default **`false`**. If false, every gate=`arm`
   command returns `E_ACCES` ("hardware gate closed") regardless of args. Reads (`hw_status`,
   `hw_read_telemetry`, `hw_analyze_frame`) still work, so the agent can look before it is allowed to
   touch. Mutually exclusive with `insecure_no_auth` (§4.3).
2. **Per-call gate — the keys do not lie (condition 7).**
   - `gate = arm` (**motion only**): **both** `"arm": true` **and** the exact
     `"confirm": "I UNDERSTAND THIS MOVES THE PHYSICAL ARM"` (case-insensitive) — a deliberate two-key:
     intent structurally *and* semantically. Missing either ⇒ `E_ACCES` with the reason in `message`.
   - `gate = confirm` (**non-motion privileged**: `hw_configure`, `hw_install`, `hw_disconnect`,
     `delete_rig`): the single `confirm` token only — no `arm`, because nothing pinches. Keeping `arm`
     reserved is what preserves its meaning.
3. **Joint-limit enforcement before TX.** No frame goes on the bus until every commanded
   `positions_deg[i]` lies within the Design B limits, `±180 / ±119.7455 / … °` (`±π / ±2.09 / …` rad).
   On violation the whole command is rejected `E_RANGE` naming `index`, value, and bound — **and nothing
   is sent** (all-or-nothing; a partially-applied pose is the dangerous case). Limits come from
   `get_robot_info`, never hard-coded twice.
4. **Rate & accel clamps.** `hw_motors_move`/`hw_live_start` **SHOULD** clamp `speed_erpm`/`accel` to
   `hardware.max_speed_erpm`; live mode **MUST** carry a `heartbeat_timeout_ms` that stops the stream if
   the agent stops sending `hw_live_update`.
5. **Fail-safe on disconnect (condition 1 + review).** On TCP drop **or** `unregister()`, the bridge
   **MUST** set `abort_flag` and call `driver.stop()` whenever *any* motion is outstanding — that is
   `hw_live_*` streaming **and** an in-flight one-shot `hw_motors_move`/`hw_motors_set_zero`, not
   streaming alone. A dead agent must never leave the arm chasing a stale target.

The e-stop is sacred: **`hw_motors_stop` is exempt from every gate, the busy guard, and the task lock**
(it *is* the gate), and is dispatched on the receiver lane so it can never be queued behind, or
refused by, the motion it stops (§5.3). It only ever *removes* motion, so it is available even when
`hardware.enabled=false` and with no `arm`/`confirm`.

---

## 8. Registration, lifecycle & UI

- `register()` **MUST** keep working under `bpy.app.background` (via the `_in_headless()` helper,
  §2.3): it registers classes but **MUST
  NOT** bind a socket in background (§2.3). Start is explicit.
- `AddonPreferences` (`Addons > PickIK arm7 (native C ABI)`): `enable_mcp_bridge` (def `False`),
  `mcp_port` (def `9876`, `0`=ephemeral), `mcp_auth_token` (def `""` → auto-generated, §4.3),
  `mcp_start_on_load` (def `False`), `mcp_hardware_enabled` (def `False`).
  The implemented set adds `mcp_export_root` and `mcp_runtime_file` — the §9.5 sandbox and the §4.1
  discovery record are otherwise not reachable from the UI — and `mcp_insecure_no_auth`, because
  §4.3's escape hatch would otherwise be unreachable at all; starting refuses it together with
  `mcp_hardware_enabled`, on the add-on side as well as inside the bridge.
  `_mcp_prefs()` **MUST** tolerate the block being absent rather than assume it: under
  `--factory-startup` — which is how the headless gates and any hand-driven `register()` arrive —
  Blender hands out no add-on preference block at all (measured on 3.4.1 and 4.5.3), and an add-on that
  treats its own preferences as certain fails to load in exactly the sessions that verify it.
- **N-Panel** section "MCP bridge" on `PICKIK_PT_main`: a `Start`/`Stop` toggle, the port, a status
  readout (`listening on 127.0.0.1:<bound> · client: yes/no · last cmd …`), and the hardware-gate
  state. Draws using only icon enums present on this Blender (the gate-9 lesson: whitelist against the
  live icon enum, `BLANK` fallback).
- **Single client (§4.1):** the bridge tracks its one session; a second `hello` ⇒ `E_ACCES`. The
  disconnect fail-safe (§7.5) keys off this one session's liveness.
- Teardown: `unregister()` **MUST** `bpy.app.timers.unregister(...)` the tick, close the listening
  socket, set `abort_flag`, and `join(timeout)` every worker/receiver thread — no detached thread may
  outlive `unregister()`. Leftovers on shutdown are not acceptable.

---

## 9. Security

- **Bind loopback only** (`127.0.0.1`); reject a request to bind a routable interface. Localhost is
  not a privilege boundary on a shared host, hence mandatory auth (§4.3), not an option.
- **No `eval`/`exec`/`getattr-by-name`** on anything off the wire. Dispatch is a closed allow-list
  (§4.4); `cmd` is a key, never a resolved identifier. The single most important rule.
- **Argument validation** on the server (types, arity, enum members, `additionalProperties:false`) and
  re-checked on the bridge (defence in depth, they are different processes). Sizes bounded (`q`=7,
  frames=16 doubles, CAN `dlc≤8`); a mismatch is `E_INVAL`, never a buffer overrun.
- **§9.4 Secrets.** token via `secrets.token_urlsafe(32)`, `hmac.compare_digest`, never logged (§4.3).
  Runtime file `0600`. `insecure_no_auth` logs loudly and is mutually exclusive with `hardware.enabled`.
- **§9.5 Filesystem & privileged sinks (condition 5).** An untrusted agent must not be handed an
  arbitrary-file-write primitive. `export_urdf` is sandboxed: it resolves `directory` against a
  configured `export_root`, and **MUST** reject `..`, absolute paths outside the root, and symlinked
  paths that escape it (`os.path.realpath` + a prefix check against the resolved root). Out-of-root ⇒
  `E_ACCES`. Same treatment for anything else that writes to disk. `hw_install` (pip → network fetch →
  code execution **inside Blender's interpreter**) is a privileged sink too: it is `confirm`-gated, and
  **SHOULD** be off by default (`hardware.allow_install=false`), documenting that it runs third-party
  code. `hw_send_frame` (raw CAN-TX) is likewise `arm`-gated and labelled advanced.

---

## 10. MCP resources & prompts

- **Tools** (`tools/list`, `tools/call`): the §6 catalogue.
- **Resources** (`resources/*`), scheme `pickik://` — **only the genuinely static ones (condition —
  resources are the wrong home for live data, clients cache them):**
  - `pickik://spec/arm7` → the Design B kinematic spec (`arm7-kinematic-spec.md`, the source of truth).
  - `pickik://limits` → joint limits (from `get_robot_info`).
  `get_state`/`pickik_status` are the **authoritative path** for live state; `pickik://state` **MAY**
  exist only as a convenience mirror and clients are warned not to rely on it for actuation. Any push
  channel, if added in Phase 2+, **MUST** be subscription-based and coalesced to ≤ 1 notification/s (at
  50 Hz continuous, live-state data would otherwise flood the agent's context with data it cannot act on
  at that rate — and MCP notifications are advisory/notify-only, so the agent has to read anyway).
- **Prompts** (`prompts/get`):
  - `plan_ik_solution` — solver choice & recovery guidance (ccd/gradient near, memetic global/recover);
    the position-only-vs-full-pose convention; `md_weight` as the anti-"arm jumps to a random pose" tool.
  - `plan_trajectory` — current `q` → target safely: `validate_pose` → `solve_ik(dry_run)` → verify →
    `execute` → optional hw move under the §7 gate.

---

## 11. Testing & acceptance (the §3.0c protocol, extended)

`test_mcp_bridge.py`, headless (`blender --background --python test_mcp_bridge.py`), reusing the
`test_acceptance.py` fixtures + the in-process client pattern from `test_api`. Gates:

- **14 bridge round-trip** — start in background (inline executor), loopback socket, complete the
  handshake (**pin the add-on's `mcp_protocol`, not the vendored copy**), `get_state`, `validate_pose`
  on the nine §5 anchors, `solve_ik(dry_run=true)` on targets A (200/100/300 mm, memetic) and B
  (300/150/300 mm, gradient); assert agreement with the C-ABI FK within a micron and `error_pos < 1 mm`.
- **15 allow-list / RCE** — an unknown `cmd`, and a `cmd` carrying a `__import__`/`eval` payload ⇒
  `E_INVAL`/`E_PROTO`; assert nothing executed (the allow-list holds).
- **16 main-thread budget (fixed)** — **instrument the tick duration directly**: time the drain callback
  from entry to exit (and the sum of `write`/`read` handler time within it), not the end-to-end socket
  round-trip (which includes tick latency and would not measure the stall). Assert the per-tick drain
  stays under the 4 ms budget for `ccd`/`gradient`; assert `memetic` is measured off-thread.
- **19 lifecycle & the panel** — `unregister()` leaves no bridge singleton, no bridge thread, and a
  **released port** (a later `register()` must be able to bind it again), and `register()` works again
  afterwards so a reload cycle is clean. The panel's MCP section draws in every branch — running / not
  running, preferences present / absent (the latter needs a stand-in block, the headless session has
  none) — with every icon it asks for validated against the **live** icon enum (the gate-9 lesson), and
  it never draws the token, whatever the preferences hold.
- **17 hardware gate** — `hardware.enabled=false` ⇒ every `arm` command `E_ACCES`; enabled but missing
  `arm`/`confirm` ⇒ `E_ACCES`; a limit-exceeding `positions_deg` ⇒ `E_RANGE` **and** zero frames on the
  bus; `hw_motors_stop` succeeds under all closed-gate conditions (the e-stop is always reachable).
- **18 e-stop priority & fail-safe (fixed)** — with an `hw_motors_move`/`hw_live_start` in flight, an
  `hw_motors_stop` is served **without** `E_BUSY` and **ahead of** the queued motion; assert `abort_flag`
  rises and the worker ceases issuing frames. Drop the socket mid-motion (both a one-shot **and** a
  live stream) ⇒ assert `driver.stop()` was called. Assert `insecure_no_auth`+`hardware.enabled` together
  refuse to bring the hardware lane up (§4.3).

Existing gates 1..13 **MUST** stay green; this layer must not regress them (byte-identical solvers,
the same `pick_ik_c.dll`, the same operators).

**The main-thread stall gate was re-shaped, and the reason is on purpose (do not "tidy" it back).**
It used to assert `p90 < 4 ms` on the synchronous solvers. What that gate exists to catch is
main-thread work where the tick must not block — an inline `memetic`, or scene work inside a `read`
handler — and that failure signature is **25-500 ms**. It is not the difference between 3.75 ms and
5.84 ms: five rounds in one process, one build, identical bytes, measured p90 2.81 / 3.48 / 3.75 /
5.84 / 4.67 ms and worst 3.25 / 4.85 / 4.22 / 10.42 / 7.56 ms. Two of those five rounds would have
failed a 4 ms gate and three would have passed it, so such a threshold is a coin flip and not a
regression test. A control run in the same process settles it: 120-360 cheap `is_valid` crossings
through the same DLL at **p90 0.003-0.011 ms**, no excursion whatsoever — the spread above is not the
machine being demonstrably busy either. The gate is therefore three measurements, where it was one:

| part | asserts | why it can be trusted |
|---|---|---|
| ceiling | solve p90 < **25 ms** | catches the actual failure mode with no ambiguity; cannot flake |
| regression | median < stored baseline × **2.0** | relative to the build's own record, so a machine that is simply slower does not read as a code change |
| control | 120 `is_valid` crossings, reported always | this run's noise floor; if it says the box is busy, the median verdict is reported and not failed |

The baseline lives in `test_acceptance_baseline.json`, keyed `"<blender>|<python>"` — one record per
build, so 3.4.1 and 4.5.3 are normalised against themselves rather than against each other. Refresh
deliberately with `PICKIK_UPDATE_BASELINE=1`; never by accepting a red gate. The 4 ms figure was a
UI-smoothness budget and remains the number to quote about frame pacing (see §5's tick budget); what it
was never entitled to be is a regression threshold. Anyone tempted to restore it should read this
paragraph first — a limit tuned to catch a 25 ms catastrophe will not also catch a 2 ms breeze, and
pretending otherwise buys a flaky suite and an afternoon of chasing it.

---

## 12. Phasing

- **Phase 1 (MVP, this contract)** — `pick_ik_mcp` scaffold; `mcp_protocol.py`, `mcp_bridge.py`
  (including the **priority lane + `abort_flag` + drain-to-budget** — these are Phase 1 plumbing even
  though the e-stop tool itself is exposed in Phase 3, because retrofitting a priority path onto a bare
  FIFO is the painful case), `mcp_server.py`, `mcp_client.py` (runtime-file reader), the observe + rig +
  IK/FK groups, registration + prefs + panel, config (`export_root`, auth, port), and gates 14–16. No
  hardware. Fully headless-testable. *Safe by default.*
  **Status (measured, not asserted):** the add-on side is in and green — `test_mcp_protocol.py` 20/20 on
  the system Python, `test_mcp_bridge.py` 77/77 headless on **4.5.3 LTS (py 3.11.11)** and **3.4.1
  (py 3.10.8)**, existing gates re-run 15/15, drain-to-budget stable at `[4,3,4,1]`/`[4,4,4]` jobs per tick
  with the slowest tick ≈4.6 ms against the 4 ms budget, a 58 ms memetic solve measured off-thread with
  the applied pose equal to the solved pose to 0.00e+00 rad. Two findings folded back above: §4.6 (a close
  must half-close, or refusals are lost) and §5.5 (the budget must be measured on `perf_counter`). A third,
  operator-facing: the `0600` intent of §4.1 is **advisory on Windows** — the CRT reports `0666` for a file
  created with `0o600`, so the bridge asserts containment instead (the runtime file must sit inside the
  operator's own home, and it warns loudly to stderr if it does not). The MCP-server side of Phase 1
  (`pick_ik_mcp/`: vendored protocol, `mcp_server.py`, `mcp_client.py`, README, requirements) is what is
  left of this phase.
- **Phase 2** — `set_continuous`, manual-FK niceties, resources & prompts, options-panel parity for the
  solver weights; optional subscription push (coalesced ≤1/s).
- **Phase 3** — `mcp_handlers_hw.py` (the CubeMars CAN group) behind the §7 split gate, gates 17–18,
  the fail-safe on disconnect, telemetry decoding via `decode_feedback`, and `hw_motors_move_to` via
  `plan_id` (decision 8) if the ergonomic is wanted.

---

## 13. Config (`mcp_config.example.json`)
```json
{
  "protocol": "pickik-bridge/1",
  "host": "127.0.0.1",
  "port": 9876,
  "runtime_file": "~/.pickik/bridge.json",
  "auth": { "token": "", "insecure_no_auth": false, "max_handshake_attempts": 5 },
  "bridge": { "idle_timeout_s": 300, "tick_interval_s": 0.05,
              "tick_budget_ms": 4, "request_timeout_ms": 5000 },
  "export_root": "~/pickik/export",
  "hardware": { "enabled": false, "allow_install": false, "max_speed_erpm": 0.0 }
}
```
(`hardware.max_speed_erpm: 0.0` = clamp all motion to zero until the operator sets a real value.)

---

## 14. Resolved decisions & open questions

**Resolved (2026-09-06):** transport TCP+timer-queue; language Python + `mcp` SDK; hardware exposed
behind the §7 split gate; plan-first; **e-stop on a priority lane with `abort_flag`**; `E_TIMEOUT` =
outcome-unknown with `get_state` recovery; `dry_run`+`seed_q` is a concurrent `pure` command; drain to
budget; `export_urdf` path-sandboxed to `export_root`; single authenticated client; `arm` reserved for
motion.

**Resolved — the five (phase-corrected):**
1. **Port** — keep 9876, never hard-code it; bind + publish the `0600` runtime file; fail loud on a
   taken configured port, never fall back silently. *(Phase 1 artifact.)*
2. **Auth** — secret required, auto-generated (`secrets.token_urlsafe(32)`), `0600` runtime file;
   `insecure_no_auth` logs loudly and is mutually exclusive with `hardware.enabled`. *(Phase 1 artifact.)*
3. **`hw_motors_move_to`** — dropped from v1; Phase 3 via `plan_id` bound to `q`+seed, stale ⇒ `E_STATE`.
   *(Phase 3.)*
4. **Live state** — snapshot-on-read `get_state`/`pickik_status` tools are authoritative; static
   `pickik://spec/arm7`,`pickik://limits` are the resources; `pickik://state` a warned-about mirror;
   push (if ever) subscription + coalesced ≤1/s. *(Phase 1 for the tools; Phase 2+ for push.)*
5. **Home** — sibling repo `pick_ik_mcp`; vendor `mcp_protocol.py` with a `proto_rev` drift guard.
   *(Phase 1 artifact — decide the import path now; gate 14 pins it.)*

**Open (none blocking for Phase 1):**
- A. `plan_id` lifetime & store for the Phase 3 `hw_motors_move_to` — in-memory ring vs the runtime file.
- B. Whether Phase 2 adds a coalesced push channel at all, or stays snapshot-on-read forever.

---
*End of plan (v2). Contract locked for Phase 1 — implementation proceeds to §12 Phase 1.*
