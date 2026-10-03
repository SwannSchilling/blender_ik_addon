# PickIK trajectory playback — living reference

> **This file is the anchor for the trajectory feature**: what is implemented
> and how, what has been verified to work, what does NOT work / is deferred,
> and what every playback parameter actually does.
>
> **Maintenance rule**: change this file in the SAME commit as any change to
> the capture → pack → stream pipeline or to a trajectory parameter. Append to
> the changelog (§11) every time. If a claim in here is wrong, the code is
> wrong or this file is — fix whichever is untrue, do not leave both.

* **Last updated:** 2026-10-03
* **Build tag:** `playback-pacing-fix-2024-09h` (`__init__.py: BUILD_TAG`)
* **Relevant commits:** `bc35a13` (Play file button), `2c5bab8` (dense capture
  + honest pacing), docs commit that introduced this file.
* **Hardware status of the current build:** code-verified headless only (see
  §2/§3) — the dense capture and the new pacing have **not yet been measured
  on the real arm**.

---

## 1. The pipeline as implemented

Three entry points share ONE capture → pack → stream path
(`__init__.py` → `trajectory.py` → `cubemars_driver.py`):

| Entry point | Operator | Capture rate | Output | Deterministic? |
|---|---|---|---|---|
| **Play smooth** | `pickik.play_trajectory` | `traj_fps` (panel, default 60) | straight to motors | No — re-solves every press (by design, see §9) |
| **Export** | `pickik.export_trajectory` | dialog `fps` field (default 60) | `pickik_trajectory.json` | Yes — file is frozen numbers |
| **Play file** | `pickik.play_trajectory_file` | the file's own `dt_s` | straight to motors | Yes — no solver runs at all |

### Step 1 — Author
`Add Keyframe` (`pickik.frame_key`) keys the IK target empty's
`rig.target.location` at the playback head (plus the mm panel fields, which
exist only for panel continuity). Blender's F-curves hold the motion.

### Step 2 — Capture (`_sample_track(context, dt_s)`)
Samples the keyed motion on a **uniform grid** from the first to the last
keyframed frame. The grid step is adjusted slightly so the LAST sample lands
exactly on the final key — the stream then paces at ONE true `dt` and the
duration is exactly the authored span.

* Target position per grid frame comes from the **keyed F-curves**
  (`fcurve.evaluate`, sub-frame exact) — the captured path IS Blender's own
  interpolation, i.e. what the scrubber shows. Fallback: stepping
  `frame_set` when the target is not keyed as F-curves.
* Each grid target is **IK-solved** with continuity seeding
  (`rig.last_q`), so the joint curve is what the arm does when driven there.
* Times use the scene's real clock `render.fps / render.fps_base`
  (`_effective_fps`) — fractional frame rates replay at the right speed.
* Returns `(pts, kf_times)`: dense `[(seconds, q_deg)]` + authored key times.

> **History:** until `2c5bab8` capture sampled ONLY the keyframed frames and a
> planner (`plan_s_curve_waypoints`, smootherstep per segment) re-invented the
> curve between them — zero velocity at every keyframe (stop-and-go) and a
> different shape than the scrubber. That re-plan step is REMOVED from all
> capture paths; `plan_s_curve*` remains in `trajectory.py` as unused legacy
> (old JSON files still play fine — the DTO format never changed).

### Step 3 — Pack (`trajectory.pack_samples(pts, kf_times)`)
Produces the replay DTO (contract in §7): per-sample positions + velocities
(central difference), `dt_s`, and `keyframe_idx` (nearest sample of every
authored key) for the CLI verifier.

### Step 4 — Stream (`cubemars_driver.stream_trajectory` → `_traj_worker`)
One Mode-6 (position+velocity) CAN frame per active motor per sample.

* **Pacing:** one deadline per sample on `time.monotonic()`. Falling behind
  **skips** the samples that can no longer be sent on their slot (counted and
  reported) instead of bursting them back-to-back. 1 ms Windows scheduler
  ticks are requested for the duration of the stream (best-effort).
* **Velocity slot:** constant per-joint ceiling (default) or the experimental
  true-velocity feedforward (`traj_vel_feed`, see §4).
* **Ramp start** (default ON): eases from the measured pose onto sample 0
  before the scripted path; otherwise a 150 ms lead-in re-enables the drives.
* **Tracking-error watchdog:** every tick compares commanded vs. measured
  (fresh feedback register) and keeps the worst error + its sample index.

### Step 5 — Tail
Hold-to-arrival: the final position is re-fed until measured is within
**3.0°** of commanded for **0.30 s** (timeout **8 s**) — one shared arrival
definition (`position_within_tolerance`) for the worker, the GUI and the CLI
verifier. Then `traj_playback_done` goes True and **Hold last frame**
(default ON) keeps feeding the pose until Stop. The end-of-run status reports
frames, duration, skipped samples and peak tracking error.

## 2. What works (and how we know)

Verified **headless** on Blender 3.4 and 4.5 (`tests/` scripts, §11). No
on-arm claim is made without an explicit note.

| # | Claim | Evidence | On-arm? |
|---|---|---|---|
| W1 | Exported/played motion is the timeline's own curve (shape AND timing) | `tests/smoke_dense_capture.py` 15/15: uniform grid, exact ends, evaluator plumbing identical to `frame_set` evaluation | **pending bench check** |
| W2 | Duration equals the authored span exactly | same smoke: `dt_s*(n-1)` == keyframe span | yes (format-level) |
| W3 | `keyframe_idx` marks every authored key | smoke + `tests/test_trajectory.py` 10/10 | yes |
| W4 | Play file replays a baked JSON deterministically — no solver, no scrub history, no drift | `tests/smoke_play_traj_file.py` 12/12; loader round-trip tests | **pending bench check** |
| W5 | Corrupt/short/missing trajectory files are refused with one clear message before anything streams | loader tests (short rows, zero `dt_s`, missing file) | n/a |
| W6 | Both Play buttons share ONE knob resolver and ONE drive-selection rule (live handoff / refuse while playing / fresh) | shared `_traj_stream_settings` + `_traj_start_stream`; both smokes | pending |
| W7 | Pacing is wall-clock honest: monotonic deadlines, skip-not-burst, skips counted and reported | `_traj_worker` (code-reviewed + suite green; no burst path remains) | **pending bench check** |
| W8 | 1 ms scheduler ticks during streaming (Windows), restored after | `_win_timer_begin/_end`, balanced in `finally` | pending |
| W9 | Mid-path tracking error is measured every tick and reported at the end (`traj_max_error`, `traj_skipped`) | worker + accessors | pending |
| W10 | Registration, panel and MCP bridge are unbroken by all of the above | `tests/test_mcp_bridge.py` 78/78; smokes on 3.4 + 4.5 | n/a |
| W11 | Old (S-curve-era) exported JSONs still load and play | DTO format unchanged; loader passes extras through | yes |

## 3. What does NOT work / limitations / deferred

| # | Issue | Status |
|---|---|---|
| N1 | **No on-arm validation yet** of dense capture + new pacing + tracking-error report. Everything above is headless-verified. | OPEN — needs one bench session before trusting it for a shoot |
| N2 | **Live scrubbing drift** ("links wander"): redundant 7-DOF arm + a local solver seeded from the previous pose wanders the self-motion manifold. Root cause analysis in §9. | BY DESIGN — mitigated by Export / Play file (deterministic), or re-home before Play smooth |
| N3 | **Play smooth re-solves on every press** — it bakes nothing; the first solve is seeded from `rig.last_q`, so it can inherit a little scrub history. | ACCEPTED (per project decision 2026-10-03) |
| N4 | **Time-scaling to physical limits** (stretch a too-fast path so it fits the drive's real speed/accel): there is no ERPM↔deg/s gearing anywhere (`_vel_degs_to_erpm` returns a constant ceiling), so there is no physical per-joint limit to scale against. | DEFERRED to the parameters discussion (§10, P6) |
| N5 | Tracking error is **report-only** — no auto-pause/abort on excess error. | DEFERRED (small follow-up; data already collected) |
| N6 | No live progress/ETA in the panel during playback (status line only; the CLI player has the richer report). | DEFERRED (UX follow-up) |
| N7 | `send_hz` API parameter is vestigial: only used as `dt` fallback when a dict has no `dt_s`. The old "safety cap" doc claim was removed in the docs commit. | CLEANUP CANDIDATE (§10, P3) |
| N8 | Two sample-rate knobs exist: `traj_fps` (Play smooth) and the Export dialog `fps` field. Easy to confuse. | CLEANUP CANDIDATE (§10, P1) |
| N9 | Velocity feedforward measured **neutral-to-slightly-worse** on this accel-limited rig (A/B on the real arm, 2026-09); the constant ceiling is the validated default. | EXPERIMENTAL — keep off unless exploring a lighter rig |

## 4. Trajectory panel parameters — current truth

The "Trajectory" box (`PickIKProps`). Defaults read from the class definition
(`__init__.py`). **Status** is the verdict of the 2026-10-03 review: `ACTIVE`
(used and needed), `EXPERIMENTAL` (off by default, measured neutral/worse),
`CANDIDATE` (see §10).

| Label | Property | Type / range | Default | Consumed by | Status |
|---|---|---|---|---|---|
| Trajectory sample FPS | `traj_fps` | int 5…500 | 60 | **Play smooth** capture rate (`dt = 1/fps`). NOT Export — Export has its own dialog field (N8). Was completely dead before `2c5bab8`. | ACTIVE · merge candidate P1 |
| Exported file | `traj_file_path` | FILE_PATH | "" (auto) | **Play file** target. Set by Export and by Play file's file browser. New in `bc35a13`. | ACTIVE |
| Play speed cap (ERPM) | `traj_move_erpm` | float 100…20000 | 2000 | Mode-6 velocity CEILING while streaming (both Play buttons). Stream shaping only — the position field carries the exact path, so raising it never moves a joint elsewhere. If the high-ratio base joint lags, raise the per-joint table instead. | ACTIVE |
| Play speed per joint | `traj_move_erpm_list` | 7 CSV | `4000,2000,2000,2000,2000,2000,2000` (`_TUNED_MOVE_ERPM`, this rig's measured profile) | Per-joint ceilings. Blank = measured profile; one number = force all 7; `0` for a joint = fall back to the cap. Mirrors CLI `--move-erpm`. | ACTIVE |
| Play accel ceiling (ERPM/s²) | `traj_accel` | float 200…40000 | 10000 (`_TUNED_ACCEL_ERPM_S2`) | Mode-6 acceleration ceiling per packet. **The dominant lever** on this rig: the base joint (J1/AK80-9) is acceleration-limited — at 2000 it lagged ~1.8 s and only landed the last pose; 10000 tracks in phase. | ACTIVE |
| Hold last frame | `traj_hold_last` | bool | ON | After arrival at the FINAL pose, keep feeding it so the drives stay enabled and HOLD (Mode-6 drops a pose the instant the stream stops). Untick = release the moment playback ends. | ACTIVE |
| Ramp start | `traj_ramp_start` | bool | ON | Ease from wherever the joints are onto the first sample before the scripted path (no snap to sample 0). | ACTIVE |
| Velocity feedforward | `traj_vel_feed` | bool | **OFF** | Command the TRUE per-sample velocity into the Mode-6 slot (peak-normalised per joint, floored to a creep) instead of the constant ceiling. A/B on this accel-limited rig: neutral-to-slightly-worse (the velocity slot is not the binding constraint). | EXPERIMENTAL (N9) |
| Velocity scale / joint | `traj_v_scale_list` | 7 CSV | "" (auto) | ERPM-per-(deg/s) overrides for the feedforward slot only. Blank = auto peak-normalise. | EXPERIMENTAL |
| Velocity floor / joint | `traj_v_floor_list` | 7 CSV | "" (auto 6 % of the joint's cap) | Minimum creep ERPM for a moving sample (eased start/apex/stop never stall) — feedforward only. | EXPERIMENTAL |

All `*_list` fields share one parser (`_opt_float_list`): drop blanks, `0` ⇒
auto/fallback, a single value ⇒ force on every joint, else per-joint; padded
to 7.

### Stream parameters with NO panel field (driver defaults)

| Parameter | Default | Meaning |
|---|---|---|
| arrival `tolerance_deg` | 3.0 | per-joint window for "reached the pose" (shared with the CLI verifier) |
| `stable_window_s` | 0.30 | the report must stay inside tolerance this long to count as ARRIVED |
| `arrival_timeout_s` | 8.0 | how long the hold tail waits for arrival |
| lead-in | 150 ms | drive re-enable burst before the path when Ramp start is off |
| ramp duration | 1.5 s | Ramp start easing duration |

## 5. Operator inputs

| Operator | Inputs | Notes |
|---|---|---|
| `pickik.export_trajectory` | `filepath` (FILE_PATH), `fps` (int 5…500, default 60) | both appear in the file dialog; `fps` = samples/second of the written file |
| `pickik.play_trajectory` | none | reads the Trajectory box; capture at `traj_fps` |
| `pickik.play_trajectory_file` | `filepath` (FILE_PATH) | file browser opens pre-pointed at `traj_file_path` (or `<blend dir>/pickik_trajectory.json`); streams at the file's `dt_s` |

## 6. Adjacent CubeMars fields (do not confuse with the play caps)

| Label | Property | Default | Note |
|---|---|---|---|
| CubeMars Actuators | `cubemars_enabled` | OFF | master switch for CAN |
| CAN interface / channel | `cubemars_interface` / `cubemars_channel` | `gs_usb` / 0 | python-can backend |
| J1…J7 CAN ID | `cubemars_j1_id` … `j7_id` | 0x68, 0x69, 0x5… | 0 = inactive joint |
| Max speed (ERPM) | `cubemars_speed_erpm` | 2000 | one-shot / live moves ONLY — not the play cap |
| Max accel (ERPM/s²) | `cubemars_accel_erpm_s2` | 2000 | one-shot / live moves ONLY — not `traj_accel` |
| Follow rate (deg/tick) | `cubemars_slew_deg_per_tick` | 1.0 | live-follow rate limiting |
| Live update | `cubemars_live` | OFF | 50 Hz pose-follow stream; Play hands it off cleanly when it is running |

## 7. The exported JSON — format contract

Written by `Export` / `standalone/export_trajectory.py` via
`trajectory.pack_samples`; read back by `trajectory.load_samples`
(validating) and `standalone/play_trajectory.py`. **The format has not
changed** — files from the S-curve era still play.

| Key | Type | Meaning |
|---|---|---|
| `dt_s` | float | seconds per sample (uniform). The stream's cadence — a file replayed with **Play file** runs at exactly this rate |
| `t_s` | [float] | per-sample time stamps (seconds) |
| `q_pos_deg` | [[float]×7] | per-sample joint angles, rig-space degrees, ALL 7 joints (the driver's active map picks which move) |
| `q_vel_deg_s` | [[float]×7] | per-sample joint velocity (central difference; endpoints 0). Optional on load — missing ⇒ zeros (constant-ceiling slot) |
| `n_samples`, `n_joints` | int | counts |
| `keyframe_idx` | [int] | nearest sample index of every authored key — the CLI verifier checks exactly there |

`load_samples` refuses (one clear message, nothing streams): unreadable /
non-JSON files, non-object roots, empty `q_pos_deg`, rows that are not 7
numbers, `dt_s ≤ 0`. Unknown extra keys pass through untouched.

## 8. CLI player mirror (`standalone/play_trajectory.py`)

Same driver, same DTO, same arrival definition — the GUI and CLI can be
tuned to match. Defaults:

| CLI flag | Default | Mirrors |
|---|---|---|
| `--move-erpm` | `4000,2000,…` (TUNED_MOVE_ERPM) | `traj_move_erpm_list` |
| `--accel` | 10000 (DEF_ACCEL_ERPM_S2) | `traj_accel` |
| `--tol-deg` | 3.0 | driver `tolerance_deg` (shared definition) |
| `--stable-window` | 0.30 s | `stable_window_s` |
| `--hold-timeout` | 8.0 s | `arrival_timeout_s` |
| `--keyframe-band` | 6 samples | how long past a key its closest approach is still judged |
| `--settle` | 0.12 s | feedback capture per keyframe |
| `--dry-run` | off | print the keyframes, touch nothing |

Extras the CLI has and the GUI does not (N6): per-keyframe want/actual/err
tables, PASS/FAIL verdict, exit codes. The GUI's **Play file** is the
in-Blender replacement for the plain `play` invocation.

## 9. Background: why live scrubbing drifts (analysis, kept)

Two facts collide when scrubbing with Live update ON:

1. the solver is seeded from the **previous pose** (`rig.last_q`) — a local
   descent converges to the nearest solution to wherever it already was;
2. the arm is **redundant** — 7 joints for a 6-DOF task ⇒ a 1-dimensional
   self-motion manifold: the same tip position is reachable by a continuum of
   elbow/wrist configurations.

So forward scrub and backward scrub walk different branches: the tool tip
tracks the target (that constraint holds) while the links "drift".
`md_weight` ("pull toward the seed") makes it worse the higher it is (left 0
by default, which already minimises it).

**Verdicts (updated 2026-10-03):**

* **Export → Play file = the canonical cure.** The file is frozen numbers; the
  player never runs the solver — no seed, no history, no drift. This is now a
  button in Blender (`bc35a13`), not just a CLI ritual.
* **Play smooth is not a bake** — it re-solves the keyframes on every press.
  It samples the KEYFRAMES (not the playhead), so scrubbing around before
  pressing Play does not wander the motion; the residual is confined to the
  first solve's seed (`rig.last_q` may carry scrub history). Accepted by
  project decision; turn Live update OFF or re-home to a known pose for a
  clean start.
* **Live scrubbing with Live update ON** is where the drift is actually seen;
  it is inherent to seeding a redundant local solver from history.

## 10. Parameter review — open proposals (working area)

For the pending parameters discussion. Nothing here is implemented; each row
is a proposal with a suggested verdict. Update the row (and §4) when decided.

| # | Proposal | Rationale | Suggested verdict |
|---|---|---|---|
| P1 | Merge the Export dialog `fps` into `traj_fps` (one capture-rate knob) | N8: two knobs doing one job; `traj_fps` was dead for months and nobody noticed | KEEP one knob |
| P2 | Group `traj_vel_feed` + scale + floor as one "experimental velocity profile" block (or drop scale/floor to CLI-only) | three fields for a feature that measured neutral-to-worse on this rig (N9) | KEEP the toggle, HIDE the two lists unless the toggle is on |
| P3 | Remove or rename `send_hz` in `stream_trajectory` (now only a `dt` fallback) | N7 | REMOVE from the public path |
| P4 | Split the panel visually: "Capture" (fps) vs "Stream shaping" (caps, hold, ramp) vs "Experimental" | users tune caps thinking they change the path; they do not | KEEP — presentation only |
| P5 | Fix stale docstrings claiming velocity feedforward is the default | contradicted the property default | **DONE** in the docs commit |
| P6 | Real per-joint vel/accel limits (deg/s, measured on the arm) → enables time-scaling of too-fast paths (N4) and an honest fit check at export time | without physical units every "limit" is a drive-side ERPM knob | MEASURE on the bench, then implement |
| P7 | Add tracking-error abort threshold (N5) and a progress readout (N6) | the data is already collected per tick | DO after the bench check (N1) |

## 11. Verifying & maintaining this document

### How to re-verify (run from the repo root; none of these touch hardware)

```sh
python blender_ik_addon/tests/test_trajectory.py                  # DTO: pack/load (any Python)
python blender_ik_addon/tests/verify_names.py blender_ik_addon/__init__.py blender_ik_addon/trajectory.py blender_ik_addon/cubemars_driver.py
blender --background --factory-startup --python blender_ik_addon/tests/smoke_play_traj_file.py   # Play file button + loader
blender --background --factory-startup --python blender_ik_addon/tests/smoke_dense_capture.py   # dense capture + export
blender --background --factory-startup --python blender_ik_addon/tests/test_mcp_bridge.py       # full regression (78 checks)
```

Expected: all PASS / `RESULT: OK` / `78 passed, 0 failed` on Blender **3.4**
and **4.5** (both installed addon copies must be synced after code changes —
see the changelog rule below).

### On-arm checks (bench session — closes N1)

1. Key a two-pose move; **Export**; **Play file** — arm should trace the
   scrubber preview exactly, same duration, no stop-and-go at the keyframe.
2. Watch the end status: `skipped 0 samples`, `peak tracking err … deg` — the
   error number is the new honest metric; record it in the changelog.
3. Repeat with Live update ON → hand-off plays cleanly without a snap.

### Maintenance rule (again)

Any change to capture/pack/stream or a parameter updates THIS file in the
same commit. Changelog:

| Date | Change | Commits |
|---|---|---|
| 2026-10-03 | **Play file** button (GUI replay of exported JSON, file-validated loader, remembered path); **dense capture** replaces keyframe-only + S-curve re-plan (motion == scrubber); pacing: monotonic deadlines + skip-not-burst + 1 ms Windows ticks; tracking-error + skip report; `traj_fps` repurposed as the Play-smooth capture rate; three stale docstrings corrected; tests: `test_trajectory.py`, `smoke_play_traj_file.py`, `smoke_dense_capture.py`. This document created as the living anchor (replaces the parameter notes + drift analysis supplied 2026-10-03). | `bc35a13`, `2c5bab8`, docs commit |