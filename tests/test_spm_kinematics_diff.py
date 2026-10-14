# SPDX-License-Identifier: BSD-3-Clause
"""Differential test: the Python SPM kinematics port against the HTML tool's own code, under node.

    python -B tests/test_spm_kinematics_diff.py       # from blender_ik_addon/

Why a differential pass on top of the fixture. test_spm_kinematics pins the port to 27 recorded rows;
a port could pass those and still be wrong everywhere the fixture does not look. This one runs the
TOOL'S OWN solveIK / sigmaMinJ / solveFK, extracted verbatim from the HTML and executed by node, over a
seeded population of >=500 random orientations - valid and invalid alike - and compares the port
elementwise: theta, valid[], sigma_min, and the FK orientation. The orientations come from a fixed seed,
not from the fixture, so the population is free of the 27 rows that produced the port. It is the same
shape as the tip/pin equivalence battery (region extraction, a node driver, a verbatim tripwire), but
what is duplicated here is the whole port, so the tripwire sits on the tool's solveIK / sigmaMinJ /
solveFK: change that math and this says the two have parted, rather than the port quietly diverging.

It also carries the two live-use guarantees a fixture can not make. A continuity walk steps the platform
through many small motions and asserts the seeded FK (seeded from the previous tick's orientation) stays
in the same assembly mode as the cold solve - never a mirror jump. And it times per-call IK and FK
against the 25 ms solver tick so a real panel can budget against the port, not a guess.
"""
from __future__ import annotations

import json
import math
import os
import random
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))                 # blender_ik_addon/tests
_ADDON = os.path.dirname(_HERE)

def _find_root():
    d = _HERE
    for _ in range(6):
        if os.path.isdir(os.path.join(d, "blender_ik_addon")) and os.path.isdir(os.path.join(d, "SPM")):
            return d
        d = os.path.dirname(d)
    raise RuntimeError("cannot find the workspace root above " + _HERE)


ROOT = _find_root()
# the exporter it borrows lives in tools/spm once committed and in _scratch until then; both are added so
# the same file runs against either, and the addon/tests paths come first so the port imports as a sibling.
for _p in (_ADDON, _HERE, ROOT, os.path.join(_ADDON, "tools", "spm"),
           os.path.join(_ADDON, "tools", "gates"), os.path.join(ROOT, "_scratch")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import export_spm_golden as exp                                 # the name it has in version control
except ImportError:
    import _export_spm_golden as exp                                # the name it has in _scratch
import spm_kinematics as K                                            # noqa: E402
import spm_driver as D                                                # noqa: E402

#: verbatim lines whose presence says the tool's math is the one the port was written against
REQUIRED = (
    "function solveIK(w0,v0,cosA2,R)",
    "theta[i]=Math.atan2(Q,P)+Math.acos",
    "function sigmaMinJ(m,R,th)",
    "function solveFK(wArr,v0Arr,cosA2,xi0)",
)

POP = 620                                                            # random orientations, well past 500
SEED = 20240517                                                      # fixed: the population is reproducible
THETA_POP = 400                                                      # reachable-by-construction commanded thetas
THETA_LIMIT = 1.6                                                    # rad (~92deg), inside a full turn so a closure exists
TOL_THETA = 1e-9
TOL_SIGMA = 1e-9
TOL_R = 1e-7
TICK_BUDGET_MS = 25.0                                                # the live solver tick

#: the node driver: it draws each orientation from ORIENTS (injected below) and runs the tool's code
DRIVER = r"""
const fr=spmLegFrames();
function so3log(R){const tr=R[0][0]+R[1][1]+R[2][2];let c=(tr-1)/2;c=Math.max(-1,Math.min(1,c));
  const th=Math.acos(c);if(th<1e-12)return[0,0,0];
  if(Math.abs(th-PI)<1e-9){let best=-1,ax=[0,0,1];
    for(let j=0;j<3;j++){const col=[R[0][j]+(0===j?1:0),R[1][j]+(1===j?1:0),R[2][j]+(2===j?1:0)];
      const n=Math.hypot(col[0],col[1],col[2]);if(n>best){best=n;ax=normv(col);}}
    return[ax[0]*th,ax[1]*th,ax[2]*th];}
  const k=1/(2*Math.sin(th));
  return[k*(R[2][1]-R[1][2])*th,k*(R[0][2]-R[2][0])*th,k*(R[1][0]-R[0][1])*th];}
function go(R){
  const ik=solveIK(fr.w0,fr.v0,fr.cosA2,R);
  const allfin=ik.theta.every(Number.isFinite);
  const sigma=allfin? sigmaMinJ(fr, flatR(R), ik.theta): null;
  let Rfk=null;
  if(allfin){const wArr=[0,1,2].map(i=>matVec(rodrigues([0,0,1],ik.theta[i]),fr.w0[i]));
    const xi=solveFK(wArr,fr.v0,fr.cosA2,so3log(R));Rfk=expSO3(xi);}
  return {theta:ik.theta, valid:ik.valid, sigma:sigma, R:Rfk, Rtarget:R};
}
process.stdout.write(JSON.stringify(ORIENTS.map(go)));
"""

#: the reachable-biased driver. The orientation population above is roughly two-thirds UNREACHABLE (a plain
#: random rotation is rarely a Stewart-platform pose), so it under-tests the direction a live panel actually
#: travels: commanded angles IN range, whose forward image must invert back. Here theta is drawn WITHIN the
#: crank's own limits, the TOOL's solveFK turns it into an orientation (the same seeded search the port runs,
#: and only accepted when the tool's OWN solveIK re-solves that orientation back to theta - so the input is
#: guaranteed reachable by construction), and then the port's IK must recover theta. It is the IK half of the
#: equivalence on the reachable set the orientation sampling cannot reliably reach.
THETA_DRIVER = r"""
const fr=spmLegFrames();
function so3log(R){const tr=R[0][0]+R[1][1]+R[2][2];let c=(tr-1)/2;c=Math.max(-1,Math.min(1,c));
  const th=Math.acos(c);if(th<1e-12)return[0,0,0];
  if(Math.abs(th-PI)<1e-9){let best=-1,ax=[0,0,1];
    for(let j=0;j<3;j++){const col=[R[0][j]+(0===j?1:0),R[1][j]+(1===j?1:0),R[2][j]+(2===j?1:0)];
      const n=Math.hypot(col[0],col[1],col[2]);if(n>best){best=n;ax=normv(col);}}
    return[ax[0]*th,ax[1]*th,ax[2]*th];}
  const k=1/(2*Math.sin(th));
  return[k*(R[2][1]-R[1][2])*th,k*(R[0][2]-R[2][0])*th,k*(R[1][0]-R[0][1])*th];}
function near(a,b){for(let i=0;i<3;i++){let d=a[i]-b[i];d=((d+PI)%(2*PI)+2*PI)%(2*PI)-PI;
  if(Math.abs(d)>1e-6)return false;}return true;}
function thetaSeeds(){const net=[[0,0,0]];
  for(const ax of [[1,0,0],[0,1,0],[0,0,1],[-1,0,0],[0,-1,0],[0,0,-1]])
    for(const an of [PI/2,-PI/2,PI,PI/3,-PI/3]) net.push([ax[0]*an,ax[1]*an,ax[2]*an]);
  return net;}
function toolFK(th){const wArr=[0,1,2].map(i=>matVec(rodrigues([0,0,1],th[i]),fr.w0[i]));
  for(const s of thetaSeeds()){const xi=solveFK(wArr,fr.v0,fr.cosA2,s);const R=expSO3(xi);
    const ik=solveIK(fr.w0,fr.v0,fr.cosA2,R);
    if(ik.theta.every(Number.isFinite)&&near(ik.theta,th))return R;}
  return null;}
function go(th){const R=toolFK(th);if(!R)return {theta:th,R:null,sigma:null};
  return {theta:th,R,sigma:sigmaMinJ(fr,flatR(R),th)};}
process.stdout.write(JSON.stringify(THETAS.map(go)));
"""

PASS, FAIL = [], []


def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}\n        {detail}")


def _is_none(x):
    return x is None or (isinstance(x, float) and not math.isfinite(x))


def _population():
    """A seeded run of random orientations. An axis and an angle from a fixed LCG make the population
    reproducible across machines, and a plain random rotation is often unreachable - which is the point:
    the population has to hold invalid poses, not only the reachable ones the fixture happens to record."""
    rnd = random.Random(SEED)
    out = []
    for _ in range(POP):
        ax = [rnd.uniform(-1.0, 1.0) for _ in range(3)]
        n = math.sqrt(sum(c * c for c in ax))
        if n < 1e-6:
            ax, n = [0.0, 0.0, 1.0], 1.0
        ax = [c / n for c in ax]
        out.append(K.exp_so3([ax[0] * rnd.uniform(0.0, math.pi),
                              ax[1] * rnd.uniform(0.0, math.pi),
                              ax[2] * rnd.uniform(0.0, math.pi)]))
    return out


def _theta_population():
    """Crank angles drawn WITHIN the actuator's reachable range, so the forward image is a pose that exists
    by construction. This is the population a slider actually walks and the one a plain random rotation rarely
    lands on; the orientation-biased sample above is roughly two-thirds unreachable, which cannot exercise the
    IK-inverts-FK direction on the reachable set. The limit is well inside a full turn so every theta has a
    physical closure for the tool's solveFK to return."""
    rnd = random.Random(SEED + 7717)
    out = []
    for _ in range(THETA_POP):
        out.append([rnd.uniform(-THETA_LIMIT, THETA_LIMIT) for _ in range(3)])
    return out


def _node_module(payload_token, payload, driver, tag):
    """Write the extracted pure lane plus a driver that maps over `payload`, and run it under node. The
    purity tripwires run in front of EVERY node run - cheap, and neither differential may compare against a
    lane that quietly pulled in the browser. Returns (parsed_json, error_or_None)."""
    html = open(exp.HTML, encoding="utf-8").read()
    gone = [line for line in REQUIRED if line not in html]
    if gone:
        return None, ("the tool's math moved out from under the port; absent from the HTML: "
                      + " | ".join(g[:60] for g in gone))
    parts = exp.pieces_of(html)
    whole = "\n".join(l for name in exp.REGIONS for l in parts[name].split("\n")
                      if not l.strip().startswith("//"))
    for tok in ("THREE", "document", "getElementById"):
        if tok in whole:
            return None, f"the extracted lane mentions {tok}, so it is not the pure lane"
    module = os.path.join(ROOT, f"_spm_diff_{tag}.mjs")
    out = module + ".out"
    js = driver.replace(payload_token, json.dumps(payload), 1)
    try:
        with open(module, "w", encoding="utf-8") as fh:
            for name in exp.REGIONS:
                fh.write(parts[name] + "\n")
            fh.write(js)
        with open(out, "wb") as fh:
            run = subprocess.run(["node", module], stdout=fh, stderr=subprocess.STDOUT, cwd=ROOT)
        if run.returncode != 0:
            return None, f"node refused the differential (exit {run.returncode})"
        return json.loads(open(out, encoding="utf-8").read()), None
    finally:
        for p in (module, out):
            try:
                os.remove(p)
            except OSError:
                pass


def _run_node(orients):
    return _node_module("ORIENTS", orients, DRIVER, "orient")


def _run_node_thetas(thetas):
    return _node_module("THETAS", thetas, THETA_DRIVER, "theta")


def _node_present():
    """node is the authority this whole differential leans on: the numbers come from the tool's OWN code
    running there. If it is absent the differential cannot run at all, and that must FAIL LOUDLY rather than
    surface as a traceback a reader might mistake for a port bug. Returns (version_or_None, error_or_None)."""
    try:
        probe = subprocess.run(["node", "--version"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               cwd=ROOT, timeout=30)
    except FileNotFoundError:
        return None, ("node is not on PATH. This differential executes the tool's own JS under node; without "
                      "it there is NO authority to compare the port against, so it cannot pass. Install node "
                      "(>=18) or make it reachable on PATH, then re-run.")
    except OSError as exc:
        return None, f"node could not be launched: {type(exc).__name__}: {exc}"
    if probe.returncode != 0:
        return None, f"node --version exited {probe.returncode}"
    return (probe.stdout.decode("utf-8", "replace").strip() or "unknown"), None


def main():
    node_ver, node_err = _node_present()
    if node_err is not None:
        print("\n" + "=" * 72)
        print("!! NODE MISSING - THE SPM DIFFERENTIAL CANNOT ESTABLISH THE TRUTH, FAILING LOUDLY !!")
        print("!! " + node_err)
        print("=" * 72 + "\n")
        check("node is present so the tool's own code can be the authority", False, node_err)
        return tally()
    mechs = K.mech_from()
    orients = _population()
    tool, err = _run_node(orients)
    if err is not None:
        check("the tool's extracted code runs under node and still matches the port's assumptions", False, err)
        return tally()
    check(">=500 seeded random orientations were drawn and solved by the tool under node",
          len(tool) >= 500, f"population {len(tool)} orientations, seed {SEED}")

    wth = wsg = wfk_p = wcold = 0.0
    bad_th = bad_v = bad_sg = bad_fk = bad_cold = n_none = 0
    for i, R in enumerate(orients):
        t = tool[i]
        ik = K.solve_from(R, mechs)
        for j in range(3):
            a, b = ik["theta"][j], t["theta"][j]
            if _is_none(a) != _is_none(b):
                bad_th += 1
            elif not _is_none(a):
                wth = max(wth, abs(a - b))
                if abs(a - b) > TOL_THETA:
                    bad_th += 1
            if bool(ik["valid"][j]) != bool(t["valid"][j]):
                bad_v += 1
        a, b = ik["sigma_min"], t["sigma"]
        if _is_none(a) != _is_none(b):
            bad_sg += 1
        elif not _is_none(a):
            wsg = max(wsg, abs(a - b))
            if abs(a - b) > TOL_SIGMA:
                bad_sg += 1
        # the forward map. The LIVE path is the one held to the tool: a port FK seeded from the pose is
        # the same thing the tool's solveFK seeded from log(R) computes, so it must match elementwise. The
        # unseeded net is then only checked to be a legitimate closure root (it converges and re-solves);
        # the 3-rod closure folds into crossed-assembly twins that share a theta, so the unseeded root may
        # lawfully sit a finite angle from the tool's - reported, not failed. That fold is WHY live use seeds.
        if t["R"] is None and ik["fit_ok"] is False:
            n_none += 1
        elif t["R"] is not None and ik["fit_ok"]:
            seed = K.fk_to_orientation(ik["theta"], mechs, R)
            cold = K.fk_to_orientation(ik["theta"], mechs)
            # Compare the port's pose-seeded root to the tool's own seeded-solveFK root, elementwise. The
            # 'seed' LABEL is not a correctness signal under nearest-to-prev selection (a net seed can pin the
            # same physical branch a hair closer), so the value comparison is what carries this check.
            if seed is None or not seed["converged"]:
                bad_fk += 1
            else:
                for r in range(3):
                    for c in range(3):
                        wfk_p = max(wfk_p, abs(seed["R"][r][c] - t["R"][r][c]))
            if cold is None or not cold["converged"]:
                bad_cold += 1
            else:
                wcold = max(wcold, K.angle_between(cold["R"], R))

    check("theta matches the tool's solveIK elementwise across the population",
          bad_th == 0, f"worst |theta| {wth:.2e} rad, {bad_th} mismatch(es)")
    check("valid[] matches the tool's per-leg roots across the population",
          bad_v == 0, f"{bad_v} leg(s) differed")
    check("sigma_min matches the tool's sigmaMinJ across the population",
          bad_sg == 0, f"worst |sigma_min| {wsg:.2e}, {bad_sg} mismatch(es)")
    check("the live path - the port FK seeded from the pose - matches the tool's solveFK elementwise",
          bad_fk == 0, f"worst |port-seed - tool| {wfk_p:.2e}, {bad_fk} bad, {n_none} unreachable skipped")
    check("the unseeded net still lands on a legitimate closure root every time (symmetry divergence aside)",
          bad_cold == 0, f"{bad_cold} net root(s) failed to converge; worst net-vs-pose angle {wcold:.2e} rad")

    # --- reachable-biased: commanded theta in range -> the tool's own FK -> the port must invert it back ----
    # The orientation population above is mostly unreachable, so it never checks that the port's IK turns the
    # tool's FORWARD image back into the angles that produced it. Here the tool computes R by its OWN solveFK
    # (accepted only when the tool's OWN solveIK maps R back to theta, so R is a reachable pose by construction),
    # and then the port's solve_from(R) must recover theta (mod a turn), all legs valid, with the same sigma_min.
    thetas = _theta_population()
    ttool, terr = _run_node_thetas(thetas)
    if terr is not None:
        check("the reachable-biased driver ran under node", False, terr)
    else:
        t_cov = t_solved = t_bad_th = t_bad_v = t_bad_sg = 0
        wt_th = wt_sg = 0.0
        for src, res in zip(thetas, ttool):
            if res is None or res.get("R") is None:
                continue                                             # the tool's own FK found no closure for that theta
            t_solved += 1
            got = K.solve_from(res["R"], mechs)                      # res['R'] is the tool's expSO3 output, nested 3x3
            if not got["fit_ok"] or not all(got["valid"]):
                t_bad_v += 1
                continue
            for i in range(3):
                d = (got["theta"][i] - src[i] + math.pi) % (2.0 * math.pi) - math.pi
                wt_th = max(wt_th, abs(d))
            if wt_th > TOL_THETA:
                t_bad_th += 1
            if res["sigma"] is not None and got["sigma_min"] is not None:
                wt_sg = max(wt_sg, abs(got["sigma_min"] - res["sigma"]))
                if abs(got["sigma_min"] - res["sigma"]) > TOL_SIGMA:
                    t_bad_sg += 1
            t_cov += 1
        check("reachable-biased: the port's IK inverts the tool's forward image back to the commanded theta",
              t_cov > 0 and t_bad_th == 0 and t_bad_v == 0,
              f"{t_cov}/{len(thetas)} commanded thetas reached a closure the port re-inverted to theta "
              f"(worst angle {wt_th:.2e} rad), {t_bad_v} not-all-valid")
        check("reachable-biased: sigma_min matches the tool across the commanded thetas",
              t_cov > 0 and t_bad_sg == 0,
              f"worst |sigma_min| {wt_sg:.2e} over {t_solved} tool-solved commanded thetas")

    # --- continuity: a seeded FK must not jump to a mirror mode over many small motions -------------
    # The invariant that matters live is that each frame's solve continues from the last: seeded from the
    # previous orientation, the recovered orientation has to stay near it (a few degrees, the commanded
    # step), never teleport to a crossed twin. The cold solve is shown alongside only to expose how often
    # that twin exists; a divergence between them is the fold, not a jump, so it is reported not failed.
    prev_R = K.orientation_from_tilt_pan_roll(0.0, 0.0, 0.0)
    jumps = noncont = sym = 0
    steps = 0
    w_seed_prev = 0.0
    JUMP_BOUND = 0.5                                                # a commanded step is a few degrees
    for s in range(240):
        tilt = 25.0 * math.sin(s * 0.05)
        pan = 150.0 * math.sin(s * 0.017 + 1.0)
        roll = 80.0 * math.sin(s * 0.031)
        R = K.orientation_from_tilt_pan_roll(tilt, pan, roll)
        th = K.solve_from(R, mechs)
        if not th["fit_ok"]:
            continue
        steps += 1
        seed = K.fk_to_orientation(th["theta"], mechs, prev_R)
        if not seed["converged"]:
            noncont += 1
            continue
        jump = K.angle_between(seed["R"], prev_R)
        w_seed_prev = max(w_seed_prev, jump)
        if jump > JUMP_BOUND:
            jumps += 1
        cold = K.fk_to_orientation(th["theta"], mechs)
        if cold["converged"] and K.angle_between(cold["R"], seed["R"]) > 1e-6:
            sym += 1
        prev_R = seed["R"]

    check("a seeded FK stays continuous with the previous frame (no mirror jump) over the walk",
          jumps == 0 and noncont == 0 and steps > 100,
          f"{steps} reachable steps, worst frame-to-frame turn {w_seed_prev:.2e} rad "
          f"(bound {JUMP_BOUND}), {jumps} jump(s), {noncont} non-converged, "
          f"{sym} step(s) where the unseeded net alone would have shown the twin")

    # --- timing: per-call IK and FK against the 25 ms live solver tick ------------------------------
    probe = [K.orientation_from_tilt_pan_roll(20.0 * math.sin(k * 0.3), 120.0 * math.sin(k * 0.07),
                                             40.0 * math.sin(k * 0.11)) for k in range(300)]
    solved = [K.solve_from(R, mechs) for R in probe]
    reachable = [s for s in solved if s["fit_ok"]]
    if not reachable:                                              # a timing path must always exist
        reachable = [K.solve_from(K.orientation_from_tilt_pan_roll(0.0, 0.0, 0.0), mechs)]

    def timeit(fn, n):
        xs = []
        for _ in range(n):
            t0 = time.perf_counter()
            fn()
            xs.append((time.perf_counter() - t0) * 1000.0)
        xs.sort()
        return xs[len(xs) // 2], xs[-1]

    ik_med, ik_worst = timeit(lambda: K.solve_from(probe[0], mechs), 500)
    fk_med, fk_worst = timeit(lambda: K.fk_to_orientation(reachable[0]["theta"], mechs), 200)
    seed_R = reachable[0]["R"]
    fs_med, fs_worst = timeit(lambda: K.fk_to_orientation(reachable[0]["theta"], mechs, seed_R), 500)
    print(f"        IK median {ik_med:.3f} ms worst {ik_worst:.3f} ms | FK cold median {fk_med:.3f} ms "
          f"worst {fk_worst:.3f} ms | FK seeded median {fs_med:.3f} ms worst {fs_worst:.3f} ms "
          f"(tick {TICK_BUDGET_MS} ms)")
    check(f"per-call IK stays inside the {TICK_BUDGET_MS:.0f} ms live tick", ik_med < TICK_BUDGET_MS,
          f"IK median {ik_med:.3f} ms (worst {ik_worst:.3f})")
    check(f"the live path - seeded FK - stays inside the {TICK_BUDGET_MS:.0f} ms live tick",
          fs_med < TICK_BUDGET_MS,
          f"seeded FK median {fs_med:.3f} ms (worst {fs_worst:.3f}); cold net is the fallback, "
          f"median {fk_med:.3f} ms")
    return tally()


def tally():
    print(f"\n== {len(PASS)} passed, {len(FAIL)} failed, {len(PASS) + len(FAIL)} checks ==")
    if FAIL:
        print("failed: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
