"""Proof that the duplicated tip/pin formula still says what the on-screen line says.

The golden block carries the crank-tip and platform-pin arithmetic a second time, because the
on-screen path is deliberately left alone (no shared helper, no risk to the readouts). Duplicated
arithmetic is only safe while it stays equal, so this battery says so two ways. First, the exact
line is required to still be present in the HTML: edit the readout and this stops, rather than the
two copies quietly parting. Second, both versions are run over a battery of poses and the points
and the printed millimetres are compared, together with the ok/warn/bad verdict the readout derives
from them - the observable behaviour of the readout, not just its numbers.

The poses come from a fixed sequence, not from the fixture: this is about the formula, and a
fixture-derived battery would only test the path that produced it.

    python -B _scratch/_test_tip_pin_equivalence.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))


def _find_root() -> str:
    """Found rather than counted, so this runs from _scratch/ today and from
    blender_ik_addon/tools/spm/ tomorrow without either being hard-coded."""
    d = _HERE
    for _ in range(6):
        if os.path.isdir(os.path.join(d, "blender_ik_addon")) and os.path.isdir(os.path.join(d, "SPM")):
            return d
        d = os.path.dirname(d)
    raise RuntimeError("cannot find the workspace root above " + _HERE)


ROOT = _find_root()
sys.path.insert(0, _HERE)
try:                                                    # the name it has in version control
    import export_spm_golden as exp                     # noqa: E0401
except ImportError:                                     # the name it has in _scratch
    import _export_spm_golden as exp                    # noqa: E0401

#: the on-screen line, verbatim, as it must still read in the HTML (the drift tripwire)
REQUIRED = (
    "const crankTips=[],platPts=[];for(let i=0;i<3;i++){crankTips.push(addv(pivot,scalev("
    "matVec(rodrigues([0,0,1],theta[i]),w0[i]),r1)));platPts.push(addv(pivot,scalev("
    "matVec(Rplat,v0[i]),r2)));}",
    "const cls=errNorm*1000<0.5?'ok':errNorm*1000<2?'warn':'bad';",
)

#: what the readout does with the numbers, transcribed so the verdict can be compared too
DRIVER = """
const tipPinReadout=(pivot,w0,v0,r1,r2,theta,Rplat)=>{
  const crankTips=[],platPts=[];for(let i=0;i<3;i++){crankTips.push(addv(pivot,scalev(matVec(rodrigues([0,0,1],theta[i]),w0[i]),r1)));platPts.push(addv(pivot,scalev(matVec(Rplat,v0[i]),r2)));}
  const mm=[];for(let i=0;i<3;i++){const err=subv(platPts[i],crankTips[i]);const errNorm=Math.hypot(err[0],err[1],err[2]);const cls=errNorm*1000<0.5?'ok':errNorm*1000<2?'warn':'bad';mm.push({d:errNorm*1000,cls});}
  return{crankTips,platPts,mm};
};
let worstTip=0,worstPin=0,worstMm=0,n=0,clsDiff=0;
const fr=spmLegFrames();
for(let k=1;k<=240;k++){
  const th=[0,1,2].map(i=>Math.sin(k*(i+2)*0.37)*Math.PI);
  const ax=normv([Math.sin(k*0.11),Math.cos(k*0.07),Math.sin(k*0.05)+0.3]);
  const R=rodrigues(ax,(k%57)/57*Math.PI);
  const a=tipPinReadout(CALIB.pivot,fr.w0,fr.v0,fr.r1,fr.r2,th,R);
  const b=spmTipsAndPoints(fr,th,R);
  const bm=spmPinErrorsMm(b.crankTips,b.platPts);
  for(let i=0;i<3;i++){
    for(let j=0;j<3;j++){
      worstTip=Math.max(worstTip,Math.abs(a.crankTips[i][j]-b.crankTips[i][j]));
      worstPin=Math.max(worstPin,Math.abs(a.platPts[i][j]-b.platPts[i][j]));
    }
    worstMm=Math.max(worstMm,Math.abs(a.mm[i].d-bm[i]));
    const clsOf=v=>v<0.5?'ok':v<2?'warn':'bad';
    if(a.mm[i].cls!==clsOf(bm[i]))clsDiff++;
  }
  n++;
}
process.stdout.write(JSON.stringify({poses:n,worstTip_m:worstTip,worstPin_m:worstPin,
  worstMm_mm:worstMm,verdictDifferences:clsDiff}));
"""

TOLERANCE_M = 1e-15


def main() -> int:
    html = open(exp.HTML, encoding="utf-8").read()
    gone = [line for line in REQUIRED if line not in html]
    if gone:
        print("FAIL: the on-screen readout is not what this battery was written against:")
        for line in gone:
            print("   not in the file any more:", line[:120])
        print("   the two copies are free to differ now, so neither proves anything")
        return 1
    print("tripwire   : the on-screen tip/pin line and its ok/warn/bad rule are still in the HTML, verbatim")

    parts = exp.pieces_of(html)
    # Code, not prose: the golden block's own header comment says out loud that it does not touch
    # THREE or the DOM, and a gate reading comments would fail on its own honesty (the exporter
    # applies the same rule for the same reason).
    whole = "\n".join(l for part in parts.values() for l in part.split("\n")
                      if not l.strip().startswith("//"))
    for token in ("THREE", "document", "getElementById"):
        if token in whole:
            print(f"FAIL: the extracted code mentions {token}, so it is not the pure lane")
            return 2

    module = os.path.join(ROOT, "_tip_pin_battery.mjs")
    out = module + ".out"
    try:
        with open(module, "w", encoding="utf-8") as fh:
            for name in exp.REGIONS:
                fh.write(parts[name] + "\n")
            fh.write(DRIVER)
        with open(out, "wb") as fh:
            run = subprocess.run(["node", module], stdout=fh, stderr=subprocess.STDOUT, cwd=ROOT)
        if run.returncode != 0:
            print(f"FAIL: node refused the battery (exit {run.returncode})")
            return 3
        got = json.loads(open(out, encoding="utf-8").read())
    finally:
        for p in (module, out):
            try:
                os.remove(p)
            except OSError:
                pass

    print(f"battery    : {got['poses']} poses, each with its own theta triple and orientation")
    print(f"             worst crank-tip deviation {got['worstTip_m']:.3e} m")
    print(f"             worst platform-pin deviation {got['worstPin_m']:.3e} m")
    print(f"             worst printed-millimetre deviation {got['worstMm_mm']:.3e} mm")
    print(f"             ok/warn/bad verdicts differing: {got['verdictDifferences']}")
    bad = [ ]
    if got["worstTip_m"] > TOLERANCE_M or got["worstPin_m"] > TOLERANCE_M:
        bad.append("the points themselves come out at different places")
    if got["worstMm_mm"] > TOLERANCE_M * 1000:
        bad.append("the readout would print a different distance")
    if got["verdictDifferences"]:
        bad.append("the readout would colour a pin differently")
    if got["poses"] < 200:
        bad.append("the battery ran fewer poses than it was written to run")
    if bad:
        print("FAIL: " + "; ".join(bad))
        return 4
    print("ok: the duplicated formula is the on-screen formula, over every pose the battery threw at it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
