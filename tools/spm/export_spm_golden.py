"""Produce the SPM golden-pose fixture by running the HTML tool's OWN code.

Why it works this way: the fixture has to be what the tool says, so nothing here recomputes a
pose. The tool's script is sliced out at its own markers - CALIB, the pure math, the geometry
constants, the tool's own MAPCORE block, and the SPMGOLDEN block - concatenated into one node
module, and `collectGoldenJSON()` is called. What lands in the fixture is the bytes that
function returns, plus a stamp of the file they came from.

The purity gate is the whole point: if any extracted line mentions THREE, the DOM, or a
network/file API, the numbers would depend on a browser and this script would be lying about
where they came from, so it refuses instead of exporting.

    python -B _scratch/_export_spm_golden.py            # writes the fixture
    python -B _scratch/_export_spm_golden.py --print    # also dump the JSON

Written to the workspace root as a sibling temp module because this session may create files
there; deleted afterwards.
"""
from __future__ import annotations

import datetime
import hashlib
import json
import os
import re
import subprocess
import sys

def _find_root() -> str:
    """The workspace root, found rather than counted, so the script runs from _scratch/ today and
    from blender_ik_addon/tools/spm/ tomorrow without either location being hard-coded."""
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(6):
        if os.path.isdir(os.path.join(d, "blender_ik_addon")) and os.path.isdir(os.path.join(d, "SPM")):
            return d
        d = os.path.dirname(d)
    raise RuntimeError("cannot find the workspace root: no folder above this script holds both "
                       "blender_ik_addon/ and SPM/")


ROOT = _find_root()
#: the path this very file is reached by, in the stamp's forward-slash form. Derived, never written
#: down: the copy that lives in tools/spm/ has to stamp tools/spm/, and the copy still in _scratch/
#: has to stamp _scratch/. A stamp that contradicts the file carrying it is the drift the whole
#: _stamp block exists to catch, so it must not be able to originate in here.
EXPORTER_REL = os.path.relpath(os.path.realpath(__file__), ROOT).replace("\\", "/")
#: the committed copy is the authority, and it moves only by a deliberate re-stamp
SNAPSHOT = os.path.join(ROOT, "blender_ik_addon", "tools", "spm",
                        "spm_mechanism_demo_meshes_v1_2.frozen.html")
WORKING = os.path.join(ROOT, "SPM", "spm_mechanism_demo_meshes_v1_2.html")
#: reading the working copy is for the moment before the first snapshot exists, and says so
#: loudly; the env form lets a sibling tool inherit the operator's decision instead of each
#: script carrying its own copy of the flag
ALLOW_WORKING = "--allow-working-copy" in sys.argv or os.environ.get("SPM_GOLDEN_ALLOW_WORKING") == "1"
HTML = SNAPSHOT if (os.path.isfile(SNAPSHOT) or not ALLOW_WORKING) else WORKING
FIXTURE = os.path.join(ROOT, "blender_ik_addon", "tests", "fixtures", "spm_golden_poses.json")
#: the second-best home, for a session that may not create the fixtures folder
FALLBACK = os.path.join(os.path.dirname(os.path.dirname(FIXTURE)), "spm_golden_poses.json")
MODULE = os.path.join(ROOT, "_spm_golden_harness.mjs")

#: the tool's own regions, in dependency order
REGIONS = ("SPMCALIB", "SPMMATH", "SPMGEOM", "SPMMATH2", "SPMMATH3", "MAPCORE", "SPMGOLDEN")
IMPURE = re.compile(r"\b(THREE\w*|document|window|localStorage|FileReader|getElementById|fetch|Blob|URL)\b")


def span(text: str, name: str) -> tuple:
    """The (begin, end) offsets of one marked region, in its own marker form."""
    for begin, end in ((f"//{name}-BEGIN", f"//{name}-END"), (f"//{name}-begin", f"//{name}-end")):
        try:
            lo, hi = text.index(begin), text.index(end)
        except ValueError:
            continue
        if hi > lo:
            return lo + len(begin), hi
    raise AssertionError(f"region {name} is not delimited in {os.path.basename(HTML)}")


def pieces_of(text: str) -> dict:
    """The regions, verbatim, each exactly once.

    The golden block was inserted inside the tool's own MAPCORE span (it belongs with that
    math), so MAPCORE would otherwise carry it a second time and node would read the second
    `const GOLDEN_BUILD` as a SyntaxError. The nested block is therefore cut out of its
    parent and emitted on its own, after the math it depends on.
    """
    spans = {name: span(text, name) for name in REGIONS}
    out = {}
    for name in REGIONS:
        lo, hi = spans[name]
        body = text[lo:hi]
        child = "SPMGOLDEN"
        if name != child and child in REGIONS:
            clo, chi = spans[child]
            if lo < clo < hi:                       # the golden block rides inside this one
                body = body[:clo - lo] + body[chi - lo:]
        out[name] = body
    return out


def main() -> int:
    if not os.path.isfile(HTML):
        print("no tool to read at", os.path.relpath(SNAPSHOT, ROOT).replace("\\", "/"))
        print("        the committed snapshot does not exist yet. Take it from the working copy,")
        print("        and only in the same breath as the fixture it belongs to:")
        print("            Copy-Item SPM\\spm_mechanism_demo_meshes_v1_2.html \\")
        print("                      blender_ik_addon\\tools\\spm\\spm_mechanism_demo_meshes_v1_2.frozen.html")
        print("        (python -B ..._export_spm_golden.py --allow-working-copy exports from the")
        print("         working copy meanwhile, and stamps it as such, which is not a committable")
        print("         fixture: its provenance points at a file version control does not hold)")
        return 2
    if os.path.isfile(SNAPSHOT) and os.path.realpath(HTML) == os.path.realpath(SNAPSHOT) \
            and os.path.isfile(WORKING) \
            and hashlib.sha256(open(WORKING, "rb").read()).hexdigest() \
            != hashlib.sha256(open(SNAPSHOT, "rb").read()).hexdigest():
        print("NOTICE  the snapshot in tools/spm/ is behind the working copy in SPM/. The fixture"
              "\n        is stamped to the snapshot, which is the rule; when the drift is meant,"
              "\n        re-stamp the snapshot and regenerate the fixture in one commit.")
    text = open(HTML, encoding="utf-8").read()
    parts = pieces_of(text)
    whole = "\n".join(parts.values())

    # Scan code, not prose: the golden block's own header comment says out loud that it does not
    # touch THREE or the DOM, and a gate that reads comments would fail on its own honesty.
    code_only = "\n".join(l for l in whole.split("\n") if not l.strip().startswith("//"))
    dirty = sorted({m[1] for m in IMPURE.finditer(code_only)})
    if dirty:
        print("REFUSING: the extracted regions are not pure; they mention", dirty)
        print("           a fixture may only carry numbers that came from DOM-free code.")
        for i, line in enumerate(code_only.split("\n"), 1):
            if IMPURE.search(line):
                print(f"   line {i}: {line.strip()[:160]}")
        return 3

    node = subprocess.run(["node", "--version"], stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text = True)
    ver = (node.stdout or "?").strip() if node.returncode == 0 else "unknown"

    with open(MODULE, "w", encoding="utf-8") as fh:
        fh.write(f"// generated by {EXPORTER_REL} from the tool's own regions\n")
        for name in REGIONS:
            fh.write(f"\n// ---- {name} ----\n")
            fh.write(parts[name])
        fh.write("\nconst __doc = collectGoldenJSON();\n")
        fh.write("__doc._export = {node_version: process.version, exported_at: new Date().toISOString(), "
                 "exporter: '" + EXPORTER_REL + "'};\n")
        fh.write("process.stdout.write(JSON.stringify(__doc, null, 2));\n")

    raw = MODULE + ".out"
    errf = MODULE + ".err"
    keep = "--keep" in sys.argv
    try:
        with open(raw, "wb") as fh, open(errf, "wb") as efh:
            run = subprocess.run(["node", MODULE], stdout=fh, stderr=efh, cwd=ROOT)
        if run.returncode != 0:
            print("node refused the extracted regions (exit %d) - the tool's own code did not run "
                  "outside its page. What it said:" % run.returncode)
            print(open(errf, encoding="utf-8", errors="replace").read()[-2500:])
            return 4
        body = open(raw, encoding="utf-8").read()
        doc = json.loads(body)
    finally:
        for p in ((MODULE, raw, errf) if not keep else ()):
            try:
                os.remove(p)
            except OSError:
                pass
        if keep:
            print("(--keep: the harness module stays at", MODULE, "for inspection)")

    raw_bytes = open(HTML, "rb").read()
    if os.path.realpath(HTML) == os.path.realpath(WORKING):
        print("NOTICE  the numbers came from the working copy in SPM/, not from the committed"
              "\n        snapshot. That is allowed only with --allow-working-copy, and only before"
              "\n        the first snapshot exists: a fixture like this one is stamped to a file"
              "\n        that version control does not hold.")
    doc["_stamp"] = {
        "html_file": os.path.relpath(HTML, ROOT).replace("\\", "/"),
        "html_sha256": hashlib.sha256(raw_bytes).hexdigest(),
        "html_bytes": len(raw_bytes),
        "build": doc.get("build"),
        "regions": list(REGIONS),
        "node_version": doc.get("_export", {}).get("node_version"),
        "exported_at": doc.get("_export", {}).get("exported_at"),
        #: which copy of the exporter produced this, derived from __file__ so the copy in
        #: tools/spm says tools/spm and the one still in _scratch says _scratch. Recorded before
        #: the transient _export carrier below is dropped, so it survives into the fixture.
        "exporter": EXPORTER_REL,
        "band_mm": doc.get("band_mm"),
        "units": "metres and radians; mm only where a key says mm; SCALE=0.001 is STL vertices only",
    }
    doc.pop("_export", None)

    # --stamp-report: assert what this run would stamp and write no fixture at all. The copy block
    # proves the relocated tool reads the committed snapshot without touching the real fixture or
    # reaching outside the two directories it may write, and without leaving a probe file to clean.
    if "--stamp-report" in sys.argv:
        print("STAMP html_file=%s" % doc["_stamp"]["html_file"])
        print("STAMP html_sha256=%s" % doc["_stamp"]["html_sha256"])
        print("STAMP html_bytes=%d" % doc["_stamp"]["html_bytes"])
        print("STAMP exporter=%s" % doc["_stamp"]["exporter"])
        print("STAMP poses=%d build=%s" % (doc["counts"]["poses"], doc["_stamp"]["build"]))
        return 0

    # Two homes, tried in order. fixtures/ is where the fixture belongs; a session that may not
    # create a directory has to be able to produce the artefact anyway, and tests/ beside the
    # test that reads it is the honest second best - the test accepts either, and says which.
    # --out writes the document to one specific path and nowhere else. The copy block needs it:
    # proof the relocated tool works must not clobber the real fixture or reach outside the two
    # directories it is allowed to write, so it produces a throwaway and checks that instead.
    probe = "--out" in sys.argv and sys.argv.index("--out") + 1 < len(sys.argv)
    if probe:
        homes = (sys.argv[sys.argv.index("--out") + 1],)
    else:
        homes = (FIXTURE, FALLBACK, os.path.join(ROOT, "spm_golden_poses.json"))
    saved = None
    for target in homes:
        folder = os.path.dirname(os.path.realpath(target))
        if not os.path.isdir(folder):
            try:
                os.makedirs(folder)
            except OSError as exc:
                print("cannot create %s (%s)" % (folder, exc.strerror))
                continue
        if not os.path.isdir(folder):
            continue
        try:
            with open(target, "w", encoding="utf-8", newline="\n") as fh:
                json.dump(doc, fh, indent=2, sort_keys=False)
                fh.write("\n")
        except OSError as exc:
            print("cannot write %s (%s)" % (target, exc.strerror))
            continue
        saved = target
        break
    if saved is None:
        print("no writable home for the fixture; from your shell:")
        print("    New-Item -ItemType Directory -Force -Path '%s'" % os.path.dirname(FIXTURE))
        return 5

    print("fixture        :", os.path.relpath(saved, ROOT).replace("\\", "/"),
          "" if saved == FIXTURE else "  <-- second-best home; make fixtures/ and move it")
    print("html build     :", doc["_stamp"]["build"], "| sha256", doc["_stamp"]["html_sha256"][:16], "...")
    print("node           :", doc["_stamp"]["node_version"])
    print("poses          :", doc["counts"]["poses"], "by kind:", json.dumps(doc["counts"]["by_kind"]))
    print("band           :", doc["band_mm"], "mm | quaternion order", doc["convention"]["quaternion_order"])
    print("pending        :", json.dumps(doc.get("pending")))
    if "--print" in sys.argv:
        print(json.dumps(doc, indent=2)[:4000])
    return 0


if __name__ == "__main__":
    sys.exit(main())
