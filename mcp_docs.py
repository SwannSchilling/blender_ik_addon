"""The catalogue tables in the docs are derived from the catalogue, not typed into them.

Why this exists: a tool table was once hand-written into `pick_ik_mcp/SERVER_DESIGN.md`, it said twelve
tools where there are fourteen, and it invented a `confirm` gate for `export_urdf`, which has none. A
human will do that again, and it is not a diligence problem: those numbers have one authority
(`mcp_protocol`) and a prose copy of them can never be trusted over it. So the mechanical columns are
generated, and every table whose prose must stay hand-written is *checked* against the authority
instead of being believed.

    python -m blender_ik_addon.mcp_docs            # regenerate the generated blocks, then check
    python -m blender_ik_addon.mcp_docs --check    # check only; non-zero exit on any disagreement

`tests/test_mcp_bridge.py` calls check_docs() as a gate, so a doc that drifts fails the suite rather
than waiting for a reader who happens to know the answer.
"""
from __future__ import annotations

import os
import re
import sys

from . import mcp_protocol as P

_HERE = os.path.dirname(os.path.realpath(__file__))
TARGETS = (os.path.join(_HERE, "MCP_INTEGRATION_PLAN.md"),
           os.path.normpath(os.path.join(_HERE, "..", "pick_ik_mcp", "SERVER_DESIGN.md")))

BEGIN, END = "<!-- CATALOGUE-BEGIN:%s -->", "<!-- CATALOGUE-END:%s -->"
#: The catalogue's gate strings as the docs draw them. `none` may also be drawn as an em dash.
GATE_DRAWN = {"none": "—", "confirm": "`confirm`", "arm": "`arm`"}


def _answered() -> frozenset | None:
    """Commands this build actually answers. The handler table needs `bpy`; without it the column says
    `?`, which is honest, rather than `—`, which would be a lie about the build."""
    try:
        from . import mcp_handlers_obs as H
    except BaseException:
        return None
    return frozenset(H.HANDLERS)


def render_tools() -> str:
    """The generated table. Every cell comes from the authority; nothing here is typed twice."""
    have = _answered()
    rows = sorted(P.COMMANDS)
    lines = ["| cmd | class | exec | gate | answered here |", "|---|---|---|---|---|"]
    for cmd in rows:
        spec = P.lookup(cmd)
        lines.append("| `{}` | {} | {} | {} | {} |".format(
            cmd, spec.cls, spec.executor, GATE_DRAWN.get(spec.gate, spec.gate),
            "?" if have is None else ("yes" if cmd in have else "—")))
    lines += ["", f"_Generated from `mcp_protocol`: proto_rev `{P.proto_rev()}`, "
                  f"{len(P.COMMANDS)} commands, {0 if have is None else len(have)} answered in this "
                  f"build. Nothing between the sentinels is hand-written; edit `mcp_docs.py` or the "
                  f"catalogue instead._"]
    return "\n".join(lines)


def _tables(text: str) -> list:
    """Every pipe table in a document, as (header_cells_lower, [rows_of_cells])."""
    out, lines, i = [], text.splitlines(), 0
    sep = re.compile(r"^\|[\s:|\-]+\|$")
    while i < len(lines):
        row = lines[i].strip()
        if row.startswith("|") and i + 1 < len(lines) and sep.match(lines[i + 1].strip()):
            hdr = [c.strip().lower().replace("`", "") for c in row.strip("|").split("|")]
            rows, j = [], i + 2
            while j < len(lines) and lines[j].strip().startswith("|"):
                rows.append([c.strip() for c in lines[j].strip().strip("|").split("|")])
                j += 1
            out.append((hdr, rows))
            i = j
        else:
            i += 1
    return out


def _bare(cell: str) -> str:
    return cell.strip().strip("`* ").replace("**", "")


def check_docs(verbose: bool = True) -> list:
    """Every catalogue column the docs repeat, against the authority. Returns the disagreements."""
    bad, checked = [], 0
    for path in TARGETS:
        name = os.path.basename(path)
        try:
            with open(path, encoding="utf-8") as fh:
                text = fh.read()
        except OSError as exc:
            bad.append(f"{name}: cannot be read ({exc})")
            continue
        for hdr, rows in _tables(text):
            if "cmd" not in hdr or "gate" not in hdr:
                continue                                    # not a catalogue table; its prose is its own
            col = {k: hdr.index(k) for k in ("cmd", "gate") if k in hdr}
            for key in ("class", "exec", "tool"):
                if key in hdr:
                    col[key] = hdr.index(key)
            for row in rows:
                if col["cmd"] >= len(row):
                    continue
                cmd = _bare(row[col["cmd"]])
                if cmd not in P.COMMANDS:
                    continue                                # a label, a group, or a command gone away
                spec = P.lookup(cmd)
                checked += 1
                want = _bare(GATE_DRAWN[spec.gate])
                got = row[col["gate"]] if col["gate"] < len(row) else ""
                base = _bare(got)
                # An open gate may be drawn as an em dash and may carry a note about what guards it
                # instead (§9.5's sandbox), so that column is matched by prefix; a named gate is
                # matched exactly once the emphasis and code ticks are off.
                ok_gate = base.startswith(("—", "-", "(")) or base == "" if want == "—" \
                    else base == want
                if not ok_gate:
                    bad.append(f"{name}: `{cmd}` is gated **{want or 'open'}** in the catalogue; the "
                              f"doc draws {got!r}")
                if "tool" in col and col["tool"] < len(row) and "(" in row[col["tool"]]:
                    continue        # a variant row: arguments pick its class, so prose owns that cell
                for key, want2 in (("class", spec.cls), ("exec", spec.executor)):
                    if key in col and col[key] < len(row):
                        got2 = _bare(row[col[key]])
                        if got2 in ("", "—", "-"):
                            continue
                        if want2 not in got2:
                            bad.append(f"{name}: `{cmd}` is **{want2}** in {key} in the catalogue; the "
                                       f"doc draws {got2!r}")
                        else:
                            checked += 1
    if verbose:
        for line in bad:
            print(f"[mcp_docs] {line}")
        print(f"[mcp_docs] {checked} catalogue cell(s) checked against the authority: "
              f"{len(bad)} disagreement(s)")
    return bad


def write_blocks() -> list:
    """Regenerate the sentinel-delimited blocks. Returns which files it had to change."""
    changed = []
    for path in TARGETS:
        mark = BEGIN % "tools"
        try:
            with open(path, encoding="utf-8") as fh:
                text = fh.read()
        except OSError:
            continue
        if mark not in text:
            continue
        _had, tail = text.split(mark, 1)
        _old, after = tail.split(END % "tools", 1)
        fresh = _had + mark + "\n\n" + render_tools() + "\n\n" + (END % "tools") + after
        if fresh != text:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(fresh)
            changed.append(os.path.basename(path))
    return changed


def main(argv = None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if "--check" not in argv:
        changed = write_blocks()
        print(f"[mcp_docs] regenerated the block in: {', '.join(changed) or 'nothing: none were stale'}")
    bad = check_docs()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
