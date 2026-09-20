"""Every exception name a file expects to resolve, checked against what actually exists.

`py_compile` proves bytes parse. It does not notice `FileNotFoundError` written with a missing `r`, or
`SHUT_SD` where `SHUT_WR` was meant: those compile happily and raise at the worst possible moment, in
front of a user. That matters more than usual here, because the text of these files has been observed
arriving altered in transit -- identifiers really do sometimes lose a letter on the way in -- and a
file that imports cleanly is not yet a file that is correct.

So this walks the AST and asks, for each class named in an `except` or a `raise`, whether either
builtins or the module's own definitions actually provide it, and reports the nearest real candidate
when they do not.

    python blender_ik_addon/tests/verify_names.py path/to/file.py [more.py ...]
"""
from __future__ import annotations

import ast
import builtins
import difflib
import sys


def _defined_names(tree: ast.Module) -> set:
    """Names the module itself provides, so a home-grown exception is not reported as missing."""
    out = set()
    for node in tree.body:
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)):
            out.add(node.name)
        elif isinstance(node, ast.Assign):
            out |= {t.id for t in node.targets if isinstance(t, ast.Name)}
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            out.add(node.target.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            out |= {(a.asname or a.name.split(".")[0]) for a in node.names}
        elif isinstance(node, ast.Try):                     # a compat shim: `except: X = Something`
            for handler in node.handlers:
                for sub in ast.walk(handler):
                    if isinstance(sub, ast.Assign):
                        out |= {t.id for t in sub.targets if isinstance(t, ast.Name)}
    for node in ast.walk(tree):                            # module-level defs inside `if` blocks count
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.col_offset == 0:
            out.add(node.name)
    return out


def _handler_types(node: ast.ExceptHandler) -> list:
    t = node.type
    if t is None:
        return []
    if isinstance(t, ast.Name):
        return [t]
    if isinstance(t, ast.Tuple):
        return [e for e in t.elts if isinstance(e, ast.Name)]
    if isinstance(t, ast.Attribute):                       # `except mod.Error` is fine, not our business
        return []
    return []


def check_file(path: str) -> list:
    try:
        src = open(path, encoding="utf-8").read()
    except OSError as exc:
        return [f"{path}: cannot be read ({exc})"]
    try:
        tree = ast.parse(src, filename=path)
    except SyntaxError as exc:
        return [f"{path}: does not parse ({exc})"]
    local = _defined_names(tree)
    universe = set(dir(builtins)) | local
    problems = []
    for node in ast.walk(tree):
        candidates = []
        if isinstance(node, ast.ExceptHandler):
            candidates = [(t.id, node.lineno, "except") for t in _handler_types(node)]
        elif isinstance(node, ast.Raise) and isinstance(node.exc, ast.Call):
            f = node.exc.func
            if isinstance(f, ast.Name) and f.id[0].isupper():
                candidates = [(f.id, node.lineno, "raise")]
        for name, line, where in candidates:
            if name in universe:
                continue
            near = difflib.get_close_matches(name, sorted(universe), n=2, cutoff=0.72)
            problems.append(f"{path}:{line}: {where} {name} resolves to nothing here; "
                            f"nearest real candidates: {near or 'none'}")
    return problems


def main(argv: list) -> int:
    if not argv:
        print(__doc__.splitlines()[1].strip())
        return 2
    problems = []
    for path in argv:
        found = check_file(path)
        print(f"--- {path}: {len(found)} unresolved name(s)")
        for one in found:
            print("   ", one)
        problems += found
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
