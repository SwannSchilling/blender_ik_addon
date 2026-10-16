# tools/spm - dev tooling for the standalone SPM. Not part of the add-on.

Nothing in this folder is imported by the add-on, is imported by Blender, or is reachable from
`__init__.py`. The add-on's runtime is the modules at the repository root plus `meshes/`; this folder
exists so that the SPM's offline tooling is versioned and reproducible instead of living in an
untracked `_scratch/` that a fresh clone cannot see.

## What the package will need from the SPM later (not this folder)

This folder ships nothing. But two modules that do NOT live here are runtime, not dev tooling, and are
carried by the future package once the SPM rig registers - `spm_kinematics.py` (the port, pure Python,
no `bpy`) and `spm_driver.py` (signs, gear, offsets). When the SPM rig is wired at the driver slice they
go on the package's explicit module list beside `arm7_rig.py` and `ik_core.py`; until then they are on
disk but nothing in `__init__.py` imports them, so they cost the current package nothing. The one-line
test that a file is dev-only is this folder's rule: if `__init__.py` can reach it, it is not dev-only,
and these two are the deliberate exception - runtime modules kept out only because the registration that
would pull them in has not landed yet.


Why it can sit inside the package directory without shipping: the add-on is a `bl_info` classic
add-on and its package has never been "the whole folder". What ships today is five files
(`__init__.py`, `arm7_rig.py`, `ik_core.py`, `test_acceptance.py`, `README.md`) - no `tests/`, no
`standalone/`, no DLLs, no meshes. Dev tooling living inside the package folder and being left out
of the package is therefore already this repository's convention (`tests/`, `standalone/` do
exactly that), not a new idea introduced here. `tools/gates/README.md` states what that implies for
whoever regenerates a package.

## The frozen snapshot

`spm_mechanism_demo_meshes_v1_2.frozen.html` is a **snapshot**, byte-for-byte, of the working copy at
`SPM/spm_mechanism_demo_meshes_v1_2.html`. It is not a file to edit. It changes in exactly one
situation: a deliberate re-stamp, in the same commit as the fixture regenerated from it. That rule is
what makes the fixture's `_stamp.html_sha256` mean something - it is a digest of a file version
control holds, so the numbers can be traced back to bytes somebody can still recover. A snapshot that
drifts from the working copy silently turns that digest into a claim.

`COMMIT_PHASE2_1A.ps1 -ReStamp` performs the copy and refuses to commit a snapshot that is behind the
working copy without being told to; `COMMIT_PHASE2_1B.ps1` refuses to commit a fixture whose stamp
does not equal the snapshot's digest. The export itself prints a notice, and never an error, when the
snapshot lags - lag is expected between a tool change and its re-stamp.

## The four scripts, and why each exists

| script | what it proves |
|---|---|
| `export_spm_golden.py` | The fixture is the tool's own output. It slices the tool's own marked regions out of the snapshot, concatenates them into one node module, and calls the tool's `collectGoldenJSON()`. It refuses to write if extracted code reaches for `THREE` or the DOM, or if an exported quaternion does not reproduce the matrix it claims. Nothing here recomputes a pose: a second opinion about the geometry would be a second authority, and the tool is the one. |
| `tip_pin_equivalence.py` | The crank-tip and platform-pin arithmetic exists twice on purpose (the on-screen readout was deliberately left alone). This requires the on-screen line to still be present verbatim, then runs both copies over 240 poses and compares the points, the printed millimetres, and the `ok/warn/bad` verdicts derived from them. |
| `peek_spm_golden.py` | Every pose in one table, for a human: angles, conditioning, closure residual, quaternion. A fixture nobody can read is a fixture nobody checks. |
| `stl_manifest.py` | The nine SPM meshes by weight and content, so no binary enters the repository without being named first. Prints; writes only when told. |

Reading the *working* copy instead of the snapshot is possible with `--allow-working-copy` (or
`SPM_GOLDEN_ALLOW_WORKING=1`, which sibling scripts inherit) and prints a notice saying why the
result is not a committable fixture. It exists for the moment before the first snapshot exists.

## Running them, from the workspace root

```powershell
Set-Location 'F:\GithubProjects\URDF_BIO_IK'
python -B blender_ik_addon\tools\spm\export_spm_golden.py      # regenerate the fixture
python -B blender_ik_addon\tools\spm\tip_pin_equivalence.py    # the equivalence battery
python -B blender_ik_addon\tools\spm\peek_spm_golden.py        # read the fixture
python -B blender_ik_addon\tools\spm\stl_manifest.py           # the meshes, weighed and hashed
```
They locate the workspace by walking up until they find the folder holding both `SPM/` and
`blender_ik_addon/`, so none of them hard-codes where the checkout lives. The assert suite over the
fixture is not here: `blender_ik_addon/tests/test_spm_golden.py` is a test, and lives with the tests.
