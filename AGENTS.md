# Rhino_PyWorkshop — Research Summary & Agent Guidelines

This document is a research summary of the repository plus a practical playbook for
coding agents (Claude Code, Codex, or others) working in it. Read this before making
non-trivial changes — the runtime environment here has sharp edges that are easy to
trip over silently (no test suite, no way to execute most of the code outside Rhino,
an implicit import convention that isn't declared anywhere in code).

## 1. What this repository is

`Rhino_PyWorkshop` is a personal collection of Python tools for **Rhino 3D /
Grasshopper (GH)**, focused on digital fabrication: parametric geometry generation,
image/surface-driven pattern control, and machine-code export for both desktop FFF/FDM
3D printers (G-code) and an industrial KUKA robot arm (KRL). It is not a packaged
Python library or application — it's a working set of Grasshopper Python (GHPython)
components plus a shared helper library that those components import.

Two top-level folders matter:

- **`libs/`** — shared helper modules imported by scripts. Domain-organized (geometry
  math, curve ops, print-path prep, G-code assembly, KUKA KRL assembly, surface/image
  sampling, file I/O + validation), not class-hierarchy-organized.
- **`scripts/`** — individual Grasshopper Python components. Each file is the body of
  one GHPython component on some `.gh` canvas (not included in this repo — only the
  script bodies are version-controlled). Each script reads GH-supplied inputs as bare
  global variables and assigns outputs the same way.

A third folder, **`machine_settings/`**, holds JSON machine profiles (build volume,
nozzle diameter, G-code flavor) consumed by the G-code export path.

`REPO_DOCUMENTATION.md` in the repo root is an academic-report-oriented summary
(capabilities, evidence suggestions). This file is oriented at agents that need to
*edit* the code safely and efficiently.

## 2. Runtime environment — read this before writing any code

This is the single most important section. Getting this wrong produces code that
looks correct, compiles, and is subtly or completely broken in Grasshopper.

### 2.1 IronPython inside Grasshopper, not CPython

Every file in `libs/` except `iolib.py` imports `rhinoscriptsyntax`, `Rhino`, and/or
`Grasshopper` at module scope. These packages only exist inside Rhino's embedded
IronPython interpreter (GHPython components). **None of these modules can be
`import`-ed or executed in a plain CPython session** — attempting to `import
geometrylib` from a normal `python` shell will raise `ModuleNotFoundError:
rhinoscriptsyntax` immediately.

- `iolib.py` is the **one exception**: pure standard library + optional `openpyxl`,
  no Rhino imports. It is fully importable and testable in plain CPython.
- Everything else can only be **syntax-checked** headless (`python -m py_compile
  path/to/file.py`, or `ast.parse(open(path).read())`), not executed or unit-tested
  directly.

### 2.2 Grasshopper's implicit globals

GHPython components receive their inputs as bare global variables injected by
Grasshopper at run time (matching the component's named input parameters on the
canvas), and produce outputs the same way (assigning to variables Grasshopper reads
back out by name, e.g. `a`, `preview`, `previewpts`). Two additional names are always
injected:

- `ghenv` — the component wrapper; `ghenv.Component.AddRuntimeMessage(level, msg)` is
  the standard way to surface warnings/errors on the GH canvas.
- `ghdoc` — the active GH document; `ghdoc.Path` is used to resolve output directories
  relative to the `.gh` file.

**Static analysis / linters will flag `ghenv`, `ghdoc`, and every GH input variable
(e.g. `toolpath`, `nozzle`, `filename`) as "undefined name."** This is normal and
expected for every script in this repo — it is not a bug to fix. Do not add
`# noqa` comments or reorganize code to "fix" these; they are correct as-is.

Consequences for editing scripts:

- **Never rename a script's top-level input/output variable names** unless you are
  also updating the corresponding `.gh` file (which is not in this repo, so in
  practice: don't rename them). The GH canvas wires named parameters to these exact
  Python names; a rename silently breaks the component the next time someone opens it
  in Rhino, with no error until then.
- Optional inputs are conventionally handled with `try: x = float(x) except NameError:
  x = default` or `x if 'x' in globals() else default` — this pattern means "the GH
  input wasn't connected." Preserve it; don't replace with `.get()`-style code that
  assumes a dict.

### 2.3 The import convention: bare top-level modules, not a package

Despite `libs/__init__.py` existing (and now exporting `iolib`'s functions), **no
script in `scripts/` imports through the `libs` package**. Every script does e.g.:

```python
import geometrylib as gl
import curvelib as cl
import printlib as pl
import gcodelib as gcl
import kukalib as kl
from srflib import sample_surface_color
```

This works because Grasshopper appends the `libs/` folder directly to `sys.path` at
run time (external to this repo — a GH component or Rhino plugin setting), so every
module in `libs/` is importable as a top-level name, and modules within `libs/`
import each other the same bare way (e.g. `curvelib.py` does `import geometrylib as
gl`, not `from . import geometrylib` or `from libs import geometrylib`).

**Implications for agents:**

- When adding a new function to `libs/`, add it as a top-level function/class in the
  relevant flat file — do not introduce subpackages or relative imports; they will
  not resolve the same way IronPython's GH-injected `sys.path` does, and there is no
  way to test that assumption without opening Rhino.
- When you rename or move a function between `libs/` files, you must **grep the
  entire `scripts/` directory** for every call site (`pl.old_name`, `from printlib
  import old_name`, etc.) and update each one. There is no import system that will
  catch a missed call site for you — it fails silently at GH-canvas-run time, not at
  edit time.
- Cross-module dependencies inside `libs/` currently look like this (leaf → dependent):
  `iolib` ← `geometrylib` ← `curvelib`, `srflib` ← `printlib`, `gcodelib`; `printlib`
  also imports `gcodelib` (for the shared `gcodeline`/flow logic) and `curvelib` (for
  `centercrv`, `curveselfintersection`). Keep this a DAG — `gcodelib.py` and
  `kukalib.py` must never import `printlib.py` or `curvelib.py` back, or you'll create
  a cycle.

### 2.4 No test suite, no CI, no way to execute geometry code headless

There is no `pytest`/`unittest` suite in this repo, and there cannot easily be one for
the 7 of 8 `libs/` files that touch Rhino. Practical verification tiers, in order of
what's actually available to an agent:

1. **Syntax check** (`python -m py_compile <file>`) — catches typos, indentation,
   bad renames, mismatched parens/f-strings. Always do this on every touched file
   before considering a change done.
2. **Pure-logic smoke test** — for functions whose *body* does not call
   `rs.*`/`Rhino.*`/`gh.*` (e.g. `geometrylib.lerp`, `minmaxcap`, `srflib._gradient_factor`,
   `gcodelib.calculate_flow`, `gcodelib.is_within_build_volume`, anything in `iolib.py`),
   copy just that function's body into a throwaway script in a scratch location, exercise
   it with plain values via `python`, then delete the scratch script. This repo's IronPython
   files cannot be imported directly in CPython even for their Rhino-free functions,
   because the module-level `import rhinoscriptsyntax` at the top of the file fails
   immediately — the workaround is copying the isolated logic out, not trying to import
   the real module.
3. **Manual Grasshopper run** — the only way to verify anything that touches curve/surface
   geometry, GH component messaging, or file output. This repo's maintainer has Rhino
   installed and can do this; an agent generally cannot. **When you cannot run this
   tier yourself, say so explicitly** rather than claiming a change is verified — flag
   exactly which components/scripts need a manual re-test and why (e.g. "this changes
   the emitted G-code line order for X.py, please re-run the GH component and diff the
   output file").

### 2.5 Line endings & style

Files use CRLF line endings (Windows repo) — `git diff`/`git status` will warn about
LF→CRLF conversion; that's expected on this Windows checkout, not an error.
Indentation and quote style are inconsistent across files (mix of `return(x)` and
`return x`, single/double quotes, f-strings vs `.format()` vs `%`) — match the
*surrounding* file's existing style rather than imposing a repo-wide style pass
unless specifically asked to.

## 3. Architecture reference

### 3.1 `libs/` — current state

| File | Role | Rhino-dependent? |
|---|---|---|
| `iolib.py` | CSV/XLSX read/write **and** shared input validation (`ValidationError`, `report_issue`, `is_number`, `require_list`/`require_nonempty_list`, `validate_scalar`, `validate_type`). The only pure-CPython file. | No |
| `geometrylib.py` | Interpolation (`lerp`, `lerp_bias`, `invlerp`, `remap`, `lerppts`), clamping (`minmaxcap`/`minmaxcaplist`), `bbox_bounds`, angle math (`shortestangle`, `absoluteangle`, `plane_to_abc`), `timestamp`, `normalize_pattern`, `flattenlist`, and `validate_input` (thin wrapper over `iolib.validate_type`). Leaf module — no other `libs/` dependency besides `iolib`. | Yes |
| `curvelib.py` | Curve construction/measurement helpers: `centercrv`/`centercrv_dir`, `divide_crv_equal`, `bboxplanedomain`, `curveselfintersection`/`curveselfintersection2`, `sort_curves_z`, `spiralise`, `rounded_rectangle`, `offset_crv_both_sides`, etc. Imports `geometrylib`, `iolib`. | Yes |
| `printlib.py` | Print-path prep: `centerobject`, `leveltoplatform`, `selfclosestpt2`, `slice_brep_uniform` (older `rs.AddSrfContourCrvs`-based slicer, currently unused by any script), `stack_curves_by_pattern`, `materialestimation` (FFF/3DCP volume-from-length estimate). `gcodeline` here is a **thin re-export of `gcodelib.gcodeline`** — kept only because `ultimaker_standalone.py` still calls `pl.gcodeline`. Imports `curvelib`, `geometrylib`, `gcodelib`, `iolib`. | Yes |
| `gcodelib.py` | The shared, machine-agnostic G-code module (rebuilt — see §4). Machine-profile abstraction (`load_machine_properties`, `is_within_build_volume` — supports both rectangular `x/y/z` and radial/delta `r/h` build volumes), `calculate_flow` (canonical formula), `gcodeline`/`travel_move`/`print_move`/`retract`/`unretract`, `purge_duplicate_points`/`purge_collinear_points`/`center_points_to_origin`, `curve_to_polyline_points`/`curve_to_points`, `build_header`, `save_gcode_file`, and a `GCodeLib` class wrapping the stateful per-run workflow (`get_part_dims`, `check_print`, `add_header`, `save`). Imports `geometrylib`, `iolib`. | Yes |
| `kukalib.py` | `KukaKRL` class: KRL program assembly (`krl_header`, `ptp`, `lin`, `set_output`, `wait`, fold/comment helpers), tool/base/velocity validation (delegates to `iolib.validate_scalar`), safe filename truncation on write (`write_file`). Imports `iolib`. | No (pure string assembly + file I/O — could in principle be tested headless, but isn't yet wired that way) |
| `srflib.py` | Image/surface sampling and pattern generation: `sample_surface_color`, `closest_srf`, `remap_rgb_channels`, `evaluate_parameter` (fixed/image/gradient sourcing — see §5 for a **known gap**), `get_division_parameters`, `build_alternating_polylines`, `generate_pattern_stack`, `build_gradient_pattern`/`build_stack_pattern` (share a private `_gradient_factor` helper). Imports `geometrylib`, `iolib`. | Yes |

`libs/__init__.py` only re-exports `iolib`'s public names (CSV/XLSX + validation) —
it is **not** used as an import path by any script (see §2.3); treat it as documentation
of `iolib`'s public surface, not a functional package boundary.

### 3.2 `scripts/` — catalog by purpose

- **G-code export (Ultimaker/Wasp FFF)**: `ultimaker.py` (canonical, rebuilt to call
  `gcodelib`), `ultimaker_speeds.py` (velocity-list variant, also rebuilt),
  `wasp_delta.py` (delta/radial printer, uses the `GCodeLib` class), `generic_gcode.py`
  (minimal machine-agnostic exporter, deliberately has no header/temps/retraction).
  `ultimaker_standalone.py` and `ultimaker_contained.py` are **deliberately excluded**
  from the shared-module consolidation — see §4 and §6.1. Do not "clean up" these two
  without explicit instruction.
- **KUKA robotic deposition**: `kuka.py` — the best example in the repo of a script
  properly delegating to its library (`kukalib.KukaKRL`); includes its own
  well-structured GH-input validation layer and two material-estimation functions
  (`estimate_print_volume`, `estimate_material_requirements`) that are still
  script-local (candidates for promotion into `kukalib.py` — not yet done, see §6.2).
- **Pattern generation (porous/truss/thermal walls)**: `porous_triangular.py`,
  `porous_DC26.py`, `porous_test_patterns.py`, `sampled_porous_pattern.py`,
  `sampled_triangles.py`, `sampled_triangles_columns.py`,
  `sampled_freeform_triangles.py` (large, ~2300 lines — the most evolved member of
  this family), `thermal_patterns.py`, `e3d_wall.py`/`e3d_wall_test.py`. These share
  substantial duplicated logic (parameter-evaluation dispatch, wall-opening geometry,
  curve-stacking, brep-slicing) that has **not yet** been elevated into `libs/` — see
  §6.2.
- **Geometry/utility**: `spiraliser.py`, `filament_mesh.py`, `adaptive_slicing.py`,
  `img_projection.py`.

### 3.3 `machine_settings/`

Two JSON profiles today: `ultimaker2.json` (`type: cartesian`, `build_volume: {x,y,z}`,
`nozzle_diameter`, `gcode_flavour: UltiGCode`) and `wasp.json` (`type: delta`,
`build_volume: {r,h}`, same flavour). `gcodelib.load_machine_properties(machine_id)`
resolves a bare id (e.g. `"ultimaker2"`) against this folder; pass a full `.json` path
instead if you need a profile outside this folder. No material-property JSON files
exist yet (referenced by `kuka.py`'s `estimate_material_requirements` as a
runtime-supplied GH input path, not a repo file).

## 4. What changed in the most recent refactor pass (context for future work)

A phased consistency/deduplication refactor addressing six goals (validation tooling
in `iolib`, "libs shouldn't overlap," G-code generalization, KUKA material estimation,
elevating duplicated script logic, and an analysis of folder-per-function
restructuring) was scoped into 6 phases; **phases 0–3 are complete**, phases 4–5 are
not yet started. Completed work, briefly:

- **Phase 0**: removed two accidental dead imports from `curvelib.py`.
- **Phase 1**: built `iolib.py`'s validation module and migrated
  `geometrylib.validate_input`, `kukalib`'s percent/tool/base checks,
  `srflib.remap_rgb_channels`, `printlib.slice_brep_uniform`, and
  `curvelib.rounded_rectangle`/`divide_crv_equal` to use it, with unchanged public
  signatures.
- **Phase 2**: removed the byte-identical `curveselfintersection` duplicate between
  `curvelib.py`/`printlib.py`; fixed a real bug in `geometrylib.minmaxcaplist` (was
  clamping the same value for every list item); added `geometrylib.bbox_bounds` and
  deduplicated three independent bbox-corner-extraction call sites; fixed a real bug
  in `srflib.build_gradient_pattern` (was returning `2N` items instead of `N`) while
  extracting a shared `_gradient_factor` helper; fixed `kukalib.write_file` checking
  the wrong variable's length before truncating filenames.
- **Phase 3**: rebuilt `gcodelib.py` from a mostly-unused, partially-broken class
  (`save()` referenced an undefined variable; `check_print()` hardcoded a `223` buildplate
  bound instead of reading it from the machine profile, and only supported rectangular
  builds) into the shared module described in §3.1, then rewrote `ultimaker.py`,
  `ultimaker_speeds.py`, `wasp_delta.py`, and `generic_gcode.py` to call into it instead
  of each re-implementing header/flow/save/retraction logic independently.

**Important, explicit exclusions** (do not "fix" these without asking first):

- `ultimaker_standalone.py` is currently byte-identical to `ultimaker.py`, but it is
  a **deliberate, separate self-contained script** (per repo maintainer), not an
  accidental duplicate — do not delete or merge it.
- `ultimaker_contained.py` is a documented "self-contained experimental alpha" that
  intentionally reimplements geometry/G-code helpers locally instead of importing
  `libs/` — left untouched by design.
- `printlib.gcodeline` still exists (as a one-line re-export of
  `gcodelib.gcodeline`) specifically because `ultimaker_standalone.py` depends on it;
  don't delete it while that script still calls `pl.gcodeline`.

The folder-per-function restructuring question (splitting each `libs/*.py` into a
folder with one file per function) was analyzed and **rejected** — it would conflict
with the bare-module import convention (§2.3), add IronPython import-path risk for
little benefit at this codebase's current size (largest file ~340 lines), and doesn't
improve the Rhino-only testing constraint either way. Don't propose it again without
a materially different justification (e.g. a single file growing past ~1000 lines).

## 5. Known technical debt / good next targets

If asked to continue this repo's consolidation work, these are the highest-value,
already-scoped-out targets (not yet implemented):

1. **KUKA material estimation** (`scripts/kuka.py` → `libs/kukalib.py`): move
   `estimate_print_volume`/`estimate_material_requirements` from script scope into
   `kukalib.py` as plain functions. Note while doing this: `kuka.py` calls `json.load`
   inside `estimate_material_requirements` but never imports `json` at module scope —
   this is a live `NameError` in the current script, worth fixing in the same pass.
2. **Elevate duplicated pattern-generation logic** into `libs/` (`srflib.py`/
   `curvelib.py`): the `_gradient_factor`/`_coerce_axis_value`/`evaluate_parameter`
   trio is independently reimplemented (with `gradient_stops`/`composite` modes that
   `srflib.evaluate_parameter` doesn't have yet) across `porous_triangular.py`,
   `sampled_triangles.py`, `sampled_triangles_columns.py`, and
   `sampled_freeform_triangles.py` — this is the single largest remaining duplication
   surface in the repo (~150–250 near-identical lines × 3–4 files for the
   parameter-config validation alone). `add_wall_opening`, curve-stack construction,
   and brep-to-Z-ordered-contour-rows logic are each duplicated 3–5 times across the
   same family plus `e3d_wall.py`/`e3d_wall_test.py`/`sampled_triangles_columns.py`.
3. **`e3d_wall.py` vs `e3d_wall_test.py`**: ~50% verbatim-duplicated (differ mainly in
   `build_contactlines` and its sub-helpers) — flagged as its own scoping exercise
   rather than a quick fix, since deciding what's shared vs. script-private needs a
   deliberate design pass.
4. Several scripts have local reimplementations that already exist in `libs/` and
   just need a call-site swap: `spiraliser.py` reimplements `curvelib.spiralise`
   and duplicates `curvelib.sort_curves_z`'s purpose; `img_projection.py`
   reimplements `srflib.closest_srf` verbatim (it imports `geometrylib` but never
   `srflib`); `filament_mesh.py`'s `create_rounded_rectangle` is a RhinoCommon port of
   `curvelib.rounded_rectangle` and contains ~60 lines of dead/unreachable code after
   its first `return` statement.

## 6. Guidelines for coding agents working in this repo

### 6.1 Before touching anything

- **Grep before you rename or delete.** Any function in `libs/` may be called from
  any script via the bare-import convention — a rename that looks locally complete
  can still break a script you didn't open. Search all of `scripts/` (not just the
  ones you're actively editing) for the old name before finalizing.
- **A file that "looks like" a duplicate might not be one.** `ultimaker_standalone.py`
  looked byte-identical to `ultimaker.py` and was still deliberately excluded from
  consolidation per the maintainer. When you find apparent duplication, especially in
  a script whose docstring calls it "experimental," "alpha," or "self-contained,"
  **ask before deleting/merging** rather than assuming it's dead weight.
- **Preserve GH component input/output variable names exactly.** These are the
  contract with an external `.gh` file this repo doesn't contain. Internal helper
  function names can be freely renamed; top-level script globals that GH reads/writes
  cannot.

### 6.2 While making changes

- **Preserve exact numeric/string output, not just "equivalent" logic**, when
  refactoring G-code or KRL generation. This codebase has real quirks that look like
  bugs but are existing, load-bearing behavior — e.g. one script's main loop emits a
  `G1` move with no extrusion value on alternate branches instead of the `G0` you'd
  expect from a "travel move" reading of the surrounding code. When consolidating such
  logic into a shared helper, either verify the helper reproduces the exact original
  call shape, or call the low-level primitive (`gcodeline(...)`) directly rather than
  forcing the code through a higher-level convenience wrapper (`travel_move`/
  `print_move`) that doesn't fit the specific case. Getting this wrong changes the
  physical machine's motion/extrusion behavior, not just code style.
- **Keep the `libs/` dependency graph a DAG.** Check what a target file already
  imports before adding a new cross-import; `gcodelib.py` and `kukalib.py` are
  currently leaf-adjacent (only depend on `geometrylib`/`iolib`) — don't have them
  import `printlib`/`curvelib` back.
- **New validation should go through `iolib`'s helpers**
  (`is_number`, `require_list`/`require_nonempty_list`, `validate_scalar`,
  `validate_type`, `ValidationError`, `report_issue`) rather than hand-rolling a new
  `isinstance`/`raise ValueError` pattern — this repo had 6+ independently
  reimplemented versions of the same numeric-range check before the Phase 1 cleanup;
  don't reintroduce a seventh.
- **`report_issue(message, level, component=None)`** is the standard way to emit a GH
  warning/error with a headless-safe fallback (prints instead of calling
  `ghenv.Component.AddRuntimeMessage` when no `component` is passed). Use it — and its
  `component=` parameter — instead of hardcoding `ghenv.Component.AddRuntimeMessage(...)`
  or a bare global `ghenv` reference inside library code (library code should not
  assume it's running inside a live GH component; scripts calling into it can pass
  `ghenv.Component` explicitly).
- **Match the surrounding file's style** (quote choice, `return(x)` vs `return x`,
  f-string vs `.format()`) rather than reformatting unrelated lines in a file you're
  touching for one specific change — these files have inconsistent style baked in
  from years of incremental edits, and an unrelated reformat makes the diff harder to
  review.

### 6.3 Verifying your work

- Run `python -m py_compile <every touched file>` as a baseline — always, for every
  change, including changes to `iolib.py`.
- For pure-math/pure-string logic changes, write and run a small standalone smoke
  test (copy just the function bodies you changed into a scratch script, assert
  expected outputs, delete the scratch script afterward) before declaring the change
  correct. Don't rely on "it looks right" for anything with a formula, an off-by-one
  risk, or a branch condition.
- **State plainly what you could and couldn't verify.** Anything touching curve/
  surface/mesh geometry, GH component messaging, or the exact byte content of a saved
  `.gcode`/`.src` file needs a manual Grasshopper run to confirm — you very likely
  cannot do this yourself. Say so explicitly rather than reporting the task as fully
  verified. When a change alters *output values* (not just code location — e.g. a bug
  fix that changes what a function returns for the same input), call that out
  specifically as something the next manual GH test needs to check, since it's easy
  to miss in a large diff.
- If your change is broad (many files, a new shared module, several rewritten
  scripts), prefer landing it in small, independently-revertible phases over one
  large commit — this repo has no CI safety net, so the ability to bisect by hand
  matters more here than in a tested codebase.

### 6.4 Scope discipline

- This repo has accumulated substantial uncommitted/in-progress work at any given
  time (check `git status` before starting — don't assume a clean tree). Only touch
  files relevant to the requested task; don't opportunistically "clean up" adjacent
  files you notice are messy unless asked.
- When a task is large enough to plausibly need phasing (e.g. "deduplicate the
  pattern-generation scripts," §5 item 2), confirm scope and sequencing with the user
  before generating a large diff — the six-phase structure used for the `gcodelib`
  consolidation (§4) is a reasonable template: land the shared library changes,
  verify, then migrate call sites script by script.
