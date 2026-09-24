# Rhino_PyWorkshop

A personal collection of Python tools for Rhino 8 / Grasshopper, focused on digital
fabrication: parametric pattern generation (porous, truss and thermal walls), image- and
surface-driven parameter control, and machine-code export for desktop FFF printers
(G-code) and a KUKA robot arm (KRL).

## Requirements

- **Rhino 8** with Grasshopper. The scripts target the CPython 3.9 runtime of the
  Rhino 8 *Script* component (`#! python3` shebang). They do **not** run under the legacy
  IronPython 2.7 GHPython component: the libraries use f-strings, `from __future__ import
  annotations`, `str | Path` type hints and `pathlib`.
- Optional: `openpyxl` for the XLSX helpers in `libs/iolib.py`.

## Repository layout

| Folder | Contents |
|---|---|
| `libs/` | Shared helper modules imported by the scripts (see below). |
| `scripts/` | One file per Grasshopper Script component. Each script reads its inputs as bare globals injected by Grasshopper (`toolpath`, `nozzle`, ...) and assigns its outputs the same way (`a`, `preview`, ...). The `.gh` canvases themselves are not versioned. |
| `machine_settings/` | JSON machine/material profiles: `ultimaker2.json` (cartesian), `wasp.json` (delta) and `kuka.json` (robot bead geometry + material data). |
| `AGENTS.md` | Detailed working notes for anyone (human or coding agent) editing the code: runtime constraints, import convention, verification tiers, known debt. Read it before changing anything non-trivial. |
| `REPO_DOCUMENTATION.md` | Capability summary aimed at academic reporting. |

## Setup: making `libs/` importable

Every script imports the libraries as bare top-level modules (`import geometrylib as gl`,
`from srflib import sample_surface_color`), so the `libs/` folder must be on the Python
search path of the Rhino 8 script runtime. Either:

1. add the absolute path of `libs/` to the module search paths in the Rhino 8 Script
   Editor options (Tools > Options > Python 3), or
2. append it at run time at the top of a component, after the shebang:

   ```python
   import sys
   sys.path.append(r"C:\path\to\Rhino_PyWorkshop\libs")
   ```

After editing a library file, restart Rhino (or reset the script engine) so the cached
module is reloaded; `scripts/kuka.py` even checks for a stale `kukalib` and tells you.

## Libraries (`libs/`)

| Module | Role | Needs Rhino? |
|---|---|---|
| `iolib.py` | CSV/XLSX IO and the shared validation helpers (`ValidationError`, `report_issue`, `validate_scalar`, `validate_type`, ...). | No |
| `geometrylib.py` | Interpolation and remapping (`lerp`, `invlerp`, `remap`, `minmaxcap`), bounding boxes, angles, timestamps, pattern normalisation. | Yes |
| `curvelib.py` | Curve construction and measurement helpers (centred lines, self-intersections, spiralising, rounded rectangles, two-sided offsets). | Yes |
| `srflib.py` | Image sampling on surfaces (`sample_surface_color`, cached bitmaps), RGB remapping, gradient profiles (`GRADIENT_MODES`) and alternating zig-zag pattern stacks. | Yes |
| `printlib.py` | Print-path preparation: centring on the build plate, levelling to Z=0, uniform Brep slicing, pattern-based curve stacking, material estimation. | Yes |
| `gcodelib.py` | Machine profiles (cartesian and delta build volumes), the canonical flow formula, G-code line/header/file helpers and the stateful `GCodeLib` class. | Yes |
| `kukalib.py` | `KukaKRL`: KRL program assembly with header, PTP/LIN/LIN_REL moves, outputs, waits, local subprograms and declarations, speed safety caps and safe file writing. | No |

The libraries form a DAG: `iolib` <- `geometrylib` <- `curvelib`, `srflib`, `gcodelib`
<- `printlib`; `kukalib` only depends on `iolib`. Keep it that way.

## Scripts (`scripts/`)

**Machine export**

| Script | Purpose |
|---|---|
| `ultimaker.py` | Canonical Ultimaker 2+ (UltiGCode) exporter for a list of planar curves: validation, centring, build-volume check, optional test line, header, time estimate, save to `gcode/` next to the `.gh` file. |
| `ultimaker_speeds.py` | Variant taking a point list plus per-point flow multipliers (`PTS`, `VEL`). |
| `ultimaker_standalone.py` | Deliberately kept, self-contained twin of `ultimaker.py` (only uses `printlib.gcodeline`). |
| `ultimaker_contained.py` | Experimental alpha exporter with no library dependencies at all. |
| `wasp_delta.py` | Wasp 2040 (delta) exporter using the `GCodeLib` class; centres the part on the origin and checks the radial build volume. |
| `generic_gcode.py` | Minimal machine-agnostic G1 exporter (no header, temperatures or retraction). |
| `kuka.py` | KUKA KRL generator with per-layer subprograms, standby/resume via digital inputs, extruder control, speed caps and a material estimate from `machine_settings/kuka.json`. Output goes to `krl/` next to the `.gh` file. |

**Pattern generation**

| Script | Purpose |
|---|---|
| `porous_triangular.py`, `porous_DC26.py` | Stacked curves with grid-placed, tapered triangular openings (fixed or surface-gradient driven). |
| `sampled_triangles.py`, `sampled_triangles_columns.py`, `sampled_freeform_triangles.py` | Guide-surface / image-driven versions of the triangular openings, for flat walls, sliced columns and freeform front + planar back walls. |
| `sampled_porous_pattern.py`, `porous_test_patterns.py`, `biofab_porous_pattern.py` | Alternating zig-zag polylines whose amplitude and shift are sampled from an image mapped on a surface (the biofab variant handles polycurves and tapers near segment ends). |
| `thermal_patterns.py` | Layered truss / sine / bezier infill patterns with amplitude modulation. |
| `e3d_wall.py`, `e3d_wall_test.py` | Alternating truss between a sliced wall surface and its offset. |

**Geometry utilities**

| Script | Purpose |
|---|---|
| `spiraliser.py` | Turns a stack of planar polylines into one continuous spiral (optional fade-out lap). |
| `adaptive_slicing.py` | Computes adaptive layer heights from the overhang angle of a Brep. |
| `img_projection.py` | Displaces points along a surface normal by the luminance of a projected image. |
| `filament_mesh.py` | Sweeps a rounded-rectangle section along a curve into a mesh (filament preview). |

## Machine settings

`gcodelib.load_machine_properties("ultimaker2")` resolves a bare id against
`machine_settings/`; pass a full `.json` path to use a profile elsewhere. Cartesian
profiles declare `build_volume: {x, y, z}`, delta profiles `{r, h}`. `kuka.json` carries
`bead_width`, `layer_height`, `flow_factor`, `Density` (kg/m3), `Water ratio` (%) and
`Waste per batch (kg)` for the KUKA material estimate.

## Safety notes (KUKA)

`kukalib.KukaKRL` clamps LIN speed to 250 mm/s and PTP speed to 25 % (warning above
10 %); `kuka.py` applies the same caps before calling the library. `$OUT[3]` drives the
extruder with **inverted logic** (`FALSE` = running). Always dry-run a generated `.src`
on the controller before printing.

## Verifying changes

There is no test suite and most code only runs inside Rhino. Minimum checks before
committing:

```bash
python -m py_compile libs/*.py scripts/*.py
```

Pure logic (`iolib`, `kukalib`, `gcodelib.calculate_flow`, `srflib` gradients, ...) can be
exercised headless by putting stub `rhinoscriptsyntax`/`Rhino`/`System` modules on
`sys.path`; see `AGENTS.md` section 2.4. Anything touching geometry, Grasshopper
messages or the exact content of a saved `.gcode`/`.src` file needs a manual run of the
corresponding component in Grasshopper.
