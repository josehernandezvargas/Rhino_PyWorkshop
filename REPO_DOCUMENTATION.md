# Rhino_PyWorkshop: Repository Documentation for Academic Reporting

## 1. Repository Purpose
`Rhino_PyWorkshop` is a Python-based digital design and fabrication toolchain built for Rhino/Grasshopper environments. It connects:
- geometric design generation,
- computational pattern control,
- toolpath preparation,
- machine-oriented export (G-code and KUKA KRL),
- and fabrication-aware validation.

The repository is structured as a reusable framework rather than a single script, combining modular libraries (`libs/`) and fabrication-oriented workflows (`scripts/`).

## 2. Digital Toolchain Capabilities

### 2.1 Design-to-Toolpath Pipeline
The codebase supports a full computational workflow:
1. Parametric geometry definition (curves, contours, offsets, stacks).
2. Pattern modulation (truss, porous, triangular, thermal, spiral).
3. Data-driven control (surface/image sampling, gradient and alternating pattern logic).
4. Path conditioning (centering, leveling, ordering by layer/height).
5. Machine export for additive fabrication and robotic execution.

### 2.2 Fabrication Output Targets
The repository generates machine-ready outputs for:
- FFF/FDM printers (`scripts/ultimaker.py`, `scripts/ultimaker_speeds.py`, `scripts/wasp_delta.py`, `scripts/generic_gcode.py`).
- Industrial robot trajectories in KUKA KRL (`scripts/kuka.py`, `libs/kukalib.py`).

This demonstrates interoperability across desktop additive manufacturing and robotic fabrication contexts.

### 2.3 Process-Aware Logic
Implemented scripts include process-oriented checks and controls such as:
- build-volume and printability checks,
- curve ordering and non-planar warnings,
- parameter validation with runtime feedback in Grasshopper,
- machine profiles loaded from JSON (`machine_settings/`).

These features position the repository as a practical digital fabrication tool, not only a geometry generator.

## 3. Library and Script Ecosystem

### 3.1 Core Libraries (`libs/`)
- `geometrylib.py`: interpolation, remapping, normalization, orientation conversion, input validation.
- `curvelib.py`: curve operations, self-intersection logic, division, offset and spiral utilities.
- `printlib.py`: print-path preparation, G-code line formatting, slicing and stacking helpers.
- `gcodelib.py`: machine-aware G-code object with headers, comments and bounds extraction.
- `kukalib.py`: KUKA KRL program builder with safety-oriented motion parameter checks.
- `srflib.py`: image/surface sampling and data-driven pattern construction.
- `iolib.py`: CSV/XLSX data IO for pipeline integration and reporting workflows.

### 3.2 Applied Scripts (`scripts/`)
- Pattern generation: `thermal_patterns.py`, `e3d_wall_test.py`, `porous_*`, `sampled_porous_pattern.py`.
- Geometry transformation: `spiraliser.py`, `filament_mesh.py`, `adaptive_slicing.py`.
- Export workflows: `ultimaker.py`, `ultimaker_speeds.py`, `wasp_delta.py`, `generic_gcode.py`, `kuka.py`.
- Image-driven modulation: `img_projection.py`, porous sampling workflows.

This separation shows sound software architecture: reusable computation in libraries, fabrication scenarios in scripts.

## 4. Main Points to Highlight in an Academic Report

1. **End-to-end digital fabrication workflow**: the repository spans from computational design to machine-level code export.
2. **Modular architecture**: shared libraries reduce duplication and enable reproducible experimentation across scripts.
3. **Data-driven design methods**: image/surface sampling and parameter remapping are used to encode material and geometric behavior.
4. **Multi-platform manufacturing**: one codebase targets both FDM printers and robotic KUKA systems.
5. **Fabrication-aware validation**: runtime checks and machine constraints are embedded in script logic.
6. **Research prototyping capacity**: multiple pattern families (porous, thermal, truss, spiral) support comparative studies.
7. **Configurability and reproducibility**: machine settings are externalized in JSON and key parameters are exposed through Grasshopper inputs.

## 5. Programming Skills Demonstrated

1. **Computational geometry programming** using RhinoCommon/`rhinoscriptsyntax`.
2. **Parametric algorithm design** for path generation, layering, and pattern logic.
3. **Digital fabrication programming** through custom G-code and KRL generation pipelines.
4. **Software modularization** via reusable helper libraries and script-level orchestration.
5. **Input validation and defensive coding** with explicit warnings/errors in Grasshopper runtime.
6. **Machine abstraction** by separating machine profiles and export logic from geometry logic.
7. **Data integration skills** (CSV/XLSX + image sampling) for hybrid design workflows.
8. **Versioned technical scripting practice** with script metadata, structured parameters, and clear function boundaries.

## 6. Suggested Evidence for the Report
To strengthen academic reporting, include:
- one pipeline diagram (design input -> script processing -> machine output),
- comparative outputs from at least two scripts (e.g., thermal vs porous),
- example G-code and KRL excerpts,
- machine compatibility table (Ultimaker, Wasp, KUKA),
- validation examples (warnings/errors and how they prevent failures),
- metrics: path length, layer count, estimated material, and execution/print time.

## 7. Summary
`Rhino_PyWorkshop` demonstrates a mature digital design-to-fabrication workflow with clear evidence of advanced programming skills in computational geometry, toolpath engineering, machine-code generation, and robust script architecture for research and prototyping in digital fabrication.
