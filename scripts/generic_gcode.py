#! python3
"""
Grasshopper G‑code exporter
=============================

Inputs
------
* **crv** *(required)* : GUID, Curve, or list thereof.
* **tol** *(optional float)* : max segment length for tessellation (default 0.1).
* **filename** *(optional string)* : base name (no extension, default "export").
* **z_offset** *(optional float)* : added to every Z coordinate (default 0.0).
* **run** *(optional bool)* : regenerate outputs **a**, **b** when **True** (default True).
* **save** *(optional bool)* : write file when **True** (default False).

Outputs
-------
* **a** : list of G‑code lines.
* **b** : list matching **a**; `Point3d` values for moves, `Point3d.Unset` for `;Layer` lines.
* **preview** *(optional output)* : lightweight preview as a list of `Point3d` values centered near origin.

Requirements
------------
"""

import rhinoscriptsyntax as rs
import Rhino
import geometrylib as gl
import gcodelib as gcl
import os


# --- Defaults & coerce ---
curves = crv if isinstance(crv, list) else [crv]
tol = tol if 'tol' in globals() and tol is not None else 0.1
name = filename if 'filename' in globals() and filename else 'export'
z_offset = z_offset if 'z_offset' in globals() and z_offset is not None else 0.0
run_btn = run if 'run' in globals() and run is not None else True
save_btn = save if 'save' in globals() and save is not None else False
preview = []

try:
    tol = float(tol)
except (TypeError, ValueError):
    tol = 0.1
tol = max(tol, 0.1)

a = []  # gcode lines
b = []  # Rhino Point3d values or placeholder for comments

# --- Helpers ---
def make_preview_points(point_groups):
    return [pt for group in point_groups for pt in group]

def get_output_dir():
    try:
        if ghdoc and ghdoc.Path:
            return os.path.dirname(os.path.realpath(ghdoc.Path))
    except NameError:
        pass
    return os.getcwd()

# --- Main generation ---
if run_btn:
    point_groups = []
    for c in curves:
        cid = rs.coercecurve(c)
        if not cid:
            print(f"Invalid input: {c}")
            continue
        pts = gcl.curve_to_points(cid, tol)
        if not pts:
            continue
        pts = gcl.purge_duplicate_points(pts)
        pts = gcl.purge_collinear_points(pts)
        if pts:
            point_groups.append(pts)

    point_groups = gcl.center_points_to_origin(point_groups)
    preview = make_preview_points(point_groups)

    E = 0.0
    current_z = None
    layer = 0
    for pts in point_groups:
        prev = None
        for pt in pts:
            x, y, z0 = pt.X, pt.Y, pt.Z
            z1 = z0 + z_offset
            if current_z is None or abs(z1 - current_z) > tol / 2.0:
                layer += 1
                a.append(f";Layer {layer}")
                b.append(Rhino.Geometry.Point3d.Unset)
                current_z = z1
            if prev:
                E += rs.Distance(prev, pt)
            out_pt = Rhino.Geometry.Point3d(round(x, 1), round(y, 1), round(z1, 1))
            line = gcl.gcodeline(
                1,
                x=out_pt.X,
                y=out_pt.Y,
                z=out_pt.Z,
                e=round(E, 1)
            )
            a.append(line)
            b.append(out_pt)
            prev = pt
    print(f"Generated {len(a)} lines in {layer} layers.")

# --- Saving ---
if save_btn and a:
    ts = gl.timestamp()
    hourstamp = " at " + gl.timestamp(format=3)
    base_dir = get_output_dir()
    gcode_dir = os.path.join(base_dir, "gcode")
    try:
        fp = gcl.save_gcode_file(gcode_dir, name, [], a, timestamp=ts)
        print("File Saved  " + fp + hourstamp)
    except Exception as e:
        print(f"Save failed: {e}")

c = preview

# outputs: a, b, c/preview
