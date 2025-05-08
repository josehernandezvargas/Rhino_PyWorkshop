#! python
"""
Grasshopper G‑code exporter 
=============================

Inputs
------
* **crv** *(required)* : GUID, Curve, or list thereof.
* **tol** *(optional float)* : max segment length for tessellation (default 0.05).
* **filename** *(optional string)* : base name (no extension, default "export").
* **z_offset** *(optional float)* : added to every Z coordinate (default 0.0).
* **run** *(optional bool)* : regenerate outputs **a**, **b** when **True** (default True).
* **save** *(optional bool)* : write file when **True** (default False).

Outputs
-------
* **a** : list of G‑code lines.
* **b** : list matching **a**; `(x, y, z)` tuples for moves, `(0.0, 0.0, 0.0)` for `;Layer` lines.

Requirements
------------
"""

import rhinoscriptsyntax as rs
import printlib as pl
import geometrylib as gl
import os


# --- Defaults & coerce ---
curves   = crv if isinstance(crv, list) else [crv]
tol      = tol if 'tol' in globals() else 0.05
name     = filename if 'filename' in globals() and filename else 'export'
z_offset = z_offset if 'z_offset' in globals() else 0.0
run_btn  = run if 'run' in globals() else True
save_btn = save if 'save' in globals() else False

a = []  # gcode lines
b = []  # point tuples or placeholder for comments

# --- Helpers ---
def purge_duplicates(pts, tol_dup=1e-6):
    unique, last = [], None
    for p in pts:
        if last is None or rs.Distance(p, last) > tol_dup:
            unique.append(p)
            last = p
    return unique

def purge_collinear(pts, tol_col=1e-6):
    if len(pts) < 3:
        return pts
    cleaned = [pts[0]]
    for i in range(1, len(pts)-1):
        prev, curr, nxt = pts[i-1], pts[i], pts[i+1]
        v1 = rs.VectorCreate(curr, prev)
        v2 = rs.VectorCreate(nxt, curr)
        if rs.VectorLength(rs.VectorCrossProduct(v1, v2)) > tol_col:
            cleaned.append(curr)
    cleaned.append(pts[-1])
    return cleaned

# --- Main generation ---
if run_btn:
    all_pts = []
    for c in curves:
        cid = rs.coercecurve(c)
        if not cid:
            print(f"Invalid input: {c}")
            continue
        if rs.IsPolyline(cid):
            pts = rs.PolylineVertices(cid)
        else:
            length = rs.CurveLength(cid)
            segments = max(int(length / tol), 1)
            pts = rs.DivideCurve(cid, segments)
        pts = purge_duplicates(pts)
        pts = purge_collinear(pts)
        all_pts.extend(pts)

    E = 0.0
    prev = None
    current_z = None
    layer = 0
    for pt in all_pts:
        x, y, z0 = pt.X, pt.Y, pt.Z
        z1 = z0 + z_offset
        # Insert layer comment when Z changes beyond half tolerance
        if current_z is None or abs(z1 - current_z) > tol/2:
            layer += 1
            a.append(f";Layer {layer}")
            b.append((0.0, 0.0, 0.0))  # placeholder for layer comment
            current_z = z1
        if prev:
            E += rs.Distance(prev, pt)
        line = pl.gcodeline(
            1,
            x=round(x,1),
            y=round(y,1),
            z=round(z1,1),
            e=round(E,1)
        )
        a.append(line)
        b.append((round(x,1), round(y,1), round(z1,1)))
        prev = pt
    print(f"Generated {len(a)} lines in {layer} layers.")

# --- Saving ---
if save_btn and a:
    ts = gl.timestamp()
    gh_dir = os.path.dirname(os.path.realpath(ghdoc.Path))
    fp = os.path.join(gh_dir, f"{ts}_{name}.gcode")
    try:
        with open(fp, 'w') as f:
            for L in a:
                f.write(L + "\n")
        print(f"Saved to {fp}")
    except Exception as e:
        print(f"Save failed: {e}")

# outputs: a, b
