#! python3

"""Grasshopper script to create a stacked porous triangular pattern.

Builds a stack of copied curves along +Z, divides each curve into points,
selects opening locations with a grid pattern, and displaces those points
along +Y with a tapered width over a number of layers.

Inputs:
    base_crv: Base curve to copy and divide.
    layers: Number of stacked layers (int).
    layer_height: Z spacing between layers.
    div_dist: Division distance for each curve.
    num_u: Opening periodicity along the curve divisions (int, > 0).
    num_v: Opening periodicity along the stacked layers (int, > 0).
    steps: Number of layers to propagate each opening (int, > 0).
    opening_width: Maximum Y displacement for an opening.

Outputs:
    a: List of points (original or displaced) for all layers.
"""
__author__ = "joseh"
__version__ = "2026.01.14"


import rhinoscriptsyntax as rs
import curvelib as cl
import geometrylib as gl
from ghpythonlib import treehelpers as th
import math

# Expected parameter ranges (GH inputs):
# base_crv: Rhino curve.
# layers: int >= 1.
# layer_height: > 0.
# div_dist: > 0.
# num_u: int >= 1.
# num_v: int >= 1.
# steps: int >= 1.
# opening_width: >= 0.
# angle: degrees (0..360 recommended).
# displacement: any real value.
# Optional modulation params:
# u_bias: 0..2 (1.0 = linear; <1 eases down, >1 eases up).
# v_bias: unused (layer height is constant).
# u_reverse/v_reverse: bool.
# u_shift: 0..1 phase shift.
# v_shift: int offset in rows (positive or negative).
# u_scale/v_scale: > 0 (1.0 = neutral).

num_u = int(num_u)
num_v = int(num_v)
step_count = int(steps)

def _get_param(name, default):
    return globals().get(name, default)

u_bias = float(_get_param("u_bias", 1.0))
# v_bias = float(_get_param("v_bias", 1.5))
u_reverse = bool(_get_param("u_reverse", False))
# v_reverse = bool(_get_param("v_reverse", False))
u_shift = float(_get_param("u_shift", 0.0))
# v_shift = float(_get_param("v_shift", 0.0))
u_scale = float(_get_param("u_scale", 1.0))
# v_scale = float(_get_param("v_scale", 1.0))

if num_u <= 0 or num_v <= 0:
    raise ValueError("num_u and num_v must be positive integers")
if step_count <= 0:
    raise ValueError("steps must be a positive integer")



def add_wall_opening(pt, base_crv, width, angle, disp):
    """Create a four-point opening profile around a point on a curve.

    The point is projected to the closest curve parameter, then the curve
    tangent and a rotated normal (around +Z) define the opening direction.

    Args:
        pt: Base point to open around.
        base_crv: Curve used to compute tangent/normal directions.
        width: Opening width measured along the curve tangent.
        angle: Rotation angle (degrees) from the tangent to the normal.
        disp: Offset distance along the rotated normal.

    Returns:
        Tuple of points: (before, before_disp, after_disp, after).
    """
    pt_param = rs.CurveClosestPoint(base_crv, pt)
    tangent = rs.VectorUnitize(rs.CurveTangent(base_crv, pt_param))
    normal = rs.VectorRotate(tangent, 90 - angle, [0, 0, 1])
    pt_before = rs.CopyObject(pt, tangent * -width / 2)
    pt_after = rs.CopyObject(pt, tangent * width / 2)
    pt_before_disp = rs.CopyObject(pt_before, normal * disp)
    pt_after_disp = rs.CopyObject(pt_after, normal * disp)
    return (pt_before, pt_before_disp, pt_after_disp, pt_after)


point_stack = []
param_stack = []
curve_stack = []

# Create stack of curves and per-curve point divisions.
for l in range(int(layers)):

    curve = rs.CopyObject(base_crv, [0, 0, l * layer_height])

    # OLD: divide curve directly

    # points = cl.divide_crv_equal(curve, div_dist)
    num_divs = int(rs.CurveLength(base_crv) / div_dist )
    # print(f"numdivs: {num_divs}")
    division_module = 1/num_divs
    div_params = [division_module + l*division_module for i in range(num_divs)]
    points = [rs.EvaluateCurve(base_crv, rs.CurveParameter(base_crv, t)) for t in div_params]
    
    # HACK: shift the points along the curve direction
    rs.MoveObject(points[1:-1], rs.VectorUnitize(rs.VectorCreate(rs.CurveStartPoint(base_crv), rs.CurveEndPoint(base_crv))) * u_shift)
    point_stack.append(points) # two dimensional array of points
    param_stack.append(div_params)
    curve_stack.append(curve)

# Set opening pattern with the same dimensions as point_stack.

bool_stack = []
row_count = len(point_stack)
opening_rows = set()
v_cursor = 5
while v_cursor < row_count:
    v_inc = int((v_scale - 1) * v_cursor)
    local_v = max(1, num_v + v_inc)
    opening_rows.add(v_cursor)
    v_cursor += local_v

for v, row in enumerate(point_stack):
    bool_row = []

    # Uniform u/v periods
    for u, point in enumerate(row):
        v_idx = (row_count - 1 - v) if v_reverse else v
        v_idx_shifted = max(0, min(row_count - 1, v_idx + int(v_shift)))
        is_open_v = v_idx_shifted in opening_rows
        if u == 0 or u == len(row):
            bool_row.append(0)
        elif u % num_u == 0 and is_open_v:
            bool_row.append(1)
        else:
            bool_row.append(0)
    bool_stack.append(bool_row)

# Propagate opening widths upward with a decreasing value over step_count.
# Array with same dimensions as bool_stack with calculated widths.
openings = [[0 for _ in row] for row in bool_stack]

base_step_count = step_count

for v, row in enumerate(bool_stack):
    v_idx = (row_count - 1 - v) if v_reverse else v
    v_idx_shifted = max(0, min(row_count - 1, v_idx + int(v_shift)))
    v_inc = int((v_scale - 1) * v_idx_shifted)
    local_v = max(1, num_v + v_inc)
    local_step_count = max(1, int(base_step_count * (local_v / max(1, num_v))))
    for u, is_opening in enumerate(row):
        if is_opening:
            # Propagate opening upwards
            for n in range(local_step_count):
                if v + n >= len(openings):
                    break
                t = n / local_step_count
                w =  gl.remap(0, len(row), opening_width * opening_scale, opening_width, u) # decrease the width as u increases
                openings[v + n][u] = gl.lerp(w, 0 , t)

# Displace points using the calculated widths and build a tree output.

output = []

for i, row in enumerate(point_stack):
    row_pts = []
    for j, pt in enumerate(row):
        width =  openings[i][j] # gl.remap(0, j/len(row), 20, openings[i][j], t) openings[i][j] * j/len(row) * 2 
        if width:
            newpts = add_wall_opening(pt, curve_stack[i], width, angle, displacement / math.cos(math.radians(angle))) # compensate for angled displacement
            row_pts.extend(newpts)
        else:
            row_pts.append(pt)
    output.append(row_pts)

# Back side

back_side = rs.CopyObject(base_crv, [0 , -displacement - 18])
back_bracket = rs.AddPolyline((rs.CurveStartPoint(base_crv), rs.CurveStartPoint(back_side), rs.CurveEndPoint(back_side), rs.CurveEndPoint(base_crv)))

closed_layers = []

for l in range(int(layers)):
    back_side_z = rs.CopyObject(back_side, [0, 0, l * layer_height])
    # output_polyline = rs.AddPolyline(output[l])
    output_pts = output[l]
    output_pts.extend((rs.CurvePoints(back_side_z))[::-1])
    closed_layers.append(output_pts)

# a = closed_layers
a = th.list_to_tree(output)
