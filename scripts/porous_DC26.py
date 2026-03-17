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
nozzle = 20.0

def _get_param(name, default):
    return globals().get(name, default)


def _optional_width(value):
    """Normalize optional GH width inputs: non-positive values disable override."""
    if value is None:
        return None
    try:
        value = float(value)
    except Exception:
        return None
    return value if value > 0.0 else None

u_bias = float(_get_param("u_bias", 1.0))
# v_bias = float(_get_param("v_bias", 1.5))
u_reverse = bool(_get_param("u_reverse", False))
# v_reverse = bool(_get_param("v_reverse", False))
u_shift = float(_get_param("u_shift", 0.0))
# v_shift = float(_get_param("v_shift", 0.0))
u_scale = float(_get_param("u_scale", 1.0))
# v_scale = float(_get_param("v_scale", 1.0))
width_front_input = _optional_width(_get_param("width_front", None))
width_back_input = _optional_width(_get_param("width_back", None))
width_back_reduction = float(_get_param("width_back_reduction", 0.0))
width_back_reduction = max(0.0, min(1.0, width_back_reduction))

if num_u <= 0 or num_v <= 0:
    raise ValueError("num_u and num_v must be positive integers")
if step_count <= 0:
    raise ValueError("steps must be a positive integer")



def add_wall_opening(
    pt,
    base_crv,
    width,
    angle,
    disp,
    reference_tangent=None,
    width_front=None,
    width_back=None,
):
    """Create a four-point opening profile around a point on a curve.

    The point is projected to the closest curve parameter, then the curve
    tangent and a rotated normal (around +Z) define the opening direction.

    Args:
        pt: Base point to open around.
        base_crv: Curve used to compute tangent/normal directions.
        width: Default opening width measured along the curve tangent.
        angle: Rotation angle (degrees) from the tangent to the normal.
        disp: Offset distance along the rotated normal.
        reference_tangent: Optional tangent used to keep orientation stable.
        width_front: Optional front width for (pt_before, pt_after).
        width_back: Optional back width for (pt_before_disp, pt_after_disp).

    Returns:
        Tuple of points: (before, before_disp, after_disp, after).
    """
    pt_param = rs.CurveClosestPoint(base_crv, pt)
    tangent = rs.VectorUnitize(rs.CurveTangent(base_crv, pt_param))
    if reference_tangent and rs.VectorDotProduct(tangent, reference_tangent) < 0:
        tangent = rs.VectorReverse(tangent)
    normal = rs.VectorRotate(tangent, 90 - angle, [0, 0, 1])
    w_front = width if width_front is None else float(width_front)
    w_back = w_front if width_back is None else float(width_back)

    pt_before = rs.PointAdd(pt, rs.VectorScale(tangent, -w_front / 2.0))
    pt_after = rs.PointAdd(pt, rs.VectorScale(tangent, w_front / 2.0))
    pt_before_back = rs.PointAdd(pt, rs.VectorScale(tangent, -w_back / 2.0))
    pt_after_back = rs.PointAdd(pt, rs.VectorScale(tangent, w_back / 2.0))
    disp_vec = rs.VectorScale(normal, disp)
    pt_before_disp = rs.PointAdd(pt_before_back, disp_vec)
    pt_after_disp = rs.PointAdd(pt_after_back, disp_vec)
    return (pt_before, pt_before_disp, pt_after_disp, pt_after)


base_crv_tangent = rs.VectorUnitize(rs.VectorCreate(rs.CurveStartPoint(base_crv), rs.CurveEndPoint(base_crv)))
base_crv_normal = rs.VectorRotate(base_crv_tangent, -90 - angle, [0, 0, 1])
back_base_crv = rs.CopyObject(base_crv, base_crv_normal * displacement)
displacement_distance = float(displacement)


# === BEGIN FRONT_BACK_STACK_INTERVENTION ===
def _build_stacks(seed_crv):
    point_stack = []
    param_stack = []
    curve_stack = []
    num_divs = max(2, int(rs.CurveLength(seed_crv) / div_dist) + 1)
    division_module = 1.0 / (num_divs - 1)
    div_params = [i * division_module for i in range(num_divs)]
    for l in range(int(layers)):
        curve = rs.CopyObject(seed_crv, [0, 0, l * layer_height])
        points = [rs.EvaluateCurve(curve, rs.CurveParameter(curve, t)) for t in div_params]
        point_stack.append(points)
        param_stack.append(div_params)
        curve_stack.append(curve)
    return point_stack, param_stack, curve_stack

point_stack_front, param_stack_front, curve_stack_front = _build_stacks(base_crv)
point_stack_back, param_stack_back, curve_stack_back = _build_stacks(back_base_crv)
# === END FRONT_BACK_STACK_INTERVENTION ===

# Set opening pattern with the same dimensions as point_stack.
bool_stack = []
row_count = len(point_stack_front)
opening_rows = set()
v_cursor = 5
while v_cursor < row_count:
    v_inc = int((v_scale - 1) * v_cursor)
    local_v = max(1, num_v + v_inc)
    opening_rows.add(v_cursor)
    v_cursor += local_v

for v, row in enumerate(point_stack_front):
    bool_row = []
    # Support both normalized (0..1) and direct step values for u_shift.
    if abs(u_shift) <= 1.0:
        u_phase_steps = int(round(u_shift * max(1, num_u - 1))) % num_u
    else:
        u_phase_steps = int(round(u_shift)) % num_u

    # Uniform u/v periods
    v_idx = (row_count - 1 - v) if v_reverse else v
    v_idx_shifted = max(0, min(row_count - 1, v_idx + int(v_shift)))
    is_open_v = v_idx_shifted in opening_rows
    for u, point in enumerate(row):
        if u == 0 or u == len(row) - 1:
            bool_row.append(0)
        # Use interior index so endpoint clamping does not phase-shift apertures.
        elif (((u - 1) + u_phase_steps) % num_u == 0) and is_open_v:
            bool_row.append(1)
        else:
            bool_row.append(0)
    bool_stack.append(bool_row)

# === BEGIN FRONT_BACK_COMPLEMENT_BOOL_INTERVENTION ===
bool_stack_front = []
bool_stack_back = []
open_row_counter_split = 0
for i, row in enumerate(bool_stack):
    row_front = []
    row_back = []
    row_has_open = any(row)
    row_flip = open_row_counter_split % 2
    open_in_row = 0
    for j, is_open in enumerate(row):
        if is_open:
            # Alternate front/back across openings in the same row, then
            # reverse that order on the next open row (brick-like split).
            use_front = ((open_in_row + row_flip) % 2 == 0)
            row_front.append(1 if use_front else 0)
            row_back.append(0 if use_front else 1)
            open_in_row += 1
        else:
            row_front.append(0)
            row_back.append(0)
    bool_stack_front.append(row_front)
    bool_stack_back.append(row_back)
    if row_has_open:
        open_row_counter_split += 1
# === END FRONT_BACK_COMPLEMENT_BOOL_INTERVENTION ===

# === BEGIN FRONT_BACK_WIDTH_PROPAGATION_INTERVENTION ===
def _propagate_widths(bool_src, back_reduction=0.0):
    # Per opening store widths for:
    # (pt_before/pt_after, pt_before_disp/pt_after_disp)
    openings = [[(0.0, 0.0) for _ in row] for row in bool_src]
    back_reduction = max(0.0, min(1.0, float(back_reduction)))
    base_step_count = step_count
    for v, row in enumerate(bool_src):
        v_idx = (row_count - 1 - v) if v_reverse else v
        v_idx_shifted = max(0, min(row_count - 1, v_idx + int(v_shift)))
        v_inc = int((v_scale - 1) * v_idx_shifted)
        local_v = max(1, num_v + v_inc)
        local_step_count = max(1, int(base_step_count * (local_v / max(1, num_v))))
        for u, is_opening in enumerate(row):
            if is_opening:
                for n in range(local_step_count):
                    if v + n >= len(openings):
                        break
                    t = n / local_step_count
                    w = gl.remap(0, len(row), opening_width * opening_scale, opening_width, u)
                    w_front = gl.lerp(w, 0, t)
                    # 0: keep base width across propagated steps, 1: match front reduction.
                    w_back = gl.lerp(w, w_front, back_reduction)
                    openings[v + n][u] = (w_front, w_back)
    return openings

openings_front = _propagate_widths(bool_stack_front, width_back_reduction)
openings_back = _propagate_widths(bool_stack_back, width_back_reduction)
# === END FRONT_BACK_WIDTH_PROPAGATION_INTERVENTION ===

# === BEGIN FRONT_BACK_OUTPUT_INTERVENTION ===
output_front = []
output_back = []
disp_comp = (displacement_distance - nozzle * 0.8) / math.cos(math.radians(angle)) 

for i, (row_front, row_back) in enumerate(zip(point_stack_front, point_stack_back)):
    pts_front = []
    pts_back = []
    for j, (pt_front, pt_back) in enumerate(zip(row_front, row_back)):
        ref_param = rs.CurveClosestPoint(curve_stack_front[i], pt_front)
        ref_tangent = rs.VectorUnitize(rs.CurveTangent(curve_stack_front[i], ref_param))

        width_pair_front = openings_front[i][j]
        width_front = width_pair_front[0]
        width_back_profile = width_pair_front[1]
        if width_front:
            newpts_front = add_wall_opening(
                pt_front,
                curve_stack_front[i],
                width_front,
                angle,
                disp_comp,
                reference_tangent=ref_tangent,
                width_front=width_front_input,
                width_back=(width_back_input if width_back_input is not None else width_back_profile),
            )
            pts_front.extend(newpts_front)
        else:
            pts_front.append(pt_front)

        width_pair_back = openings_back[i][j]
        width_back = width_pair_back[0]
        width_back_profile = width_pair_back[1]
        if width_back:
            newpts_back = add_wall_opening(
                pt_back,
                curve_stack_back[i],
                width_back,
                angle,
                -disp_comp,
                reference_tangent=ref_tangent,
                width_front=width_front_input,
                width_back=(width_back_input if width_back_input is not None else width_back_profile),
            )
            pts_back.extend(newpts_back)
        else:
            pts_back.append(pt_back)

    output_front.append(pts_front)
    output_back.append(pts_back)

closed_layers = []
for l in range(int(layers)):
    closed_layers.append(list(output_front[l]) + list(output_back[l])[::-1])

a = th.list_to_tree(closed_layers)
# === END FRONT_BACK_OUTPUT_INTERVENTION ===
