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
    b: List of color-coded preview meshes for the active surface gradients.
"""
__author__ = "joseh"
__version__ = "2026.03.17"


import Rhino
import rhinoscriptsyntax as rs
import curvelib as cl
import geometrylib as gl
from ghpythonlib import treehelpers as th
import math
from System.Drawing import Color

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
# guide_srf: optional surface used to drive opening widths and density.
# param_config: optional dict. Defaults to fixed values matching the inputs.
# Optional modulation params:
# u_bias: 0..2 (1.0 = linear; <1 eases down, >1 eases up).
# v_bias: unused (layer height is constant).
# u_reverse/v_reverse: bool.
# u_shift: 0..1 phase shift.
# v_shift: int offset in rows (positive or negative).
# seed_row_start: first layer index allowed to contain openings.
# u_scale/v_scale: > 0 (1.0 = neutral).

num_u = int(num_u)
num_v = int(num_v)
step_count = int(steps)

def _get_param(name, default):
    return globals().get(name, default)

u_bias = float(_get_param("u_bias", 1.0))
# v_bias = float(_get_param("v_bias", 1.5))
u_reverse = bool(_get_param("u_reverse", False))
v_reverse = bool(_get_param("v_reverse", False))
u_shift = float(_get_param("u_shift", 0.0))
v_shift = float(_get_param("v_shift", 0.0))
seed_row_start = int(_get_param("seed_row_start", 0))
u_scale = float(_get_param("u_scale", 1.0))
v_scale = float(_get_param("v_scale", 1.0))
opening_scale = float(_get_param("opening_scale", 1.0))

if num_u <= 0 or num_v <= 0:
    raise ValueError("num_u and num_v must be positive integers")
if step_count <= 0:
    raise ValueError("steps must be a positive integer")



def _gradient_factor(mode, t):
    """Return a normalized gradient factor for a supported profile."""
    t = max(0.0, min(1.0, float(t)))
    if mode == "linear":
        return t
    if mode == "reverse":
        return 1.0 - t
    if mode == "peak":
        return 2.0 * t if t <= 0.5 else 2.0 * (1.0 - t)
    if mode == "valley":
        return 1.0 - (2.0 * t if t <= 0.5 else 2.0 * (1.0 - t))
    if mode == "smoothpeak":
        return abs(math.sin(t * math.pi))
    if mode == "smoothvalley":
        return 1.0 - abs(math.sin(t * math.pi))
    raise ValueError("Unsupported gradient mode: {}".format(mode))


def _coerce_axis_value(axis, pt, uv):
    """Resolve a gradient axis against normalized UV or world coordinates."""
    if axis in ("u", "U"):
        return uv[0], (0.0, 1.0)
    if axis in ("v", "V"):
        return uv[1], (0.0, 1.0)
    if axis in ("x", "X", 0):
        return pt[0], None
    if axis in ("y", "Y", 1):
        return pt[1], None
    if axis in ("z", "Z", 2):
        return pt[2], None
    raise ValueError("Unsupported gradient axis: {}".format(axis))


def _sample_guide_data(pt, guide_srf):
    """Sample normalized UV coordinates at the closest point on the guide surface."""
    uv = rs.SurfaceClosestPoint(guide_srf, pt)
    u_dom = rs.SurfaceDomain(guide_srf, 0)
    v_dom = rs.SurfaceDomain(guide_srf, 1)
    u_norm = gl.invlerp(u_dom[0], u_dom[1], uv[0])
    v_norm = gl.invlerp(v_dom[0], v_dom[1], uv[1])
    return (u_norm, v_norm)


def _stable_noise(row_idx, col_idx, uv):
    """Return a deterministic pseudo-random value in [0, 1)."""
    raw = math.sin(
        (row_idx + 1) * 12.9898
        + (col_idx + 1) * 78.233
        + uv[0] * 37.719
        + uv[1] * 19.913
    )
    return raw - math.floor(raw)


def evaluate_parameter(mode, settings, pt, uv):
    """Evaluate one scalar parameter from fixed or procedural gradient sources."""
    if mode == "fixed":
        return float(settings)

    if mode == "gradient":
        axis = settings.get("axis", "u")
        coord, default_domain = _coerce_axis_value(axis, pt, uv)
        domain = settings.get("domain", default_domain)
        if domain is None:
            raise ValueError("Gradient settings for axis '{}' require a domain".format(axis))
        factor = _gradient_factor(
            settings.get("mode", "linear"),
            gl.invlerp(domain[0], domain[1], coord),
        )
        rng = settings["range"]
        return gl.lerp(rng[0], rng[1], factor)

    if mode == "gradient_stops":
        axis = settings.get("axis", "u")
        coord, default_domain = _coerce_axis_value(axis, pt, uv)
        domain = settings.get("domain", default_domain)
        if domain is None:
            raise ValueError("Gradient stop settings for axis '{}' require a domain".format(axis))
        t = gl.minmaxcap(0.0, 1.0, gl.invlerp(domain[0], domain[1], coord))
        stops = settings["stops"]
        if t <= stops[0][0]:
            value = float(stops[0][1])
        elif t >= stops[-1][0]:
            value = float(stops[-1][1])
        else:
            value = float(stops[-1][1])
            for idx in range(len(stops) - 1):
                p0, v0 = stops[idx]
                p1, v1 = stops[idx + 1]
                if p0 <= t <= p1:
                    local_t = 0.0 if p1 == p0 else gl.invlerp(p0, p1, t)
                    value = gl.lerp(v0, v1, local_t)
                    break
        return value * float(settings.get("scale", 1.0))

    if mode == "composite":
        values = [
            evaluate_parameter(source_mode, source_settings, pt, uv)
            for source_mode, source_settings in settings["sources"]
        ]
        operation = settings.get("operation", "multiply")
        if operation == "multiply":
            result = 1.0
            for value in values:
                result *= value
        elif operation == "add":
            result = sum(values)
        elif operation == "max":
            result = max(values)
        else:
            result = min(values)
        return result * float(settings.get("scale", 1.0))

    raise ValueError("Unsupported parameter mode: {}".format(mode))


def _coerce_preview_color(settings, fallback):
    """Read an optional preview color from settings."""
    color_value = settings.get("preview_color", fallback) if isinstance(settings, dict) else fallback
    if isinstance(color_value, Color):
        return color_value
    if isinstance(color_value, (list, tuple)) and len(color_value) == 3:
        return Color.FromArgb(int(color_value[0]), int(color_value[1]), int(color_value[2]))
    return Color.FromArgb(int(fallback[0]), int(fallback[1]), int(fallback[2]))


def _build_colored_gradient_preview(guide_srf, spec, value_range, preview_color, sample_count=24):
    """Build a color-coded mesh preview for one gradient specification."""
    if guide_srf is None or spec is None:
        return None

    mode, settings = spec
    if mode == "fixed":
        return None

    mesh = Rhino.Geometry.Mesh()
    u_dom = rs.SurfaceDomain(guide_srf, 0)
    v_dom = rs.SurfaceDomain(guide_srf, 1)
    min_value, max_value = value_range
    span = max(1e-9, float(max_value) - float(min_value))

    for v_idx in range(sample_count + 1):
        v_t = float(v_idx) / float(sample_count)
        v = gl.lerp(v_dom[0], v_dom[1], v_t)
        for u_idx in range(sample_count + 1):
            u_t = float(u_idx) / float(sample_count)
            u = gl.lerp(u_dom[0], u_dom[1], u_t)
            pt = rs.EvaluateSurface(guide_srf, u, v)
            uv = (u_t, v_t)
            value = float(evaluate_parameter(mode, settings, pt, uv))
            factor = gl.minmaxcap(0.15, 1.0, (value - min_value) / span)
            red = int(round(preview_color.R * factor))
            green = int(round(preview_color.G * factor))
            blue = int(round(preview_color.B * factor))
            mesh.Vertices.Add(rs.coerce3dpoint(pt))
            mesh.VertexColors.Add(Color.FromArgb(red, green, blue))

    row_size = sample_count + 1
    for v_idx in range(sample_count):
        for u_idx in range(sample_count):
            a_idx = v_idx * row_size + u_idx
            b_idx = a_idx + 1
            c_idx = a_idx + row_size + 1
            d_idx = a_idx + row_size
            mesh.Faces.AddFace(a_idx, b_idx, c_idx, d_idx)

    mesh.Normals.ComputeNormals()
    mesh.Compact()
    return mesh


def _collect_preview_specs(param_name, spec, fallback_color, previews):
    """Collect one preview spec for each active gradient source."""
    if spec is None:
        return
    mode, settings = spec
    if mode == "fixed":
        return
    if mode == "composite":
        sources = settings.get("sources", [])
        for idx, source in enumerate(sources):
            source_color = _coerce_preview_color(
                source[1] if isinstance(source[1], dict) else {},
                fallback_color[idx % len(fallback_color)],
            )
            _collect_preview_specs(param_name, source, [source_color], previews)
        return

    preview_color = _coerce_preview_color(
        settings if isinstance(settings, dict) else {},
        fallback_color[0],
    )
    previews.append((param_name, spec, preview_color))


def build_gradient_previews(guide_srf, param_config, width_default):
    """Build preview meshes for all active non-fixed parameter gradients."""
    if guide_srf is None or not isinstance(param_config, dict):
        return []

    preview_specs = []
    _collect_preview_specs("width", param_config.get("width"), [(255, 120, 80), (255, 200, 90)], preview_specs)
    _collect_preview_specs("density", param_config.get("density"), [(80, 170, 255), (80, 230, 180)], preview_specs)
    _collect_preview_specs("steps", param_config.get("steps"), [(220, 110, 255), (180, 110, 255)], preview_specs)

    previews = []
    for param_name, spec, preview_color in preview_specs:
        if param_name == "width":
            value_range = (0.0, max(1.0, width_default))
        elif param_name == "density":
            value_range = (0.0, 1.0)
        else:
            value_range = (1.0, max(2.0, float(step_count)))
        mesh = _build_colored_gradient_preview(guide_srf, spec, value_range, preview_color)
        if mesh is not None:
            previews.append(mesh)
    return previews


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


def _build_stack(seed_crv):
    """Create evenly sampled point rows for each copied layer curve."""
    point_stack = []
    param_stack = []
    curve_stack = []
    num_divs = max(2, int(rs.CurveLength(seed_crv) / div_dist) + 1)
    division_module = 1.0 / float(num_divs - 1)
    div_params = [i * division_module for i in range(num_divs)]

    for layer_idx in range(int(layers)):
        curve = rs.CopyObject(seed_crv, [0, 0, layer_idx * layer_height])
        domain = rs.CurveDomain(curve)
        curve_params = [gl.lerp(domain[0], domain[1], t) for t in div_params]
        points = [rs.EvaluateCurve(curve, t) for t in curve_params]
        point_stack.append(points)
        param_stack.append(curve_params)
        curve_stack.append(curve)

    return point_stack, param_stack, curve_stack


default_param_config = {
    "width": ("fixed", float(opening_width) * float(opening_scale)),
    "density": ("fixed", 1.0),
    "steps": ("fixed", float(step_count)),
}

# # Gradient example:
# default_param_config = {
#     "width": (
#         "gradient",
#         {
#             "axis": "v",
#             "mode": "linear",
#             "range": (20.0, 90.0),
#             "preview_color": (255, 140, 90),
#         },
#     ),
#     "density": ("fixed", 1.0),
#     "steps": ("fixed", float(step_count)),
# }

# # Example with different gradients per parameter.
# default_param_config = {
#     "width": (
#         "gradient_stops",
#         {
#             "axis": "v",
#             "stops": [(0.0, 20.0), (0.5, 60.0), (1.0, 100.0)],
#             "preview_color": (255, 140, 90),
#         },
#     ),
#     "density": (
#         "gradient_stops",
#         {
#             "axis": "u",
#             "stops": [(0.0, 0.2), (0.5, 1.0), (1.0, 0.2)],
#             "preview_color": (80, 170, 255),
#         },
#     ),
#     "steps": ("fixed", float(step_count)),
# }

# Example with multiple gradients combined for one parameter.
# default_param_config = {
#     "width": (
#         "composite",
#         {
#             "operation": "multiply",
#             "sources": [
#                 (
#                     "gradient_stops",
#                     {
#                         "axis": "v",
#                         "stops": [(0.0, 20.0), (0.5, 70.0), (1.0, 100.0)],
#                         "preview_color": (255, 140, 90),
#                     },
#                 ),
#                 (
#                     "gradient_stops",
#                     {
#                         "axis": "u",
#                         "stops": [(0.0, 0.7), (0.5, 1.0), (1.0, 0.7)],
#                         "preview_color": (255, 210, 90),
#                     },
#                 ),
#             ],
#         },
#     ),
#     "density": (
#         "composite",
#         {
#             "operation": "multiply",
#             "sources": [
#                 (
#                     "gradient_stops",
#                     {
#                         "axis": "v",
#                         "stops": [(0.0, 0.15), (0.5, 0.9), (1.0, 0.3)],
#                         "preview_color": (80, 170, 255),
#                     },
#                 ),
#                 (
#                     "gradient_stops",
#                     {
#                         "axis": "u",
#                         "stops": [(0.0, 0.4), (0.5, 1.0), (1.0, 0.4)],
#                         "preview_color": (80, 230, 180),
#                     },
#                 ),
#             ],
#         },
#     ),
#     "steps": ("fixed", float(step_count)),
# }


# Create stack of curves and per-curve point divisions.
point_stack, param_stack, curve_stack = _build_stack(base_crv)
guide_srf_raw = _get_param("guide_srf", None)
guide_srf = rs.coercesurface(guide_srf_raw) if guide_srf_raw is not None else None
param_config = _get_param("param_config", default_param_config)
width_spec = default_param_config["width"]
density_spec = default_param_config["density"]
steps_spec = default_param_config["steps"]
if isinstance(param_config, dict):
    width_spec = param_config.get("width", width_spec)
    density_spec = param_config.get("density", density_spec)
    steps_spec = param_config.get("steps", steps_spec)
elif isinstance(param_config, (list, tuple)) and len(param_config) == 2:
    width_spec = param_config

# Set opening pattern with the same dimensions as point_stack.

bool_stack = []
row_count = len(point_stack)
opening_rows = set()
v_cursor = max(0, int(seed_row_start))
while v_cursor < row_count:
    v_inc = int((v_scale - 1) * v_cursor)
    local_v = max(1, num_v + v_inc)
    opening_rows.add(v_cursor)
    v_cursor += local_v

for v, row in enumerate(point_stack):
    bool_row = []
    if abs(u_shift) <= 1.0:
        u_phase_steps = int(round(u_shift * max(1, num_u - 1))) % num_u
    else:
        u_phase_steps = int(round(u_shift)) % num_u

    # Uniform u/v periods
    for u, point in enumerate(row):
        v_idx = (row_count - 1 - v) if v_reverse else v
        v_idx_shifted = max(0, min(row_count - 1, v_idx + int(v_shift)))
        is_open_v = v_idx_shifted in opening_rows
        if u == 0 or u == len(row) - 1:
            bool_row.append(0)
        elif ((u - 1 + u_phase_steps) % num_u == 0) and is_open_v:
            if guide_srf is None or density_spec is None:
                bool_row.append(1)
            else:
                uv = _sample_guide_data(point, guide_srf)
                density_value = evaluate_parameter(density_spec[0], density_spec[1], point, uv)
                density_value = gl.minmaxcap(0.0, 1.0, float(density_value))
                bool_row.append(1 if _stable_noise(v, u, uv) <= density_value else 0)
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
    for u, is_opening in enumerate(row):
        if is_opening:
            pt = point_stack[v][u]
            uv = _sample_guide_data(pt, guide_srf) if guide_srf is not None else None
            if guide_srf is None or width_spec is None:
                width_seed = float(opening_width) * float(opening_scale)
            else:
                width_seed = evaluate_parameter(width_spec[0], width_spec[1], pt, uv)
            width_seed = max(0.0, float(width_seed))
            if guide_srf is None or steps_spec is None:
                local_step_count = max(1, int(base_step_count * (local_v / max(1, num_v))))
            else:
                steps_value = evaluate_parameter(steps_spec[0], steps_spec[1], pt, uv)
                local_step_count = max(1, int(round(float(steps_value))))
            # Propagate opening upwards
            for n in range(local_step_count):
                if v + n >= len(openings):
                    break
                t = n / local_step_count
                openings[v + n][u] = max(openings[v + n][u], gl.lerp(width_seed, 0, t))

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
    output_pts = list(output[l])
    output_pts.extend((rs.CurvePoints(back_side_z))[::-1])
    closed_layers.append(output_pts)

a = th.list_to_tree(closed_layers)
b = build_gradient_previews(guide_srf, param_config, float(opening_width) * float(opening_scale))
