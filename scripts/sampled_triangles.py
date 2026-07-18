#! python3

"""Grasshopper scaffold for surface-guided porous triangular openings.

This script keeps the opening geometry from `porous_triangular.py`, but moves
the control logic to a guide surface. The guide can be sampled in two ways:

1. Procedural gradients:
   Parameters are derived from normalized guide-surface UV or world XYZ.
2. Encoded RGB texture:
   Parameters are remapped from the sampled image channels, as in
   `sampled_porous_pattern.py`.

Approach:
    1. Build a stacked scaffold of divided copies of `base_crv`.
    2. Reuse the original U/V seed logic to decide candidate opening points.
    3. Sample the guide surface at each candidate point.
    4. Convert guide values into opening width / depth / density parameters.
    5. Propagate tapered widths upward and emit the same four-point triangle
       opening profile used by the original porous triangle script.

Inputs:
    base_crv: Base curve to copy and divide.
    guide_srf: Surface used to evaluate gradients and/or texture sampling.
    img: Optional texture image path. Required only for image_* modes.
    layers: Number of stacked layers (int >= 1).
    layer_height: Z spacing between layers.
    div_dist: Target division distance along each curve.
    num_u: Base U periodicity for seed openings.
    num_v: Base V periodicity for seed rows.
    steps: Default propagation depth in layers.
    opening_width: Default opening width.
    angle: Opening angle in degrees.
    displacement: Opening displacement normal to the wall.

Optional GH inputs:
    param_config: Dict configuring surface-driven parameters.
    gate_threshold: Legacy threshold input kept for backward compatibility.
    density: Normalized 0..1 global multiplier for appearance density.
    u_shift, v_shift, u_scale, v_scale, v_reverse, opening_scale:
        Same modulation inputs as the existing triangular script.

Outputs:
    a: Grasshopper tree of polyline point rows.
    b: Preview geometry showing the sampled width gradient.
    c: Seed points used to generate the openings.
"""
__author__ = "joseh"
__version__ = "2026.03.13"


import math
import random

import Grasshopper as gh
import Rhino
import geometrylib as gl
import rhinoscriptsyntax as rs
from ghpythonlib import treehelpers as th
from srflib import sample_surface_color
from System.Drawing import Color


def _get_param(name, default):
    return globals().get(name, default)


def _stable_noise(seed_value, row_idx, t_norm):
    """Return a deterministic pseudo-random value in [0, 1)."""
    raw = math.sin((row_idx + 1) * 12.9898 + t_norm * 78.233 + seed_value * 37.719)
    return raw - math.floor(raw)


def _add_runtime_message(level, message):
    """Emit a Grasshopper runtime message when available."""
    try:
        ghenv.Component.AddRuntimeMessage(level, message)
    except Exception:
        print(message)


def _is_number(value):
    """Return True for numeric values except booleans."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _normalize_param_mode(mode):
    """Map public param_config aliases to the internal evaluator modes."""
    image_aliases = {
        "image.r": "image_r",
        "image.g": "image_g",
        "image.b": "image_b",
        "image.red": "image_r",
        "image.green": "image_g",
        "image.blue": "image_b",
    }
    return image_aliases.get(mode, mode)


def _coerce_required(name, value, expected_type, coercer=None):
    """Validate a required Grasshopper input using local helpers where possible."""
    try:
        gl.validate_input(name, value, expected_type, ghenv.Component, coercer=coercer)
    except Exception:
        return None
    return coercer(value) if coercer is not None else value


def _append_error(errors, message):
    """Collect an error and report it to the Grasshopper component."""
    errors.append(message)
    _add_runtime_message(gh.Kernel.GH_RuntimeMessageLevel.Error, message)


def _append_warning(message):
    """Report a non-fatal warning to the Grasshopper component."""
    _add_runtime_message(gh.Kernel.GH_RuntimeMessageLevel.Warning, message)


def _validate_scalar(name, value, errors, minimum=None, allow_zero=False):
    """Validate a numeric scalar and optionally enforce a lower bound."""
    if not _is_number(value):
        _append_error(errors, "{} must be a number.".format(name))
        return None
    value = float(value)
    if minimum is not None:
        if allow_zero:
            is_invalid = value < minimum
        else:
            is_invalid = value <= minimum
        if is_invalid:
            comparator = ">=" if allow_zero else ">"
            _append_error(errors, "{} must be {} {}.".format(name, comparator, minimum))
            return None
    return value


def _validate_param_spec(name, spec, errors):
    """Validate one param_config entry."""
    if not isinstance(spec, (list, tuple)) or len(spec) != 2:
        _append_error(errors, "param_config['{}'] must be a (mode, settings) pair.".format(name))
        return None
    mode, settings = spec
    mode = _normalize_param_mode(mode)
    if mode == "fixed":
        if not _is_number(settings):
            _append_error(errors, "param_config['{}'] fixed mode requires a number.".format(name))
            return None
    elif mode in ("image_r", "image_g", "image_b"):
        if not isinstance(settings, (list, tuple)) or len(settings) != 2:
            _append_error(errors, "param_config['{}'] image mode requires a (min, max) tuple.".format(name))
            return None
        if not all(_is_number(v) for v in settings):
            _append_error(errors, "param_config['{}'] image range values must be numeric.".format(name))
            return None
    elif mode == "gradient":
        if not isinstance(settings, dict):
            _append_error(errors, "param_config['{}'] gradient mode requires a dict.".format(name))
            return None
        if "range" not in settings:
            _append_error(errors, "param_config['{}'] gradient settings require 'range'.".format(name))
            return None
        rng = settings["range"]
        if not isinstance(rng, (list, tuple)) or len(rng) != 2 or not all(_is_number(v) for v in rng):
            _append_error(errors, "param_config['{}'] gradient 'range' must be a numeric pair.".format(name))
            return None
        try:
            _gradient_factor(settings.get("mode", "linear"), 0.5)
        except ValueError as exc:
            _append_error(errors, str(exc))
            return None
        try:
            _coerce_axis_value(settings.get("axis", "u"), (0.0, 0.0, 0.0), (0.5, 0.5))
        except ValueError as exc:
            _append_error(errors, str(exc))
            return None
        if "domain" in settings:
            domain = settings["domain"]
            if (
                not isinstance(domain, (list, tuple))
                or len(domain) != 2
                or not all(_is_number(v) for v in domain)
            ):
                _append_error(errors, "param_config['{}'] gradient 'domain' must be a numeric pair.".format(name))
                return None
    elif mode == "gradient_stops":
        if not isinstance(settings, dict):
            _append_error(errors, "param_config['{}'] gradient_stops mode requires a dict.".format(name))
            return None
        stops = settings.get("stops")
        if not isinstance(stops, (list, tuple)) or len(stops) < 2:
            _append_error(errors, "param_config['{}'] gradient_stops requires at least two stops.".format(name))
            return None
        prev_pos = None
        for stop in stops:
            if not isinstance(stop, (list, tuple)) or len(stop) != 2:
                _append_error(errors, "param_config['{}'] stops must be (position, value) pairs.".format(name))
                return None
            pos, value = stop
            if not _is_number(pos) or not _is_number(value):
                _append_error(errors, "param_config['{}'] stop values must be numeric.".format(name))
                return None
            if not 0.0 <= float(pos) <= 1.0:
                _append_error(errors, "param_config['{}'] stop positions must be within 0..1.".format(name))
                return None
            if prev_pos is not None and float(pos) < prev_pos:
                _append_error(errors, "param_config['{}'] stops must be sorted by position.".format(name))
                return None
            prev_pos = float(pos)
        try:
            _coerce_axis_value(settings.get("axis", "u"), (0.0, 0.0, 0.0), (0.5, 0.5))
        except ValueError as exc:
            _append_error(errors, str(exc))
            return None
        if "scale" in settings and not _is_number(settings["scale"]):
            _append_error(errors, "param_config['{}'] gradient_stops 'scale' must be numeric.".format(name))
            return None
    elif mode == "composite":
        if not isinstance(settings, dict):
            _append_error(errors, "param_config['{}'] composite mode requires a dict.".format(name))
            return None
        operation = settings.get("operation", "multiply")
        if operation not in ("multiply", "add", "max", "min"):
            _append_error(
                errors,
                "param_config['{}'] composite operation must be one of multiply/add/max/min.".format(name),
            )
            return None
        sources = settings.get("sources")
        if not isinstance(sources, (list, tuple)) or len(sources) < 2:
            _append_error(errors, "param_config['{}'] composite mode requires at least two sources.".format(name))
            return None
        normalized_sources = []
        for idx, source in enumerate(sources):
            if not isinstance(source, (list, tuple)) or len(source) != 2:
                _append_error(
                    errors,
                    "param_config['{}'] composite source {} must be a (mode, settings) pair.".format(name, idx),
                )
                return None
            normalized_source = _validate_param_spec("{} source {}".format(name, idx), source, errors)
            if normalized_source is None:
                return None
            normalized_sources.append(normalized_source)
        if "scale" in settings and not _is_number(settings["scale"]):
            _append_error(errors, "param_config['{}'] composite 'scale' must be numeric.".format(name))
            return None
        settings = dict(settings)
        settings["sources"] = normalized_sources
    else:
        _append_error(errors, "Unsupported parameter mode: {}".format(mode))
        return None
    return (mode, settings)


def _validate_param_config(param_config, img, errors, default_steps):
    """Validate the optional param_config dictionary."""
    if not isinstance(param_config, dict):
        _append_error(errors, "param_config must be a dictionary when provided.")
        return None

    validated = {}
    for key in ("width", "depth"):
        if key not in param_config:
            _append_error(errors, "param_config is missing '{}'.".format(key))
            continue
        validated[key] = _validate_param_spec(key, param_config[key], errors)
    if "steps" in param_config:
        validated["steps"] = _validate_param_spec("steps", param_config["steps"], errors)
    else:
        validated["steps"] = ("fixed", float(default_steps if default_steps is not None else 1.0))
    if "density" in param_config:
        validated["density"] = _validate_param_spec("density", param_config["density"], errors)
    else:
        validated["density"] = ("fixed", 1.0)

    image_modes = ("image_r", "image_g", "image_b")
    if any(
        isinstance(validated.get(key), (list, tuple)) and validated[key][0] in image_modes
        for key in validated
    ) and not img:
        _append_error(errors, "img is required when param_config uses image_* modes.")

    return validated if not errors else None


def _validate_inputs():
    """Validate Grasshopper inputs and return normalized values."""
    errors = []

    base_crv_value = _coerce_required("base_crv", _get_param("base_crv", None), object, rs.coercecurve)
    guide_srf_raw = _get_param("guide_srf", None)
    guide_srf_value = rs.coercesurface(guide_srf_raw) if guide_srf_raw is not None else None

    img_value = _get_param("img", None)
    layers_value = _validate_scalar("layers", _get_param("layers", None), errors, minimum=0)
    layer_height_value = _validate_scalar("layer_height", _get_param("layer_height", None), errors)
    div_dist_value = _validate_scalar("div_dist", _get_param("div_dist", None), errors, minimum=0)
    num_u_value = _validate_scalar("num_u", _get_param("num_u", None), errors, minimum=0)
    num_v_value = _validate_scalar("num_v", _get_param("num_v", None), errors, minimum=0)
    steps_value = _validate_scalar("steps", _get_param("steps", None), errors, minimum=0)
    opening_width_value = _validate_scalar("opening_width", _get_param("opening_width", None), errors, minimum=0, allow_zero=True)
    angle_value = _validate_scalar("angle", _get_param("angle", None), errors)
    displacement_value = _validate_scalar("displacement", _get_param("displacement", None), errors)

    if base_crv_value is None:
        _append_error(errors, "base_crv is required and must be a valid curve.")
    if guide_srf_raw is not None and guide_srf_value is None:
        _append_error(errors, "guide_srf must be a valid surface when provided.")

    if angle_value is not None and abs(math.cos(math.radians(angle_value))) < 1e-9:
        _append_error(errors, "angle must not be 90 + k*180 degrees because displacement compensation would divide by zero.")

    u_shift_value = _validate_scalar("u_shift", _get_param("u_shift", 0.0), errors)
    v_shift_value = _validate_scalar("v_shift", _get_param("v_shift", 0.0), errors)
    u_scale_value = _validate_scalar("u_scale", _get_param("u_scale", 1.0), errors)
    v_scale_value = _validate_scalar("v_scale", _get_param("v_scale", 1.0), errors)
    opening_scale_value = _validate_scalar("opening_scale", _get_param("opening_scale", 1.0), errors, minimum=0, allow_zero=True)
    gate_threshold_value = _validate_scalar("gate_threshold", _get_param("gate_threshold", 0.5), errors)
    seed_row_start_value = _validate_scalar("seed_row_start", _get_param("seed_row_start", 5), errors, minimum=0, allow_zero=True)
    density_input = _get_param("density", _get_param("density_slider", 1.0))
    density_value = _validate_scalar("density", density_input, errors, minimum=0, allow_zero=True)
    random_seed_value = _validate_scalar("random_seed", _get_param("random_seed", 1.0), errors, minimum=0, allow_zero=True)
    v_reverse_value = bool(_get_param("v_reverse", False))

    if gate_threshold_value is not None:
        gate_threshold_value = gl.minmaxcap(0.0, 1.0, gate_threshold_value)
    if density_value is not None:
        density_value = gl.minmaxcap(0.0, 1.0, density_value)

    if layers_value is not None:
        layers_value = int(layers_value)
    if num_u_value is not None:
        num_u_value = int(num_u_value)
    if num_v_value is not None:
        num_v_value = int(num_v_value)
    if steps_value is not None:
        steps_value = int(round(steps_value))
    if seed_row_start_value is not None:
        seed_row_start_value = int(seed_row_start_value)

    param_config_value = _get_param("param_config", None)
    if param_config_value is not None:
        param_config_value = _validate_param_config(param_config_value, img_value, errors, steps_value)

    if (
        opening_width_value is not None
        and div_dist_value is not None
        and opening_width_value > div_dist_value
        and param_config_value is not None
        and param_config_value["width"][0] == "fixed"
    ):
        _append_warning(
            "opening_width is larger than div_dist; clamping fixed opening_width to div_dist for seed spacing."
        )
        opening_width_value = div_dist_value
        param_config_value["width"] = ("fixed", float(opening_width_value) * opening_scale_value)

    if errors:
        return None

    return {
        "base_crv": base_crv_value,
        "guide_srf": guide_srf_value,
        "img": img_value,
        "layers": layers_value,
        "layer_height": layer_height_value,
        "div_dist": div_dist_value,
        "num_u": num_u_value,
        "num_v": num_v_value,
        "steps": steps_value,
        "opening_width": opening_width_value,
        "angle": angle_value,
        "displacement": displacement_value,
        "u_shift": u_shift_value,
        "v_shift": v_shift_value,
        "u_scale": u_scale_value,
        "v_scale": v_scale_value,
        "v_reverse": v_reverse_value,
        "opening_scale": opening_scale_value,
        "gate_threshold": gate_threshold_value,
        "seed_row_start": seed_row_start_value,
        "density": density_value,
        "random_seed": int(random_seed_value) if random_seed_value is not None else 1,
        "param_config": param_config_value,
    }


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


def _sample_guide_data(pt, guide_srf, img):
    """Sample normalized UV and optional RGB data at the closest point."""
    uv = rs.SurfaceClosestPoint(guide_srf, pt)
    u_dom = rs.SurfaceDomain(guide_srf, 0)
    v_dom = rs.SurfaceDomain(guide_srf, 1)
    u_norm = gl.invlerp(u_dom[0], u_dom[1], uv[0])
    v_norm = gl.invlerp(v_dom[0], v_dom[1], uv[1])
    rgb = sample_surface_color(pt, guide_srf, img) if img else None
    return (u_norm, v_norm), rgb


def evaluate_parameter(mode, settings, pt, uv, rgb):
    """Evaluate one scalar parameter from fixed, image, or gradient sources."""
    mode = _normalize_param_mode(mode)

    if mode == "fixed":
        return float(settings)

    if mode in ("image_r", "image_g", "image_b"):
        if rgb is None:
            raise ValueError("{} requires an image input".format(mode))
        channel_idx = {"image_r": 0, "image_g": 1, "image_b": 2}[mode]
        return gl.remap(0, 255, settings[0], settings[1], rgb[channel_idx])

    if mode == "gradient":
        axis = settings.get("axis", "u")
        coord, default_domain = _coerce_axis_value(axis, pt, uv)
        domain = settings.get("domain", default_domain)
        if domain is None:
            raise ValueError(
                "Gradient settings for axis '{}' require a domain".format(axis)
            )
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
            raise ValueError(
                "Gradient stop settings for axis '{}' require a domain".format(axis)
            )
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
            evaluate_parameter(source_mode, source_settings, pt, uv, rgb)
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


def add_wall_opening(pt, base_crv, width, angle, disp, disp_width=None):
    """Create the four-point triangular opening profile from the original script."""
    pt_param = rs.CurveClosestPoint(base_crv, pt)
    tangent = rs.VectorUnitize(rs.CurveTangent(base_crv, pt_param))
    normal = rs.VectorRotate(tangent, 90 - angle, [0, 0, 1])
    inner_width = width if disp_width is None else max(float(disp_width), float(width))
    pt_before = rs.PointAdd(pt, rs.VectorScale(tangent, -width / 2.0))
    pt_after = rs.PointAdd(pt, rs.VectorScale(tangent, width / 2.0))
    pt_before_inner = rs.PointAdd(pt, rs.VectorScale(tangent, -inner_width / 2.0))
    pt_after_inner = rs.PointAdd(pt, rs.VectorScale(tangent, inner_width / 2.0))
    disp_vec = rs.VectorScale(normal, disp)
    pt_before_disp = rs.PointAdd(pt_before_inner, disp_vec)
    pt_after_disp = rs.PointAdd(pt_after_inner, disp_vec)
    return (pt_before, pt_before_disp, pt_after_disp, pt_after)


def build_curve_stack(base_crv, layers, layer_height, div_dist):
    """Create copied curves and evenly distributed sample points for each layer."""
    point_stack = []
    curve_stack = []
    curve_length = rs.CurveLength(base_crv)
    num_divs = max(2, int(curve_length / float(div_dist)) + 1)

    for layer_idx in range(int(layers)):
        curve = rs.CopyObject(base_crv, [0, 0, layer_idx * layer_height])
        domain = rs.CurveDomain(curve)
        params = [
            gl.lerp(domain[0], domain[1], float(i) / float(num_divs - 1))
            for i in range(num_divs)
        ]
        points = [rs.EvaluateCurve(curve, t) for t in params]
        point_stack.append(points)
        curve_stack.append(curve)

    return point_stack, curve_stack


def build_regular_seed_stack(
    point_stack,
    num_u,
    num_v,
    step_count,
    u_shift,
    v_shift,
    v_scale,
    v_reverse,
    seed_row_start,
):
    """Regular scaffold-aligned sampling used when no guide surface is connected."""
    row_count = len(point_stack)
    opening_rows = set()
    bottom_margin_layers = max(2, int(seed_row_start))
    v_cursor = bottom_margin_layers

    while v_cursor < row_count:
        v_inc = int((v_scale - 1.0) * v_cursor)
        local_v = max(0, num_v + v_inc)
        if _has_vertical_clearance(v_cursor, step_count, row_count, bottom_margin_layers):
            opening_rows.add(v_cursor)
        v_cursor += max(1, int(step_count)) + local_v

    seed_stack = []
    if abs(u_shift) <= 1.0:
        u_phase_steps = int(round(u_shift * max(1, num_u - 1))) % num_u
    else:
        u_phase_steps = int(round(u_shift)) % num_u

    for v, row in enumerate(point_stack):
        v_idx = (row_count - 1 - v) if v_reverse else v
        v_idx_shifted = max(0, min(row_count - 1, v_idx + int(v_shift)))
        is_open_row = v_idx_shifted in opening_rows
        bool_row = []
        for u, _pt in enumerate(row):
            is_endpoint = u == 0 or u == len(row) - 1
            is_seed = ((u - 1 + u_phase_steps) % num_u == 0) if not is_endpoint else False
            bool_row.append(1 if (is_open_row and is_seed) else 0)
        seed_stack.append(bool_row)

    return seed_stack


def _curve_param_from_normalized(curve, t_norm):
    domain = rs.CurveDomain(curve)
    return gl.lerp(domain[0], domain[1], t_norm)


def _has_vertical_clearance(row_idx, step_count, row_count, bottom_margin_layers=2):
    """Return True when an opening fits within the stack and avoids bottom rows."""
    step_count = max(1, int(step_count))
    bottom_margin_layers = max(0, int(bottom_margin_layers))
    if row_idx < bottom_margin_layers:
        return False
    return row_idx + step_count <= row_count


def _has_edge_clearance(curve, curve_param, opening_width):
    """Return True when an opening center is far enough from the curve ends."""
    curve_length = rs.CurveLength(curve)
    if curve_length is None:
        return False
    arc_length = rs.CurveLength(curve, sub_domain=(rs.CurveDomain(curve)[0], curve_param))
    if arc_length is None:
        return False
    margin = max(0.0, float(opening_width))
    return margin <= arc_length <= max(0.0, curve_length - margin)


def build_random_seed_data(
    curve_stack,
    guide_srf,
    img,
    param_config,
    density,
    div_dist,
    random_seed,
):
    """Generate random non-overlapping seeds along curves using guide-driven density."""
    if guide_srf is None:
        return []

    rng = random.Random(int(random_seed))
    seeds = []
    all_candidates = []
    candidate_count_factor = 3
    row_count = len(curve_stack)
    bottom_margin_layers = 2

    for row_idx, curve in enumerate(curve_stack):
        curve_length = rs.CurveLength(curve)
        candidate_count = max(
            1,
            int((curve_length / float(div_dist)) * candidate_count_factor * max(0.15, density)),
        )
        row_candidates = []

        for _ in range(candidate_count):
            t_norm = rng.random()
            curve_param = _curve_param_from_normalized(curve, t_norm)
            pt = rs.EvaluateCurve(curve, curve_param)
            uv, rgb = _sample_guide_data(pt, guide_srf, img)

            width_value = max(
                0.0,
                float(
                    evaluate_parameter(
                        param_config["width"][0],
                        param_config["width"][1],
                        pt,
                        uv,
                        rgb,
                    )
                ),
            )
            if not _has_edge_clearance(curve, curve_param, width_value):
                continue
            depth_value = max(
                0.0,
                float(
                    evaluate_parameter(
                        param_config["depth"][0],
                        param_config["depth"][1],
                        pt,
                        uv,
                        rgb,
                    )
                ),
            )
            step_value = max(
                1,
                int(
                    round(
                        evaluate_parameter(
                            param_config["steps"][0],
                            param_config["steps"][1],
                            pt,
                            uv,
                            rgb,
                        )
                    )
                ),
            )
            if not _has_vertical_clearance(row_idx, step_value, row_count, bottom_margin_layers):
                continue
            density_value = gl.minmaxcap(
                0.0,
                1.0,
                float(
                    evaluate_parameter(
                        param_config["density"][0],
                        param_config["density"][1],
                        pt,
                        uv,
                        rgb,
                    )
                ),
            )
            local_density = gl.minmaxcap(0.0, 1.0, density * density_value)
            if local_density <= 0.0:
                continue
            if _stable_noise(int(random_seed), row_idx, t_norm) > local_density:
                continue

            # Low-density areas should also become sparser, not merely smaller.
            spacing_factor = gl.lerp(3.0, 1.0, local_density)
            min_u_spacing = max(width_value, float(div_dist) * spacing_factor)
            min_v_spacing = max(
                step_value,
                int(round(gl.lerp(step_value + 3.0, float(step_value), local_density))),
            )

            row_candidates.append(
                {
                    "row": row_idx,
                    "param": curve_param,
                    "arc": t_norm * curve_length,
                    "width": width_value,
                    "depth": depth_value,
                    "steps": step_value,
                    "local_density": local_density,
                    "min_u_spacing": min_u_spacing,
                    "min_v_spacing": min_v_spacing,
                    "point": pt,
                    "score": _stable_noise(int(random_seed) + 11, row_idx, t_norm),
                }
            )

        all_candidates.extend(row_candidates)
        row_candidates.sort(key=lambda item: item["score"])
        for candidate in row_candidates:
            intersects = False
            for other in seeds:
                same_band = abs(candidate["row"] - other["row"]) < max(
                    candidate["min_v_spacing"], other["min_v_spacing"]
                )
                close_u = abs(candidate["arc"] - other["arc"]) < max(
                    candidate["min_u_spacing"], other["min_u_spacing"]
                )
                if same_band and close_u:
                    intersects = True
                    break
            if not intersects:
                seeds.append(candidate)

    if not seeds and all_candidates:
        seeds.append(
            max(
                all_candidates,
                key=lambda item: (item["local_density"], -item["score"]),
            )
        )

    return seeds


def build_opening_widths(curve_stack, point_stack, seed_stack, guide_srf, img, param_config):
    """Sample the guide surface at seed points and propagate opening widths upward."""
    openings = [[(0.0, 0.0) for _ in row] for row in seed_stack]

    for v, row in enumerate(point_stack):
        curve = curve_stack[v]
        for u, pt in enumerate(row):
            if not seed_stack[v][u]:
                continue

            if guide_srf is None:
                uv, rgb = (0.0, 0.0), None
            else:
                uv, rgb = _sample_guide_data(pt, guide_srf, img)

            width = evaluate_parameter(
                param_config["width"][0],
                param_config["width"][1],
                pt,
                uv,
                rgb,
            )
            local_steps = max(
                1,
                int(
                    round(
                        evaluate_parameter(
                            param_config["steps"][0],
                            param_config["steps"][1],
                            pt,
                            uv,
                            rgb,
                        )
                    )
                ),
            )
            width = max(0.0, float(width))
            pt_param = rs.CurveClosestPoint(curve, pt)
            if not _has_edge_clearance(curve, pt_param, width):
                continue

            for n in range(local_steps):
                row_idx = v + n
                if row_idx >= len(openings):
                    break
                taper_t = float(n) / float(local_steps)
                propagated = gl.lerp(width, 0.0, taper_t)
                existing_width, existing_disp_width = openings[row_idx][u]
                openings[row_idx][u] = (
                    max(existing_width, propagated),
                    max(existing_disp_width, width),
                )

    return openings


def build_seed_points_from_stack(point_stack, seed_stack):
    """Collect the active scaffold seed points."""
    seed_points = []
    for row_idx, row in enumerate(point_stack):
        for col_idx, pt in enumerate(row):
            if seed_stack[row_idx][col_idx]:
                seed_points.append(pt)
    return seed_points


def build_opening_events(curve_stack, random_seeds):
    """Propagate random seeds into per-row opening events."""
    event_stack = [[] for _ in curve_stack]

    for seed in random_seeds:
        for n in range(seed["steps"]):
            row_idx = seed["row"] + n
            if row_idx >= len(event_stack):
                break
            taper_t = float(n) / float(seed["steps"])
            propagated = gl.lerp(seed["width"], 0.0, taper_t)
            if propagated <= 0.0:
                continue
            event_stack[row_idx].append((seed["param"], propagated, seed["width"], seed["depth"]))

    return event_stack


def build_seed_points_from_random_seeds(random_seeds):
    """Collect accepted random seed points."""
    return [seed["point"] for seed in random_seeds]


def build_output_points(point_stack, curve_stack, openings, angle, displacement):
    """Apply the propagated widths and emit a tree-ready point stack."""
    output = []
    disp_comp = displacement / math.cos(math.radians(angle))

    for row_idx, row in enumerate(point_stack):
        row_pts = []
        for col_idx, pt in enumerate(row):
            width, disp_width = openings[row_idx][col_idx]
            if width > 0.0:
                row_pts.extend(
                    add_wall_opening(
                        pt,
                        curve_stack[row_idx],
                        width,
                        angle,
                        disp_comp,
                        disp_width,
                    )
                )
            else:
                row_pts.append(pt)
        output.append(row_pts)

    return output


def build_output_points_from_events(point_stack, curve_stack, event_stack, angle, displacement):
    """Emit output rows by merging base contour points with opening events."""
    output = []

    for row_idx, row in enumerate(point_stack):
        curve = curve_stack[row_idx]
        domain = rs.CurveDomain(curve)
        row_items = [(domain[0], 0, row[0])]
        for event_param, width, disp_width, depth in event_stack[row_idx]:
            event_pt = rs.EvaluateCurve(curve, event_param)
            row_items.append((event_param, 1, (event_pt, width, disp_width, depth)))

        row_items.append((domain[1], 0, row[-1]))
        row_items.sort(key=lambda item: (item[0], item[1]))

        row_pts = []
        for _param, item_type, payload in row_items:
            if item_type == 0:
                row_pts.append(payload)
            else:
                event_pt, width, disp_width, depth = payload
                disp_comp = depth / math.cos(math.radians(angle))
                row_pts.extend(
                    add_wall_opening(
                        event_pt,
                        curve,
                        width,
                        angle,
                        disp_comp,
                        disp_width,
                    )
                )
        output.append(row_pts)

    return output


def build_closed_output(base_crv, output, layers, layer_height, displacement):
    """Close each layer with a copied back-side curve, as in the original script."""
    closed_layers = []
    back_side = rs.CopyObject(base_crv, [0, displacement + 18, 0])

    for layer_idx in range(int(layers)):
        back_side_z = rs.CopyObject(back_side, [0, 0, layer_idx * layer_height])
        layer_pts = list(output[layer_idx])
        layer_pts.extend(list(rs.CurvePoints(back_side_z))[::-1])
        closed_layers.append(layer_pts[::-1])

    return closed_layers


def build_gradient_preview(guide_srf, img, param_config, opening_width, opening_scale, sample_count=24):
    """Build a preview mesh for the active guide field on guide_srf."""
    if guide_srf is None:
        return None

    width_mode, width_settings = param_config["width"]
    if opening_width <= 0:
        return None

    mesh = Rhino.Geometry.Mesh()
    u_dom = rs.SurfaceDomain(guide_srf, 0)
    v_dom = rs.SurfaceDomain(guide_srf, 1)
    base_width = max(1e-9, float(opening_width) * float(opening_scale))

    for v_idx in range(sample_count + 1):
        v_t = float(v_idx) / float(sample_count)
        v = gl.lerp(v_dom[0], v_dom[1], v_t)
        for u_idx in range(sample_count + 1):
            u_t = float(u_idx) / float(sample_count)
            u = gl.lerp(u_dom[0], u_dom[1], u_t)
            pt = rs.EvaluateSurface(guide_srf, u, v)
            uv = (u_t, v_t)
            rgb = sample_surface_color(pt, guide_srf, img) if img else None

            if rgb is not None:
                mesh_color = Color.FromArgb(int(rgb[0]), int(rgb[1]), int(rgb[2]))
            else:
                if width_mode == "fixed":
                    width_value = float(width_settings)
                else:
                    width_value = evaluate_parameter(width_mode, width_settings, pt, uv, rgb)

                factor = gl.minmaxcap(0.2, 1.0, float(width_value) / base_width)
                gray = int(round(gl.remap(0.2, 1.0, 51, 255, factor)))
                mesh_color = Color.FromArgb(gray, gray, gray)

            mesh.Vertices.Add(rs.coerce3dpoint(pt))
            mesh.VertexColors.Add(mesh_color)

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


# param_config = {
#     "width": ("image.r", (30, 80)),
#     "steps": ("image.g", (3, 6)),
#     "density": ("image.b", (0.2, 1.0)),
#     "depth": ("fixed", 100),
# }

# param_config = {
#     "width": ("image.r", (10.0, 45.0)),
#     "depth": ("image.g", (12.0, 60.0)),
#     "density": ("image.b", (0.05, 1.0)),
#     "steps": ("fixed", 6.0),
# }

# param_config = {
#     "width": ("image.r", (20.0, 90.0)),
#     "depth": ("image.g", (30.0, 120.0)),
#     "density": ("image.b", (0.15, 0.85)),
#     "steps": ("fixed", 8.0),
# }

# param_config = {
#     "width": ("image.r", (15.0, 55.0)),
#     "depth": ("fixed", 80.0),
#     "density": ("image.b", (0.3, 1.0)),
#     "steps": ("image.g", (2.0, 7.0)),
# }


validated = _validate_inputs()

if validated is None:
    a = th.list_to_tree([])
    b = []
    c = []
    C = c
else:
    point_stack, curve_stack = build_curve_stack(
        validated["base_crv"],
        validated["layers"],
        validated["layer_height"],
        validated["div_dist"],
    )
    if validated["guide_srf"] is None or validated["param_config"] is None:
        seed_stack = build_regular_seed_stack(
            point_stack,
            validated["num_u"],
            validated["num_v"],
            validated["steps"],
            validated["u_shift"],
            validated["v_shift"],
            validated["v_scale"],
            validated["v_reverse"],
            validated["seed_row_start"],
        )
        openings = build_opening_widths(
            curve_stack,
            point_stack,
            seed_stack,
            None,
            validated["img"],
            {
                "width": ("fixed", validated["opening_width"] * validated["opening_scale"]),
                "steps": ("fixed", validated["steps"]),
                "density": ("fixed", 1.0),
                "depth": ("fixed", validated["displacement"]),
            },
        )
        output = build_output_points(
            point_stack,
            curve_stack,
            openings,
            validated["angle"],
            validated["displacement"],
        )
        c = build_seed_points_from_stack(point_stack, seed_stack)
    else:
        random_seeds = build_random_seed_data(
            curve_stack,
            validated["guide_srf"],
            validated["img"],
            validated["param_config"],
            validated["density"],
            validated["div_dist"],
            validated["random_seed"],
        )
        event_stack = build_opening_events(curve_stack, random_seeds)
        output = build_output_points_from_events(
            point_stack,
            curve_stack,
            event_stack,
            validated["angle"],
            validated["displacement"],
        )
        c = build_seed_points_from_random_seeds(random_seeds)
    closed_output = build_closed_output(
        validated["base_crv"],
        output,
        validated["layers"],
        validated["layer_height"],
        validated["displacement"],
    )
    b = (
        build_gradient_preview(
            validated["guide_srf"],
            validated["img"],
            validated["param_config"],
            validated["opening_width"],
            validated["opening_scale"],
        )
        if validated["guide_srf"] is not None and validated["param_config"] is not None
        else []
    )

    a = th.list_to_tree(closed_output)
    C = c
