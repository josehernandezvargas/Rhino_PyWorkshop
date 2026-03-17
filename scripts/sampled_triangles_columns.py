#! python3

"""Grasshopper scaffold for porous triangular openings on sliced columns.

This variation slices a Brep into horizontal contour curves at 10 mm intervals
and treats those contours like the base curves from `sampled_triangles.py`.

Differences from the base script:
    1. `brep` replaces `base_crv`.
    2. The Brep is sliced into horizontal contour curves using `layer_height`.
    3. Contour rows use a denser division (`base_div_dist`) than the regular
       opening spacing (`div_dist`), so rounded columns retain smoother bases.
    4. `point_mode` controls seed placement independently of whether a guide
       surface is connected: `"regular"` or `"random"`.
    5. The closing boundary is built from an outward contour offset at
       `displacement + 18`, instead of a translated back-side curve.

Inputs:
    brep: Brep to contour into 10 mm layers.
    guide_srf: Optional guide surface for gradient and/or image sampling.
    img: Optional texture image path. Required only for image_* modes.
    layer_height: Distance between contour slices.
    div_dist: Target spacing used for opening placement.
    num_u: Base U periodicity for regular seed openings.
    num_v: Base V periodicity for regular seed rows.
    steps: Default propagation depth in layers.
    opening_width: Default opening width.
    angle: Opening angle in degrees.
    displacement: Opening displacement normal to the wall.

Optional GH inputs:
    base_div_dist: Denser contour sampling distance for the base rows.
    point_mode: "regular" or "random".
    param_config: Dict configuring surface-driven parameters.
    gate_threshold: Seed opens when sampled gate value >= threshold.
    density: Normalized 0..1 control for overall appearance density.
    u_shift, v_shift, u_scale, v_scale, v_reverse, opening_scale:
        Same modulation inputs as the sampled triangle script.

Outputs:
    a: Grasshopper tree of closed polyline point rows.
    b: Preview geometry showing the sampled width gradient.
    c: Seed points used to generate the openings.
"""
__author__ = "joseh"
__version__ = "2026.03.16"


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
    raw = math.sin((row_idx + 1) * 12.9898 + t_norm * 78.233 + seed_value * 37.719)
    return raw - math.floor(raw)


def _add_runtime_message(level, message):
    try:
        ghenv.Component.AddRuntimeMessage(level, message)
    except Exception:
        print(message)


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _coerce_required(name, value, expected_type, coercer=None):
    try:
        gl.validate_input(name, value, expected_type, ghenv.Component, coercer=coercer)
    except Exception:
        return None
    return coercer(value) if coercer is not None else value


def _append_error(errors, message):
    errors.append(message)
    _add_runtime_message(gh.Kernel.GH_RuntimeMessageLevel.Error, message)


def _append_warning(message):
    _add_runtime_message(gh.Kernel.GH_RuntimeMessageLevel.Warning, message)


def _validate_scalar(name, value, errors, minimum=None, allow_zero=False):
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


def _validate_point_mode(value, errors):
    mode = str(value or "regular").strip().lower()
    if mode not in ("regular", "random"):
        _append_error(errors, "point_mode must be either 'regular' or 'random'.")
        return None
    return mode


def _validate_param_spec(name, spec, errors):
    if not isinstance(spec, (list, tuple)) or len(spec) != 2:
        _append_error(errors, "param_config['{}'] must be a (mode, settings) pair.".format(name))
        return None
    mode, settings = spec
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
            _coerce_axis_value(settings.get("axis", "u"), (0.0, 0.0, 0.0), (0.5, 0.5))
        except ValueError as exc:
            _append_error(errors, str(exc))
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
            pos, val = stop
            if not _is_number(pos) or not _is_number(val):
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
        for idx, source in enumerate(sources):
            if not isinstance(source, (list, tuple)) or len(source) != 2:
                _append_error(
                    errors,
                    "param_config['{}'] composite source {} must be a (mode, settings) pair.".format(name, idx),
                )
                return None
            if _validate_param_spec("{} source {}".format(name, idx), source, errors) is None:
                return None
        if "scale" in settings and not _is_number(settings["scale"]):
            _append_error(errors, "param_config['{}'] composite 'scale' must be numeric.".format(name))
            return None
    else:
        _append_error(errors, "Unsupported parameter mode: {}".format(mode))
        return None
    return spec


def _validate_param_config(param_config, img, errors):
    if not isinstance(param_config, dict):
        _append_error(errors, "param_config must be a dictionary when provided.")
        return None

    validated = {}
    for key in ("gate", "width", "steps"):
        if key not in param_config:
            _append_error(errors, "param_config is missing '{}'.".format(key))
            continue
        validated[key] = _validate_param_spec(key, param_config[key], errors)
    validated["density"] = _validate_param_spec(
        "density",
        param_config.get("density", ("fixed", 1.0)),
        errors,
    )

    image_modes = ("image_r", "image_g", "image_b")
    if any(
        isinstance(validated.get(key), (list, tuple)) and validated[key][0] in image_modes
        for key in validated
    ) and not img:
        _append_error(errors, "img is required when param_config uses image_* modes.")

    return validated if not errors else None


def _validate_inputs():
    errors = []

    brep_value = _coerce_required("brep", _get_param("brep", None), object, rs.coercebrep)
    guide_srf_raw = _get_param("guide_srf", None)
    guide_srf_value = rs.coercesurface(guide_srf_raw) if guide_srf_raw is not None else None

    img_value = _get_param("img", None)
    layer_height_value = _validate_scalar("layer_height", _get_param("layer_height", 10.0), errors, minimum=0)
    div_dist_value = _validate_scalar("div_dist", _get_param("div_dist", None), errors, minimum=0)
    num_u_value = _validate_scalar("num_u", _get_param("num_u", None), errors, minimum=0)
    num_v_value = _validate_scalar("num_v", _get_param("num_v", None), errors, minimum=0)
    steps_value = _validate_scalar("steps", _get_param("steps", None), errors, minimum=0)
    opening_width_value = _validate_scalar("opening_width", _get_param("opening_width", None), errors, minimum=0, allow_zero=True)
    angle_value = _validate_scalar("angle", _get_param("angle", None), errors)
    displacement_value = _validate_scalar("displacement", _get_param("displacement", None), errors)

    if brep_value is None:
        _append_error(errors, "brep is required and must be a valid Brep.")
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
    seed_row_start_value = _validate_scalar("seed_row_start", _get_param("seed_row_start", 2), errors, minimum=0, allow_zero=True)
    density_value = _validate_scalar("density", _get_param("density", _get_param("density_slider", 1.0)), errors, minimum=0, allow_zero=True)
    random_seed_value = _validate_scalar("random_seed", _get_param("random_seed", 1.0), errors, minimum=0, allow_zero=True)
    v_reverse_value = bool(_get_param("v_reverse", False))
    point_mode_value = _validate_point_mode(_get_param("point_mode", "regular"), errors)

    base_div_default = None
    if div_dist_value is not None:
        base_div_default = max(1.0, min(float(div_dist_value) * 0.5, 8.0))
    base_div_dist_value = _validate_scalar(
        "base_div_dist",
        _get_param("base_div_dist", base_div_default),
        errors,
        minimum=0,
    )

    if gate_threshold_value is not None:
        gate_threshold_value = gl.minmaxcap(0.0, 1.0, gate_threshold_value)
    if density_value is not None:
        density_value = gl.minmaxcap(0.0, 1.0, density_value)

    if num_u_value is not None:
        num_u_value = int(num_u_value)
    if num_v_value is not None:
        num_v_value = int(num_v_value)
    if steps_value is not None:
        steps_value = int(round(steps_value))
    if seed_row_start_value is not None:
        seed_row_start_value = int(seed_row_start_value)

    print(
        "sampled_triangles_columns inputs:",
        {
            "layer_height": layer_height_value,
            "div_dist": div_dist_value,
            "base_div_dist": base_div_dist_value,
            "num_u": num_u_value,
            "num_v": num_v_value,
            "steps": steps_value,
            "point_mode": point_mode_value,
        },
    )

    default_param_config = {
        "gate": ("fixed", 1.0),
        "width": ("fixed", float(opening_width_value or 0.0) * float(opening_scale_value or 1.0)),
        "steps": ("fixed", float(steps_value or 1)),
        "density": ("fixed", 1.0),
    }
    param_config_value = _get_param("param_config", default_param_config)
    param_config_value = _validate_param_config(param_config_value, img_value, errors)

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
        param_config_value["width"] = ("fixed", float(opening_width_value) * float(opening_scale_value))

    if errors:
        return None

    return {
        "brep": brep_value,
        "guide_srf": guide_srf_value,
        "img": img_value,
        "layer_height": layer_height_value,
        "div_dist": div_dist_value,
        "base_div_dist": base_div_dist_value,
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
        "point_mode": point_mode_value,
        "param_config": param_config_value,
    }


def _gradient_factor(mode, t):
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
    uv = rs.SurfaceClosestPoint(guide_srf, pt)
    u_dom = rs.SurfaceDomain(guide_srf, 0)
    v_dom = rs.SurfaceDomain(guide_srf, 1)
    u_norm = gl.invlerp(u_dom[0], u_dom[1], uv[0])
    v_norm = gl.invlerp(v_dom[0], v_dom[1], uv[1])
    rgb = sample_surface_color(pt, guide_srf, img) if img else None
    return (u_norm, v_norm), rgb


def evaluate_parameter(mode, settings, pt, uv, rgb):
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
            raise ValueError("Gradient settings for axis '{}' require a domain".format(axis))
        factor = _gradient_factor(settings.get("mode", "linear"), gl.invlerp(domain[0], domain[1], coord))
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
            return float(stops[0][1]) * float(settings.get("scale", 1.0))
        if t >= stops[-1][0]:
            return float(stops[-1][1]) * float(settings.get("scale", 1.0))
        for idx in range(len(stops) - 1):
            p0, v0 = stops[idx]
            p1, v1 = stops[idx + 1]
            if p0 <= t <= p1:
                local_t = 0.0 if p1 == p0 else gl.invlerp(p0, p1, t)
                return gl.lerp(v0, v1, local_t) * float(settings.get("scale", 1.0))
        return float(stops[-1][1]) * float(settings.get("scale", 1.0))

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
            return result * float(settings.get("scale", 1.0))
        if operation == "add":
            return sum(values) * float(settings.get("scale", 1.0))
        if operation == "max":
            return max(values) * float(settings.get("scale", 1.0))
        return min(values) * float(settings.get("scale", 1.0))

    raise ValueError("Unsupported parameter mode: {}".format(mode))


def add_wall_opening(pt, base_crv, width, angle, disp):
    pt_param = rs.CurveClosestPoint(base_crv, pt)
    tangent = rs.VectorUnitize(rs.CurveTangent(base_crv, pt_param))
    normal = rs.VectorRotate(tangent, 90 - angle, [0, 0, 1])
    if base_crv.IsClosed:
        center_pt = _curve_centroid(base_crv)
        inward_vec = rs.VectorCreate(center_pt, pt)
        alt_normal = rs.VectorReverse(normal)
        if rs.VectorDotProduct(alt_normal, inward_vec) > rs.VectorDotProduct(normal, inward_vec):
            normal = alt_normal
    pt_before = rs.PointAdd(pt, rs.VectorScale(tangent, -width / 2.0))
    pt_after = rs.PointAdd(pt, rs.VectorScale(tangent, width / 2.0))
    disp_vec = rs.VectorScale(normal, disp)
    pt_before_disp = rs.PointAdd(pt_before, disp_vec)
    pt_after_disp = rs.PointAdd(pt_after, disp_vec)
    return (pt_before, pt_before_disp, pt_after_disp, pt_after)


def _curve_length(curve):
    return float(curve.GetLength())


def _curve_domain(curve):
    return curve.Domain


def _point_at_normalized(curve, t_norm):
    domain = _curve_domain(curve)
    param = gl.lerp(domain.T0, domain.T1, t_norm)
    return param, curve.PointAt(param)


def _slice_brep_to_contours(brep, layer_height):
    print("slicing brep with layer_height={}".format(layer_height))
    bbox = brep.GetBoundingBox(True)
    start = Rhino.Geometry.Point3d(bbox.Center.X, bbox.Center.Y, bbox.Min.Z)
    end = Rhino.Geometry.Point3d(bbox.Center.X, bbox.Center.Y, bbox.Max.Z)
    contours = Rhino.Geometry.Brep.CreateContourCurves(brep, start, end, layer_height)
    if not contours:
        return []

    rows_by_z = {}
    for curve in contours:
        if curve is None or not curve.IsValid:
            continue
        z_key = round(curve.GetBoundingBox(True).Center.Z, 6)
        rows_by_z.setdefault(z_key, []).append(curve)

    ordered = []
    for z_key in sorted(rows_by_z.keys()):
        curves = rows_by_z[z_key]
        longest = max(curves, key=lambda item: item.GetLength())
        ordered.append(longest)
    print("contour rows generated={}".format(len(ordered)))
    return ordered


def build_curve_point_samples(curve_stack, base_div_dist):
    point_stack = []
    param_stack = []

    for curve in curve_stack:
        curve_length = max(base_div_dist, _curve_length(curve))
        sample_count = max(12, int(math.ceil(curve_length / float(base_div_dist))))
        if curve.IsClosed:
            t_values = [float(i) / float(sample_count) for i in range(sample_count)]
        else:
            sample_count = max(2, sample_count)
            t_values = [float(i) / float(sample_count - 1) for i in range(sample_count)]

        params = []
        points = []
        for t_norm in t_values:
            param, pt = _point_at_normalized(curve, t_norm)
            params.append(param)
            points.append(pt)
        point_stack.append(points)
        param_stack.append(params)

    return point_stack, param_stack


def _has_vertical_clearance(row_idx, step_count, row_count, bottom_margin_layers=2):
    step_count = max(1, int(step_count))
    bottom_margin_layers = max(0, int(bottom_margin_layers))
    if row_idx < bottom_margin_layers:
        return False
    return row_idx + step_count <= row_count


def _has_edge_clearance(curve, curve_param, opening_width):
    if curve.IsClosed:
        return True
    curve_length = _curve_length(curve)
    arc_length = curve.GetLength(Rhino.Geometry.Interval(curve.Domain.T0, curve_param))
    margin = max(0.0, float(opening_width))
    return margin <= arc_length <= max(0.0, curve_length - margin)


def _build_regular_opening_rows(row_count, num_v, step_count, v_scale, v_reverse, v_shift, seed_row_start):
    opening_rows = set()
    bottom_margin_layers = max(0, int(seed_row_start))
    v_cursor = bottom_margin_layers

    while v_cursor < row_count:
        v_idx = (row_count - 1 - v_cursor) if v_reverse else v_cursor
        v_idx_shifted = max(0, min(row_count - 1, v_idx + int(v_shift)))
        if _has_vertical_clearance(v_idx_shifted, step_count, row_count, bottom_margin_layers):
            opening_rows.add(v_idx_shifted)
        v_inc = int((v_scale - 1.0) * v_cursor)
        local_v = max(0, num_v + v_inc)
        v_cursor += max(1, int(step_count)) + local_v

    return opening_rows


def _evaluate_seed_values(pt, guide_srf, img, param_config, gate_threshold, density):
    if guide_srf is None:
        uv, rgb = (0.0, 0.0), None
        gate_value = 1.0
    else:
        uv, rgb = _sample_guide_data(pt, guide_srf, img)
        gate_value = evaluate_parameter(param_config["gate"][0], param_config["gate"][1], pt, uv, rgb)
        if gate_value < gate_threshold:
            return None

    width_value = max(
        0.0,
        float(evaluate_parameter(param_config["width"][0], param_config["width"][1], pt, uv, rgb)),
    )
    step_value = max(
        1,
        int(round(evaluate_parameter(param_config["steps"][0], param_config["steps"][1], pt, uv, rgb))),
    )
    density_value = gl.minmaxcap(
        0.0,
        1.0,
        float(evaluate_parameter(param_config["density"][0], param_config["density"][1], pt, uv, rgb)),
    )
    if not _is_number(step_value):
        print("unexpected step_value:", step_value, type(step_value))
    local_density = gl.minmaxcap(0.0, 1.0, density * density_value)
    return {
        "width": width_value,
        "steps": step_value,
        "local_density": local_density,
    }


def build_regular_seed_data(
    curve_stack,
    guide_srf,
    img,
    param_config,
    gate_threshold,
    density,
    div_dist,
    num_u,
    num_v,
    steps,
    u_shift,
    v_shift,
    v_scale,
    v_reverse,
    seed_row_start,
    random_seed,
):
    seeds = []
    row_count = len(curve_stack)
    opening_rows = _build_regular_opening_rows(
        row_count,
        num_v,
        steps,
        v_scale,
        v_reverse,
        v_shift,
        seed_row_start,
    )
    if abs(u_shift) <= 1.0:
        u_phase_steps = int(round(u_shift * max(1, num_u - 1))) % max(1, num_u)
    else:
        u_phase_steps = int(round(u_shift)) % max(1, num_u)

    for row_idx, curve in enumerate(curve_stack):
        if row_idx not in opening_rows:
            continue

        curve_length = _curve_length(curve)
        div_count = max(num_u + 1, int(curve_length / float(div_dist)))
        is_closed = curve.IsClosed
        if not is_closed:
            div_count = max(2, div_count + 1)

        for idx in range(div_count):
            if (idx + u_phase_steps) % max(1, num_u) != 0:
                continue

            if is_closed:
                t_norm = float(idx) / float(div_count)
            else:
                if idx == 0 or idx == div_count - 1:
                    continue
                t_norm = float(idx) / float(div_count - 1)

            curve_param, pt = _point_at_normalized(curve, t_norm)
            values = _evaluate_seed_values(pt, guide_srf, img, param_config, gate_threshold, density)
            if values is None or values["local_density"] <= 0.0:
                continue
            if not _has_edge_clearance(curve, curve_param, values["width"]):
                continue
            if not _has_vertical_clearance(row_idx, values["steps"], row_count, seed_row_start):
                continue
            if _stable_noise(int(random_seed), row_idx, t_norm) > values["local_density"]:
                continue

            seeds.append(
                {
                    "row": row_idx,
                    "param": curve_param,
                    "arc": t_norm * curve_length,
                    "width": values["width"],
                    "steps": values["steps"],
                    "point": pt,
                }
            )

    print("regular seed count={}".format(len(seeds)))
    return seeds


def build_random_seed_data(
    curve_stack,
    guide_srf,
    img,
    param_config,
    gate_threshold,
    density,
    div_dist,
    random_seed,
):
    rng = random.Random(int(random_seed))
    seeds = []
    all_candidates = []
    row_count = len(curve_stack)
    bottom_margin_layers = 2

    for row_idx, curve in enumerate(curve_stack):
        curve_length = _curve_length(curve)
        candidate_count = max(
            1,
            int((curve_length / float(div_dist)) * 3 * max(0.15, density)),
        )
        row_candidates = []

        for _ in range(candidate_count):
            t_norm = rng.random()
            curve_param, pt = _point_at_normalized(curve, t_norm)
            values = _evaluate_seed_values(pt, guide_srf, img, param_config, gate_threshold, density)
            if values is None or values["local_density"] <= 0.0:
                continue
            if not _has_edge_clearance(curve, curve_param, values["width"]):
                continue
            if not _has_vertical_clearance(row_idx, values["steps"], row_count, bottom_margin_layers):
                continue
            if _stable_noise(int(random_seed), row_idx, t_norm) > values["local_density"]:
                continue

            spacing_factor = gl.lerp(3.0, 1.0, values["local_density"])
            min_u_spacing = max(values["width"], float(div_dist) * spacing_factor)
            min_v_spacing = max(
                values["steps"],
                int(round(gl.lerp(values["steps"] + 3.0, float(values["steps"]), values["local_density"]))),
            )

            row_candidates.append(
                {
                    "row": row_idx,
                    "param": curve_param,
                    "arc": t_norm * curve_length,
                    "width": values["width"],
                    "steps": values["steps"],
                    "point": pt,
                    "local_density": values["local_density"],
                    "min_u_spacing": min_u_spacing,
                    "min_v_spacing": min_v_spacing,
                    "score": _stable_noise(int(random_seed) + 11, row_idx, t_norm),
                }
            )

        row_candidates.sort(key=lambda item: item["score"])
        all_candidates.extend(row_candidates)
        for candidate in row_candidates:
            intersects = False
            for other in seeds:
                same_band = abs(candidate["row"] - other["row"]) < max(
                    candidate["min_v_spacing"], other.get("min_v_spacing", 0)
                )
                close_u = abs(candidate["arc"] - other["arc"]) < max(
                    candidate["min_u_spacing"], other.get("min_u_spacing", 0)
                )
                if same_band and close_u:
                    intersects = True
                    break
            if not intersects:
                seeds.append(candidate)

    if not seeds and all_candidates:
        seeds.append(max(all_candidates, key=lambda item: (item["local_density"], -item["score"])))

    print("random seed count={}".format(len(seeds)))
    return seeds


def build_opening_events(curve_stack, seed_data):
    event_stack = [[] for _ in curve_stack]
    for seed in seed_data:
        for n in range(seed["steps"]):
            row_idx = seed["row"] + n
            if row_idx >= len(event_stack):
                break
            taper_t = float(n) / float(seed["steps"])
            propagated = gl.lerp(seed["width"], 0.0, taper_t)
            if propagated <= 0.0:
                continue
            event_stack[row_idx].append((seed["param"], propagated))
    return event_stack


def build_output_points_from_events(point_stack, param_stack, curve_stack, event_stack, angle, displacement):
    output = []
    disp_comp = displacement / math.cos(math.radians(angle))

    for row_idx, row in enumerate(point_stack):
        curve = curve_stack[row_idx]
        row_items = list(zip(param_stack[row_idx], [0] * len(row), row))
        for event_param, width in event_stack[row_idx]:
            event_pt = curve.PointAt(event_param)
            row_items.append((event_param, 1, (event_pt, width)))

        row_items.sort(key=lambda item: (item[0], item[1]))
        row_pts = []
        for _param, item_type, payload in row_items:
            if item_type == 0:
                row_pts.append(payload)
            else:
                event_pt, width = payload
                row_pts.extend(add_wall_opening(event_pt, curve, width, angle, disp_comp))
        output.append(row_pts)

    return output


def _curve_area(curve):
    amp = Rhino.Geometry.AreaMassProperties.Compute(curve)
    return abs(amp.Area) if amp else 0.0


def _curve_centroid(curve):
    amp = Rhino.Geometry.AreaMassProperties.Compute(curve)
    if amp:
        return amp.Centroid
    bbox = curve.GetBoundingBox(True)
    return bbox.Center


def _dense_curve_points(curve, target_dist):
    curve_length = max(target_dist, _curve_length(curve))
    sample_count = max(12, int(math.ceil(curve_length / float(target_dist))))
    t_values = [float(i) / float(sample_count) for i in range(sample_count)] if curve.IsClosed else [
        float(i) / float(sample_count - 1) for i in range(max(2, sample_count))
    ]
    pts = []
    for t_norm in t_values:
        _param, pt = _point_at_normalized(curve, t_norm)
        pts.append(pt)
    return pts


def _average_radius_from_point(curve, center_pt, target_dist):
    pts = _dense_curve_points(curve, target_dist)
    if not pts:
        return 0.0
    total = 0.0
    for pt in pts:
        total += pt.DistanceTo(center_pt)
    return total / float(len(pts))


def _build_outer_offset_curve(curve, offset_dist):
    success, plane = curve.TryGetPlane()
    if not success:
        bbox = curve.GetBoundingBox(True)
        plane = Rhino.Geometry.Plane(Rhino.Geometry.Point3d(0, 0, bbox.Center.Z), Rhino.Geometry.Vector3d.ZAxis)

    tol = Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance if Rhino.RhinoDoc.ActiveDoc else 0.01
    candidates = []
    for signed_dist in (abs(offset_dist), -abs(offset_dist)):
        offsets = curve.Offset(plane, signed_dist, tol, Rhino.Geometry.CurveOffsetCornerStyle.Round)
        if not offsets:
            continue
        for offset in offsets:
            if offset and offset.IsValid:
                candidates.append(offset)

    if not candidates:
        return None

    base_center = _curve_centroid(curve)
    base_radius = _average_radius_from_point(curve, base_center, max(1.0, abs(offset_dist) * 0.5))
    ranked = []
    for candidate in candidates:
        candidate_radius = _average_radius_from_point(candidate, base_center, max(1.0, abs(offset_dist) * 0.5))
        radius_delta = candidate_radius - base_radius
        ranked.append((radius_delta, candidate))

    ranked.sort(key=lambda item: item[0], reverse=True)
    print("offset radius deltas:", [round(item[0], 3) for item in ranked])

    for radius_delta, candidate in ranked:
        if radius_delta > 0:
            return candidate

    return ranked[0][1]


def build_closed_output(curve_stack, output, base_div_dist, displacement):
    closed_layers = []
    offset_dist = displacement + 18.0

    for row_idx, curve in enumerate(curve_stack):
        offset_curve = _build_outer_offset_curve(curve, offset_dist)
        if offset_curve is None:
            closed_layers.append(list(output[row_idx]))
            continue
        offset_pts = _dense_curve_points(offset_curve, base_div_dist)
        layer_pts = list(output[row_idx])
        layer_pts.extend(offset_pts[::-1])
        closed_layers.append(layer_pts)

    return closed_layers


def build_gradient_preview(guide_srf, img, param_config, opening_width, opening_scale, sample_count=24):
    if guide_srf is None or opening_width <= 0:
        return None

    width_mode, width_settings = param_config["width"]
    base_width = max(1e-9, float(opening_width) * float(opening_scale))
    mesh = Rhino.Geometry.Mesh()
    u_dom = rs.SurfaceDomain(guide_srf, 0)
    v_dom = rs.SurfaceDomain(guide_srf, 1)

    for v_idx in range(sample_count + 1):
        v_t = float(v_idx) / float(sample_count)
        v = gl.lerp(v_dom[0], v_dom[1], v_t)
        for u_idx in range(sample_count + 1):
            u_t = float(u_idx) / float(sample_count)
            u = gl.lerp(u_dom[0], u_dom[1], u_t)
            pt = rs.EvaluateSurface(guide_srf, u, v)
            uv = (u_t, v_t)
            rgb = sample_surface_color(pt, guide_srf, img) if img else None
            width_value = float(width_settings) if width_mode == "fixed" else evaluate_parameter(width_mode, width_settings, pt, uv, rgb)
            factor = gl.minmaxcap(0.2, 1.0, float(width_value) / base_width)
            gray = int(round(gl.remap(0.2, 1.0, 51, 255, factor)))
            mesh.Vertices.Add(rs.coerce3dpoint(pt))
            mesh.VertexColors.Add(Color.FromArgb(gray, gray, gray))

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


validated = _validate_inputs()

if validated is None:
    a = th.list_to_tree([])
    b = []
    c = []
    C = c
else:
    curve_stack = _slice_brep_to_contours(validated["brep"], validated["layer_height"])
    if not curve_stack:
        _append_warning("No contour curves were created from the Brep.")
        a = th.list_to_tree([])
        b = []
        c = []
        C = c
    else:
        point_stack, param_stack = build_curve_point_samples(curve_stack, validated["base_div_dist"])

        if validated["point_mode"] == "random":
            seed_data = build_random_seed_data(
                curve_stack,
                validated["guide_srf"],
                validated["img"],
                validated["param_config"],
                validated["gate_threshold"],
                validated["density"],
                validated["div_dist"],
                validated["random_seed"],
            )
        else:
            seed_data = build_regular_seed_data(
                curve_stack,
                validated["guide_srf"],
                validated["img"],
                validated["param_config"],
                validated["gate_threshold"],
                validated["density"],
                validated["div_dist"],
                validated["num_u"],
                validated["num_v"],
                validated["steps"],
                validated["u_shift"],
                validated["v_shift"],
                validated["v_scale"],
                validated["v_reverse"],
                validated["seed_row_start"],
                validated["random_seed"],
            )

        event_stack = build_opening_events(curve_stack, seed_data)
        output = build_output_points_from_events(
            point_stack,
            param_stack,
            curve_stack,
            event_stack,
            validated["angle"],
            validated["displacement"],
        )
        closed_output = build_closed_output(
            curve_stack,
            output,
            validated["base_div_dist"],
            validated["displacement"],
        )

        a = th.list_to_tree(closed_output)
        b = build_gradient_preview(
            validated["guide_srf"],
            validated["img"],
            validated["param_config"],
            validated["opening_width"],
            validated["opening_scale"],
        )
        c = [seed["point"] for seed in seed_data]
        C = c
