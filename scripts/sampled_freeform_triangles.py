#! python3

"""Grasshopper scaffold for triangular openings between two sliced wall surfaces.

This script supports two opening-distribution modes:

1. Regular grid mode:
   Used when no preset or `param_config` is active. Openings are distributed
   by `num_u`, `num_v`, and the sliced division points.
2. Guide-driven mode:
   Used when a preset or `param_config` is active. Width, steps, density, and
   profile modifiers are sampled from gradients or image channels on a guide
   surface.

Each opening is resolved as a front profile plus an overlap-offset back profile,
with optional back-side support indents propagated into lower layers.

Inputs:
    front_srf: Freeform front surface to slice and sample.
    back_srf: Planar back surface to slice and use for displacement normals.
    guide_srf: Optional guide surface/Brep for gradient and texture sampling.
    img: Optional texture image path. Required only for image_* modes.
    layers: Optional cap on the number of sliced rows for testing.
    layer_height: Z slicing distance in the global +Z direction.
    div_dist: Target division distance along each sliced front curve.
    num_u: Base U periodicity for seed openings.
    num_v: Base V periodicity for seed rows.
    steps: Default propagation depth in layers.
    opening_width: Default opening width.
    angle: Opening angle in degrees.
    nozzle/N: Nozzle width used to compute the overlap stand-off.
    overlap: Overlap ratio as 0..1 or 0..100 percent.
    max_overhang: Maximum support overhang angle in degrees for back-side runout.
    preset: `0` disables presets and keeps the regular grid distribution.

Optional GH inputs:
    param_config: Dict configuring surface-driven parameters.
    gate_threshold: Legacy threshold input kept for backward compatibility.
    density: Normalized 0..1 global multiplier for appearance density.
    backface_reduction: Normalized 0..1 blend between constant and tapered back face.
    u_shift, v_shift, u_scale, v_scale, v_reverse, opening_scale:
        Same modulation inputs as the existing triangular script.

Outputs:
    a: Grasshopper tree of open polyline point rows.
    b: Preview geometry showing the sampled width gradient.
    c: Seed points used to generate the openings.
    d: Grasshopper tree of opening contour loops.
"""
__author__ = "joseh"
__version__ = "2026.03.24"


import math
import random

import Grasshopper as gh
import Rhino
import geometrylib as gl
import rhinoscriptsyntax as rs
from ghpythonlib import treehelpers as th
from srflib import sample_surface_color
from System.Drawing import Color


# Regular seed layout defaults used when GH inputs are not connected.
DEFAULT_SHIFT_U = 1
DEFAULT_SHIFT_V = -5


def _get_param(name, default):
    return globals().get(name, default)


def _get_first_param(names, default=None):
    for name in names:
        value = globals().get(name, None)
        if value is not None:
            return value
    return default


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


def _model_tolerance():
    doc = Rhino.RhinoDoc.ActiveDoc
    return doc.ModelAbsoluteTolerance if doc is not None else 0.01


def _surface_bbox(surface):
    return surface.GetBoundingBox(True)


def _coerce_wall_geometry(value):
    """Accept Rhino surfaces and Breps, including extruded-curve Breps."""
    brep = rs.coercebrep(value, False)
    if brep is not None:
        return brep
    surface = rs.coercesurface(value, False)
    if surface is not None:
        return surface
    return None


def _as_brep(geometry):
    if isinstance(geometry, Rhino.Geometry.Brep):
        return geometry
    if hasattr(geometry, "ToBrep"):
        return geometry.ToBrep()
    raise TypeError("Expected a Rhino surface or Brep.")


def _primary_face(geometry):
    """Return the main face used for UV sampling and preview evaluation."""
    if isinstance(geometry, Rhino.Geometry.Brep):
        return max(
            geometry.Faces,
            key=lambda face: Rhino.Geometry.AreaMassProperties.Compute(face).Area,
        )
    return geometry


def _closest_surface_data(geometry, pt):
    """Return closest UV, domains, and face/surface for a wall geometry."""
    pt3d = rs.coerce3dpoint(pt)
    if isinstance(geometry, Rhino.Geometry.Brep):
        face = _primary_face(geometry)
        success, u, v = face.ClosestPoint(pt3d)
        if not success:
            raise ValueError("Could not find closest point on the input brep face.")
        u_dom = face.Domain(0)
        v_dom = face.Domain(1)
        return (u, v), face, (u_dom.T0, u_dom.T1), (v_dom.T0, v_dom.T1)

    success, u, v = geometry.ClosestPoint(pt3d)
    if not success:
        raise ValueError("Could not find closest point on the input surface.")
    u_dom = geometry.Domain(0)
    v_dom = geometry.Domain(1)
    return (u, v), geometry, (u_dom.T0, u_dom.T1), (v_dom.T0, v_dom.T1)


def _is_planar_wall(geometry, tol):
    if isinstance(geometry, Rhino.Geometry.Brep):
        return geometry.Faces.Count == 1 and geometry.Faces[0].IsPlanar(tol)
    return geometry.IsPlanar(tol)


def _normalize_overlap(value):
    overlap = float(value)
    if overlap > 1.0:
        overlap /= 100.0
    return gl.minmaxcap(0.0, 1.0, overlap)


def _planar_wall_plane(geometry):
    face = _primary_face(geometry)
    success, plane = face.TryGetPlane()
    if not success:
        raise ValueError("Could not resolve a plane from back_srf.")
    return plane


def _point3d(value):
    return rs.coerce3dpoint(value)


def _vector3d(value):
    return rs.coerce3dvector(value)


def _dot(a, b):
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z


def _scale(vector, factor):
    return Rhino.Geometry.Vector3d(vector.X * factor, vector.Y * factor, vector.Z * factor)


def _add_point_vector(point, vector):
    return Rhino.Geometry.Point3d(point.X + vector.X, point.Y + vector.Y, point.Z + vector.Z)


def _sub_points(a, b):
    return Rhino.Geometry.Vector3d(a.X - b.X, a.Y - b.Y, a.Z - b.Z)


def _distance(a, b):
    return _sub_points(a, b).Length


def _offset_data_from_back_surface(pt, back_srf, back_plane, tangent, angle, stand_off):
    """Return displaced-point data using depth = front-to-back gap minus stand-off."""
    pt = _point3d(pt)
    uv, face, _u_dom, _v_dom = _closest_surface_data(back_srf, pt)
    back_pt = face.PointAt(uv[0], uv[1])
    to_front = _sub_points(pt, back_pt)
    if to_front.Length <= 1e-9:
        normal = Rhino.Geometry.Vector3d(face.NormalAt(uv[0], uv[1]))
        normal.Unitize()
        to_front = normal
    else:
        to_front.Unitize()

    plane_normal = Rhino.Geometry.Vector3d(back_plane.Normal)
    plane_normal.Unitize()
    if _dot(plane_normal, to_front) < 0.0:
        plane_normal.Reverse()

    tangent_on_back = Rhino.Geometry.Vector3d(tangent)
    tangent_on_back -= _scale(plane_normal, _dot(tangent_on_back, plane_normal))
    if tangent_on_back.Length <= 1e-9:
        tangent_on_back = Rhino.Geometry.Vector3d.CrossProduct(plane_normal, Rhino.Geometry.Vector3d.ZAxis)
        if tangent_on_back.Length <= 1e-9:
            tangent_on_back = Rhino.Geometry.Vector3d.CrossProduct(plane_normal, Rhino.Geometry.Vector3d.XAxis)
    tangent_on_back.Unitize()

    front_to_back_gap = abs(_dot(plane_normal, _sub_points(pt, back_pt)))
    depth = max(0.0, front_to_back_gap - float(stand_off))
    angle_shift = math.tan(math.radians(angle)) * depth
    shifted_back_pt = _add_point_vector(back_pt, _scale(tangent_on_back, angle_shift))
    target_plane_pt = _add_point_vector(back_pt, _scale(plane_normal, float(stand_off)))
    displaced_pt = _add_point_vector(target_plane_pt, _scale(tangent_on_back, angle_shift))
    return {
        "back_shifted_point": shifted_back_pt,
        "point": displaced_pt,
        "depth": depth,
    }


def _balance_pair(value, balance):
    left = float(value) * (1.0 - max(0.0, float(balance)))
    right = float(value) * (1.0 + min(0.0, float(balance)))
    return left, right


def _interpolate_point(a, b, t):
    return Rhino.Geometry.Point3d(
        gl.lerp(a.X, b.X, t),
        gl.lerp(a.Y, b.Y, t),
        gl.lerp(a.Z, b.Z, t),
    )


def _profile_envelope(t, division_points):
    division_points = max(0, int(round(float(division_points))))
    if division_points <= 0:
        return 0.0
    sample_count = division_points + 2
    ts = [float(i) / float(sample_count - 1) for i in range(sample_count)]
    values = [4.0 * s * (1.0 - s) for s in ts]
    if t <= ts[0]:
        return values[0]
    if t >= ts[-1]:
        return values[-1]
    for idx in range(sample_count - 1):
        t0 = ts[idx]
        t1 = ts[idx + 1]
        if t0 <= t <= t1:
            local_t = 0.0 if t1 == t0 else gl.invlerp(t0, t1, t)
            return gl.lerp(values[idx], values[idx + 1], local_t)
    return 0.0


def _profile_side_widths(base_width, ref_width, t, division_points, side_shift, shift_balance):
    half_width = max(0.0, float(base_width) / 2.0)
    shift_ref = max(0.0, float(ref_width) / 2.0)
    profile_shift = _profile_envelope(float(t), division_points) * float(side_shift) * shift_ref
    left_shift, right_shift = _balance_pair(profile_shift, shift_balance)
    left_width = max(0.0, half_width + left_shift)
    right_width = max(0.0, half_width + right_shift)
    return left_width, right_width


def _sample_curve_points(curve, div_dist, sample_count=None):
    """Return evenly sampled points along a curve."""
    curve_length = rs.CurveLength(curve)
    if sample_count is None:
        sample_count = max(2, int(curve_length / float(div_dist)) + 1)
    domain = rs.CurveDomain(curve)
    params = [
        gl.lerp(domain[0], domain[1], float(i) / float(sample_count - 1))
        for i in range(sample_count)
    ]
    return [rs.EvaluateCurve(curve, t) for t in params]


def _curve_point_at_length(curve, distance):
    """Return (point, parameter) at a cumulative arc length along a curve."""
    curve_geom = rs.coercecurve(curve)
    if curve_geom is None:
        raise ValueError("Invalid curve input.")

    curve_length = curve_geom.GetLength()
    if curve_length <= 1e-9:
        domain = curve_geom.Domain
        return curve_geom.PointAt(domain.Min), domain.Min

    distance = gl.minmaxcap(0.0, curve_length, float(distance))
    if distance <= 1e-9:
        domain = curve_geom.Domain
        return curve_geom.PointAt(domain.Min), domain.Min
    if abs(distance - curve_length) <= 1e-9:
        domain = curve_geom.Domain
        return curve_geom.PointAt(domain.Max), domain.Max

    success, curve_param = curve_geom.LengthParameter(distance)
    if not success:
        domain = rs.CurveDomain(curve)
        curve_param = gl.lerp(domain[0], domain[1], distance / curve_length)
    return rs.EvaluateCurve(curve, curve_param), curve_param


def _sample_curve_points_by_target_distance(curve, base_div_dist, front_srf, img, param_config):
    """Return curve points from a varying target division distance field.

    The row starts from the first endpoint and advances by the locally evaluated
    `div_dist` target along arc length. This makes the base point distribution
    itself carry the horizontal gradient, instead of applying spacing only as a
    later seed filter.
    """
    curve_length = rs.CurveLength(curve)
    if curve_length is None or curve_length <= 1e-9:
        return _sample_curve_points(curve, base_div_dist, 2)

    min_step = max(1e-3, 0.25 * float(base_div_dist))
    pts = []
    walked = 0.0

    while walked < curve_length - 1e-9:
        pt, _curve_param = _curve_point_at_length(curve, walked)
        pts.append(pt)

        uv, rgb = _sample_guide_data(pt, front_srf, img)
        target_step = max(min_step, float(_evaluate_config_value(param_config, "div_dist", pt, uv, rgb)))
        next_walked = min(curve_length, walked + target_step)
        if next_walked <= walked + 1e-9:
            break
        walked = next_walked

    end_pt = rs.CurveEndPoint(curve)
    if not pts or _distance(pts[-1], end_pt) > 1e-6:
        pts.append(end_pt)
    return pts


def _slice_surface_rows(surface, layer_height, max_rows=None):
    """Slice a wall geometry in global +Z and order contour curves by height."""
    brep = _as_brep(surface)
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
        ordered.append(max(curves, key=lambda item: item.GetLength()))
    if max_rows is not None:
        ordered = ordered[: max(0, int(max_rows))]
    return ordered


def _report(message, level=gh.Kernel.GH_RuntimeMessageLevel.Warning, errors=None):
    """Emit a runtime message and optionally collect it as an error."""
    if errors is not None and level == gh.Kernel.GH_RuntimeMessageLevel.Error:
        errors.append(message)
    _add_runtime_message(level, message)


def _validate_value(value, expected_type, message, coercer=None, allow_none=False):
    """Return (is_valid, normalized_value) and warn when a value has the wrong type."""
    if value is None:
        if allow_none:
            return True, None
        _report(message)
        return False, None

    normalized = value
    if coercer is not None:
        try:
            normalized = coercer(value)
        except Exception:
            _report(message)
            return False, None

    if not isinstance(normalized, expected_type):
        _report(message)
        return False, None

    return True, normalized


def _validate_scalar(name, value, errors, minimum=None, allow_zero=False):
    """Validate a numeric scalar and optionally enforce a lower bound."""
    if not _is_number(value):
        _report("{} must be a number.".format(name), gh.Kernel.GH_RuntimeMessageLevel.Error, errors)
        return None
    value = float(value)
    if minimum is not None:
        if allow_zero:
            is_invalid = value < minimum
        else:
            is_invalid = value <= minimum
        if is_invalid:
            comparator = ">=" if allow_zero else ">"
            _report(
                "{} must be {} {}.".format(name, comparator, minimum),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
    return value


def _validate_param_spec(name, spec, errors):
    """Validate one param_config entry."""
    if not isinstance(spec, (list, tuple)) or len(spec) != 2:
        _report(
            "param_config['{}'] must be a (mode, settings) pair.".format(name),
            gh.Kernel.GH_RuntimeMessageLevel.Error,
            errors,
        )
        return None
    mode, settings = spec
    mode = _normalize_param_mode(mode)
    if mode == "fixed":
        if not _is_number(settings):
            _report(
                "param_config['{}'] fixed mode requires a number.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
    elif mode in ("image_r", "image_g", "image_b"):
        if not isinstance(settings, (list, tuple)) or len(settings) != 2:
            _report(
                "param_config['{}'] image mode requires a (min, max) tuple.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        if not all(_is_number(v) for v in settings):
            _report(
                "param_config['{}'] image range values must be numeric.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
    elif mode == "gradient":
        if not isinstance(settings, dict):
            _report(
                "param_config['{}'] gradient mode requires a dict.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        if "range" not in settings:
            _report(
                "param_config['{}'] gradient settings require 'range'.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        rng = settings["range"]
        if not isinstance(rng, (list, tuple)) or len(rng) != 2 or not all(_is_number(v) for v in rng):
            _report(
                "param_config['{}'] gradient 'range' must be a numeric pair.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        try:
            _gradient_factor(settings.get("mode", "linear"), 0.5)
        except ValueError as exc:
            _report(str(exc), gh.Kernel.GH_RuntimeMessageLevel.Error, errors)
            return None
        try:
            _coerce_axis_value(settings.get("axis", "u"), (0.0, 0.0, 0.0), (0.5, 0.5))
        except ValueError as exc:
            _report(str(exc), gh.Kernel.GH_RuntimeMessageLevel.Error, errors)
            return None
        if "domain" in settings:
            domain = settings["domain"]
            if (
                not isinstance(domain, (list, tuple))
                or len(domain) != 2
                or not all(_is_number(v) for v in domain)
            ):
                _report(
                    "param_config['{}'] gradient 'domain' must be a numeric pair.".format(name),
                    gh.Kernel.GH_RuntimeMessageLevel.Error,
                    errors,
                )
                return None
    elif mode == "gradient_stops":
        if not isinstance(settings, dict):
            _report(
                "param_config['{}'] gradient_stops mode requires a dict.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        stops = settings.get("stops")
        if not isinstance(stops, (list, tuple)) or len(stops) < 2:
            _report(
                "param_config['{}'] gradient_stops requires at least two stops.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        prev_pos = None
        for stop in stops:
            if not isinstance(stop, (list, tuple)) or len(stop) != 2:
                _report(
                    "param_config['{}'] stops must be (position, value) pairs.".format(name),
                    gh.Kernel.GH_RuntimeMessageLevel.Error,
                    errors,
                )
                return None
            pos, value = stop
            if not _is_number(pos) or not _is_number(value):
                _report(
                    "param_config['{}'] stop values must be numeric.".format(name),
                    gh.Kernel.GH_RuntimeMessageLevel.Error,
                    errors,
                )
                return None
            if not 0.0 <= float(pos) <= 1.0:
                _report(
                    "param_config['{}'] stop positions must be within 0..1.".format(name),
                    gh.Kernel.GH_RuntimeMessageLevel.Error,
                    errors,
                )
                return None
            if prev_pos is not None and float(pos) < prev_pos:
                _report(
                    "param_config['{}'] stops must be sorted by position.".format(name),
                    gh.Kernel.GH_RuntimeMessageLevel.Error,
                    errors,
                )
                return None
            prev_pos = float(pos)
        try:
            _coerce_axis_value(settings.get("axis", "u"), (0.0, 0.0, 0.0), (0.5, 0.5))
        except ValueError as exc:
            _report(str(exc), gh.Kernel.GH_RuntimeMessageLevel.Error, errors)
            return None
        if "scale" in settings and not _is_number(settings["scale"]):
            _report(
                "param_config['{}'] gradient_stops 'scale' must be numeric.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
    elif mode == "composite":
        if not isinstance(settings, dict):
            _report(
                "param_config['{}'] composite mode requires a dict.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        operation = settings.get("operation", "multiply")
        if operation not in ("multiply", "add", "max", "min"):
            _report(
                "param_config['{}'] composite operation must be one of multiply/add/max/min.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        sources = settings.get("sources")
        if not isinstance(sources, (list, tuple)) or len(sources) < 2:
            _report(
                "param_config['{}'] composite mode requires at least two sources.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        normalized_sources = []
        for idx, source in enumerate(sources):
            if not isinstance(source, (list, tuple)) or len(source) != 2:
                _report(
                    "param_config['{}'] composite source {} must be a (mode, settings) pair.".format(name, idx),
                    gh.Kernel.GH_RuntimeMessageLevel.Error,
                    errors,
                )
                return None
            normalized_source = _validate_param_spec("{} source {}".format(name, idx), source, errors)
            if normalized_source is None:
                return None
            normalized_sources.append(normalized_source)
        if "scale" in settings and not _is_number(settings["scale"]):
            _report(
                "param_config['{}'] composite 'scale' must be numeric.".format(name),
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                errors,
            )
            return None
        settings = dict(settings)
        settings["sources"] = normalized_sources
    else:
        _report(
            "Unsupported parameter mode: {}".format(mode),
            gh.Kernel.GH_RuntimeMessageLevel.Error,
            errors,
        )
        return None
    return (mode, settings)


def _validate_param_config(param_config, img, errors, default_steps):
    """Validate the optional param_config dictionary."""
    if not isinstance(param_config, dict):
        _report("param_config must be a dictionary when provided.", gh.Kernel.GH_RuntimeMessageLevel.Error, errors)
        return None

    validated = {}
    for key in ("width", "depth"):
        if key not in param_config:
            _report("param_config is missing '{}'.".format(key), gh.Kernel.GH_RuntimeMessageLevel.Error, errors)
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
    optional_profile_defaults = {
        "angle": ("fixed", 0.0),
        "row_shift": ("fixed", 0.0),
        "division_points": ("fixed", 0.0),
        "side_shift": ("fixed", 0.0),
        "shift_balance": ("fixed", 0.0),
        "skew_shift": ("fixed", 0.0),
        "div_dist": ("fixed", 0.0),
        "row_spacing": ("fixed", 0.0),
    }
    for key, default_value in optional_profile_defaults.items():
        if key in param_config:
            validated[key] = _validate_param_spec(key, param_config[key], errors)
        else:
            validated[key] = default_value

    image_modes = ("image_r", "image_g", "image_b")
    if any(
        isinstance(validated.get(key), (list, tuple)) and validated[key][0] in image_modes
        for key in validated
    ) and not img:
        _report(
            "img is required when param_config uses image_* modes.",
            gh.Kernel.GH_RuntimeMessageLevel.Error,
            errors,
        )

    return validated if not errors else None


def _max_slice_rows(surface, layer_height):
    """Return the number of +Z contour rows available for a wall geometry."""
    bbox = _surface_bbox(surface)
    height = max(0.0, bbox.Max.Z - bbox.Min.Z)
    return max(1, int(math.floor(height / float(layer_height))) + 1)


def _validate_surface_stack(front_srf, back_srf, layers, layer_height, errors):
    """Validate wall inputs and return the effective row count cap."""
    if front_srf is None or back_srf is None or layer_height is None:
        return None

    tol = _model_tolerance()
    if back_srf is not None and not _is_planar_wall(back_srf, tol):
        _report("back_srf must be planar.", gh.Kernel.GH_RuntimeMessageLevel.Error, errors)
        return None

    max_rows = min(_max_slice_rows(front_srf, layer_height), _max_slice_rows(back_srf, layer_height))
    if layers is None or int(layers) <= 0:
        return max_rows
    return min(int(layers), max_rows)


def _coerce_preset_value(value):
    if value is None:
        return -1
    preset_idx = int(round(float(value)))
    if preset_idx <= 0:
        return -1
    return gl.minmaxcap(0, 14, preset_idx - 1)


def _get_preset_distribution_mode(preset_idx):
    """Return whether a preset should use regular grid or guide/random seeding."""
    return "grid" if int(preset_idx) in (9, 10, 11, 12, 13, 14) else "guide"


def _get_preset_param_config(preset_idx, defaults):
    base_width = max(8.0, float(defaults["opening_width"] if defaults["opening_width"] is not None else 8.0))
    base_steps = max(1.0, float(defaults["steps"] if defaults["steps"] is not None else 1.0))
    base_density = gl.minmaxcap(0.1, 1.0, float(defaults["density"] if defaults["density"] is not None else 1.0))
    base_division_points = max(0.0, float(defaults["division_points"] if defaults["division_points"] is not None else 0.0))
    base_div_dist = max(1.0, float(defaults["div_dist"] if defaults["div_dist"] is not None else 1.0))
    base_num_u = max(1.0, float(defaults["num_u"] if defaults["num_u"] is not None else 1.0))
    base_grid_spacing = max(base_width, base_div_dist * base_num_u)
    base_row_shift = float(defaults["row_shift"] if defaults["row_shift"] is not None else 0.0)
    base_side_shift = float(defaults["side_shift"] if defaults["side_shift"] is not None else 0.0)
    base_shift_balance = gl.minmaxcap(-1.0, 1.0, float(defaults["shift_balance"] if defaults["shift_balance"] is not None else 0.0))
    base_skew_shift = float(defaults["skew_shift"] if defaults["skew_shift"] is not None else 0.0)

    presets = {
        0: {
            "width": ("gradient", {"axis": "u", "mode": "linear", "range": (0.55 * base_width, 1.15 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", base_density),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", base_division_points),
            "side_shift": ("fixed", base_side_shift),
            "shift_balance": ("fixed", base_shift_balance),
            "skew_shift": ("fixed", base_skew_shift),
        },
        1: {
            "width": ("gradient", {"axis": "u", "mode": "reverse", "range": (0.55 * base_width, 1.2 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", base_density),
            "steps": ("gradient", {"axis": "v", "mode": "linear", "range": (base_steps, 0.6 * base_steps)}) ,
            "division_points": ("fixed", base_division_points),
            "side_shift": ("fixed", base_side_shift),
            "shift_balance": ("fixed", base_shift_balance),
            "skew_shift": ("gradient", {"axis": "v", "mode": "linear", "range": (0, 1.5 *base_skew_shift)}),
        },
        2: {
            "width": ("gradient", {"axis": "u", "mode": "peak", "range": (0.35 * base_width, 1.25 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("gradient", {"axis": "v", "mode": "valley", "range": (0.25 * base_density, min(1.0, 1.25 * base_density))}),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", base_division_points),
            "side_shift": ("fixed", base_side_shift),
            "shift_balance": ("fixed", base_shift_balance),
            "skew_shift": ("fixed", base_skew_shift),
        },
        3: {
            "width": ("gradient", {"axis": "u", "mode": "smoothpeak", "range": (0.45 * base_width, 1.15 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("gradient", {"axis": "u", "mode": "smoothvalley", "range": (0.35 * base_density, 1.0)}),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", max(1.0, base_division_points)),
            "side_shift": ("gradient", {"axis": "u", "mode": "smoothpeak", "range": (0.0, max(0.18, abs(base_side_shift), 0.22))}),
            "shift_balance": ("fixed", 0.0),
            "skew_shift": ("fixed", base_skew_shift),
        },
        4: {
            "width": (
                "gradient_stops",
                {"axis": "v", "stops": ((0.0, 0.5 * base_width),  (0.7, 1 * base_width))},
            ),
            "depth": ("fixed", 0.0),
            "density": (
                "gradient_stops",
                {"axis": "v", "stops": ((0.0, 0.2 * base_density), (0.35, 1.0 * base_density), (0.5, 0.0 * base_density), (1.0, 1.2 * base_density))},
            ),
            "steps": ("fixed", base_steps),
            "division_points": ("gradient", {"axis": "v", "mode": "linear", "range": (0.0, max(1.0, base_steps - 1.0))}),
            "side_shift": ("fixed", base_side_shift if abs(base_side_shift) > 1e-9 else 0.12),
            "shift_balance": ("fixed", 0.0),
            "skew_shift": ("gradient", {"axis": "v", "mode": "linear", "range": (0.0, base_skew_shift)}),
        },
        5: {
            "width": ("gradient", {"axis": "z", "mode": "linear", "domain": (0.0, 1800.0), "range": (0.45 * base_width, 0.95 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", min(1.0, 1.1 * base_density)),
            "steps": ("gradient", {"axis": "z", "mode": "reverse", "domain": (0.0, 1800.0), "range": (max(1.0, base_steps - 2.0), base_steps)}),
            "division_points": ("fixed", max(1.0, base_division_points)),
            "side_shift": ("fixed", base_side_shift),
            "shift_balance": ("fixed", base_shift_balance),
            "skew_shift": ("gradient", {"axis": "z", "mode": "linear", "domain": (0.0, 1800.0), "range": (0.0, max(12.0, abs(base_skew_shift), 18.0))}),
        },
        6: {
            "width": (
                "composite",
                {
                    "operation": "multiply",
                    "sources": (
                        ("gradient", {"axis": "u", "mode": "smoothpeak", "range": (0.45, 1.0)}),
                        ("gradient", {"axis": "v", "mode": "peak", "range": (0.55 * base_width, 1.35 * base_width)}),
                    ),
                },
            ),
            "depth": ("fixed", 0.0),
            "density": ("gradient", {"axis": "v", "mode": "linear", "range": (0.35 * base_density, min(1.0, 1.15 * base_density))}),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", max(2.0, base_division_points)),
            "side_shift": ("fixed", base_side_shift if abs(base_side_shift) > 1e-9 else 0.18),
            "shift_balance": ("gradient", {"axis": "u", "mode": "linear", "range": (-0.75, 0.75)}),
            "skew_shift": ("fixed", base_skew_shift),
        },
        7: {
            "width": ("gradient", {"axis": "u", "mode": "linear", "range": (0.45 * base_width, 1.1 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("gradient", {"axis": "v", "mode": "linear", "range": (0.4 * base_density, 1.0)}),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", max(2.0, base_division_points)),
            "side_shift": ("fixed", -max(0.15, abs(base_side_shift))),
            "shift_balance": ("gradient", {"axis": "u", "mode": "linear", "range": (-1.0, 1.0)}),
            "skew_shift": ("fixed", base_skew_shift),
        },
        8: {
            "width": ("gradient", {"axis": "u", "mode": "peak", "range": (0.4 * base_width, 0.95 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", min(1.0, 1.1 * base_density)),
            "steps": ("gradient", {"axis": "v", "mode": "linear", "range": (max(1.0, base_steps - 2.0), base_steps)}),
            "division_points": ("fixed", max(1.0, base_division_points)),
            "side_shift": ("fixed", base_side_shift if abs(base_side_shift) > 1e-9 else 0.1),
            "shift_balance": ("fixed", 0.0),
            "skew_shift": ("gradient", {"axis": "v", "mode": "reverse", "range": (0.0, max(15.0, abs(base_skew_shift), 24.0))}),
        },
        9: {
            "width": (
                "composite",
                {
                    "operation": "multiply",
                    "sources": (
                        ("gradient", {"axis": "u", "mode": "linear", "range": (20.0, 150.0)}),
                        ("gradient", {"axis": "v", "mode": "linear", "range": (1, 1.5)}),
                    ),
                },
            ),
            "angle": ("gradient", {"axis": "v", "mode": "linear", "range": (0.0, 45.0)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", 1.0),
            "row_shift": ("fixed", base_row_shift),
            "steps": ("gradient", {"axis": "u", "mode": "linear", "range": (max(1.0, base_steps - 5), base_steps + 15)}),
            "division_points": ("gradient", {"axis": "u", "mode": "linear", "range": (0, 5)}),
            "side_shift": ("gradient", {"axis": "v", "mode": "linear", "range": (0,1.5)}),
            "shift_balance": ("gradient", {"axis": "u", "mode": "linear", "range": (-1.0, 1.0)}),
            "skew_shift": ("gradient", {"axis": "u", "mode": "linear", "range": (0 , -100)}),
            "div_dist": (
                "composite",
                {
                    "operation": "multiply",
                    "sources": (
                        ("gradient", {"axis": "u", "mode": "linear", "range": (50.0, 200.0)}),
                        ("gradient", {"axis": "u", "mode": "linear", "range": (1, 1.5)}),
                        ("gradient", {"axis": "v", "mode": "linear", "range": (1, 1.5)}),
                    ),
                },
            ),
            "row_spacing": ("gradient", {"axis": "u", "mode": "linear", "range": (max(0.0, base_steps - 8), base_steps + 5)}),
        },
        10: {
            "width": (
                "composite",
                {
                    "operation": "multiply",
                    "sources": (
                        ("gradient", {"axis": "u", "mode": "linear", "range": (30.0, 200.0)}),
                        ("gradient", {"axis": "v", "mode": "linear", "range": (1, 1.5)}),
                    ),
                },
            ),
            #"angle": ("gradient", {"axis": "v", "mode": "linear", "range": (0.0, 45.0)}),
            "angle": ("fixed", 45.0),
            "depth": ("fixed", 0.0),
            "density": ("fixed", 1.0),
            "row_shift": ("fixed", base_row_shift),
            "steps": ("gradient", {"axis": "u", "mode": "linear", "range": (max(1.0, base_steps - 5), base_steps + 15)}),
            "division_points": ("gradient", {"axis": "u", "mode": "linear", "range": (0, 5)}),
            "side_shift": ("gradient", {"axis": "v", "mode": "linear", "range": (0,1.5)}),
            "shift_balance": ("gradient", {"axis": "u", "mode": "linear", "range": (-1.0, 1.0)}),
            "skew_shift": ("gradient", {"axis": "u", "mode": "linear", "range": (0 , -100)}),
            "div_dist": (
                "composite",
                {
                    "operation": "multiply",
                    "sources": (
                        ("gradient", {"axis": "u", "mode": "linear", "range": (60.0, 300.0)}),
                        ("gradient", {"axis": "v", "mode": "linear", "range": (1, 1.5)}),
                    ),
                },
            ),
            "row_spacing": ("gradient", {"axis": "u", "mode": "linear", "range": (max(0.0, base_steps - 8), base_steps + 5)}),
        },
        11: {
            "width": ("gradient", {"axis": "u", "mode": "smoothvalley", "range": (0.7 * base_width, 1.05 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", 1.0),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", 0.0),
            "side_shift": ("gradient", {"axis": "u", "mode": "smoothpeak", "range": (0.0, max(0.12, abs(base_side_shift), 0.18))}),
            "shift_balance": ("gradient", {"axis": "u", "mode": "linear", "range": (-0.35, 0.35)}),
            "skew_shift": ("fixed", base_skew_shift),
            "div_dist": ("gradient_stops", {"axis": "u", "stops": ((0.0, 0.9 * base_grid_spacing), (0.45, 1.4 * base_grid_spacing), (1.0, 2.3 * base_grid_spacing))}),
        },
        12: {
            "width": ("gradient", {"axis": "u", "mode": "linear", "range": (0.85 * base_width, 1.0 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", 1.0),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", 0.0),
            "side_shift": ("fixed", base_side_shift),
            "shift_balance": ("fixed", base_shift_balance),
            "skew_shift": ("fixed", base_skew_shift),
            "div_dist": ("gradient", {"axis": "u", "mode": "reverse", "range": (0.9 * base_grid_spacing, 2.6 * base_grid_spacing)}),
        },
        13: {
            "width": ("gradient", {"axis": "u", "mode": "smoothpeak", "range": (0.75 * base_width, 1.05 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", 1.0),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", 0.0),
            "side_shift": ("fixed", base_side_shift),
            "shift_balance": ("fixed", base_shift_balance),
            "skew_shift": ("fixed", base_skew_shift),
            "div_dist": ("gradient", {"axis": "u", "mode": "peak", "range": (1.0 * base_grid_spacing, 2.8 * base_grid_spacing)}),
        },
        14: {
            "width": ("gradient", {"axis": "u", "mode": "smoothvalley", "range": (0.8 * base_width, 1.0 * base_width)}),
            "depth": ("fixed", 0.0),
            "density": ("fixed", 1.0),
            "steps": ("fixed", base_steps),
            "division_points": ("fixed", 0.0),
            "side_shift": ("fixed", base_side_shift),
            "shift_balance": ("fixed", base_shift_balance),
            "skew_shift": ("fixed", base_skew_shift),
            "div_dist": ("gradient", {"axis": "u", "mode": "valley", "range": (1.05 * base_grid_spacing, 2.5 * base_grid_spacing)}),
        },
    }
    return presets.get(int(preset_idx))


def _validate_inputs():
    """Validate Grasshopper inputs and return normalized values."""
    errors = []

    _, front_srf_value = _validate_value(
        _get_first_param(("front_srf", "front"), None),
        object,
        "front_srf/front is required and must be a valid surface or Brep.",
        coercer=_coerce_wall_geometry,
    )
    _, back_srf_value = _validate_value(
        _get_first_param(("back_srf", "back"), None),
        object,
        "back_srf/back is required and must be a valid planar surface or Brep.",
        coercer=_coerce_wall_geometry,
    )
    guide_srf_raw = _get_first_param(("guide_srf", "guide"), None)
    guide_srf_ok, guide_srf_value = _validate_value(
        guide_srf_raw,
        object,
        "guide_srf/guide must be a valid surface or Brep when provided.",
        coercer=_coerce_wall_geometry,
        allow_none=True,
    )

    img_value = _get_param("img", None)
    layers_value = _validate_scalar("layers", _get_param("layers", None), errors, minimum=0)
    layer_height_value = _validate_scalar("layer_height", _get_param("layer_height", None), errors)
    div_dist_value = _validate_scalar("div_dist", _get_param("div_dist", None), errors, minimum=0)
    num_u_value = _validate_scalar("num_u", _get_param("num_u", None), errors, minimum=0)
    num_v_value = _validate_scalar("num_v", _get_param("num_v", None), errors, minimum=0)
    steps_value = _validate_scalar("steps", _get_param("steps", None), errors, minimum=0)
    opening_width_value = _validate_scalar("opening_width", _get_param("opening_width", None), errors, minimum=0, allow_zero=True)
    angle_value = _validate_scalar("angle", _get_param("angle", None), errors)
    nozzle_value = _validate_scalar("nozzle", _get_first_param(("nozzle", "N"), None), errors, minimum=0)
    overlap_value = _validate_scalar("overlap", _get_param("overlap", 0.1), errors, minimum=0, allow_zero=True)
    division_points_value = _validate_scalar("division_points", _get_param("division_points", 0.0), errors, minimum=0, allow_zero=True)
    side_shift_value = _validate_scalar("side_shift", _get_param("side_shift", 0.0), errors, allow_zero=True)
    shift_balance_value = _validate_scalar("shift_balance", _get_param("shift_balance", 0.0), errors, allow_zero=True)
    skew_shift_value = _validate_scalar("skew_shift", _get_param("skew_shift", 0.0), errors, allow_zero=True)
    backface_reduction_value = _validate_scalar(
        "backface_reduction",
        _get_first_param(("backface_reduction", "backfaceReduction", "backface"), 0.0),
        errors,
        minimum=0,
        allow_zero=True,
    )

    if front_srf_value is None:
        _report(
            "front_srf/front is required and must be a valid surface or Brep.",
            gh.Kernel.GH_RuntimeMessageLevel.Error,
            errors,
        )
    if back_srf_value is None:
        _report(
            "back_srf/back is required and must be a valid planar surface or Brep.",
            gh.Kernel.GH_RuntimeMessageLevel.Error,
            errors,
        )
    if guide_srf_raw is not None and not guide_srf_ok:
        _report(
            "guide_srf/guide must be a valid surface or Brep when provided.",
            gh.Kernel.GH_RuntimeMessageLevel.Error,
            errors,
        )

    u_shift_value = _validate_scalar("u_shift", _get_param("u_shift", DEFAULT_SHIFT_U), errors)
    v_shift_value = _validate_scalar("v_shift", _get_param("v_shift", DEFAULT_SHIFT_V), errors)
    row_shift_value = _validate_scalar("row_shift", _get_param("row_shift", 0.0), errors, allow_zero=True)
    u_scale_value = _validate_scalar("u_scale", _get_param("u_scale", 1.0), errors)
    v_scale_value = _validate_scalar("v_scale", _get_param("v_scale", 1.0), errors)
    opening_scale_value = _validate_scalar("opening_scale", _get_param("opening_scale", 1.0), errors, minimum=0, allow_zero=True)
    gate_threshold_value = _validate_scalar("gate_threshold", _get_param("gate_threshold", 0.5), errors)
    seed_row_start_value = _validate_scalar("seed_row_start", _get_param("seed_row_start", 5), errors, minimum=0, allow_zero=True)
    density_input = _get_param("density", _get_param("density_slider", 1.0))
    density_value = _validate_scalar("density", density_input, errors, minimum=0, allow_zero=True)
    random_seed_value = _validate_scalar("random_seed", _get_param("random_seed", 1.0), errors, minimum=0, allow_zero=True)
    preset_value = _validate_scalar("preset", _get_first_param(("preset", "Preset", "slider", "Slider"), -1), errors, minimum=-1, allow_zero=True)
    max_overhang_value = _validate_scalar("max_overhang", _get_param("max_overhang", 0.0), errors, minimum=0, allow_zero=True)
    generate_support_value = bool(_get_param("generate_support", False))
    v_reverse_value = bool(_get_param("v_reverse", False))

    if max_overhang_value is not None and max_overhang_value >= 90.0:
        _report(
            "max_overhang must be less than 90 degrees.",
            gh.Kernel.GH_RuntimeMessageLevel.Error,
            errors,
        )

    if gate_threshold_value is not None:
        gate_threshold_value = gl.minmaxcap(0.0, 1.0, gate_threshold_value)
    if density_value is not None:
        density_value = gl.minmaxcap(0.0, 1.0, density_value)
    if overlap_value is not None:
        overlap_value = _normalize_overlap(overlap_value)
    if shift_balance_value is not None:
        shift_balance_value = gl.minmaxcap(-1.0, 1.0, shift_balance_value)
    if backface_reduction_value is not None:
        backface_reduction_value = gl.minmaxcap(0.0, 1.0, backface_reduction_value)

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
    if preset_value is not None:
        preset_value = _coerce_preset_value(preset_value)

    effective_layers_value = _validate_surface_stack(
        front_srf_value,
        back_srf_value,
        layers_value,
        layer_height_value,
        errors,
    )

    param_config_value = _get_param("param_config", None)
    if preset_value is not None and preset_value >= 0:
        param_config_value = _get_preset_param_config(
            preset_value,
            {
                "opening_width": opening_width_value,
                "steps": steps_value,
                "density": density_value,
                "division_points": int(round(division_points_value)) if division_points_value is not None else 0,
                "div_dist": div_dist_value,
                "num_u": num_u_value,
                "row_shift": row_shift_value,
                "side_shift": side_shift_value,
                "shift_balance": shift_balance_value,
                "skew_shift": skew_shift_value,
            },
        )
        if param_config_value is None:
            _report("preset must be an integer from 0 to 15, where 0 disables presets.", gh.Kernel.GH_RuntimeMessageLevel.Error, errors)
    if param_config_value is not None:
        param_config_value = _validate_param_config(param_config_value, img_value, errors, steps_value)

    distribution_mode_value = "random" if param_config_value is not None else "grid"
    if preset_value is not None and preset_value >= 0:
        distribution_mode_value = _get_preset_distribution_mode(preset_value)

    if (
        opening_width_value is not None
        and div_dist_value is not None
        and opening_width_value > div_dist_value
        and param_config_value is not None
        and param_config_value["width"][0] == "fixed"
    ):
        _report(
            "opening_width is larger than div_dist; clamping fixed opening_width to div_dist for seed spacing."
        )
        opening_width_value = div_dist_value
        param_config_value["width"] = ("fixed", float(opening_width_value) * opening_scale_value)

    if errors:
        return None

    return {
        "front_srf": front_srf_value,
        "back_srf": back_srf_value,
        "guide_srf": guide_srf_value if guide_srf_value is not None else front_srf_value,
        "effective_layers": effective_layers_value,
        "img": img_value,
        "layers": layers_value,
        "layer_height": layer_height_value,
        "div_dist": div_dist_value,
        "num_u": num_u_value,
        "num_v": num_v_value,
        "steps": steps_value,
        "opening_width": opening_width_value,
        "angle": angle_value,
        "nozzle": nozzle_value,
        "overlap": overlap_value,
        "division_points": int(round(division_points_value)) if division_points_value is not None else 0,
        "side_shift": side_shift_value,
        "shift_balance": shift_balance_value,
        "skew_shift": skew_shift_value,
        "backface_reduction": backface_reduction_value,
        "u_shift": u_shift_value,
        "v_shift": v_shift_value,
        "row_shift": row_shift_value,
        "u_scale": u_scale_value,
        "v_scale": v_scale_value,
        "v_reverse": v_reverse_value,
        "opening_scale": opening_scale_value,
        "gate_threshold": gate_threshold_value,
        "seed_row_start": seed_row_start_value,
        "density": density_value,
        "random_seed": int(random_seed_value) if random_seed_value is not None else 1,
        "preset": preset_value,
        "max_overhang": max_overhang_value,
        "generate_support": generate_support_value,
        "param_config": param_config_value,
        "distribution_mode": distribution_mode_value,
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


def _sample_guide_data(pt, front_srf, img):
    """Sample normalized UV and optional RGB data at the closest point."""
    uv, _face, u_dom, v_dom = _closest_surface_data(front_srf, pt)
    u_norm = gl.invlerp(u_dom[0], u_dom[1], uv[0])
    v_norm = gl.invlerp(v_dom[0], v_dom[1], uv[1])
    rgb = sample_surface_color(pt, front_srf, img) if img else None
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


def _evaluate_config_value(param_config, key, pt, uv, rgb):
    mode, settings = param_config[key]
    return evaluate_parameter(mode, settings, pt, uv, rgb)


def _evaluate_opening_params(param_config, pt, uv, rgb):
    width = max(0.0, float(_evaluate_config_value(param_config, "width", pt, uv, rgb)))
    step_value = max(1, int(round(_evaluate_config_value(param_config, "steps", pt, uv, rgb))))
    division_points_value = max(
        0,
        int(round(_evaluate_config_value(param_config, "division_points", pt, uv, rgb))),
    )
    return {
        "width": width,
        "angle": float(_evaluate_config_value(param_config, "angle", pt, uv, rgb)),
        "row_shift": int(round(_evaluate_config_value(param_config, "row_shift", pt, uv, rgb))),
        "steps": step_value,
        "density": gl.minmaxcap(0.0, 1.0, float(_evaluate_config_value(param_config, "density", pt, uv, rgb))),
        "division_points": min(division_points_value, step_value),
        "div_dist": max(0.0, float(_evaluate_config_value(param_config, "div_dist", pt, uv, rgb))),
        "row_spacing": max(0, int(round(_evaluate_config_value(param_config, "row_spacing", pt, uv, rgb)))),
        "side_shift": float(_evaluate_config_value(param_config, "side_shift", pt, uv, rgb)),
        "shift_balance": gl.minmaxcap(
            -1.0,
            1.0,
            float(_evaluate_config_value(param_config, "shift_balance", pt, uv, rgb)),
        ),
        "skew_shift": float(_evaluate_config_value(param_config, "skew_shift", pt, uv, rgb)),
    }


def _build_fixed_param_config(validated):
    return {
        "width": ("fixed", validated["opening_width"] * validated["opening_scale"]),
        "angle": ("fixed", validated["angle"]),
        "row_shift": ("fixed", float(validated["row_shift"])),
        "steps": ("fixed", validated["steps"]),
        "density": ("fixed", 1.0),
        "depth": ("fixed", 0.0),
        "division_points": ("fixed", float(validated["division_points"])),
        "div_dist": ("fixed", 0.0),
        "row_spacing": ("fixed", float(validated["num_v"])),
        "side_shift": ("fixed", float(validated["side_shift"])),
        "shift_balance": ("fixed", float(validated["shift_balance"])),
        "skew_shift": ("fixed", float(validated["skew_shift"])),
    }


def _resolve_opening_geometry(
    pt,
    front_crv,
    back_srf,
    back_plane,
    width,
    angle,
    nozzle,
    overlap,
    disp_width=None,
    division_points=0,
    side_shift=0.0,
    shift_balance=0.0,
    skew_shift=0.0,
    skew_t=0.0,
    backface_reduction=0.0,
):
    """Resolve the opening geometry and support axes for one row event."""
    pt = _point3d(pt)
    pt_param = rs.CurveClosestPoint(front_crv, pt)
    tangent = _vector3d(rs.VectorUnitize(rs.CurveTangent(front_crv, pt_param)))
    pt = _add_point_vector(pt, _scale(tangent, float(skew_shift) * float(skew_t)))
    constant_back_width = width if disp_width is None else max(float(disp_width), float(width))
    backface_reduction = gl.minmaxcap(0.0, 1.0, float(backface_reduction))
    inner_width = gl.lerp(constant_back_width, float(width), backface_reduction)
    ref_width = max(float(width), float(constant_back_width))
    left_outer, right_outer = _profile_side_widths(width, ref_width, skew_t, division_points, side_shift, shift_balance)
    left_inner, right_inner = _profile_side_widths(inner_width, ref_width, skew_t, division_points, side_shift, shift_balance)
    pt_before = _add_point_vector(pt, _scale(tangent, -left_outer))
    pt_after = _add_point_vector(pt, _scale(tangent, right_outer))
    pt_before_inner = _add_point_vector(pt, _scale(tangent, -left_inner))
    pt_after_inner = _add_point_vector(pt, _scale(tangent, right_inner))
    stand_off = float(nozzle) * (1.0 - float(overlap))
    before_data = _offset_data_from_back_surface(pt_before_inner, back_srf, back_plane, tangent, angle, stand_off)
    after_data = _offset_data_from_back_surface(pt_after_inner, back_srf, back_plane, tangent, angle, stand_off)
    pt_before_disp = before_data["point"]
    pt_after_disp = after_data["point"]
    return {
        "points": (pt_before, pt_before_disp, pt_after_disp, pt_after),
        "support_axes": (
            (pt_before_inner, pt_before_disp, _sub_points(pt_before_disp, pt_before_inner), before_data["depth"], 1),
            (pt_after_inner, pt_after_disp, _sub_points(pt_after_disp, pt_after_inner), after_data["depth"], -1),
        ),
    }


def add_wall_opening(
    pt,
    front_crv,
    back_srf,
    back_plane,
    width,
    angle,
    nozzle,
    overlap,
    disp_width=None,
    division_points=0,
    side_shift=0.0,
    shift_balance=0.0,
    skew_shift=0.0,
    skew_t=0.0,
    backface_reduction=0.0,
):
    """Create the standard row opening with side widths biased over step progress."""
    return _resolve_opening_geometry(
        pt,
        front_crv,
        back_srf,
        back_plane,
        width,
        angle,
        nozzle,
        overlap,
        disp_width,
        division_points,
        side_shift,
        shift_balance,
        skew_shift,
        skew_t,
        backface_reduction,
    )["points"]


def add_support_indent(pt, front_crv, axis_vector, width, disp_distance, full_depth, inner_edge_shift=0.0, inner_side=0):
    """Create a back-side support indentation from a propagated axis.

    On the last support layer below an opening, only the inner edge point is
    shifted forward in the support direction: `pt_after` for the left support
    and `pt_before` for the right support. The displaced target points stay
    unchanged so the support reach is preserved.
    """
    axis_vector = _vector3d(axis_vector)
    full_depth = float(full_depth)
    if axis_vector.Length <= 1e-9 or disp_distance <= 1e-9 or full_depth <= 1e-9:
        return (pt,)

    pt = _point3d(pt)
    pt_param = rs.CurveClosestPoint(front_crv, pt)
    tangent = _vector3d(rs.VectorUnitize(rs.CurveTangent(front_crv, pt_param)))
    half_width = max(0.0, float(width) / 2.0)
    axis_scale = float(disp_distance) / full_depth
    target_pt = _add_point_vector(pt, _scale(axis_vector, axis_scale))
    pt_before = _add_point_vector(pt, _scale(tangent, -half_width))
    pt_after = _add_point_vector(pt, _scale(tangent, half_width))
    pt_before_disp = _add_point_vector(target_pt, _scale(tangent, -half_width))
    pt_after_disp = _add_point_vector(target_pt, _scale(tangent, half_width))
    if inner_edge_shift > 1e-9 and inner_side != 0:
        forward = Rhino.Geometry.Vector3d(axis_vector)
        forward.Unitize()
        if inner_side > 0:
            pt_after = _add_point_vector(pt_after, _scale(forward, float(inner_edge_shift)))
        else:
            pt_before = _add_point_vector(pt_before, _scale(forward, float(inner_edge_shift)))
    return (pt_before, pt_before_disp, pt_after_disp, pt_after)


def _build_angled_back_stacks(front_point_stack, front_curve_stack, back_srf, angle, nozzle, overlap):
    """Build back-row points/curves in the same parametric order as the front rows.

    Each back row point is projected from its corresponding front sample onto the
    back surface, then shifted laterally by the same angle term used by the
    opening geometry. The normal stand-off is not applied here so the support
    baseline stays on the back face while still matching the opening angle in plan.
    """
    back_plane = _planar_wall_plane(back_srf)
    stand_off = float(nozzle) * (1.0 - float(overlap))
    angled_point_stack = []
    angled_curve_stack = []

    for row_pts, front_curve in zip(front_point_stack, front_curve_stack):
        angled_row = []
        for pt in row_pts:
            curve_param = rs.CurveClosestPoint(front_curve, pt)
            tangent = _vector3d(rs.VectorUnitize(rs.CurveTangent(front_curve, curve_param)))
            offset_data = _offset_data_from_back_surface(pt, back_srf, back_plane, tangent, angle, stand_off)
            angled_row.append(offset_data["back_shifted_point"])

        angled_point_stack.append(angled_row)
        if len(angled_row) >= 3:
            angled_curve_stack.append(Rhino.Geometry.Curve.CreateInterpolatedCurve(angled_row, 3))
        elif len(angled_row) == 2:
            angled_curve_stack.append(Rhino.Geometry.LineCurve(angled_row[0], angled_row[1]))
        elif len(angled_row) == 1:
            angled_curve_stack.append(Rhino.Geometry.PolylineCurve(angled_row))
        else:
            angled_curve_stack.append(None)

    return angled_point_stack, angled_curve_stack


def build_curve_stacks(
    front_srf,
    back_srf,
    layer_height,
    div_dist,
    angle,
    nozzle,
    overlap,
    max_rows=None,
    img=None,
    param_config=None,
    distribution_mode="grid",
):
    """Slice both surfaces in +Z and build front/back rows.

    In the regular scaffold mode with a preset-driven `param_config`, front rows
    can be sampled from a varying target `div_dist` field instead of a uniform
    equal-length subdivision. Other modes keep the existing uniform sampling.
    """
    front_curve_stack = _slice_surface_rows(front_srf, layer_height, max_rows)
    raw_back_curve_stack = _slice_surface_rows(back_srf, layer_height, max_rows)
    row_count = min(len(front_curve_stack), len(raw_back_curve_stack))
    if row_count <= 0:
        raise ValueError("Unable to create contour rows from front_srf and back_srf.")
    front_curve_stack = front_curve_stack[:row_count]
    raw_back_curve_stack = raw_back_curve_stack[:row_count]

    front_point_stack = []
    for front_curve in front_curve_stack:
        if distribution_mode == "grid" and param_config is not None:
            front_point_stack.append(
                _sample_curve_points_by_target_distance(
                    front_curve,
                    div_dist,
                    front_srf,
                    img,
                    param_config,
                )
            )
        else:
            sample_count = max(2, int(rs.CurveLength(front_curve) / float(div_dist)) + 1)
            front_point_stack.append(_sample_curve_points(front_curve, div_dist, sample_count))

    back_point_stack, back_curve_stack = _build_angled_back_stacks(
        front_point_stack,
        front_curve_stack,
        back_srf,
        angle,
        nozzle,
        overlap,
    )

    return front_point_stack, front_curve_stack, back_point_stack, back_curve_stack


def build_regular_seed_stack(
    point_stack,
    curve_stack,
    num_u,
    num_v,
    step_count,
    u_shift,
    v_shift,
    v_scale,
    v_reverse,
    seed_row_start,
    use_all_u=False,
    front_srf=None,
    img=None,
    param_config=None,
):
    """Regular scaffold-aligned sampling used for the grid distribution mode."""
    row_count = len(point_stack)
    opening_rows = set()
    bottom_margin_layers = max(2, int(seed_row_start))
    row_shift = 0
    v_cursor = bottom_margin_layers

    while v_cursor < row_count:
        local_step = max(1, int(step_count))
        v_inc = int((v_scale - 1.0) * v_cursor)
        local_v = max(0, num_v + v_inc)

        if param_config is not None and point_stack[v_cursor]:
            row = point_stack[v_cursor]
            sample_indices = sorted(set([0, len(row) // 2, len(row) - 1]))
            row_steps = []
            row_spacings = []
            for idx in sample_indices:
                row_pt = row[idx]
                uv, rgb = _sample_guide_data(row_pt, front_srf, img)
                row_params = _evaluate_opening_params(param_config, row_pt, uv, rgb)
                row_steps.append(int(row_params["steps"]))
                row_spacings.append(int(row_params["row_spacing"]))
                row_shift = int(row_params["row_shift"])
            if row_steps:
                local_step = max(1, max(row_steps))
            if row_spacings:
                local_v = max(local_v, max(0, max(row_spacings)))

        if _has_vertical_clearance(v_cursor, local_step, row_count, bottom_margin_layers):
            opening_rows.add(v_cursor)
        v_cursor += local_step + local_v

    if row_shift != 0:
        opening_rows = {
            max(bottom_margin_layers, min(row_count - 1, row_idx + row_shift))
            for row_idx in opening_rows
        }

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
            if is_endpoint:
                is_seed = False
            elif use_all_u:
                is_seed = True
            else:
                is_seed = ((u - 1 + u_phase_steps) % num_u == 0)
            bool_row.append(1 if (is_open_row and is_seed) else 0)
        seed_stack.append(bool_row)

    return seed_stack


def _curve_param_from_normalized(curve, t_norm):
    domain = rs.CurveDomain(curve)
    return gl.lerp(domain[0], domain[1], t_norm)


def _normalized_curve_parameter(curve, curve_param):
    domain = rs.CurveDomain(curve)
    return gl.invlerp(domain[0], domain[1], curve_param)


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
    front_srf,
    img,
    param_config,
    density,
    div_dist,
    random_seed,
):
    """Generate random non-overlapping seeds along curves using guide-driven density."""
    if front_srf is None:
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
            uv, rgb = _sample_guide_data(pt, front_srf, img)
            opening_params = _evaluate_opening_params(param_config, pt, uv, rgb)
            width_value = opening_params["width"]
            if not _has_edge_clearance(curve, curve_param, width_value):
                continue
            step_value = opening_params["steps"]
            if not _has_vertical_clearance(row_idx, step_value, row_count, bottom_margin_layers):
                continue
            local_density = gl.minmaxcap(0.0, 1.0, density * opening_params["density"])
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
                    "angle": opening_params["angle"],
                    "steps": step_value,
                    "local_density": local_density,
                    "min_u_spacing": min_u_spacing,
                    "min_v_spacing": min_v_spacing,
                    "point": pt,
                    "division_points": opening_params["division_points"],
                    "side_shift": opening_params["side_shift"],
                    "shift_balance": opening_params["shift_balance"],
                    "skew_shift": opening_params["skew_shift"],
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


def build_seed_opening_events(point_stack, curve_stack, seed_stack, front_srf, img, param_config, enforce_div_dist_spacing=True):
    """Collect full-strength opening source events at seed rows only.

    When base points are already sampled from a varying target `div_dist` field,
    a second `div_dist` rejection pass would over-thin the row and appear random.
    In that mode we keep only the width-overlap safeguard here.
    """
    event_stack = [[] for _ in curve_stack]

    for row_idx, row in enumerate(point_stack):
        curve = curve_stack[row_idx]
        row_acceptance = []
        for col_idx, pt in enumerate(row):
            if not seed_stack[row_idx][col_idx]:
                continue

            if front_srf is None:
                uv, rgb = (0.0, 0.0), None
            else:
                uv, rgb = _sample_guide_data(pt, front_srf, img)

            opening_params = _evaluate_opening_params(param_config, pt, uv, rgb)
            local_density = gl.minmaxcap(0.0, 1.0, float(opening_params["density"]))
            if local_density <= 0.0:
                continue
            t_norm = gl.minmaxcap(0.0, 1.0, float(col_idx) / float(max(1, len(row) - 1)))
            if _stable_noise(17, row_idx, t_norm) > local_density:
                continue
            width = opening_params["width"]
            pt_param = rs.CurveClosestPoint(curve, pt)
            if not _has_edge_clearance(curve, pt_param, width):
                continue
            pt_arc = rs.CurveLength(curve, sub_domain=(rs.CurveDomain(curve)[0], pt_param))
            if enforce_div_dist_spacing:
                required_u_spacing = max(float(width), float(opening_params["div_dist"]))
            else:
                required_u_spacing = float(width)
            if any(abs(pt_arc - other_arc) < max(required_u_spacing, other_spacing) for other_arc, other_spacing in row_acceptance):
                continue

            event_stack[row_idx].append(
                (
                    pt_param,
                    width,
                    opening_params["angle"],
                    width,
                    0.0,
                    opening_params["steps"],
                    opening_params["division_points"],
                    opening_params["side_shift"],
                    opening_params["shift_balance"],
                    opening_params["skew_shift"],
                )
            )
            row_acceptance.append((pt_arc, required_u_spacing))

    return event_stack


def build_seed_points_from_stack(point_stack, seed_stack):
    """Collect the active scaffold seed points."""
    seed_points = []
    for row_idx, row in enumerate(point_stack):
        for col_idx, pt in enumerate(row):
            if seed_stack[row_idx][col_idx]:
                seed_points.append(pt)
    return seed_points


def build_seed_points_from_source_events(curve_stack, source_event_stack):
    """Collect the actual accepted source opening points from event rows."""
    seed_points = []
    for row_idx, row_events in enumerate(source_event_stack):
        curve = curve_stack[row_idx]
        for event in row_events:
            seed_points.append(rs.EvaluateCurve(curve, event[0]))
    return seed_points


def _opening_taper_width(width, step_count, step_idx, nozzle, overlap):
    """Return propagated opening width with a fixed nozzle-overlap top standoff."""
    step_count = max(1, int(step_count))
    stand_off = float(nozzle) * (1.0 - float(overlap))
    taper_t = 0.0 if step_count <= 1 else float(step_idx) / float(step_count - 1)
    return gl.lerp(float(width), stand_off, taper_t), taper_t


def _param_in_interval(param, interval, tol):
    return interval[0] - tol <= param <= interval[1] + tol


def build_opening_events(curve_stack, random_seeds, nozzle, overlap):
    """Propagate random seeds into per-row opening events."""
    event_stack = [[] for _ in curve_stack]

    for seed in random_seeds:
        for n in range(seed["steps"]):
            row_idx = seed["row"] + n
            if row_idx >= len(event_stack):
                break
            propagated, taper_t = _opening_taper_width(seed["width"], seed["steps"], n, nozzle, overlap)
            if propagated <= 0.0:
                continue
            event_stack[row_idx].append(
                (
                    seed["param"],
                    propagated,
                    seed["angle"],
                    seed["width"],
                    taper_t,
                    seed["division_points"],
                    seed["side_shift"],
                    seed["shift_balance"],
                    seed["skew_shift"],
                )
            )

    return event_stack


def build_opening_events_from_sources(curve_stack, source_event_stack, nozzle, overlap):
    """Propagate opening source events upward into per-row opening events."""
    event_stack = [[] for _ in curve_stack]

    for row_idx, source_events in enumerate(source_event_stack):
        for event in source_events:
            (
                event_param,
                width,
                angle,
                disp_width,
                _skew_t,
                step_count,
                division_points,
                side_shift,
                shift_balance,
                skew_shift,
            ) = event

            for n in range(int(step_count)):
                target_row_idx = row_idx + n
                if target_row_idx >= len(event_stack):
                    break
                propagated, taper_t = _opening_taper_width(width, step_count, n, nozzle, overlap)
                if propagated <= 0.0:
                    continue
                event_stack[target_row_idx].append(
                    (
                        event_param,
                        propagated,
                        angle,
                        disp_width,
                        taper_t,
                        division_points,
                        side_shift,
                        shift_balance,
                        skew_shift,
                    )
                )

    return event_stack


def build_random_seed_source_events(curve_stack, random_seeds):
    """Collect full-strength opening source events from accepted random seeds."""
    event_stack = [[] for _ in curve_stack]

    for seed in random_seeds:
        event_stack[seed["row"]].append(
            (
                seed["param"],
                seed["width"],
                seed["angle"],
                seed["width"],
                0.0,
                seed["steps"],
                seed["division_points"],
                seed["side_shift"],
                seed["shift_balance"],
                seed["skew_shift"],
            )
        )

    return event_stack


def build_opening_contours(source_event_stack, curve_stack, back_srf, angle, nozzle, overlap, backface_reduction):
    """Build one contour loop per source opening from pt_before and pt_after rows."""
    contours = []
    back_plane = _planar_wall_plane(back_srf)

    for row_idx, source_events in enumerate(source_event_stack):
        for event in source_events:
            (
                event_param,
                width,
                event_angle,
                disp_width,
                _skew_t,
                step_count,
                division_points,
                side_shift,
                shift_balance,
                skew_shift,
            ) = event

            before_pts = []
            after_pts = []
            for n in range(int(step_count)):
                target_row_idx = row_idx + n
                if target_row_idx >= len(curve_stack):
                    break
                propagated, taper_t = _opening_taper_width(width, step_count, n, nozzle, overlap)
                if propagated <= 0.0:
                    continue

                target_curve = curve_stack[target_row_idx]
                event_pt = rs.EvaluateCurve(target_curve, event_param)
                geometry = _resolve_opening_geometry(
                    event_pt,
                    target_curve,
                    back_srf,
                    back_plane,
                    propagated,
                    event_angle,
                    nozzle,
                    overlap,
                    disp_width,
                    division_points,
                    side_shift,
                    shift_balance,
                    skew_shift,
                    taper_t,
                    backface_reduction,
                )
                pt_before, _pt_before_disp, _pt_after_disp, pt_after = geometry["points"]
                before_pts.append(pt_before)
                after_pts.append(pt_after)

            if before_pts and after_pts:
                contours.append(before_pts + list(reversed(after_pts)))

    return contours


def build_seed_points_from_random_seeds(random_seeds):
    """Collect accepted random seed points."""
    return [seed["point"] for seed in random_seeds]


def build_support_events(curve_stack, back_curve_stack, back_srf, source_event_stack, angle, nozzle, overlap, layer_height, max_overhang, backface_reduction):
    """Propagate minimal back-side support indents into lower rows.

    The support depth tapers by `layer_height * tan(max_overhang)` per layer.
    Each support row is anchored on the angle-shifted back-face base point
    projected from the corresponding front-side support point. The support
    direction reuses the opening helper line on that row, reversed toward the
    front, so angle and translation are solved independently but coherently.
    On the last support layer below an opening, the inner segment between the
    two supports is shifted forward by the same ratio evaluated over one nozzle width:
    `(nozzle / layer_height) * reduction_per_layer`, i.e. `nozzle * tan(max_overhang)`.
    """
    support_event_stack = [[] for _ in back_curve_stack]
    if max_overhang is None or float(max_overhang) < 0.0:
        return support_event_stack

    back_plane = _planar_wall_plane(back_srf)
    stand_off = float(nozzle) * (1.0 - float(overlap))
    overlap_ratio = float(overlap)
    if overlap_ratio > 1.0:
        overlap_ratio /= 100.0
    reduction_per_layer = float(layer_height) * math.tan(math.radians(float(max_overhang)))
    support_width = stand_off
    support_alignment_offset = float(nozzle) * (0.5 + overlap_ratio)
    inner_edge_shift = 0.0
    if float(layer_height) > 1e-9:
        inner_edge_shift = float(nozzle) * reduction_per_layer / float(layer_height)
    pair_id = 0

    for row_idx, row_events in enumerate(source_event_stack):
        curve = curve_stack[row_idx]
        back_curve = back_curve_stack[row_idx]
        for event_param, width, event_angle, disp_width, skew_t, step_count, division_points, side_shift, shift_balance, skew_shift in row_events:
            event_pt = rs.EvaluateCurve(curve, event_param)
            geometry = _resolve_opening_geometry(
                event_pt,
                curve,
                back_srf,
                back_plane,
                width,
                event_angle,
                nozzle,
                overlap,
                disp_width,
                division_points,
                side_shift,
                shift_balance,
                skew_shift,
                skew_t,
                backface_reduction,
            )

            pair_id += 1
            for support_pt, _support_target_pt, _axis_vector, support_depth, inner_side in geometry["support_axes"]:
                initial_distance = max(0.0, float(support_depth))
                if initial_distance <= 1e-9:
                    continue

                source_param = rs.CurveClosestPoint(curve, support_pt)
                source_t_norm = _normalized_curve_parameter(curve, source_param)
                for layer_offset in range(1, row_idx + 1):
                    disp_distance = initial_distance - reduction_per_layer * float(layer_offset - 1)
                    if disp_distance <= 1e-9:
                        break

                    target_row_idx = row_idx - layer_offset
                    target_front_curve = curve_stack[target_row_idx]
                    target_curve = back_curve_stack[target_row_idx]
                    target_front_param = _curve_param_from_normalized(target_front_curve, source_t_norm)
                    target_front_pt = rs.EvaluateCurve(target_front_curve, target_front_param)
                    target_tangent = _vector3d(rs.VectorUnitize(rs.CurveTangent(target_front_curve, target_front_param)))
                    target_data = _offset_data_from_back_surface(
                        target_front_pt,
                        back_srf,
                        back_plane,
                        target_tangent,
                        event_angle,
                        stand_off,
                    )
                    target_back_pt = target_data["back_shifted_point"]
                    target_param = rs.CurveClosestPoint(target_curve, target_back_pt)
                    available_depth = max(0.0, float(target_data["depth"]))
                    if available_depth <= 1e-9:
                        continue

                    axis_vector = _sub_points(target_front_pt, target_data["point"])
                    back_tangent = _vector3d(rs.VectorUnitize(rs.CurveTangent(target_curve, target_param)))
                    if back_tangent.Length <= 1e-9:
                        continue
                    target_back_pt = _add_point_vector(
                        target_back_pt,
                        _scale(back_tangent, support_alignment_offset),
                    )
                    target_param = rs.CurveClosestPoint(target_curve, target_back_pt)
                    support_distance = min(float(disp_distance), available_depth)
                    if support_distance <= 1e-9:
                        continue
                    support_event_stack[target_row_idx].append(
                        (
                            target_param,
                            target_back_pt,
                            support_width,
                            support_distance,
                            axis_vector,
                            available_depth,
                            inner_edge_shift if layer_offset == 1 else 0.0,
                            inner_side,
                            pair_id,
                        )
                    )

                    if reduction_per_layer <= 1e-9:
                        break

    return support_event_stack


def build_output_points_from_events(point_stack, curve_stack, back_srf, event_stack, angle, nozzle, overlap, backface_reduction):
    """Emit front rows by merging base contour points with opening events."""
    output = []
    back_plane = _planar_wall_plane(back_srf)

    for row_idx, row in enumerate(point_stack):
        curve = curve_stack[row_idx]
        domain = rs.CurveDomain(curve)
        blocked_params = []
        blocked_intervals = []
        row_items = []
        for event_param, width, event_angle, disp_width, skew_t, division_points, side_shift, shift_balance, skew_shift in event_stack[row_idx]:
            event_pt = rs.EvaluateCurve(curve, event_param)
            blocked_params.append(event_param)
            geometry = _resolve_opening_geometry(
                event_pt,
                curve,
                back_srf,
                back_plane,
                width,
                event_angle,
                nozzle,
                overlap,
                disp_width,
                division_points,
                side_shift,
                shift_balance,
                skew_shift,
                skew_t,
                backface_reduction,
            )
            pt_before, pt_before_disp, pt_after_disp, pt_after = geometry["points"]
            before_param = rs.CurveClosestPoint(curve, pt_before)
            after_param = rs.CurveClosestPoint(curve, pt_after)
            blocked_intervals.append((min(before_param, after_param), max(before_param, after_param)))
            row_items.append(
                (
                    event_param,
                    1,
                    (pt_before, pt_before_disp, pt_after_disp, pt_after),
                )
            )
        param_tol = max(1e-9, abs(domain[1] - domain[0]) * 1e-6)
        for pt in row:
            base_param = rs.CurveClosestPoint(curve, pt)
            if any(abs(base_param - blocked_param) <= param_tol for blocked_param in blocked_params):
                continue
            if any(_param_in_interval(base_param, interval, param_tol) for interval in blocked_intervals):
                continue
            row_items.append((base_param, 0, pt))

        row_items.sort(key=lambda item: (item[0], item[1]))

        row_pts = []
        for _param, item_type, payload in row_items:
            if item_type == 0:
                row_pts.append(payload)
            else:
                row_pts.extend(payload)
        output.append(row_pts)

    return output


def build_back_output_points_from_supports(back_point_stack, back_curve_stack, support_event_stack):
    """Emit back rows by merging base contour points with support indent events."""
    output = []

    for row_idx, row in enumerate(back_point_stack):
        curve = back_curve_stack[row_idx]
        domain = rs.CurveDomain(curve)
        blocked_params = []
        blocked_intervals = []
        row_items = []

        pair_bounds = {}
        for support_param, support_pt, support_width, support_distance, axis_vector, full_depth, inner_edge_shift, inner_side, pair_id in support_event_stack[row_idx]:
            blocked_params.append(support_param)
            if pair_id is not None:
                pair_bounds.setdefault(pair_id, []).append(support_param)
            support_points = add_support_indent(
                support_pt,
                curve,
                axis_vector,
                support_width,
                support_distance,
                full_depth,
                inner_edge_shift,
                inner_side,
            )
            if len(support_points) >= 2:
                support_start = rs.CurveClosestPoint(curve, support_points[0])
                support_end = rs.CurveClosestPoint(curve, support_points[-1])
                blocked_intervals.append((min(support_start, support_end), max(support_start, support_end)))
            row_items.append(
                (
                    support_param,
                    1,
                    support_points,
                )
            )

        for support_params in pair_bounds.values():
            if len(support_params) >= 2:
                blocked_intervals.append((min(support_params), max(support_params)))

        param_tol = max(1e-9, abs(domain[1] - domain[0]) * 1e-6)
        for pt in row:
            base_param = rs.CurveClosestPoint(curve, pt)
            if any(abs(base_param - blocked_param) <= param_tol for blocked_param in blocked_params):
                continue
            if any(_param_in_interval(base_param, interval, param_tol) for interval in blocked_intervals):
                continue
            row_items.append((base_param, 0, pt))

        row_items.sort(key=lambda item: (item[0], item[1]))

        row_pts = []
        for _param, item_type, payload in row_items:
            if item_type == 0:
                row_pts.append(payload)
            else:
                row_pts.extend(payload)
        output.append(row_pts)

    return output


# Legacy closure path kept for reference. Not used in the current output.
def build_closed_output(front_output, back_output):
    """Connect front and back rows on one side only, leaving the opposite side open."""
    open_layers = []
    for layer_idx, row_pts in enumerate(front_output):
        back_row = list(back_output[layer_idx])[::-1]
        layer_pts = list(row_pts)
        layer_pts.extend(back_row)
        open_layers.append(layer_pts)
    return open_layers


def _point_at_polyline_distance(points, distance):
    """Interpolate a point at a cumulative distance along a point list."""
    if not points:
        return None
    if len(points) == 1:
        return points[0]

    target = max(0.0, float(distance))
    walked = 0.0
    for idx in range(len(points) - 1):
        a = _point3d(points[idx])
        b = _point3d(points[idx + 1])
        seg_len = _distance(a, b)
        if seg_len <= 1e-9:
            continue
        if walked + seg_len >= target:
            t = (target - walked) / seg_len
            return _interpolate_point(a, b, t)
        walked += seg_len
    return _point3d(points[-1])


def _split_back_row_with_start_gap(back_row, gap_length):
    """Remove a segment from the start of a row and return the remaining open path."""
    if not back_row:
        return []

    back_points = [_point3d(pt) for pt in back_row]
    if len(back_points) == 1:
        return [back_points[0]]

    total_length = 0.0
    for idx in range(len(back_points) - 1):
        total_length += _distance(back_points[idx], back_points[idx + 1])

    if total_length <= 1e-9:
        return [back_points[0]]

    gap_length = max(0.0, min(float(gap_length), total_length))
    start_cut = _point_at_polyline_distance(back_points, gap_length)
    remaining = [start_cut]

    walked = 0.0
    for idx in range(len(back_points) - 1):
        a = back_points[idx]
        b = back_points[idx + 1]
        seg_len = _distance(a, b)
        if seg_len <= 1e-9:
            continue
        next_walked = walked + seg_len
        if next_walked > gap_length:
            remaining.extend(back_points[idx + 1 :])
            break
        walked = next_walked

    return remaining


def build_open_back_gap_output(front_output, back_output, nozzle):
    """Connect front and back on both sides, leaving a removed segment at the back-row start."""
    open_layers = []
    for layer_idx, front_row in enumerate(front_output):
        back_row = list(back_output[layer_idx])
        trimmed_back = _split_back_row_with_start_gap(back_row, nozzle)
        if not trimmed_back:
            open_layers.append(list(front_row))
            continue

        back_start = _point3d(back_row[0])
        layer_pts = [back_start]
        layer_pts.extend(list(front_row))
        layer_pts.extend(list(reversed(trimmed_back)))
        open_layers.append(layer_pts)
    return open_layers


def build_gradient_preview(front_srf, img, param_config, opening_width, opening_scale, sample_count=24):
    """Build a preview mesh for the active guide field on front_srf."""
    if front_srf is None:
        return None

    width_mode, width_settings = param_config["width"]
    if opening_width <= 0:
        return None

    mesh = Rhino.Geometry.Mesh()
    preview_face = _primary_face(front_srf)
    u_dom = preview_face.Domain(0)
    v_dom = preview_face.Domain(1)
    base_width = max(1e-9, float(opening_width) * float(opening_scale))

    for v_idx in range(sample_count + 1):
        v_t = float(v_idx) / float(sample_count)
        v = gl.lerp(v_dom[0], v_dom[1], v_t)
        for u_idx in range(sample_count + 1):
            u_t = float(u_idx) / float(sample_count)
            u = gl.lerp(u_dom[0], u_dom[1], u_t)
            pt = preview_face.PointAt(u, v)
            uv = (u_t, v_t)
            rgb = sample_surface_color(pt, front_srf, img) if img else None

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


def _run(validated):
    point_stack, curve_stack, back_point_stack, back_curve_stack = build_curve_stacks(
        validated["front_srf"],
        validated["back_srf"],
        validated["layer_height"],
        validated["div_dist"],
        validated["angle"],
        validated["nozzle"],
        validated["overlap"],
        validated["effective_layers"],
        validated["img"],
        validated["param_config"],
        validated["distribution_mode"],
    )

    if validated["distribution_mode"] == "grid":
        seed_stack = build_regular_seed_stack(
            point_stack,
            curve_stack,
            validated["num_u"],
            validated["num_v"],
            validated["steps"],
            validated["u_shift"],
            validated["v_shift"],
            validated["v_scale"],
            validated["v_reverse"],
            validated["seed_row_start"],
            use_all_u=validated["param_config"] is not None,
            front_srf=validated["guide_srf"],
            img=validated["img"],
            param_config=validated["param_config"],
        )
        source_event_stack = build_seed_opening_events(
            point_stack,
            curve_stack,
            seed_stack,
            validated["guide_srf"],
            validated["img"],
            validated["param_config"] if validated["param_config"] is not None else _build_fixed_param_config(validated),
            enforce_div_dist_spacing=validated["param_config"] is None,
        )
        opening_event_stack = build_opening_events_from_sources(
            curve_stack,
            source_event_stack,
            validated["nozzle"],
            validated["overlap"],
        )
        support_event_stack = (
            build_support_events(
                curve_stack,
                back_curve_stack,
                validated["back_srf"],
                source_event_stack,
                validated["angle"],
                validated["nozzle"],
                validated["overlap"],
                validated["layer_height"],
                validated["max_overhang"],
                validated["backface_reduction"],
            )
            if validated["generate_support"]
            else [[] for _ in back_curve_stack]
        )
        output = build_output_points_from_events(
            point_stack,
            curve_stack,
            validated["back_srf"],
            opening_event_stack,
            validated["angle"],
            validated["nozzle"],
            validated["overlap"],
            validated["backface_reduction"],
        )
        seed_points = (
            build_seed_points_from_stack(point_stack, seed_stack)
            if validated["param_config"] is None
            else build_seed_points_from_source_events(curve_stack, source_event_stack)
        )
        opening_contours = build_opening_contours(
            source_event_stack,
            curve_stack,
            validated["back_srf"],
            validated["angle"],
            validated["nozzle"],
            validated["overlap"],
            validated["backface_reduction"],
        )
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
        source_event_stack = build_random_seed_source_events(curve_stack, random_seeds)
        opening_event_stack = build_opening_events(
            curve_stack,
            random_seeds,
            validated["nozzle"],
            validated["overlap"],
        )
        support_event_stack = (
            build_support_events(
                curve_stack,
                back_curve_stack,
                validated["back_srf"],
                source_event_stack,
                validated["angle"],
                validated["nozzle"],
                validated["overlap"],
                validated["layer_height"],
                validated["max_overhang"],
                validated["backface_reduction"],
            )
            if validated["generate_support"]
            else [[] for _ in back_curve_stack]
        )
        output = build_output_points_from_events(
            point_stack,
            curve_stack,
            validated["back_srf"],
            opening_event_stack,
            validated["angle"],
            validated["nozzle"],
            validated["overlap"],
            validated["backface_reduction"],
        )
        seed_points = build_seed_points_from_random_seeds(random_seeds)
        opening_contours = build_opening_contours(
            source_event_stack,
            curve_stack,
            validated["back_srf"],
            validated["angle"],
            validated["nozzle"],
            validated["overlap"],
            validated["backface_reduction"],
        )

    preview = []
    if validated["param_config"] is not None:
        preview = build_gradient_preview(
            validated["guide_srf"],
            validated["img"],
            validated["param_config"],
            validated["opening_width"],
            validated["opening_scale"],
        )

    back_output = build_back_output_points_from_supports(
        back_point_stack,
        back_curve_stack,
        support_event_stack,
    )
    return (
        th.list_to_tree(build_open_back_gap_output(output, back_output, validated["nozzle"])),
        preview,
        seed_points,
        th.list_to_tree(opening_contours),
    )


validated = _validate_inputs()

if validated is None:
    a = th.list_to_tree([])
    b = []
    c = []
    d = th.list_to_tree([])
    C = c
    D = d
else:
    a, b, c, d = _run(validated)
    C = c
    D = d
