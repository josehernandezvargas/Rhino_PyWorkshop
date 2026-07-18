#! python3

"""Generate an alternating truss from sliced wall-surface guide curves.

Inputs:
    base_srf: Wall surface/Brep sliced horizontally to define the front AB rail.
    layer_height: Z spacing between contour slices.
    layers: Optional int cap on the number of layers for testing.
    target_dist: Target spacing between alternating truss nodes along the rails.
    truss_width: Inward offset distance from the sliced AB rail to the CD rail.
    contact_length: Length of each truss contact segment along the local tangent.
    nozzle: Optional nozzle width override. Defaults to NOZZLE.

Outputs:
    a: Grasshopper tree of closed wall-loop point rows for each valid layer.
    b: Grasshopper tree of individual line segments as curves.
"""

__author__ = "joseh"
__version__ = "2026.04.27"


import Grasshopper as gh
import Rhino
import rhinoscriptsyntax as rs
from ghpythonlib import treehelpers as th


NOZZLE = 20.0
POINT_TOL = 1e-6


def _get_param(name, default=None):
    return globals().get(name, default)


def _report(message, level=gh.Kernel.GH_RuntimeMessageLevel.Warning):
    try:
        ghenv.Component.AddRuntimeMessage(level, message)
    except Exception:
        print(message)


def _as_float(name, value, minimum=None, allow_zero=False):
    if value is None:
        raise ValueError("{} is required.".format(name))
    result = float(value)
    if minimum is not None:
        threshold = minimum if allow_zero else max(minimum, POINT_TOL)
        if result < threshold or (not allow_zero and result <= minimum):
            raise ValueError("{} must be greater than {}.".format(name, minimum))
    return result


def _model_tolerance():
    doc = Rhino.RhinoDoc.ActiveDoc
    return doc.ModelAbsoluteTolerance if doc is not None else 0.01


def _as_brep(geometry):
    if isinstance(geometry, Rhino.Geometry.Brep):
        return geometry
    if isinstance(geometry, Rhino.Geometry.Surface):
        return geometry.ToBrep()
    brep = rs.coercebrep(geometry, False)
    if brep is not None:
        return brep
    surface = rs.coercesurface(geometry, False)
    if surface is not None:
        return surface.ToBrep()
    return None


def _slice_surface_rows(surface, layer_height_value, max_rows=None):
    brep = _as_brep(surface)
    if brep is None:
        return []

    bbox = brep.GetBoundingBox(True)
    start = Rhino.Geometry.Point3d(bbox.Center.X, bbox.Center.Y, bbox.Min.Z)
    end = Rhino.Geometry.Point3d(bbox.Center.X, bbox.Center.Y, bbox.Max.Z)
    contours = Rhino.Geometry.Brep.CreateContourCurves(brep, start, end, layer_height_value)
    if not contours:
        _report("No contour curves were generated from base_srf.")
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

    if ordered:
        ref_vector = ordered[0].PointAtEnd - ordered[0].PointAtStart
        for index in range(1, len(ordered)):
            current_vector = ordered[index].PointAtEnd - ordered[index].PointAtStart
            if Rhino.Geometry.Vector3d.Multiply(ref_vector, current_vector) < 0.0:
                ordered[index].Reverse()
        for curve in ordered:
            curve.Reverse()

    if max_rows is not None:
        ordered = ordered[: max(0, int(max_rows))]
    return ordered


def _curve_point_at_normalized(curve, t_norm):
    domain = curve.Domain
    t_value = domain.T0 + (domain.T1 - domain.T0) * t_norm
    return curve.PointAt(t_value), t_value


def _centered_segment(mid_pt, tangent, length):
    length = max(0.0, float(length))
    direction = Rhino.Geometry.Vector3d(tangent)
    if length <= POINT_TOL or not direction.Unitize():
        return Rhino.Geometry.Point3d(mid_pt), Rhino.Geometry.Point3d(mid_pt)
    half_vector = direction * (length * 0.5)
    return mid_pt - half_vector, mid_pt + half_vector


def _move_point_toward(point, target, distance):
    vector = target - point
    if not vector.Unitize():
        return Rhino.Geometry.Point3d(point)
    return point + vector * distance


def _move_point_away(point, target, distance):
    vector = point - target
    if not vector.Unitize():
        return Rhino.Geometry.Point3d(point)
    return point + vector * distance


def _offset_segment_toward(segment, target_point, distance):
    """Move a whole segment toward a target point by a fixed distance."""
    start_pt = segment["start"]
    end_pt = segment["end"]
    mid_pt = Rhino.Geometry.Point3d(
        0.5 * (start_pt.X + end_pt.X),
        0.5 * (start_pt.Y + end_pt.Y),
        0.5 * (start_pt.Z + end_pt.Z),
    )
    move_vector = target_point - mid_pt
    if not move_vector.Unitize():
        return _segment_record(start_pt, end_pt, segment["tangent"], segment["side"], segment["station_index"])
    move_vector *= float(distance)
    return _segment_record(
        start_pt + move_vector,
        end_pt + move_vector,
        segment["tangent"],
        segment["side"],
        segment["station_index"],
    )


def _rotate90_xy(vector):
    return Rhino.Geometry.Vector3d(-vector.Y, vector.X, 0.0)


def _offset_connector_segment(start_pt, end_pt, hint_vector, distance):
    """Shift a connector segment by a perpendicular XY vector.

    The sign is chosen from ``hint_vector`` so DA and CB can be displaced on the
    outside of the wall instead of toward the truss interior.
    """
    connector_vector = end_pt - start_pt
    normal_vector = _rotate90_xy(connector_vector)
    if not normal_vector.Unitize():
        return Rhino.Geometry.Point3d(start_pt), Rhino.Geometry.Point3d(end_pt)

    hint = Rhino.Geometry.Vector3d(hint_vector)
    if hint.Unitize() and Rhino.Geometry.Vector3d.Multiply(normal_vector, hint) < 0.0:
        normal_vector *= -1.0

    offset_vector = normal_vector * float(distance)
    return start_pt + offset_vector, end_pt + offset_vector


def _offset_midpoint_distance(source_curve, candidate_curve):
    source_mid, _ = _curve_point_at_normalized(source_curve, 0.5)
    candidate_mid, _ = _curve_point_at_normalized(candidate_curve, 0.5)
    return source_mid.DistanceTo(candidate_mid)


def _align_curve_direction(reference_curve, curve):
    if curve is None or reference_curve is None:
        return curve
    ref_vector = reference_curve.PointAtEnd - reference_curve.PointAtStart
    curve_vector = curve.PointAtEnd - curve.PointAtStart
    if Rhino.Geometry.Vector3d.Multiply(ref_vector, curve_vector) < 0.0:
        curve.Reverse()
    return curve


def _append_unique_points(target, points):
    for point in points:
        if not target or target[-1].DistanceTo(point) > POINT_TOL:
            target.append(point)
    return target


def _build_back_curve(base_curve, truss_width_value):
    if base_curve is None or not base_curve.IsValid:
        return None

    offset_plane = Rhino.Geometry.Plane.WorldXY
    tol = _model_tolerance()
    for signed_dist in (-float(truss_width_value), float(truss_width_value)):
        offsets = base_curve.Offset(offset_plane, signed_dist, tol, Rhino.Geometry.CurveOffsetCornerStyle.Sharp)
        if not offsets:
            continue
        candidates = [curve for curve in offsets if curve is not None and curve.IsValid]
        if not candidates:
            continue
        target_width = abs(float(truss_width_value))
        candidate = min(
            candidates,
            key=lambda item: abs(_offset_midpoint_distance(base_curve, item) - target_width),
        )
        return _align_curve_direction(base_curve, candidate)
    return None


def _segment_record(start_pt, end_pt, tangent, side, station_index):
    return {
        "start": start_pt,
        "end": end_pt,
        "tangent": tangent,
        "side": side,
        "station_index": station_index,
    }


def _build_station_params(curve_length, target_dist_value):
    num_divs = max(1, int(curve_length // float(target_dist_value)))
    # Truss divisions stay odd so the alternating path starts on A and ends on C.
    if num_divs % 2 == 0:
        num_divs += 1
    return [float(index) / float(num_divs) for index in range(num_divs + 1)]


def _build_contact_segment(point, tangent, segment_length, side, station_index):
    start_pt, end_pt = _centered_segment(point, tangent, segment_length)
    return _segment_record(start_pt, end_pt, tangent, side, station_index)


def _segment_curve(segment):
    return Rhino.Geometry.LineCurve(segment["start"], segment["end"])


def _sequence_segments_by_proximity(segments):
    if not segments:
        return []

    ordered_points = [segments[0]["start"], segments[0]["end"]]
    prev_point = segments[0]["end"]
    for segment in segments[1:]:
        start_pt = segment["start"]
        end_pt = segment["end"]
        if prev_point.DistanceTo(end_pt) < prev_point.DistanceTo(start_pt):
            start_pt, end_pt = end_pt, start_pt
        ordered_points.append(start_pt)
        ordered_points.append(end_pt)
        prev_point = end_pt
    return ordered_points


def _build_station_data(base_curve, back_curve, station_params, nozzle_value):
    """Sample matching rail points and their one-nozzle inward offsets.

    Each station stores:
        - the raw front/back points on the two rails
        - the raw tangents used to center contact segments
        - the inward copies of both rail points, always moved by exactly one
          nozzle unless the rail gap is too small
    """
    station_data = []
    for station_index, t_norm in enumerate(station_params):
        front_point, front_param = _curve_point_at_normalized(base_curve, t_norm)
        back_point, back_param = _curve_point_at_normalized(back_curve, t_norm)
        front_tangent = base_curve.TangentAt(front_param)
        back_tangent = back_curve.TangentAt(back_param)
        gap = front_point.DistanceTo(back_point)
        inward_dist = min(float(nozzle_value), max(0.0, gap * 0.5 - POINT_TOL))
        station_data.append({
            "index": station_index,
            "t_norm": t_norm,
            "front_point": front_point,
            "back_point": back_point,
            "front_offset": _move_point_toward(front_point, back_point, inward_dist),
            "back_offset": _move_point_toward(back_point, front_point, inward_dist),
            "back_outer": _move_point_away(back_point, front_point, nozzle_value),
            "front_tangent": front_tangent,
            "back_tangent": back_tangent,
        })
    return station_data


def _build_base_contact_segments(station_data, contact_length_value, nozzle_value):
    """Create the base alternating contact lines on the raw front/back rails."""
    contact_segments = []
    for station in station_data:
        station_index = station["index"]
        side = "front" if station_index % 2 == 0 else "back"
        segment_length = float(nozzle_value) if station_index == 0 or station_index == len(station_data) - 1 else float(contact_length_value)
        point = station["front_point"] if side == "front" else station["back_point"]
        tangent = station["front_tangent"] if side == "front" else station["back_tangent"]
        segment = _build_contact_segment(point, tangent, segment_length, side, station_index)
        contact_segments.append(segment)
    return contact_segments


def _build_offset_contact_segments(base_segments, station_data, nozzle_value):
    """Offset each contact line inward toward the opposite rail by one nozzle."""
    offset_segments = []
    station_lookup = {station["index"]: station for station in station_data}
    for segment in base_segments:
        station = station_lookup[segment["station_index"]]
        target_point = station["back_point"] if segment["side"] == "front" else station["front_point"]
        offset_segments.append(_offset_segment_toward(segment, target_point, nozzle_value))
    return offset_segments


def _build_truss_path_points(base_segments, offset_segments, start_point, end_point, use_complement=False):
    """Build one open truss-side path from the paired contact-line lists.

    Forward pass:
        0e 1i 2e 3i ...

    Complementary paired side:
        0i 1e 2i 3e ...

    The full closed sequence is created later by reversing the complementary
    path, which yields:
        ... 5e 4i 3e 2i 1e 0i
    """
    if not base_segments or not offset_segments:
        return []

    path_points = [start_point]
    for index in range(len(base_segments)):
        base_segment = base_segments[index]
        offset_segment = offset_segments[index]
        if use_complement:
            segment = offset_segment if index % 2 == 0 else base_segment
        else:
            segment = base_segment if index % 2 == 0 else offset_segment
        segment_points = [segment["start"], segment["end"]]
        _append_unique_points(path_points, segment_points)
    _append_unique_points(path_points, [end_point])
    return path_points


def _build_connector_pairs(station_data, nozzle_value):
    """Build the DA and CB segment pairs.

    Raw connectors use the exact rail endpoints.
    Offset connectors are shifted outward, normal to the connector segments.
    """
    first_station = station_data[0]
    last_station = station_data[-1]
    da_raw = [first_station["back_point"], first_station["front_point"]]
    cb_raw = [last_station["back_point"], last_station["front_point"]]
    da_hint = Rhino.Geometry.Vector3d(first_station["front_tangent"]) * -1.0
    cb_hint = Rhino.Geometry.Vector3d(last_station["front_tangent"])
    da_start, da_end = _offset_connector_segment(da_raw[0], da_raw[1], da_hint, nozzle_value)
    cb_start, cb_end = _offset_connector_segment(cb_raw[0], cb_raw[1], cb_hint, nozzle_value)
    da_offset = [da_start, da_end]
    cb_offset = [cb_start, cb_end]
    return da_raw, cb_raw, da_offset, cb_offset


def _build_closed_loop(primary_points, secondary_points):
    """Build one closed loop from a raw path and its inward parallel path."""
    if not primary_points or not secondary_points:
        return []
    loop_points = []
    _append_unique_points(loop_points, primary_points)
    _append_unique_points(loop_points, list(reversed(secondary_points)))
    _append_unique_points(loop_points, [primary_points[0]])
    return loop_points


def _build_segment_curves(points):
    """Convert a point chain into individual line-curve segments."""
    segment_curves = []
    for index in range(len(points) - 1):
        start_pt = points[index]
        end_pt = points[index + 1]
        if start_pt.DistanceTo(end_pt) <= POINT_TOL:
            continue
        segment_curves.append(Rhino.Geometry.LineCurve(start_pt, end_pt))
    return segment_curves


def build_contactlines(base_curve, target_dist_value, truss_width_value, contact_length_value, nozzle_value):
    """Build the wall loop from truss pairs plus CD, DA and CB.

    Steps:
        1. Build the raw alternating truss contact lines.
        2. Build their inward copies, one nozzle away.
        3. Sequence the truss as alternating outer and inner segments.
        4. Add the remaining CD, DA and CB path segments with the same raw/offset rule.
        5. Build one closed loop from the raw path and the reversed offset path.
    """
    if base_curve is None or not base_curve.IsValid or base_curve.IsClosed:
        return [], [], None
    if target_dist_value <= 0.0 or truss_width_value <= 0.0:
        return [], [], None

    back_curve = _build_back_curve(base_curve, truss_width_value)
    if back_curve is None:
        return [], [], None

    front_length = base_curve.GetLength()
    back_length = back_curve.GetLength()
    curve_length = min(front_length, back_length)
    if curve_length <= POINT_TOL:
        return [], [], None

    station_params = _build_station_params(curve_length, target_dist_value)
    station_data = _build_station_data(
        base_curve,
        back_curve,
        station_params,
        nozzle_value,
    )
    base_contact_segments = _build_base_contact_segments(
        station_data,
        contact_length_value,
        nozzle_value,
    )
    offset_contact_segments = _build_offset_contact_segments(
        base_contact_segments,
        station_data,
        nozzle_value,
    )

    raw_truss_points = _build_truss_path_points(
        base_contact_segments,
        offset_contact_segments,
        station_data[0]["front_point"],
        station_data[-1]["back_point"],
        use_complement=False,
    )
    offset_truss_points = _build_truss_path_points(
        base_contact_segments,
        offset_contact_segments,
        station_data[0]["front_offset"],
        station_data[-1]["back_offset"],
        use_complement=True,
    )

    back_raw_points = list(reversed([station["back_point"] for station in station_data]))
    back_offset_points = list(reversed([station["back_outer"] for station in station_data]))
    da_raw, cb_raw, da_offset, cb_offset = _build_connector_pairs(station_data, nozzle_value)

    raw_output_points = list(back_raw_points)
    _append_unique_points(raw_output_points, da_raw[1:])
    _append_unique_points(raw_output_points, raw_truss_points[1:])
    _append_unique_points(raw_output_points, cb_raw[1:])

    offset_output_points = list(back_offset_points)
    _append_unique_points(offset_output_points, da_offset[1:])
    _append_unique_points(offset_output_points, offset_truss_points[1:])
    _append_unique_points(offset_output_points, cb_offset[1:])

    closed_loop_points = _build_closed_loop(raw_output_points, offset_output_points)
    segment_curves = _build_segment_curves(closed_loop_points)
    return closed_loop_points, segment_curves, back_curve


def build_truss_layers(base_surface, layer_height_value, layer_limit, target_dist_value, truss_width_value, contact_length_value, nozzle_value):
    contour_rows = _slice_surface_rows(base_surface, layer_height_value, layer_limit)
    if not contour_rows:
        return [], []

    layer_rows = []
    debug_rows = []
    for layer_index, contour in enumerate(contour_rows):
        loop_pts, segment_polylines, back_curve = build_contactlines(
            contour,
            target_dist_value,
            truss_width_value,
            contact_length_value,
            nozzle_value,
        )
        if not loop_pts:
            _report(
                "Skipped layer {}: contour must be an open valid rail and offset by truss_width.".format(layer_index)
            )
            continue
        layer_rows.append(loop_pts)
        debug_rows.append(segment_polylines)

    return layer_rows, debug_rows

# Layer sequencing
# FIXME: Review this sequence to guide the code output

# front_crv >> Start_pt == A , End_pt == B
# back_crv >> Start_pt == D , End_pt == C
# Truss is the middle section
# Truss divisions are always uneven
# Truss always starts at A and ends in C
# output sequence: back_crv (CD) , DA, truss (AC), CB



a = th.list_to_tree([])
b = th.list_to_tree([])

try:
    base_srf_value = _get_param("base_srf")
    layer_height_value = _as_float("layer_height", _get_param("layer_height"), minimum=0.0)
    target_dist_value = _as_float("target_dist", _get_param("target_dist"), minimum=0.0)
    truss_width_value = _as_float("truss_width", _get_param("truss_width"), minimum=0.0)
    contact_length_value = _as_float("contact_length", _get_param("contact_length"), minimum=0.0, allow_zero=True)
    nozzle_value = _as_float("nozzle", _get_param("nozzle", NOZZLE), minimum=0.0)

    layers_value = _get_param("layers", None)
    layer_limit = None
    if layers_value is not None:
        layer_limit = max(0, int(layers_value))
        if layer_limit == 0:
            layer_limit = None

    if _as_brep(base_srf_value) is None:
        raise ValueError("base_srf must be a valid surface or Brep.")

    output, debug_output = build_truss_layers(
        base_srf_value,
        layer_height_value,
        layer_limit,
        target_dist_value,
        truss_width_value,
        contact_length_value,
        nozzle_value,
    )

    if not output:
        _report("No valid truss layers were generated.")

    a = th.list_to_tree(output)
    b = th.list_to_tree(debug_output)
except Exception as exc:
    _report(str(exc), gh.Kernel.GH_RuntimeMessageLevel.Error)
