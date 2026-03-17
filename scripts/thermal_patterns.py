#! python3
"""Generate layered thermal pattern curves for Rhino/Grasshopper.

Inputs (expected as script globals):
    input_crv: Rhino curve GUID for the base input curve.
    target_dist: Float spacing for curve division.
    amplitude: Float base amplitude for patterns.
    ext_amplitude: Float amplitude offset for external layer base.
    contact_length: Float contact segment length (0 for none).
    handle_length: Float handle length for bezier/sine-like curves.
    phase: Float phase offset for sine curve.
    layer_height: Float Z increment per layer.
    layer_count: Int number of layers to create.
    amp_modulation: Tuple (base_layers, transition_layers) controlling amplitude steps.
    draw_contour: Bool flag to include contour in output patterns.

Outputs:
    a: Grasshopper data tree of truss pattern layers (Polyline curves).
    e: Grasshopper data tree of mid-layer indent data.
"""

import rhinoscriptsyntax as rs
import math
import curvelib as cl
import geometrylib as gl
from ghpythonlib import treehelpers as th
from itertools import chain

contact_tol = 1e-6
nozzle = 20
sample_count = 50
offset_dist = nozzle / 2

base_layers, transition_layers = amp_modulation
base_layers = int(base_layers)
transition_layers = int(transition_layers)

def _curve_normal_xy(tangent):
    """Return a planar normal vector (Z-up) given a tangent vector."""
    # Rotate tangent 90 degrees in the XY plane.
    return rs.VectorRotate(tangent, 90, [0, 0, 1])

def create_contour_curve(base_curve, amplitude, nozzle):
    """Create a rectangular contour around a base curve and offset it.

    Args:
        base_curve: Rhino curve GUID.
        amplitude: Float offset distance for top/bottom curves.
        offset_dist: Float offset for final contour.

    Returns:
        GUID of the offset contour curve, or None if creation fails.
    """
    tangent_vector = rs.VectorCreate(rs.CurveEndPoint(base_curve), rs.CurveStartPoint(base_curve))
    normal_vector = rs.VectorRotate(tangent_vector, 90, [0, 0, 1])
    curve_top = rs.CopyObject(base_curve, rs.VectorScale(rs.VectorUnitize(normal_vector), amplitude))
    curve_bottom = rs.CopyObject(base_curve, rs.VectorScale(rs.VectorUnitize(normal_vector), -amplitude))
    pt_0 = rs.CurveStartPoint(curve_bottom)
    pt_1 = rs.CurveStartPoint(curve_top)
    pt_2 = rs.CurveEndPoint(curve_top)
    pt_3 = rs.CurveEndPoint(curve_bottom)
    bbox = rs.AddPolyline([pt_3, pt_2, pt_1, pt_0])
    return rs.OffsetCurve(bbox, [0, 0, 1], nozzle / 2)

def _add_contact_segment(container, contact):
    """Append a contact line segment if it is non-degenerate."""
    start_pt, end_pt, _ = contact
    if rs.Distance(start_pt, end_pt) > contact_tol:
        container.append(rs.AddLine(start_pt, end_pt))


#FIXME: this function doesn't work properly
def _join_contour_with_pattern(contour_curve, pattern_curve):
    """Join a contour curve to a pattern curve with a connector, if needed."""
    if not contour_curve or not pattern_curve:
        return pattern_curve or contour_curve
    contour_start = rs.CurveStartPoint(contour_curve)
    pattern_start = rs.CurveStartPoint(pattern_curve)
    connector = None
    if rs.Distance(contour_start, pattern_start) > contact_tol:
        connector = rs.AddLine(contour_start, pattern_start)
    curves = [contour_curve]
    if connector:
        curves.append(connector)
    curves.append(pattern_curve)
    joined = rs.JoinCurves(curves)
    if joined:
        return joined
    return None

def _prepare_curve_for_connection(curve, gap):
    """Return an open curve ready to be chained by endpoint with another curve."""
    if not curve:
        return None

    curve_len = rs.CurveLength(curve) or 0.0
    if curve_len <= contact_tol:
        return rs.CopyObject(curve)

    gap = max(float(gap or 0.0), 0.0)

    # Closed curves need a non-zero opening, otherwise both ends coincide and join fails.
    if rs.IsCurveClosed(curve):
        cut_dist = gap if gap > contact_tol else min(max(curve_len * 1e-4, contact_tol), curve_len * 0.25)
        cut_dist = min(cut_dist, max(curve_len - contact_tol, contact_tol))
        cut_pt = rs.CurveArcLengthPoint(curve, cut_dist, True)
        if not cut_pt:
            return rs.CopyObject(curve)
        cut_param = rs.CurveClosestPoint(curve, cut_pt)
        split = rs.SplitCurve(curve, cut_param, False)
        if not split:
            return rs.CopyObject(curve)
        if len(split) == 1:
            return split[0]
        return max(split, key=lambda c: rs.CurveLength(c) or 0.0)

    # Open curves: only trim a start segment when gap is positive.
    if gap <= contact_tol:
        return rs.CopyObject(curve)

    cut_dist = min(gap, max(curve_len - contact_tol, 0.0))
    if cut_dist <= contact_tol:
        return rs.CopyObject(curve)
    cut_pt = rs.CurveArcLengthPoint(curve, cut_dist, True)
    if not cut_pt:
        return rs.CopyObject(curve)
    cut_param = rs.CurveClosestPoint(curve, cut_pt)
    split = rs.SplitCurve(curve, cut_param, False)
    if not split:
        return rs.CopyObject(curve)
    return split[-1]


def connect_two_curves(curve1, curve2, gap):
    new_crv1 = _prepare_curve_for_connection(curve1, gap)
    new_crv2 = _prepare_curve_for_connection(curve2, gap)
    if not new_crv1 or not new_crv2:
        return None

    end_1 = rs.CurveEndPoint(new_crv1)
    start_2 = rs.CurveStartPoint(new_crv2)
    end_2 = rs.CurveEndPoint(new_crv2)
    if rs.Distance(end_1, end_2) < rs.Distance(end_1, start_2):
        rs.ReverseCurve(new_crv2)
        start_2 = rs.CurveStartPoint(new_crv2)

    connector = rs.AddLine(end_1, start_2)
    joined = rs.JoinCurves((new_crv1, connector, new_crv2))
    if not joined:
        return None
    return joined[0]



def build_contactlines(base_curve, target_dist, amplitude, contact_length):
    """Compute truss points and contact line definitions along a base curve.

    Args:
        base_curve: Rhino curve GUID.
        target_dist: Float spacing between division points.
        amplitude: Float offset applied along the normal.
        contact_length: Float contact segment length (0 for point contacts).

    Returns:
        Tuple (truss_pts, contactlines).
    """
    if not base_curve or target_dist <= 0:
        return [], []
    
    curve_length = int(rs.CurveLength(base_curve))
    num_divs = int(curve_length // target_dist // 2) * 2
    division_module = 1/num_divs
    # divides the curve in modules leaving half a module at the beginning and end
    div_params = [i*division_module for i in range(num_divs + 1)]
    div_pts = [rs.EvaluateCurve(base_curve, rs.CurveParameter(base_curve, t)) for t in div_params]
    # print(f"numdivs: {num_divs}; div_pts: {len(div_pts)}")

    ##
    truss_pts = []
    contactlines = []
    for i, pt in enumerate(div_pts):
        t_param = rs.CurveClosestPoint(base_curve, pt)
        if t_param is None:
            continue
        tangent = rs.VectorUnitize(rs.CurveTangent(base_curve, t_param))
        normal = _curve_normal_xy(tangent)
        if i % 2 == 0:
            normal = -normal

        # skip first and last
        # if i == 0 or i == len(div_pts) - 1 or contact_length == 0:
        #     moved_pt = rs.PointAdd(pt, rs.VectorScale(normal, amplitude))
        #     truss_pts.append(moved_pt)
        #     contactlines.append((moved_pt, moved_pt, tangent))
        #     continue

        contactline = cl.centercrv_dir(pt, tangent, contact_length)
        contactline = rs.MoveObject(contactline, rs.VectorScale(normal, amplitude))
        start_pt = rs.CurveStartPoint(contactline)
        end_pt = rs.CurveEndPoint(contactline)
        truss_pts.append(start_pt)
        truss_pts.append(end_pt)
        contactlines.append((start_pt, end_pt, tangent))
    return truss_pts, contactlines



def truss_polyline(base_curve, target_dist, amplitude, contact_length):
    """Create a polyline connecting truss points."""
    truss_pts, _ = build_contactlines(base_curve, target_dist, amplitude, contact_length)
    start_param = rs.CurveClosestPoint(base_curve, rs.CurveStartPoint(base_curve))
    start_tangent = rs.CurveTangent(base_curve, start_param)
    start_normal = rs.VectorUnitize(rs.VectorRotate(start_tangent, 90, [0,0,1]))
    start_pt = rs.PointAdd(rs.EvaluateCurve(base_curve, start_param), start_normal * amplitude)
    end_param = rs.CurveClosestPoint(base_curve, rs.CurveEndPoint(base_curve))
    end_tangent = rs.CurveTangent(base_curve, end_param)
    end_normal = rs.VectorUnitize(rs.VectorRotate(end_tangent, 90, [0,0,1]))
    end_pt = rs.PointAdd(rs.EvaluateCurve(base_curve, end_param), end_normal * amplitude)
    # truss_pts.insert(0, start_pt)
    # truss_pts.append(end_pt)
    if not truss_pts:
        return None
    return rs.AddPolyline(truss_pts)

# Bexier tangent curve
def sine_like_curve(base_curve, target_dist, amplitude, contact_length, handle_length):
    """Create a sine-like cubic Bezier chain between contact segments."""
    _, contactlines = build_contactlines(base_curve, target_dist, amplitude, contact_length)
    if len(contactlines) < 2:
        return None
    sine_like = []
    for i in range(len(contactlines) - 1):
        _, start_pt, start_tangent = contactlines[i]
        end_pt, _, end_tangent = contactlines[i + 1]
        start_handle = rs.PointAdd(start_pt, rs.VectorScale(start_tangent, handle_length))
        end_handle = rs.PointAdd(end_pt, rs.VectorScale(end_tangent, -handle_length))
        if i >= 1:
            _add_contact_segment(sine_like, contactlines[i])
        sine_like.append(rs.AddCurve([start_pt, start_handle, end_handle, end_pt], 3))
    # _add_contact_segment(sine_like, contactlines[-1])
    return rs.JoinCurves(sine_like)


def bezier_curve(base_curve, target_dist, amplitude, contact_length, handle_length):
    """Create a Bezier chain with normal-based handles."""
    _, contactlines = build_contactlines(base_curve, target_dist, amplitude, contact_length)
    if len(contactlines) < 2:
        return None
    bezier_crvs = []
    for i in range(len(contactlines) - 1):
        _, start_pt, start_tangent = contactlines[i]
        end_pt, _, end_tangent = contactlines[i + 1]
        start_normal = rs.VectorRotate(start_tangent, -90, [0, 0, 1])
        end_normal = rs.VectorRotate(end_tangent, 90, [0, 0, 1])
        start_normal = rs.VectorScale(rs.VectorUnitize(start_normal), amplitude / 100)
        end_normal = rs.VectorScale(rs.VectorUnitize(end_normal), amplitude / 100)
        if i % 2 == 0:
            start_normal = -start_normal
            end_normal = -end_normal
        start_handle = rs.PointAdd(start_pt, rs.VectorScale(start_normal, handle_length))
        end_handle = rs.PointAdd(end_pt, rs.VectorScale(end_normal, handle_length))
        # remove first segment
        if i >= 1:
            _add_contact_segment(bezier_crvs, contactlines[i])
        bezier_crvs.append(rs.AddCurve([start_pt, start_handle, end_handle, end_pt], 3))
    # _add_contact_segment(bezier_crvs, contactlines[-1]) # remove last segment
    return rs.JoinCurves(bezier_crvs)


def sine_curve_on_base(base_curve, wavelength, amplitude, phase, sample_count=50):
    """Create a sampled sine curve along a base curve."""
    if sample_count < 2 or not base_curve:
        return None
    pts = []
    base_pts = rs.DivideCurve(base_curve, (sample_count - 1))
    for base_pt in base_pts:
        t_param = rs.CurveClosestPoint(base_curve, base_pt)
        if t_param is None:
            continue
        tangent = rs.VectorUnitize(rs.CurveTangent(base_curve, t_param))
        normal = _curve_normal_xy(tangent)
        dist = rs.CurveLength(base_curve, 0, [0.0, t_param])
        phase_offset = 2.0 * math.pi * (phase + 0.75)
        offset = amplitude * math.sin((2.0 * math.pi * (dist / wavelength / 2.0)) + phase_offset)
        pts.append(rs.PointAdd(base_pt, rs.VectorScale(normal, offset)))
    return rs.AddInterpCurve(pts)


def _amplitude_factors(layer_count, base_layers, transition_layers):
    """Generate repeating amplitude factors based on rest and transition layers."""
    if layer_count <= 0:
        return []
    base_layers = max(0, int(base_layers))
    transition_layers = max(0, int(transition_layers))
    if base_layers == 0 and transition_layers == 0:
        return [1.0] * layer_count
    pattern = []
    if base_layers:
        pattern.extend([1.0] * base_layers)
    if transition_layers:
        if transition_layers == 1:
            pattern.append(-1.0)
        else:
            for j in range(transition_layers):
                pattern.append(gl.remap(0, transition_layers - 1, 1, -1, j))
    if base_layers:
        pattern.extend([-1.0] * base_layers)
    if transition_layers:
        if transition_layers == 1:
            pattern.append(1.0)
        else:
            for j in range(transition_layers):
                pattern.append(gl.remap(0, transition_layers - 1, -1, 1, j))
    factors = []
    while len(factors) < layer_count:
        for val in pattern:
            factors.append(val)
            if len(factors) >= layer_count:
                break
    return factors


def _move_curve_z(curve, z_offset):
    """Copy a curve at a Z offset; returns original if no move is needed."""
    if not curve or z_offset == 0:
        return curve
    return rs.CopyObject(curve, [0, 0, z_offset])

def create_indents(points, base_crv, width, depth):
    """Create point tuples for indent geometry along a base curve."""
    output = []
    for pt in points:
        pt_param = rs.CurveClosestPoint(base_crv, pt)
        tangent = rs.VectorUnitize(rs.CurveTangent(base_crv, pt_param))
        normal = rs.VectorRotate(tangent, -90, [0, 0, 1])
        pt_before = rs.CopyObject(pt, tangent * -width / 2)
        pt_after = rs.CopyObject(pt, tangent * width / 2)
        pt_before_disp = rs.CopyObject(pt_before, normal * depth)
        pt_after_disp = rs.CopyObject(pt_after, normal * depth)
        output.append((pt_before, pt_before_disp, pt_after_disp, pt_after))
    return output


def create_indents_target(points, base_crv, width, depth, target_crv):
    """Create point tuples for indent geometry along a base curve that connects with a target curve."""
    output = []
    for pt in points:
        pt_param = rs.CurveClosestPoint(base_crv, pt)
        tangent = rs.VectorUnitize(rs.CurveTangent(base_crv, pt_param))
        normal = rs.VectorRotate(tangent, 90, [0, 0, 1])
        pt_before = rs.PointAdd(pt, tangent * -width / 2)
        pt_after = rs.PointAdd(pt, tangent * width / 2)
        target_param = rs.CurveClosestPoint(target_crv, pt)
        target_pt = rs.EvaluateCurve(target_crv, target_param)
        target_vector = rs.VectorCreate(target_pt, pt)
        # if rs.VectorLength(target_vector) < depth:
        target_pt = rs.PointAdd(pt, -normal * depth)
        # print(f"d: {depth}, vector: {rs.VectorLength(target_vector)}")
        target_tangent = rs.VectorUnitize(rs.CurveTangent(target_crv, rs.CurveClosestPoint(target_crv, target_pt)))
        pt_before_target = rs.PointAdd(target_pt, target_tangent * -width / 2)
        pt_after_target = rs.PointAdd(target_pt, target_tangent * width / 2)
        output.append((pt_before, pt_before_target, pt_after_target, pt_after))
    return output



def build_layer_stack(base_curve, layer_height, layer_count, base_layers, transition_layers,
                      target_dist, amplitude, contact_length, handle_length, phase, sample_count,
                      offset_dist):
    """Build stacked pattern layers with amplitude modulation.

    Returns:
        List of layer curve GUIDs (currently only truss pattern).
    """
    num_divs = int(rs.CurveLength(base_curve) // target_dist // 2) * 2
    division_module = 1/num_divs
    # divides the curve in modules leaving half a module at the beginning and end
    # div_params = [division_module/2 + i*division_module for i in range(num_divs)]
    div_params = [i*division_module for i in range(num_divs + 1)]
    div_pts = [rs.EvaluateCurve(base_curve, rs.CurveParameter(base_curve, t)) for t in div_params]

    input_tangent = rs.VectorUnitize(rs.VectorCreate(rs.CurveEndPoint(base_curve), rs.CurveStartPoint(base_curve)))
    input_normal = rs.VectorRotate(input_tangent, 90, [0,0,1])

    # Create base_crv as an input for the inner layer
    # shift start and end points by contact_length/2 to match contactlines at the start and end

    inner_base_crv = rs.CopyObject(base_curve, input_normal * (amplitude + nozzle))
    new_start_pt = rs.CurveArcLengthPoint(inner_base_crv, contact_length/2, True)
    new_end_pt = rs.CurveArcLengthPoint(inner_base_crv, contact_length/2, False)
    pattern_base_crv = rs.SplitCurve(inner_base_crv, [rs.CurveClosestPoint(inner_base_crv, new_start_pt), rs.CurveClosestPoint(inner_base_crv, new_end_pt)])[1]


    factors = _amplitude_factors(layer_count, base_layers, transition_layers)
    normalised_factors = [gl.remap(1, -1, 1, 0.1, f) for f in factors]
    # print(f"factors: {factors}")
    # print(f"normalised factors: {normalised_factors}")
    if not factors:
        return [], [], [], []
    a_layers = []
    b_layers = []
    c_layers = []
    d_layers = []
    e_layers = []

    for i, factor in enumerate(factors):
        layer_curve = _move_curve_z(pattern_base_crv, layer_height * i)
        mid_layer_curve = _move_curve_z(base_curve, layer_height * i)
        amp = amplitude * factor
        # add a contour curve for each layer
        contour_curve = create_contour_curve(layer_curve, amplitude, nozzle)
        # A: truss pattern
        a_pattern = truss_polyline(layer_curve, target_dist, amp, contact_length)
        a_pattern_pts = rs.CurvePoints(a_pattern)[1:-1] # remove first and last point
        start_pt = rs.PointAdd(rs.CurveStartPoint(layer_curve), input_normal * -amplitude)
        start_pt = rs.PointAdd(start_pt, input_tangent * contact_length/2)
        start_corner_pt = rs.PointAdd(start_pt, input_tangent * -contact_length)
        end_pt = rs.PointAdd(rs.CurveEndPoint(layer_curve), input_normal * -amplitude)
        end_pt = rs.PointAdd(end_pt, input_tangent * -contact_length/2)
        end_corner_pt = rs.PointAdd(end_pt, input_tangent * contact_length)
        a_pattern_pts.insert(0, start_pt)
        a_pattern_pts.insert(0, start_corner_pt)
        a_pattern_pts.append(end_pt)
        a_pattern_pts.append(end_corner_pt)
        # rs.CurveSeam(contour_curve, rs.CurveClosestPoint(contour_curve, rs.CurveStartPoint(a_pattern)))
        if draw_contour:
            a_pattern_pts.extend(rs.CurvePoints(contour_curve))
        a_layers.append(rs.AddPolyline(a_pattern_pts))

        # B: sine-like pattern
        b_pattern = sine_like_curve(layer_curve, target_dist, amp, contact_length, handle_length)
        b_pattern = rs.ConvertCurveToPolyline(b_pattern, 5, 1, False, 5, 1000)
        b_start = rs.CurveStartPoint(b_pattern)
        b_start_pt = rs.PointAdd(rs.CurveStartPoint(layer_curve), input_normal * -amplitude)
        b_start_pt = rs.PointAdd(b_start_pt, input_tangent * -contact_length/2)
        b_corner_pt = rs.PointAdd(b_start_pt, input_tangent * contact_length/2)
        b_end = rs.CurveEndPoint(b_pattern)
        b_end_pt = rs.PointAdd(rs.CurveEndPoint(layer_curve), input_normal * -amplitude)
        b_end_pt = rs.PointAdd(end_pt, input_tangent * -contact_length)
        b_end_corner_pt = rs.PointAdd(end_pt, input_tangent * contact_length)
        
        b_start_segment = rs.AddPolyline((b_start, b_start_pt, b_corner_pt))
        b_end_segment = rs.AddPolyline((b_end, b_end_pt, b_end_corner_pt))
        b_pattern = rs.JoinCurves((b_start_segment, b_pattern, b_end_segment))
        # b_pattern_pts.append(end_pt)
        # b_pattern_pts.append(end_corner_pt)
        # rs.CurveSeam(contour_curve, rs.CurveClosestPoint(contour_curve, rs.CurveStartPoint(b_pattern)))
        start_corner_pt = rs.PointAdd(rs.CurveStartPoint(layer_curve), input_normal * -amplitude)
        end_corner_pt = rs.PointAdd(rs.CurveEndPoint(layer_curve), input_normal * -amplitude)
        if draw_contour:
            b_layers.append(rs.JoinCurves((b_pattern, contour_curve)))
            # b_layers.append(contour_curve)
        else:
            b_layers.append(b_pattern)
        # b_layers.append(sine_like_curve(layer_curve, target_dist, amp, contact_length, handle_length))

        # C: bezier pattern
        c_pattern = bezier_curve(layer_curve, target_dist, amp, contact_length, handle_length)
        c_pattern = rs.ConvertCurveToPolyline(c_pattern, 5, 1, False, 5, 1000)
        c_start = rs.CurveStartPoint(c_pattern)
        c_start_pt = rs.PointAdd(rs.CurveStartPoint(layer_curve), input_normal * -amplitude)
        c_start_pt = rs.PointAdd(c_start_pt, input_tangent * -contact_length/2)
        c_corner_pt = rs.PointAdd(c_start_pt, input_tangent * contact_length/2)
        c_end = rs.CurveEndPoint(c_pattern)
        c_end_pt = rs.PointAdd(rs.CurveEndPoint(layer_curve), input_normal * -amplitude)
        c_end_pt = rs.PointAdd(end_pt, input_tangent * contact_length)
        c_end_corner_pt = rs.PointAdd(end_pt, input_tangent * contact_length)
        
        c_start_segment = rs.AddPolyline((c_start, c_start_pt, c_corner_pt))
        c_end_segment = rs.AddPolyline((c_end, c_end_pt, c_end_corner_pt))
        c_pattern = rs.JoinCurves((c_start_segment, c_pattern, c_end_segment))
        rs.CurveSeam(contour_curve, rs.CurveClosestPoint(contour_curve, rs.CurveStartPoint(c_pattern)))
        if draw_contour:
            c_layers.append(rs.JoinCurves((c_pattern, contour_curve)))
        else:
            c_layers.append(c_pattern)
        # c_layers.append(bezier_curve(layer_curve, target_dist, amp, contact_length, handle_length))

        # D: true sine wave pattern
        layer_length = rs.CurveLength(layer_curve)
        div_length = max(1, int(round(layer_length / target_dist)))
        wavelength = layer_length / div_length
        d_pattern = sine_curve_on_base(layer_curve, wavelength, amp, phase, sample_count)
        d_pattern_pts = rs.CurvePoints(d_pattern)
        rs.CurveSeam(contour_curve, rs.CurveClosestPoint(contour_curve, rs.CurveStartPoint(d_pattern)))
        if draw_contour:
            d_pattern_pts.extend(rs.CurvePoints(contour_curve))
        d_layers.append(rs.AddPolyline(d_pattern_pts))
        # d_layers.append(sine_curve_on_base(layer_curve, wavelength, amp, phase, sample_count))

        # E: Mid_layer

        # External
        mid_layer_base = rs.CopyObject(mid_layer_curve, input_normal * nozzle/2)
        ext_pts = [rs.EvaluateCurve(mid_layer_base, rs.CurveParameter(input_crv, t)) for t in div_params[1:-1:2]] # exclude first and last points and take every other point

        # ext_layer_base = rs.CopyObject(mid_layer_base, input_normal * ext_amplitude)
        ext_layer_target = rs.CopyObject(mid_layer_base, -input_normal * ext_amplitude)
        mid_layer = []

        mid_layer = [rs.CurveStartPoint(mid_layer_base)]
        # print(f"normalised: {normalised_factors[i]}, input: {ext_amplitude * normalised_factors[i]}")
        for indent_pts in create_indents_target(ext_pts, mid_layer_base, 20, (ext_amplitude - nozzle/2) * normalised_factors[i], ext_layer_target):
            # indent_pts is a 4-tuple; expand into a flat list
            mid_layer.extend(indent_pts)
        mid_layer.append(rs.CurveEndPoint(mid_layer_base))

        # Close curve
        mid_layer.append(rs.CurveEndPoint(ext_layer_target))
        mid_layer.append(rs.CurveStartPoint(ext_layer_target))
        mid_layer.append(rs.CurveStartPoint(mid_layer_base))
        

        e_layers.append(rs.AddPolyline(mid_layer))
        # TEST:
        # e_layers.append(layer_curve)
        # print(mid_layer)
        a_layers[i] = connect_two_curves(a_layers[i], e_layers[i], 0)
        b_layers[i] = connect_two_curves(b_layers[i], e_layers[i], 0)
        c_layers[i] = connect_two_curves(c_layers[i], e_layers[i], 0)
        d_layers[i] = connect_two_curves(d_layers[i], e_layers[i], 0)
    return a_layers, b_layers, c_layers, d_layers, e_layers

# mid_layer.append(rs.CurveEndPoint(input_crv))

# TODO: Calculate overhang angle
def _as_float(value, default=0.0):
    """Convert GH inputs (scalar or single-item sequence) to float."""
    if isinstance(value, (list, tuple)):
        value = value[0] if value else default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default

base = _as_float(amplitude) * 2.0
height = _as_float(layer_height) * _as_float(transition_layers)
angle = math.degrees(math.atan2(height, base)) if base > contact_tol else 90.0

# Fix length input curve
input_start_tangent = rs.VectorUnitize(rs.CurveTangent(input_crv, rs.CurveClosestPoint(input_crv, rs.CurveStartPoint(input_crv))))
input_end_tangent = rs.VectorUnitize(rs.CurveTangent(input_crv, rs.CurveClosestPoint(input_crv, rs.CurveEndPoint(input_crv))))
new_input_start_pt = rs.PointAdd(rs.CurveStartPoint(input_crv), input_start_tangent * nozzle /2)
new_input_end_pt = rs.PointAdd(rs.CurveEndPoint(input_crv), -input_end_tangent * nozzle /2)
input_crv = rs.SplitCurve(input_crv, [rs.CurveClosestPoint(input_crv, new_input_start_pt), rs.CurveClosestPoint(input_crv, new_input_end_pt)])[1]

# a_list, b_list, c_list, d_list 
a_list, b_list, c_list, d_list , e_list = build_layer_stack(
    input_crv, layer_height, layer_count, base_layers, transition_layers,
    target_dist, amplitude, contact_length, handle_length, phase, sample_count,
    offset_dist
)
a = th.list_to_tree(a_list)
b = th.list_to_tree(b_list)
c = th.list_to_tree(c_list)
d = th.list_to_tree(d_list)
e = th.list_to_tree(e_list)

# e = rs.CurveEndPoint(mid_layer_base)
