#! python3

import rhinoscriptsyntax as rs
import math
import curvelib as cl
import geometrylib as gl
from ghpythonlib import treehelpers as th
from itertools import chain

contact_tol = 1e-6

base_layers, transition_layers = amp_modulation

def _curve_normal_xy(tangent):
    return rs.VectorRotate(tangent, 90, [0, 0, 1])

def create_contour_curve(base_curve, amplitude, offset_dist):
    tangent_vector = rs.VectorCreate(rs.CurveEndPoint(base_curve), rs.CurveStartPoint(base_curve))
    normal_vector = rs.VectorRotate(tangent_vector, 90, [0, 0, 1])
    curve_top = rs.CopyObject(base_curve, rs.VectorScale(rs.VectorUnitize(normal_vector), amplitude))
    curve_bottom = rs.CopyObject(base_curve, rs.VectorScale(rs.VectorUnitize(normal_vector), -amplitude))
    pt_0 = rs.CurveStartPoint(curve_bottom)
    pt_1 = rs.CurveStartPoint(curve_top)
    pt_2 = rs.CurveEndPoint(curve_top)
    pt_3 = rs.CurveEndPoint(curve_bottom)
    bbox = rs.AddPolyline([pt_0, pt_1, pt_2, pt_3, pt_0])
    return rs.OffsetCurve(bbox, [0, 0, 1], offset_dist)

def _add_contact_segment(container, contact):
    start_pt, end_pt, _ = contact
    if rs.Distance(start_pt, end_pt) > contact_tol:
        container.append(rs.AddLine(start_pt, end_pt))


#FIXME: this function doesn't work properly
def _join_contour_with_pattern(contour_curve, pattern_curve):
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


def build_contactlines(base_curve, target_dist, amplitude, contact_length):
    if not base_curve or target_dist <= 0:
        return [], []
    div_pts = cl.divide_crv_equal(base_curve, target_dist)
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

        if i == 0 or i == len(div_pts) - 1 or contact_length == 0:
            moved_pt = rs.PointAdd(pt, rs.VectorScale(normal, amplitude))
            truss_pts.append(moved_pt)
            contactlines.append((moved_pt, moved_pt, tangent))
            continue

        contactline = cl.centercrv_dir(pt, tangent, contact_length)
        contactline = rs.MoveObject(contactline, rs.VectorScale(normal, amplitude))
        start_pt = rs.CurveStartPoint(contactline)
        end_pt = rs.CurveEndPoint(contactline)
        truss_pts.append(start_pt)
        truss_pts.append(end_pt)
        contactlines.append((start_pt, end_pt, tangent))

    return truss_pts, contactlines


def truss_polyline(base_curve, target_dist, amplitude, contact_length):
    truss_pts, _ = build_contactlines(base_curve, target_dist, amplitude, contact_length)
    if not truss_pts:
        return None
    return rs.AddPolyline(truss_pts)


def sine_like_curve(base_curve, target_dist, amplitude, contact_length, handle_length):
    _, contactlines = build_contactlines(base_curve, target_dist, amplitude, contact_length)
    if len(contactlines) < 2:
        return None
    sine_like = []
    for i in range(len(contactlines) - 1):
        _, start_pt, start_tangent = contactlines[i]
        end_pt, _, end_tangent = contactlines[i + 1]
        start_handle = rs.PointAdd(start_pt, rs.VectorScale(start_tangent, handle_length))
        end_handle = rs.PointAdd(end_pt, rs.VectorScale(end_tangent, -handle_length))
        _add_contact_segment(sine_like, contactlines[i])
        sine_like.append(rs.AddCurve([start_pt, start_handle, end_handle, end_pt], 3))
    _add_contact_segment(sine_like, contactlines[-1])
    return rs.JoinCurves(sine_like)


def bezier_curve(base_curve, target_dist, amplitude, contact_length, handle_length):
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
        _add_contact_segment(bezier_crvs, contactlines[i])
        bezier_crvs.append(rs.AddCurve([start_pt, start_handle, end_handle, end_pt], 3))
    _add_contact_segment(bezier_crvs, contactlines[-1])
    return rs.JoinCurves(bezier_crvs)


def sine_curve_on_base(base_curve, wavelength, amplitude, phase, sample_count=50):
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
    if not curve or z_offset == 0:
        return curve
    return rs.CopyObject(curve, [0, 0, z_offset])


def build_layer_stack(base_curve, layer_height, layer_count, base_layers, transition_layers,
                      target_dist, amplitude, contact_length, handle_length, phase, sample_count,
                      offset_dist):
    factors = _amplitude_factors(layer_count, base_layers, transition_layers)
    if not factors:
        return [], [], [], []
    a_layers = []
    b_layers = []
    c_layers = []
    d_layers = []
    for i, factor in enumerate(factors):
        layer_curve = _move_curve_z(base_curve, layer_height * i)
        amp = amplitude * factor
        # add a contour curve for each layer
        contour_curve = create_contour_curve(layer_curve, amplitude, offset_dist)
        # A: truss pattern
        a_pattern = truss_polyline(layer_curve, target_dist, amp, contact_length)
        a_pattern_pts = rs.CurvePoints(a_pattern)
        rs.CurveSeam(contour_curve, rs.CurveClosestPoint(contour_curve, rs.CurveStartPoint(a_pattern)))
        if draw_contour:
            a_pattern_pts.extend(rs.CurvePoints(contour_curve))
        a_layers.append(rs.AddPolyline(a_pattern_pts))

        # B: sine-like pattern
        b_pattern = sine_like_curve(layer_curve, target_dist, amp, contact_length, handle_length)
        rs.CurveSeam(contour_curve, rs.CurveClosestPoint(contour_curve, rs.CurveStartPoint(b_pattern)))
        if draw_contour:
            b_layers.append(_join_contour_with_pattern(contour_curve, b_pattern))
        else:
            b_layers.append(b_pattern)
        # b_layers.append(sine_like_curve(layer_curve, target_dist, amp, contact_length, handle_length))

        # C: bezier pattern
        c_pattern = bezier_curve(layer_curve, target_dist, amp, contact_length, handle_length)
        rs.CurveSeam(contour_curve, rs.CurveClosestPoint(contour_curve, rs.CurveStartPoint(c_pattern)))
        if draw_contour:
            c_layers.append(_join_contour_with_pattern(contour_curve, c_pattern))
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
    return a_layers, b_layers, c_layers, d_layers


sample_count = 50
offset_dist = 10
a_list, b_list, c_list, d_list = build_layer_stack(
    base_crv, layer_height, layer_count, base_layers, transition_layers,
    target_dist, amplitude, contact_length, handle_length, phase, sample_count,
    offset_dist
)
a = th.list_to_tree(a_list)
b = th.list_to_tree(b_list)
c = th.list_to_tree(c_list)
d = th.list_to_tree(d_list)
