#! python3

"""
Image Pattern Stack — Modular Version (Grasshopper Script)

Fully modular and reusable version with control over:
  - independent parameter sources (image, fixed value, gradients)
  - optional parameter correlations
  - pattern normalization and alternation control

Inputs:
    crvs: list of slicing curves
    srf: mapped surface for image sampling
    img: loaded Bitmap image
    pattern: list of alternating pattern instructions (A/B/0/1/True/False)
    param_sources: dict specifying 'image', 'fixed', or 'gradient' source for each parameter
    param_values: dict containing ranges or fixed values per parameter

Outputs:
    a: list of PolylineCurve GUIDs for Grasshopper
"""

import rhinoscriptsyntax as rs
import geometrylib as gl
import curvelib as cl
from itertools import cycle
from srflib import sample_surface_color


# UTILITIES

def remap_rgb_channels(rgb, *channel_ranges):
    return tuple(gl.remap(0, 255, *rng, channel) for channel, rng in zip(rgb, channel_ranges))

def normalize_pattern(pattern):
    if not pattern:
        return [False]
    normalized = []
    for p in pattern:
        if p in [0, False, '0', 'A', 'a']:
            normalized.append(False)
        elif p in [1, True, '1', 'B', 'b']:
            normalized.append(True)
        else:
            raise ValueError(f"Invalid pattern value: {p}")
    return normalized


def evaluate_parameter(source, value_def, pt, rgb=None):
    if source == 'fixed':
        return value_def
    elif source == 'image':
        remapped = gl.remap(0, 255, *value_def, rgb)
        return remapped
    elif source == 'gradient':
        dom = value_def['domain']
        rng = value_def['range']
        coord = pt[value_def['axis']]
        t = gl.invlerp(dom[0], dom[1], coord)
        return gl.lerp(rng[0], rng[1], t)
    else:
        raise ValueError(f"Invalid source type: {source}")


# MAIN SAMPLE FUNCTION

def get_division_parameters(crv, srf, img, param_sources, param_values):
    div_data = []
    domain = rs.CurveDomain(crv)
    t = domain[0]
    t_max = domain[1]

    while t < t_max:
        pt = rs.EvaluateCurve(crv, t)
        rgb = sample_surface_color(pt, srf, img)

        dist_rgb = rgb[0]
        amp_rgb = rgb[1]
        shift_rgb = rgb[2]

        dist = evaluate_parameter(param_sources['dist'], param_values['dist'], pt, dist_rgb)
        amp  = evaluate_parameter(param_sources['amp'], param_values['amp'], pt, amp_rgb)
        shift = evaluate_parameter(param_sources['shift'], param_values['shift'], pt, shift_rgb)

        div_data.append((pt, amp, shift))
        t += dist

    return div_data


def build_alternating_polylines(crv, div_data):
    A1, B1, A2, B2 = [], [], [], []
    for i, (pt, amp, shift) in enumerate(div_data):
        param = rs.CurveClosestPoint(crv, pt)
        tan = rs.VectorUnitize(rs.CurveTangent(crv, param))
        perp = (-tan[1], tan[0], 0)

        A_base = rs.PointAdd(pt, rs.VectorScale(perp, amp))
        B_base = rs.PointAdd(pt, rs.VectorScale(perp, -amp))

        sign = 1 if i % 2 else -1
        A1.append(rs.PointAdd(A_base, rs.VectorScale(tan, shift * sign)))
        B1.append(rs.PointAdd(B_base, rs.VectorScale(tan, -shift * sign)))
        A2.append(rs.PointAdd(A_base, rs.VectorScale(tan, -shift * sign)))
        B2.append(rs.PointAdd(B_base, rs.VectorScale(tan, shift * sign)))

    poly1, poly2 = [], []
    for i in range(len(div_data) - 1):
        if i % 2 == 0:
            poly1.extend([A1[i], B1[i]])
            poly2.extend([B2[i], A2[i]])
        else:
            poly1.extend([B1[i], A1[i]])
            poly2.extend([A2[i], B2[i]])
    if div_data:
        poly1.append(B1[-1])
        poly2.append(A2[-1])

    return poly1, poly2


# FINAL STACK GENERATOR

def generate_modular_pattern_stack(crvs, srf, img, pattern, param_sources, param_values):
    stacked = []
    normalized_pattern = normalize_pattern(pattern)
    pattern_gen = cycle(normalized_pattern)

    for crv in crvs:
        div_data = get_division_parameters(crv, srf, img, param_sources, param_values)
        poly1, poly2 = build_alternating_polylines(crv, div_data)
        tag = next(pattern_gen)
        polyline = rs.AddPolyline(poly1 if not tag else poly2)
        if polyline:
            stacked.append(polyline)

    return stacked


# Grasshopper usage example input preparation:

param_sources = {
    'dist': 'image',
    'amp' : 'gradient',
    'shift': 'image'
}

param_values = {
    'dist': (40, 100),
    'amp' : { 'domain': (0, 200), 'range': (20, 80), 'axis': 0 },
    'shift': (-20, 20)
}

a = generate_modular_pattern_stack(crvs, srf, img, pattern, param_sources, param_values)
