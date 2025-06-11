#! python3

"""Grasshopper script


Inputs:
    crvs (list of curves): Curve slices representing geometry layers
    srf (surface): Rhino surface with image mapping
    img (Bitmap): Texture image to be sampled
    pattern (str): Repeating pattern (e.g. 'AABB') used to alternate the generated line types
    dist_range (tuple): (min, max) output range mapped from red channel
    amp_range (tuple): (min, max) output range mapped from green channel
    shift_range (tuple): (min, max) output range mapped from blue channel

Output:
    a (list of PolylineCurve GUIDs): Alternating polylines stacked according to the image-driven pattern

"""
__author__ = "joseh"
__version__ = "2025.05.03"


import rhinoscriptsyntax as rs
import geometrylib as gl
import curvelib as cl
from itertools import cycle
from srflib import sample_surface_color
from geometrylib import normalize_pattern


def rgb_to_parameters(rgb, dist_range, amp_range, shift_range):
    """
    Remaps RGB values (0-255) to custom ranges for division distance, amplitude, and shift.

    Parameters:
        rgb (tuple): RGB tuple (R, G, B), each 0-255.
        dist_range (tuple): (min, max) for spacing between divisions.
        amp_range (tuple): (min, max) for amplitude of perpendicular offsets.
        shift_range (tuple): (min, max) for tangential shift.

    Returns:
        tuple: (dist, amp, shift) as remapped float values.
    """
    r, g, b = rgb
    dist = gl.remap(0, 255, *dist_range, r)
    amp = gl.remap(0, 255, *amp_range, g)
    shift = gl.remap(0, 255, *shift_range, b)
    return dist, amp, shift


def get_div_pts_and_params(crv, srf, img, dist_range, amp_range, shift_range):
    """
    Samples along a curve using dynamic distance based on image colour mapping.

    Parameters:
        crv (GUID): Curve to sample along.
        srf (GUID): Surface with texture mapping.
        img (Bitmap): Image used for sampling.
        dist_range, amp_range, shift_range (tuple): Remap ranges for RGB values.

    Returns:
        list: Tuples of (point, amplitude, shift).
    """
    div_data = []
    domain = rs.CurveDomain(crv)
    t = domain[0]
    t_max = domain[1]

    while t < t_max:
        pt = rs.EvaluateCurve(crv, t)
        rgb = sample_surface_color(pt, srf, img)
        dist, amp, shift = rgb_to_parameters(
            rgb, dist_range, amp_range, shift_range)
        div_data.append((pt, amp, shift))
        t += dist

    return div_data


def build_alternating_polylines(crv, div_data):
    """
    Builds two interleaved polylines from sampled points with alternating amplitude and shift.

    Parameters:
        crv (GUID): Original curve used for reference tangents.
        div_data (list): List of (point, amp, shift) from sampling function.

    Returns:
        tuple: (polyline1, polyline2) as point lists.
    """
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


def generate_pattern_stack(crvs, srf, img, pattern, dist_range, amp_range, shift_range):
    """
    Generates a stack of alternating Rhino polylines from image-mapped surface data and curve slices.

    Returns:
        list: List of PolylineCurve GUIDs
    """
    stacked = []
    normalized_pattern = normalize_pattern(pattern)
    pattern_gen = cycle(normalized_pattern)

    for crv in crvs:
        div_data = get_div_pts_and_params(
            crv, srf, img, dist_range, amp_range, shift_range)
        poly1, poly2 = build_alternating_polylines(crv, div_data)
        tag = next(pattern_gen)
        polyline = rs.AddPolyline(poly1 if tag == 'A' else poly2)
        if polyline:
            stacked.append(polyline)

    return stacked


def sample_coloured_parameters_from_image(crvs, srf, img, dist_range, amp_range, shift_range):
    """
    Extracts colour-based parameters from multiple curves for inspection or analysis.

    Parameters:
        crvs (list): List of curves to evaluate.
        srf (GUID): Surface with image mapping.
        img (Bitmap): Texture image.
        dist_range, amp_range, shift_range (tuple): RGB remap target ranges.

    Returns:
        list: Tuples of (point, dist, amp, shift).
    """
    results = []
    for curve in crvs:
        div_data = get_div_pts_and_params(
            curve, srf, img, dist_range, amp_range, shift_range)
        for pt, amp, shift in div_data:
            rgb = sample_surface_color(pt, srf, img)
            dist, _, _ = rgb_to_parameters(
                rgb, dist_range, amp_range, shift_range)
            results.append((pt, dist, amp, shift))

    return results


# Grasshopper-compatible example usage:
# Connect these as inputs in the Python component:
# Inputs: crvs (list of curves), srf (surface), img (bitmap), pattern (str),
#         dist_range (tuple), amp_range (tuple), shift_range (tuple)
# Output: a (list of polylines)

dist_range = (40, 100)
amp_range = (80, 20)
shift_range = (-20, 20)

a = generate_pattern_stack(crvs, srf, img, pattern,
                           dist_range, amp_range, shift_range)
