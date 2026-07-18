import Rhino
import rhinoscriptsyntax as rs
import System
from System.Drawing import Bitmap
from itertools import cycle
import geometrylib as gl
import iolib as io
import math


def _coerce_bitmap(image_source):
    """Return a Bitmap from a file path or an existing bitmap-like input."""
    if isinstance(image_source, Bitmap):
        return image_source
    if hasattr(image_source, "Width") and hasattr(image_source, "Height") and hasattr(image_source, "GetPixel"):
        return image_source
    return Bitmap(image_source)


def _surface_domains_and_uv(surface, pt3d):
    """Resolve closest UV and domains for Rhino GUIDs or RhinoCommon surfaces/breps."""
    if isinstance(surface, (str, System.Guid)):
        uv = rs.SurfaceClosestPoint(surface, pt3d)
        u_dom = rs.SurfaceDomain(surface, 0)
        v_dom = rs.SurfaceDomain(surface, 1)
        return uv, u_dom, v_dom

    if isinstance(surface, Rhino.Geometry.Surface):
        success, u, v = surface.ClosestPoint(pt3d)
        if not success:
            raise ValueError("Could not find closest point on the input surface.")
        u_dom = surface.Domain(0)
        v_dom = surface.Domain(1)
        return (u, v), (u_dom.T0, u_dom.T1), (v_dom.T0, v_dom.T1)

    if isinstance(surface, Rhino.Geometry.Brep):
        success, u, v, face_index = surface.ClosestPoint(pt3d)
        if not success or face_index < 0:
            raise ValueError("Could not find closest point on the input brep.")
        face = surface.Faces[face_index]
        u_dom = face.Domain(0)
        v_dom = face.Domain(1)
        return (u, v), (u_dom.T0, u_dom.T1), (v_dom.T0, v_dom.T1)

    coerced_brep = rs.coercebrep(surface, False)
    if coerced_brep:
        return _surface_domains_and_uv(coerced_brep, pt3d)

    coerced_surface = rs.coercesurface(surface, False)
    if coerced_surface:
        return _surface_domains_and_uv(coerced_surface, pt3d)

    raise TypeError("Surface input must be a Rhino surface, brep, or Guid.")

def closest_srf(pt, srf0, srf1):
    """returns the surface that is closest to a certain point"""
    param0 = rs.SurfaceClosestPoint(srf0, pt)
    param1 = rs.SurfaceClosestPoint(srf1, pt)
    srf0_pt = rs.EvaluateSurface(srf0, param0[0] , param0[1])
    srf1_pt = rs.EvaluateSurface(srf1, param1[0] , param1[1])
    d0 = rs.Distance(pt, srf0_pt)
    d1 = rs.Distance(pt, srf1_pt)
    print('distance: ', d0, d1)
    if d0 >= d1:
        return(srf1, 1)
    else:
        return(srf0, 0)

def sample_surface_color(pt, surface, image_path, as_hsl=False):
    """
    Sample the color of a texture mapped to a Rhino surface at the closest point to a given 3D point.

    Parameters:
        pt (Point3d or list/tuple of 3 floats): The 3D point to sample from.
        surface (GUID): The Rhino surface or polysurface to sample.
        image_path (str): File path to the texture image.
        as_hsl (bool): If True, returns the color as (H, S, L); otherwise as (R, G, B).

    Returns:
        tuple: (R, G, B) or (H, S, L) values of the sampled texture at the closest surface point.
    """
    # Ensure we have a Point3d
    pt3d = rs.coerce3dpoint(pt)

    # Load the image
    img = _coerce_bitmap(image_path)

    # Find UV parameter on surface for closest point
    (u, v), u_dom, v_dom = _surface_domains_and_uv(surface, pt3d)

    # Image dimensions
    img_w = img.Width
    img_h = img.Height

    # Map U to pixel X and V to pixel Y (flip V because image origin is top-left)
    px = int(round(gl.remap(u_dom[0], u_dom[1], 0, img_w - 1, u)))
    py = int(round(gl.remap(v_dom[0], v_dom[1], img_h - 1, 0, v)))

    # Sample pixel color
    color = img.GetPixel(px, py)

    if as_hsl:
        # Rhino's ColorRGBToHLS returns (H, L, S)
        h, l, s = rs.ColorRGBToHLS(color)
        # Return as (H, S, L)
        return h, s, l
    else:
        # Return (R, G, B)
        return color.R, color.G, color.B

def remap_rgb_channels(rgb, channel_ranges):
    """
    Remaps RGB channels to value ranges.

    Parameters:
        rgb (tuple): RGB tuple (R, G, B), each 0–255.
        channel_ranges: Three tuples, each (min, max), corresponding to
                         remap targets for R, G, B channels.

    Returns:
        tuple: Remapped float values, one per channel.

    TODO: add example

    """
    if len(channel_ranges) != 3:
        raise ValueError("Expected remap ranges for R, G and B channels.")
    try:
        r, g, b = rgb
    except Exception as exc:
        raise ValueError("RGB input must unpack into three values.") from exc
    for channel_name, value in (("R", r), ("G", g), ("B", b)):
        io.validate_scalar(channel_name, value, min_value=0, max_value=255, allow_zero=True)


    range_1, range_2, range_3 = channel_ranges

    param1 = gl.remap(0, 255, *range_1, r)
    param2 = gl.remap(0, 255, *range_2, g)
    param3 = gl.remap(0, 255, *range_3, b)
    return param1, param2, param3

def get_division_parameters(crv, srf, img, param_config):
    div_data = []
    domain = rs.CurveDomain(crv)

    t = domain[0]
    t_max = domain[1]
    while t < t_max:
        pt = rs.EvaluateCurve(crv, t)
        rgb = sample_surface_color(pt, srf, img)
        dist = evaluate_parameter(*param_config['dist'], pt, rgb)
        amp  = evaluate_parameter(*param_config['amp'], pt, rgb)
        shift= evaluate_parameter(*param_config['shift'], pt, rgb)
        div_data.append((pt, amp, shift))
        t += dist
    # Safety: ensure end of curve is included
    if t > t_max and t - dist < t_max:
        pt_end = rs.EvaluateCurve(crv, t_max)
        rgb = sample_surface_color(pt_end, srf, img)
        dist = evaluate_parameter(*param_config['dist'], pt_end, rgb)
        amp  = evaluate_parameter(*param_config['amp'], pt_end, rgb)
        shift= evaluate_parameter(*param_config['shift'], pt_end, rgb)
        div_data.append((pt_end, amp, shift))
    
    if div_data[-1][0] != rs.EvaluateCurve(crv, t_max):
        pt_end = rs.EvaluateCurve(crv, t_max)
        rgb = sample_surface_color(pt_end, srf, img)
        dist = evaluate_parameter(*param_config['dist'], pt_end, rgb)
        amp  = evaluate_parameter(*param_config['amp'], pt_end, rgb)
        shift= evaluate_parameter(*param_config['shift'], pt_end, rgb)
        div_data.append((pt_end, amp, shift))
    return div_data

def build_alternating_polylines(crv, div_data):
    """
    Build the alternating polyline pairs based on amplitude and shift values.
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
    
    poly1 = [A1[0]]
    poly2 = [A2[0]]

    for i in range(len(div_data)):
        if i % 2 == 0:
            poly1.extend([A1[i], B1[i]])
            poly2.extend([B2[i], A2[i]])
        else:
            poly1.extend([B1[i], A1[i]])
            poly2.extend([A2[i], B2[i]])

    return poly1, poly2

def generate_pattern_stack(crvs, srf, img, pattern, param_config):
    stacked = []
    normalized_pattern = gl.normalize_pattern(pattern)
    pattern_gen = cycle(normalized_pattern)
    for crv in crvs:
        div_data = get_division_parameters(crv, srf, img, param_config)
        poly1, poly2 = build_alternating_polylines(crv, div_data)
        tag = next(pattern_gen)
        polyline = rs.AddPolyline(poly1 if not tag else poly2)
        if polyline:
            stacked.append(polyline)
    return stacked


def evaluate_parameter(mode, settings, pt, rgb):
    """
    Evaluates parameter value based on the given mode.

    mode: string, one of ['fixed', 'image_r', 'image_g', 'image_b', 'gradient']
    settings: value depending on mode:
        - 'fixed': scalar value
        - 'image_*': tuple (min, max)
        - 'gradient': dict with 'domain', 'range', 'axis', 'mode'
    pt: 3D point on curve (Point3d)
    rgb: tuple of (R, G, B) sampled colour values (0-255)

    Returns:
        float: remapped parameter value
    """
    if mode == 'fixed':
        return settings
    elif mode == 'image_r':
        return gl.remap(0, 255, *settings, rgb[0])
    elif mode == 'image_g':
        return gl.remap(0, 255, *settings, rgb[1])
    elif mode == 'image_b':
        return gl.remap(0, 255, *settings, rgb[2])
    elif mode == 'gradient':
        coord = pt[settings['axis']]
        t = gl.invlerp(*settings['domain'], coord)
        if settings['mode'] == 'linear':
            factor = t
        elif settings['mode'] == 'peak':
            factor = 2 * t if t <= 0.5 else 2 * (1 - t)
        elif settings['mode'] == 'valley':
            factor = 1 - (2 * t if t <= 0.5 else 2 * (1 - t))
        elif settings['mode'] == 'smoothpeak':
            factor = abs(math.sin(t * math.pi))
        elif settings['mode'] == 'smoothvalley':
            factor = 1 - abs(math.sin(t * math.pi))
        else:
            raise ValueError("Unknown gradient mode")
        return gl.lerp(*settings['range'], factor)
    else:
        raise ValueError("Unknown source mode")

def _gradient_factor(mode, t):
    """Return a normalized gradient factor for linear/reverse/peak/valley modes."""
    if mode == 'linear':
        return t
    if mode == 'reverse':
        return 1 - t
    if mode == 'peak':
        return 2 * t if t <= 0.5 else 2 * (1 - t)
    if mode == 'valley':
        return 1 - (2 * t if t <= 0.5 else 2 * (1 - t))
    raise ValueError("Invalid gradient mode.")


def build_gradient_pattern(crvs, mode='linear', min_val=0.0, max_val=1.0):
    """
    Build deterministic boolean pattern with flexible gradients.
    Modes: 'linear', 'reverse', 'peak', 'valley'
    """
    N = len(crvs)
    if N <= 1: # Prevents division by zero
        return [False] * N

    pattern = []
    for i in range(N):
        t = i / (N - 1)
        value = _gradient_factor(mode, t)

        # Apply threshold logic
        if value < min_val:
            pattern.append(False)
        elif value > max_val:
            pattern.append(True)
        else:
            pattern.append(i % 2 == 1)
    return pattern

def build_stack_pattern(crvs, mode='gradient', sequence=None, gradient_mode='linear', min_val=0.0, max_val=1.0):
    if not crvs:
        return []
    N = len(crvs)
    if N == 1:
        return [False]

    if mode == 'sequence':
        if not sequence:
            raise ValueError("Sequence mode requires a valid sequence list.")
        seq_length = len(sequence)
        return [bool(sequence[i % seq_length]) for i in range(N)]

    pattern = []
    for i in range(N):
        t = i / (N - 1)
        value = _gradient_factor(gradient_mode, t)
        if value < min_val:
            pattern.append(False)
        elif value > max_val:
            pattern.append(True)
        else:
            pattern.append(i % 2 == 1)
    return pattern
