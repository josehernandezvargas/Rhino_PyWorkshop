#! python3

"""A script variant that creates porous structures from a stack of curves composed of several segments
surface gradients can be procedurally generated

Each entry in crvs may be a simple curve or a polycurve made of several segments (lines, arcs, or
free-form spans). Every curve is exploded into its constituent segments and sampled independently
(each segment needs its own local tangent, and its own equal-length division), but every segment
shares its endpoint with its neighbour - so instead of blindly trimming points off both ends of every
segment (which cut real teeth out of short segments and still left the shared point duplicated),
the duplicate point at every segment joint (including the seam where a closed curve wraps around) is
dropped once the segments' points are concatenated into a single, continuous division list. The
alternating zigzag is then built once for that whole continuous list, so its A/B parity carries
through every joint instead of restarting at each segment.

Division spacing (TARGET_DIST) is a single fixed value shared by every curve and segment in the
stack - it no longer varies with image colour or the procedural gradient. This guarantees each layer
gets a consistent, predictable number of divisions relative to its length, which previously caused
points to scramble when different layers happened to sample different colours and therefore produced
different division counts. Amplitude and shift are still driven by image colour and/or the procedural
gradient as before.

Inputs:
    crvs (list of curves): Curve slices representing geometry layers. Simple curves and polycurves
        are both supported; polycurves are exploded into segments before sampling and rejoined after.
    srf (surface): Rhino surface with image mapping. Also used as the UV domain for the procedural
        gradient (GRADIENT_AXIS/GRADIENT_MODE/GRADIENT_BLEND below).
    img (Bitmap, optional): Texture image to be sampled. May be left unconnected (None); when no
        image is supplied, amp/shift are driven entirely by the procedural gradient below,
        regardless of GRADIENT_BLEND.
    pattern (str): Repeating pattern (e.g. 'AABB') used to alternate the generated line types.
    amp_range (tuple): (min, max) output range mapped from green channel / amplitude gradient.
    shift_range (tuple): (min, max) output range mapped from blue channel / shift gradient.

Constants (defined together at the bottom of the file):
    TARGET_DIST (float): Fixed division spacing shared by every curve/segment in the stack.
    GRADIENT_AXIS (str): Surface direction driving the gradient - 'U' or 'V'.
    GRADIENT_MODE (str): Gradient shape - 'linear', 'reverse', 'peak', or 'valley'.
    GRADIENT_BLEND (float 0-1): 0.0 uses only the image-sampled value, 1.0 uses only the procedural
        gradient value; values in between blend the two, both expressed in amp/shift_range.
    SKIP_POINTS (int): Division points at each end of every segment kept fully straight (no
        offset) - prevents the zigzag from overlapping itself when input curves are small
        relative to amp_range.
    AMP_TAPER_POINTS (int): Additional division points, beyond SKIP_POINTS at each end, over
        which amplitude ramps linearly back up to full instead of jumping straight to it.
    DEBUG (bool): When True, prints division counts, curve lengths, and other troubleshooting info.

Output:
    a (list of PolylineCurve GUIDs): One polyline per input curve, alternating line type according to
        the pattern and closed only when the corresponding source curve was closed.

"""
__author__ = "joseh"
__version__ = "2025.05.03"


import rhinoscriptsyntax as rs
import geometrylib as gl
import iolib as io
from itertools import cycle
from srflib import sample_surface_color
from geometrylib import normalize_pattern

# Prints division counts, curve lengths, and other troubleshooting info when True.
DEBUG = True


def gradient_factor(t, mode):
    """
    Maps a normalized surface parameter to a normalized (0-1) gradient factor.

    Parameters:
        t (float): Normalized surface parameter (0-1) along the chosen gradient axis.
        mode (str): One of 'linear', 'reverse', 'peak', 'valley'.

    Returns:
        float: Gradient factor in the 0-1 range.
    """
    t = gl.minmaxcap(0, 1, t)
    if mode == 'linear':
        return t
    if mode == 'reverse':
        return 1 - t
    if mode == 'peak':
        return 2 * t if t <= 0.5 else 2 * (1 - t)
    if mode == 'valley':
        return 1 - (2 * t if t <= 0.5 else 2 * (1 - t))
    raise ValueError(f"Unknown gradient mode: {mode}")


def sample_surface_gradient_t(pt, srf, axis):
    """
    Finds the closest point to pt on srf and normalizes its UV coordinate along axis to 0-1.

    Parameters:
        pt (point): 3D point to project onto the surface.
        srf (GUID): Surface used as the gradient domain.
        axis (str): 'U' or 'V'.

    Returns:
        float: Normalized (0-1) parameter along the requested surface axis.
    """
    u, v = rs.SurfaceClosestPoint(srf, pt)
    u_dom = rs.SurfaceDomain(srf, 0)
    v_dom = rs.SurfaceDomain(srf, 1)
    if axis.upper() == 'U':
        return gl.invlerp(u_dom[0], u_dom[1], u)
    return gl.invlerp(v_dom[0], v_dom[1], v)


def rgb_to_amp_shift(rgb, grad_t, amp_range, shift_range, gradient_mode, gradient_blend):
    """
    Remaps RGB values (0-255) to amplitude/shift ranges, blended against a procedural gradient
    sampled from the same ranges. Division spacing is no longer derived here - see TARGET_DIST.

    Parameters:
        rgb (tuple or None): RGB tuple (R, G, B), each 0-255, or None if no image is connected.
            When None, the result is driven entirely by the procedural gradient.
        grad_t (float): Normalized (0-1) surface parameter used for the procedural gradient.
        amp_range (tuple): (min, max) for amplitude of perpendicular offsets.
        shift_range (tuple): (min, max) for tangential shift.
        gradient_mode (str): Gradient shape - 'linear', 'reverse', 'peak', or 'valley'.
        gradient_blend (float): 0-1 weight between image-sampled (0.0) and gradient-driven (1.0) values.
            Ignored when rgb is None.

    Returns:
        tuple: (amp, shift) as remapped, gradient-blended float values.
    """
    factor = gradient_factor(grad_t, gradient_mode)
    grad_amp = gl.lerp(*amp_range, factor)
    grad_shift = gl.lerp(*shift_range, factor)

    if rgb is None:
        return grad_amp, grad_shift

    _, g, b = rgb
    image_amp = gl.remap(0, 255, *amp_range, g)
    image_shift = gl.remap(0, 255, *shift_range, b)

    amp = gl.lerp(image_amp, grad_amp, gradient_blend)
    shift = gl.lerp(image_shift, grad_shift, gradient_blend)
    return amp, shift


def compute_divisions(length, target_dist):
    """
    Determines how many equal-length divisions a segment should be split into.

    A segment shorter than twice target_dist can't be split into two halves that
    each meet target_dist, so dividing it at all would never satisfy the spacing
    requirement - it is kept as a single straight span (one division) instead of
    forcing a partial zigzag tooth that doesn't fit.

    Parameters:
        length (float): Segment length.
        target_dist (float): Target spacing between divisions.

    Returns:
        int: Number of divisions (>= 1).
    """
    if length < 2 * target_dist:
        return 1
    return max(1, int(round(length / target_dist)))


def compute_amp_factors(num_points, divs, skip_points, taper_points):
    """
    Determines, per division point of a segment, whether the point is kept fully straight and
    how much its amplitude should be scaled - so the zigzag tapers to zero near both ends of the
    segment instead of jumping straight to full amplitude, which causes overlap when amp is large
    relative to segment length.

    Parameters:
        num_points (int): Number of division points sampled on the segment.
        divs (int): Division count the segment resolved to (num_points - 1). A segment forced to
            a single division (see compute_divisions) is always kept fully straight.
        skip_points (int): Number of points at each end of the segment to keep fully straight
            (amp/shift zeroed), regardless of taper_points.
        taper_points (int): Number of additional points, beyond skip_points at each end, over
            which amplitude ramps linearly from 0 back up to full - 0 disables the ramp (hard
            cutoff straight into full amplitude right after the skipped points).

    Returns:
        list: (is_straight, amp_factor) tuples, one per division point, in order.
    """
    if num_points == 0:
        return []

    last = num_points - 1
    factors = []
    for i in range(num_points):
        distance = min(i, last - i)
        if divs == 1 or distance < skip_points:
            factors.append((True, 0.0))
        elif taper_points > 0 and distance < skip_points + taper_points:
            factors.append((False, (distance - skip_points) / taper_points))
        else:
            factors.append((False, 1.0))
    return factors


def get_div_pts_and_params(crv, srf, img, target_dist, amp_range, shift_range,
                            gradient_axis, gradient_mode, gradient_blend):
    """
    Samples a curve at equal-length divisions (target_dist, fixed for the whole stack) and
    computes amplitude/shift at each division point from image colour and/or procedural gradient.
    Segments too short to fit even two divisions at target_dist are kept as a single straight
    span - see compute_divisions.

    Parameters:
        crv (GUID): Curve to sample along.
        srf (GUID): Surface with texture mapping and gradient domain.
        img (Bitmap or None): Image used for sampling. If None, the procedural gradient alone
            drives amp/shift (equivalent to gradient_blend=1.0).
        target_dist (float): Fixed target spacing between divisions, shared by every curve and
            segment in the stack so each layer ends up with a consistent, predictable division count.
        amp_range, shift_range (tuple): Remap ranges for amplitude/shift.
        gradient_axis (str): 'U' or 'V' surface direction driving the gradient.
        gradient_mode (str): Gradient shape - 'linear', 'reverse', 'peak', or 'valley'.
        gradient_blend (float): 0-1 weight between image-sampled and gradient-driven values.

    Returns:
        list: Tuples of (point, amplitude, shift).
    """
    length = rs.CurveLength(crv) or 0.0
    divs = compute_divisions(length, target_dist) if length > 0 else 0
    points = rs.DivideCurve(crv, divs, False, True) if divs > 0 else []

    if DEBUG:
        print(f"[get_div_pts_and_params] crv={crv} length={length:.3f} "
              f"target_dist={target_dist} divisions={(len(points) - 1) if points else 0} "
              f"points={len(points) if points else 0}")

    div_data = []
    if not points:
        return div_data

    for pt in points:
        rgb = sample_surface_color(pt, srf, img) if img is not None else None
        grad_t = sample_surface_gradient_t(pt, srf, gradient_axis)
        amp, shift = rgb_to_amp_shift(rgb, grad_t, amp_range, shift_range, gradient_mode, gradient_blend)
        div_data.append((pt, amp, shift))

    return div_data


def collect_layer_div_data(segments, srf, img, target_dist, amp_range, shift_range,
                            gradient_axis, gradient_mode, gradient_blend,
                            skip_points=0, taper_points=0):
    """
    Samples every segment of a curve and concatenates their division points into one continuous
    list. Consecutive segments share an endpoint where they meet, so each segment (after the
    first) drops its own first sample - it is a duplicate of the previous segment's last sample -
    instead of independently re-sampling and re-zigzagging across the same point twice.

    A segment gets exactly one division (see compute_divisions) only when it is too short to
    fit two divisions at target_dist, in which case it is kept fully straight. skip_points and
    taper_points additionally flatten/taper the amplitude near the ends of every segment (see
    compute_amp_factors), since curves that are small relative to amp_range otherwise overlap
    themselves right where segments meet. Either way, the point a segment shares with the
    previous one is flagged/zeroed to match, since that shared point survives in the combined
    list as the previous segment's own last sample.

    Parameters:
        segments (list of GUID): The curve's exploded segments, in order.
        srf, img, target_dist, amp_range, shift_range, gradient_axis, gradient_mode, gradient_blend:
            Passed straight through to get_div_pts_and_params.
        skip_points (int): Division points at each end of every segment to keep fully straight.
        taper_points (int): Additional division points, beyond skip_points, over which amplitude
            ramps linearly back up to full - 0 disables the ramp.

    Returns:
        tuple: (combined, segment_divisions)
            combined (list): Tuples of (point, amp, shift, segment, is_straight) - segment is the
                exploded curve the point was sampled from (used for tangent evaluation);
                is_straight marks points that must be rendered without any zigzag offset.
            segment_divisions (list of int): Division count used for each segment, in order -
                used by generate_pattern_stack to check that division counts stay constant
                across layers.
    """
    segment_points = []
    segment_divisions = []
    for segment in segments:
        div_data = get_div_pts_and_params(
            segment, srf, img, target_dist, amp_range, shift_range,
            gradient_axis, gradient_mode, gradient_blend)
        divs = len(div_data) - 1 if div_data else 0
        segment_divisions.append(divs)
        factors = compute_amp_factors(len(div_data), divs, skip_points, taper_points)
        segment_points.append([
            [pt, 0.0, 0.0, True] if is_straight else [pt, amp * factor, shift, False]
            for (pt, amp, shift), (is_straight, factor) in zip(div_data, factors)
        ])

    for idx, points in enumerate(segment_points):
        if not points or not points[0][3]:
            continue
        if idx > 0 and segment_points[idx - 1]:
            prev_last = segment_points[idx - 1][-1]
            prev_last[1] = 0.0
            prev_last[2] = 0.0
            prev_last[3] = True

    combined = []
    for idx, points in enumerate(segment_points):
        segment = segments[idx]
        if idx > 0 and points:
            points = points[1:]
        combined.extend((pt, amp, shift, segment, is_straight) for pt, amp, shift, is_straight in points)
    return combined, segment_divisions


def build_alternating_polylines(div_data):
    """
    Builds two interleaved polylines from sampled points with alternating amplitude and shift.
    The A/B parity runs continuously across the whole div_data list, so when it spans several
    segments (see collect_layer_div_data) the zigzag carries through segment joints unbroken.
    Points flagged is_straight (segments too short to fit a zigzag - see collect_layer_div_data)
    are passed straight through unmodified instead of being offset into a tooth.

    Parameters:
        div_data (list): List of (point, amp, shift, segment, is_straight) tuples. segment is
            the curve used to evaluate the tangent at that point (its own exploded segment, so
            tangents stay correct on either side of a kink).

    Returns:
        tuple: (polyline1, polyline2) as point lists.
    """
    A1, B1, A2, B2 = [], [], [], []

    for i, (pt, amp, shift, segment, is_straight) in enumerate(div_data):
        if is_straight:
            A1.append(pt)
            B1.append(pt)
            A2.append(pt)
            B2.append(pt)
            continue

        param = rs.CurveClosestPoint(segment, pt)
        tan = rs.VectorUnitize(rs.CurveTangent(segment, param))
        perp = (-tan[1], tan[0], 0)

        A_base = rs.PointAdd(pt, rs.VectorScale(perp, amp))
        B_base = rs.PointAdd(pt, rs.VectorScale(perp, -amp))

        sign = 1 if i % 2 else -1
        A1.append(rs.PointAdd(A_base, rs.VectorScale(tan, shift * sign)))
        B1.append(rs.PointAdd(B_base, rs.VectorScale(tan, -shift * sign)))
        A2.append(rs.PointAdd(A_base, rs.VectorScale(tan, -shift * sign)))
        B2.append(rs.PointAdd(B_base, rs.VectorScale(tan, shift * sign)))

    if not div_data:
        return [], []

    poly1 = [A1[0]]
    poly2 = [A2[0]]

    for i in range(len(div_data)):
        if div_data[i][4]:
            # Straight point: A/B already collapse to the same point, so just pass through once.
            poly1.append(A1[i])
            poly2.append(A2[i])
        elif i % 2 == 0:
            poly1.extend([A1[i], B1[i]])
            poly2.extend([B2[i], A2[i]])
        else:
            poly1.extend([B1[i], A1[i]])
            poly2.extend([A2[i], B2[i]])

    return poly1, poly2


def explode_to_segments(crv):
    """
    Explodes a curve into its constituent segments so each span can be sampled independently.

    Parameters:
        crv (GUID): Curve to explode. Polycurves are split into their segments; curves that
            cannot be exploded (already a single span) are returned unchanged.

    Returns:
        tuple: (segments, is_closed, exploded)
            segments (list of GUID): The curve's segments, or [crv] if it could not be exploded.
            is_closed (bool): Whether the original curve was closed.
            exploded (bool): True if new segment objects were added to the document and should
                be cleaned up by the caller once they are no longer needed.
    """
    is_closed = rs.IsCurveClosed(crv)
    segments = rs.ExplodeCurves(crv, delete_input=False)
    if segments:
        return segments, is_closed, True
    return [crv], is_closed, False


def generate_pattern_stack(crvs, srf, img, pattern, target_dist, amp_range, shift_range,
                            gradient_axis, gradient_mode, gradient_blend,
                            skip_points=0, taper_points=0):
    """
    Generates a stack of alternating Rhino polylines from image-mapped and gradient-driven surface
    data. Each input curve is exploded into segments, every segment is divided at the same fixed
    target_dist, and the segments' division points are stitched into one continuous list (dropping
    the duplicate point at every joint, including the seam of a closed curve) before a single
    continuous zigzag is built across the whole curve.

    skip_points/taper_points keep the zigzag away from the ends of every segment (see
    compute_amp_factors) - needed because curves that are small relative to amp_range otherwise
    produce heavy overlap right where segments meet.

    Returns:
        list: List of PolylineCurve GUIDs
    """
    stacked = []
    normalized_pattern = normalize_pattern(pattern)
    pattern_gen = cycle(normalized_pattern)
    reference_divisions = None

    if DEBUG:
        print(f"[generate_pattern_stack] curves={len(crvs)} target_dist={target_dist} "
              f"amp_range={amp_range} shift_range={shift_range} gradient_axis={gradient_axis} "
              f"gradient_mode={gradient_mode} gradient_blend={gradient_blend} "
              f"skip_points={skip_points} taper_points={taper_points}")

    for i, crv in enumerate(crvs):
        segments, is_closed, exploded = explode_to_segments(crv)
        tag = next(pattern_gen)

        if DEBUG:
            print(f"[layer {i}] crv={crv} closed={is_closed} segments={len(segments)} tag={tag}")

        combined, segment_divisions = collect_layer_div_data(
            segments, srf, img, target_dist, amp_range, shift_range,
            gradient_axis, gradient_mode, gradient_blend,
            skip_points, taper_points)

        if reference_divisions is None:
            reference_divisions = segment_divisions
        elif segment_divisions != reference_divisions:
            io.report_issue(
                f"[layer {i}] division count {segment_divisions} differs from layer 0's "
                f"{reference_divisions}; division counts should stay constant across the "
                "stack for consistent layer alignment.",
                level="warning")

        if is_closed and len(combined) > 1 and combined[0][0] == combined[-1][0]:
            combined = combined[:-1]  # drop the seam point duplicated by wrap-around

        if DEBUG:
            print(f"[layer {i}] combined_div_points={len(combined)}")

        poly1, poly2 = build_alternating_polylines(combined)
        layer_pts = poly2 if tag else poly1

        if exploded:
            rs.DeleteObjects(segments)

        if is_closed and layer_pts and layer_pts[0] != layer_pts[-1]:
            layer_pts.append(layer_pts[0])

        if DEBUG:
            print(f"[layer {i}] total_points={len(layer_pts)}")

        polyline = rs.AddPolyline(layer_pts) if layer_pts else None
        if polyline:
            stacked.append(polyline)

    return stacked


def sample_coloured_parameters_from_image(crvs, srf, img, target_dist, amp_range, shift_range,
                                           gradient_axis, gradient_mode, gradient_blend,
                                           skip_points=0, taper_points=0):
    """
    Extracts colour- and gradient-based amplitude/shift parameters from multiple curves for
    inspection or analysis. Each curve is exploded into segments before sampling, matching
    generate_pattern_stack, including the skip_points/taper_points amplitude tapering near
    segment ends (see compute_amp_factors).

    Parameters:
        crvs (list): List of curves to evaluate.
        srf (GUID): Surface with image mapping and gradient domain.
        img (Bitmap or None): Texture image. If None, amp/shift are driven entirely by the
            procedural gradient (equivalent to gradient_blend=1.0).
        target_dist (float): Fixed division spacing, shared by every curve/segment in the stack.
        amp_range, shift_range (tuple): RGB/gradient remap target ranges.
        gradient_axis (str): 'U' or 'V' surface direction driving the gradient.
        gradient_mode (str): Gradient shape - 'linear', 'reverse', 'peak', or 'valley'.
        gradient_blend (float): 0-1 weight between image-sampled and gradient-driven values.
        skip_points (int): Division points at each end of every segment kept fully straight.
        taper_points (int): Additional division points, beyond skip_points, over which amplitude
            ramps linearly back up to full.

    Returns:
        list: Tuples of (point, amp, shift).
    """
    results = []
    for curve in crvs:
        segments, _, exploded = explode_to_segments(curve)
        for segment in segments:
            div_data = get_div_pts_and_params(
                segment, srf, img, target_dist, amp_range, shift_range,
                gradient_axis, gradient_mode, gradient_blend)
            divs = len(div_data) - 1 if div_data else 0
            factors = compute_amp_factors(len(div_data), divs, skip_points, taper_points)
            results.extend(
                (pt, 0.0, 0.0) if is_straight else (pt, amp * factor, shift)
                for (pt, amp, shift), (is_straight, factor) in zip(div_data, factors))
        if exploded:
            rs.DeleteObjects(segments)

    return results


# Grasshopper-compatible example usage:
# Connect these as inputs in the Python component:
# Inputs: crvs (list of curves), srf (surface), img (bitmap, optional - leave unconnected to use
#         only the procedural gradient), pattern (str),
#         amp_range (tuple), shift_range (tuple)
# Output: a (list of polylines)

# Fixed division spacing for the whole stack - every curve/segment uses this same target distance,
# so layers end up with a consistent, predictable division count instead of one driven by image
# colour (which previously caused mismatched division counts and scrambled points between layers).
TARGET_DIST = 40

amp_range = (25, 15)
shift_range = (0, 0)

# Procedural gradient constants - sampled from srf's UV domain and expressed in the ranges above.
GRADIENT_AXIS = 'U'      # 'U' or 'V': surface direction driving the gradient
GRADIENT_MODE = 'linear'  # 'linear', 'reverse', 'peak', or 'valley'
GRADIENT_BLEND = 0.0     # 0.0 = purely image-driven, 1.0 = purely gradient-driven, in between blends both

# Division points at each end of every segment kept fully straight (no offset), and additional
# points beyond that over which amplitude ramps back up to full. Needed when input curves are
# small relative to amp_range, otherwise the zigzag overlaps itself near segment ends.
SKIP_POINTS = 1
AMP_TAPER_POINTS = 2 
a = generate_pattern_stack(crvs, srf, img, pattern,
                           TARGET_DIST, amp_range, shift_range,
                           GRADIENT_AXIS, GRADIENT_MODE, GRADIENT_BLEND,
                           SKIP_POINTS, AMP_TAPER_POINTS)
