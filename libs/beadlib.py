# libs/beadlib.py
"""Printed-filament (bead) footprint from toolpath centrelines.

The footprint of a bead of width w is the area swept by a disc of diameter w
moving along the toolpath. It is computed for the whole polyline at once
(GEOS buffer via shapely), never as left and right offset curves: one-sided
offsets self-intersect where the curvature radius is below w/2 or the path
crosses itself, and segment-by-segment sweeps leave wedges at every bend.

Rhino-free: depends only on shapely (>= 2.0) and iolib, so it can be imported
and tested in plain CPython like iolib. The Rhino side (curves in, Breps out)
lives in printlib.bead_footprint.

Pipeline of footprint():
    1. clean     drop duplicate points and zero-length segments
    2. corner    a registered corner function sweeps each path
    3. union     merge all beads into regions (or keep one per path)
    4. fuse      optional closing: fuse beads closer than fuse_gap
    5. finalize  make valid, snap to a precision grid

Corner types (CORNERS registry):
    round   exact disc sweep; outer corners are arcs of radius w/2 (default)
    mitre   sharp outer corners, cut where the spike exceeds corner_mitre * w/2
    bevel   outer corners cut straight across
    blend   centreline filleted with corner_blend first (robot zone), then round
    bulge   wider bead where the path turns: w * (1 + corner_bulge * turn / pi),
            turn summed over about one bead width of arc length

Adding a corner type:
    @register_corner("my_corner")
    def _my_corner(pts, closed, width, opt):
        return sweep_constant(pts, closed, width, "mitre", "square", 10.0, opt.tolerance)

    It receives the cleaned 2D points of one path, the closed flag, the bead
    width and the BeadOptions, and returns a shapely geometry. Extra parameters
    go in BeadOptions.extra (a dict).
"""
import math

import iolib as io

try:
    import shapely
    from shapely.geometry import LinearRing, LineString, MultiPoint, Point, Polygon
    from shapely.geometry.polygon import orient
    from shapely.ops import unary_union
except ImportError as _exc:  # pragma: no cover - depends on the Rhino install
    raise ImportError(
        "beadlib needs shapely >= 2.0. In a Rhino 8 Script component add the line "
        "'# r: shapely>=2.0' to the script; in plain Python run 'pip install shapely'."
    ) from _exc


CAPS = ("round", "flat", "square")


# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------

class BeadOptions:
    """Settings shared by all corner types. Lengths in model units (mm).

    bead_width           bead width, used when a path has no width of its own
    corner_style         key in CORNERS: round | mitre | bevel | blend | bulge | custom
    end_cap              end of open paths: round | flat | square
    corner_mitre         'mitre': max spike length as a multiple of bead_width/2
    corner_blend         'blend': fillet radius applied to the centreline
    corner_bulge         'bulge': extra width at a 180 deg turn, as a fraction of bead_width
    corner_bulge_window  'bulge': arc length over which turning is summed (0 = bead_width)
    fuse_gap             close gaps narrower than this (0 = off)
    union                True: merged regions, False: one footprint per path
    tolerance            max deviation of polygonised arcs
    min_seg              shorter segments are dropped while cleaning
    grid                 precision grid of the output coordinates
    extra                dict with parameters for custom corner types
    """

    def __init__(self, bead_width=40.0, corner_style="round", end_cap="round", corner_mitre=2.0,
                 corner_blend=20.0, corner_bulge=0.25, corner_bulge_window=0.0, fuse_gap=0.0,
                 union=True, tolerance=0.1, min_seg=1e-3, grid=1e-3, extra=None):
        self.bead_width = io.validate_scalar("bead_width", bead_width, min_value=0, allow_zero=False)
        self.corner_style = str(corner_style).strip().lower()
        self.end_cap = str(end_cap).strip().lower()
        self.corner_mitre = io.validate_scalar("corner_mitre", corner_mitre, min_value=1.0)
        self.corner_blend = io.validate_scalar("corner_blend", corner_blend, min_value=0)
        self.corner_bulge = io.validate_scalar("corner_bulge", corner_bulge, min_value=0)
        self.corner_bulge_window = io.validate_scalar("corner_bulge_window", corner_bulge_window, min_value=0)
        self.fuse_gap = io.validate_scalar("fuse_gap", fuse_gap, min_value=0)
        self.union = bool(union)
        self.tolerance = io.validate_scalar("tolerance", tolerance, min_value=0, allow_zero=False)
        self.min_seg = io.validate_scalar("min_seg", min_seg, min_value=0, allow_zero=False)
        self.grid = io.validate_scalar("grid", grid, min_value=0, allow_zero=False)
        self.extra = dict(extra or {})
        if self.end_cap not in CAPS:
            raise io.ValidationError("end_cap must be one of: {}.".format(", ".join(CAPS)))

    def __repr__(self):
        return "BeadOptions(bead_width={}, corner_style={!r}, end_cap={!r})".format(
            self.bead_width, self.corner_style, self.end_cap)


# ---------------------------------------------------------------------------
# Corner registry
# ---------------------------------------------------------------------------

CORNERS = {}


def register_corner(name):
    """Decorator: make a corner function available under `name`."""
    def deco(fn):
        CORNERS[str(name).strip().lower()] = fn
        return fn
    return deco


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------

def clean_path(pts, closed=False, min_seg=1e-3):
    """2D copy of the points without duplicates / near-zero segments."""
    out = []
    for p in pts:
        q = (float(p[0]), float(p[1]))
        if not out or math.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) > min_seg:
            out.append(q)
    if closed and len(out) > 2 and math.hypot(out[0][0] - out[-1][0],
                                              out[0][1] - out[-1][1]) <= min_seg:
        out.pop()
    return out


def quad_segs(radius, chord_tol):
    """Segments per quarter circle so a polygonised arc deviates < chord_tol."""
    if radius <= chord_tol:
        return 1
    step = 2.0 * math.acos(1.0 - chord_tol / radius)
    return max(1, int(math.ceil((math.pi / 2.0) / step)))


def turn_angles(pts, closed):
    """Turning angle at each vertex in radians (0 = straight, pi = reversal)."""
    n = len(pts)
    out = [0.0] * n
    for i in range(n):
        if not closed and (i == 0 or i == n - 1):
            continue
        a, b, c = pts[i - 1], pts[i], pts[(i + 1) % n]
        u = (b[0] - a[0], b[1] - a[1])
        v = (c[0] - b[0], c[1] - b[1])
        lu, lv = math.hypot(*u), math.hypot(*v)
        if lu == 0 or lv == 0:
            continue
        cosang = max(-1.0, min(1.0, (u[0] * v[0] + u[1] * v[1]) / (lu * lv)))
        out[i] = math.acos(cosang)
    return out


def densify(pts, closed, max_len):
    """Insert points so no segment is longer than max_len (original vertices kept)."""
    p = list(pts) + ([pts[0]] if closed else [])
    out = []
    for a, b in zip(p[:-1], p[1:]):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / max_len)))
        for k in range(n):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    if not closed:
        out.append(p[-1])
    return out


def windowed_turn(pts, closed, window):
    """Turning angle summed over +-window/2 of arc length around each vertex.
    A sharp corner and a tight arc of the same total turn give the same value,
    independent of how finely the path is sampled."""
    n = len(pts)
    turns = turn_angles(pts, closed)
    s = [0.0]
    for i in range(1, n):
        s.append(s[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    total = s[-1] + (math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) if closed else 0.0)
    half = window / 2.0
    out = []
    for i in range(n):
        acc = 0.0
        for j in range(n):
            d = abs(s[j] - s[i])
            if closed:
                d = min(d, total - d)
            if d <= half:
                acc += turns[j]
        out.append(min(acc, math.pi))
    return out


def fillet_polyline(pts, closed, radius, chord_tol=0.1):
    """Replace each corner by a tangent arc of `radius`, clamped so neighbouring
    fillets never overlap. Models robot corner blending (zone data)."""
    n = len(pts)
    if n < 3 or radius <= 0:
        return list(pts)
    out = []
    for i in range(n):
        b = pts[i]
        if not closed and (i == 0 or i == n - 1):
            out.append(b)
            continue
        a, c = pts[i - 1], pts[(i + 1) % n]
        ua = (a[0] - b[0], a[1] - b[1])
        uc = (c[0] - b[0], c[1] - b[1])
        la, lc = math.hypot(*ua), math.hypot(*uc)
        ua = (ua[0] / la, ua[1] / la)
        uc = (uc[0] / lc, uc[1] / lc)
        theta = math.acos(max(-1.0, min(1.0, ua[0] * uc[0] + ua[1] * uc[1])))  # interior angle
        if theta > math.pi - 1e-6 or theta < 1e-6:
            out.append(b)  # straight or full reversal
            continue
        d = radius / math.tan(theta / 2.0)  # tangent length
        r = radius
        d_max = 0.5 * min(la, lc)
        if d > d_max:
            d = d_max
            r = d * math.tan(theta / 2.0)
        t1 = (b[0] + ua[0] * d, b[1] + ua[1] * d)
        t2 = (b[0] + uc[0] * d, b[1] + uc[1] * d)
        bis = (ua[0] + uc[0], ua[1] + uc[1])
        lb = math.hypot(*bis)
        h = r / math.sin(theta / 2.0)
        ctr = (b[0] + bis[0] / lb * h, b[1] + bis[1] / lb * h)
        a1 = math.atan2(t1[1] - ctr[1], t1[0] - ctr[0])
        a2 = math.atan2(t2[1] - ctr[1], t2[0] - ctr[0])
        sweep = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi  # short arc
        step = 2.0 * math.acos(max(-1.0, 1.0 - chord_tol / r)) if r > chord_tol else abs(sweep)
        k = max(1, int(math.ceil(abs(sweep) / step)))
        for j in range(k + 1):
            ang = a1 + sweep * j / k
            out.append((ctr[0] + r * math.cos(ang), ctr[1] + r * math.sin(ang)))
    return clean_path(out, closed)


# ---------------------------------------------------------------------------
# Sweeps
# ---------------------------------------------------------------------------

def sweep_constant(pts, closed, width, join="round", cap="round", mitre_limit=2.0, chord_tol=0.1):
    """Disc sweep of constant width (GEOS buffer of the whole polyline)."""
    if cap not in CAPS:
        raise io.ValidationError("cap must be one of: {}.".format(", ".join(CAPS)))
    r = width / 2.0
    if len(pts) == 1:
        geom = Point(pts[0])
    elif closed and len(pts) > 2:
        geom = LinearRing(pts)
    else:
        geom = LineString(pts)
    return geom.buffer(r, quad_segs=quad_segs(r, chord_tol), cap_style=cap,
                       join_style=join, mitre_limit=mitre_limit)


def sweep_variable(pts, closed, widths, chord_tol=0.1):
    """Disc sweep with one width per vertex: union of the convex hulls of
    consecutive discs (tapered capsules). Ends are always round."""
    if len(widths) != len(pts):
        raise io.ValidationError("sweep_variable: {} widths for {} points.".format(len(widths), len(pts)))
    p = list(pts) + ([pts[0]] if closed else [])
    w = list(widths) + ([widths[0]] if closed else [])
    discs = [Point(q).buffer(wi / 2.0, quad_segs=quad_segs(wi / 2.0, chord_tol))
             for q, wi in zip(p, w)]
    if len(discs) == 1:
        return discs[0]
    hulls = [MultiPoint(list(a.exterior.coords) + list(b.exterior.coords)).convex_hull
             for a, b in zip(discs[:-1], discs[1:])]
    return shapely.union_all(hulls)


# ---------------------------------------------------------------------------
# Corner types
# ---------------------------------------------------------------------------

@register_corner("round")
def _round(pts, closed, width, opt):
    """Exact disc sweep: outer corners are arcs of radius w/2. Physical default."""
    return sweep_constant(pts, closed, width, "round", opt.end_cap, chord_tol=opt.tolerance)


@register_corner("mitre")
def _mitre(pts, closed, width, opt):
    """Sharp outer corners, cut off where the spike exceeds corner_mitre * w/2."""
    return sweep_constant(pts, closed, width, "mitre", opt.end_cap, opt.corner_mitre, opt.tolerance)


@register_corner("bevel")
def _bevel(pts, closed, width, opt):
    """Outer corners cut straight across."""
    return sweep_constant(pts, closed, width, "bevel", opt.end_cap, chord_tol=opt.tolerance)


@register_corner("blend")
def _blend(pts, closed, width, opt):
    """Centreline filleted with corner_blend (robot zone), then swept round.
    Inner corners become rounded too, not only the outer ones."""
    p = fillet_polyline(pts, closed, opt.corner_blend, opt.tolerance)
    return sweep_constant(p, closed, width, "round", opt.end_cap, chord_tol=opt.tolerance)


@register_corner("bulge")
def _bulge(pts, closed, width, opt):
    """Wider bead where the path turns (deceleration at constant flow).
    Width = w * (1 + corner_bulge * turn / pi), with turn summed within one
    corner_bulge_window (default w) of arc length. Ends are always round."""
    window = opt.corner_bulge_window or width
    p = densify(pts, closed, window / 4.0)
    ws = [width * (1.0 + opt.corner_bulge * t / math.pi) for t in windowed_turn(p, closed, window)]
    return sweep_variable(p, closed, ws, opt.tolerance)


# ---------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------

def polygons(geom):
    """Explode any geometry into its non-empty polygons."""
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    return [g for g in getattr(geom, "geoms", []) if isinstance(g, Polygon) and not g.is_empty]


def close_gaps(geom, gap, chord_tol=0.1):
    """Morphological closing: fuses beads closer than `gap` and fills notches
    narrower than `gap`. Keep gap well below the bead width."""
    r = gap / 2.0
    qs = quad_segs(r, chord_tol)
    return geom.buffer(r, quad_segs=qs, join_style="mitre", mitre_limit=10) \
               .buffer(-r, quad_segs=qs, join_style="mitre", mitre_limit=10)


def _finalize(geom, grid):
    if not geom.is_valid:
        geom = shapely.make_valid(geom)
    return shapely.set_precision(geom, grid)


def footprint(paths, opt=None):
    """Footprint of the printed filament.

    paths  sequence of (points, closed) or (points, closed, width); points are
           (x, y) or (x, y, z) in the plane of the layer, z is ignored.
    opt    BeadOptions (defaults if None).

    Returns a list of branches, each a list of shapely Polygons (holes
    included): one branch with all merged regions if opt.union, else one
    branch per path.
    """
    opt = opt or BeadOptions()
    if opt.corner_style not in CORNERS:
        raise io.ValidationError('Unknown corner type "{}". Available: {}.'.format(
            opt.corner_style, ", ".join(sorted(CORNERS))))
    fn = CORNERS[opt.corner_style]
    beads = []
    for item in io.require_list("paths", list(paths)):
        pts, closed = item[0], bool(item[1])
        width = item[2] if len(item) > 2 and item[2] else opt.bead_width
        width = io.validate_scalar("path width", width, min_value=0, allow_zero=False)
        p = clean_path(pts, closed, opt.min_seg)
        beads.append(fn(p, closed and len(p) > 2, width, opt) if p else None)

    def post(geom):
        if opt.fuse_gap > 0:
            geom = close_gaps(geom, opt.fuse_gap, opt.tolerance)
        return polygons(_finalize(geom, opt.grid))

    if opt.union:
        return [post(unary_union([b for b in beads if b is not None]))]
    return [post(b) if b is not None else [] for b in beads]


def islands(polys, min_area=0.0):
    """Regions enclosed by the bead (the holes of the footprint), as Polygons.

    A bead lying inside a hole is cut out of that island, so an island can have
    holes of its own. Islands smaller than min_area are dropped.
    """
    holes = [Polygon(h) for p in polys for h in p.interiors]
    if not holes:
        return []
    enclosed = unary_union(holes).difference(unary_union(list(polys)))
    return [g for g in polygons(enclosed) if g.area > min_area]


def rings(poly):
    """(outer, holes) as open coordinate lists: outer CCW, holes CW.
    Use this to rebuild the polygon in another geometry kernel (Rhino)."""
    poly = orient(poly, 1.0)
    outer = list(poly.exterior.coords)[:-1]
    holes = [list(h.coords)[:-1] for h in poly.interiors]
    return outer, holes


# ---------------------------------------------------------------------------
# Process relations
# ---------------------------------------------------------------------------

def width_from_flow(flow_mm3_s, speed_mm_s, layer_height, profile="rounded"):
    """Bead width from volumetric flow, travel speed and layer height.

    rect:    A = w * h
    rounded: A = (w - h) * h + pi * h^2 / 4   (flat top/bottom, round sides)
    """
    flow = io.validate_scalar("flow_mm3_s", flow_mm3_s, min_value=0, allow_zero=False)
    speed = io.validate_scalar("speed_mm_s", speed_mm_s, min_value=0, allow_zero=False)
    h = io.validate_scalar("layer_height", layer_height, min_value=0, allow_zero=False)
    area = flow / speed
    if profile == "rect":
        return area / h
    return area / h + h * (1 - math.pi / 4)
