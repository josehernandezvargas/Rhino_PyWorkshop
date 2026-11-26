import rhinoscriptsyntax as rs
from curvelib import centercrv, curveselfintersection
import geometrylib as gl
import gcodelib as gcl
import iolib as io


def selfclosestpt2(pts, i, diam):
    """
    calculates the distance between a point in a list and its closest non-consecutive point
    i.e. the maximum printing width to avoid self collision
    Inputs:
        pts:list of points (presliced)
        i: index of the point in the list to evaluate
        diam: max distance to ignore expressed in number of points ( = mm when curve is sliced at regular 1 mm distance )
    Output:
        distance, or None when every point lies inside the +-diam index window
    """

    low = max(i-diam, 0)  # some range before and after the point
    list1 = pts[0:low]  # points more than `diam` indices before i
    list2 = pts[i+diam+1:]  # points more than `diam` indices after i
    newlist = list1+list2  # new list some points contiguous to the point
    if not newlist:
        return None  # every point is within the ignored window
    pt = pts[i]
    closestpt = newlist[rs.PointArrayClosestPoint(newlist, pt)]
    # returns a single distance
    return (rs.Distance(pts[i], closestpt))


def testline(object, length, distance):
    # gets the lower (min y) side on the x axis
    end = rs.BoundingBox(object)[1]
    start = rs.BoundingBox(object)[0]
    offvect = (0, -distance, 0)
    line = centercrv(end, start, length)
    line = rs.MoveObject(line, offvect)
    return (line)


def leveltoplatform(input, height=0):
    """
    Moves the input geometry to be in touch with the platform

    Arguments

    input: item or list
        input geometry
    height: number
        custom height of the platform (default = 0)
    Returns
    item or list
        Geometry at the specified height
    """

    bounds = gl.bbox_bounds(input)
    if not bounds:
        raise io.ValidationError("leveltoplatform: could not compute a bounding box for the input.")
    minx, miny, minz, maxx, maxy, maxz = bounds
    disp = height - minz
    vector = rs.CreateVector(0, 0, disp)
    if type(input) is list:
        output = []
        for i, crv in enumerate(input):
            output.append(rs.MoveObject(crv, vector))
    else:
        output = rs.MoveObject(input, vector)
    return (output)


def centerobject(input, buildplate=(223, 223), delta=False):
    """
    Centres the input geometry on the buildplate. Height is unchanged.

    Arguments

    input: item or list
        input geometry
    buidplate: (number,number)
        custom dimensions of the platform. Default = Ultimaker 2+ (223 x 223)
    Returns
    item or list
        Geometry centred on the platform
    """

    bounds = gl.bbox_bounds(input)
    if not bounds:
        raise io.ValidationError("centerobject: could not compute a bounding box for the input.")
    minx, miny, minz, maxx, maxy, maxz = bounds
    centrex = (minx + maxx) / 2  # part midpoint in x and y
    centrey = (miny + maxy) / 2
    if delta:
        dispx = -centrex
        dispy = -centrey
    else:
        dispx = buildplate[0]/2 - centrex
        dispy = buildplate[1]/2 - centrey
    dispvector = rs.CreateVector(dispx, dispy, 0)
    if type(input) is list:
        output = []
        for i, crv in enumerate(input):
            output.append(rs.CopyObject(crv, dispvector))
    else:
        output = rs.CopyObject(input, dispvector)
    return output


def gcodeline(g, pt=None, x=None, y=None, z=None, f=None, e=None, v=None):
    """
    Creates a line of gcode from a variable set of keyword arguments.

    Thin re-export of gcodelib.gcodeline, kept here for scripts that still
    call pl.gcodeline(...).
    """
    return gcl.gcodeline(g, pt=pt, x=x, y=y, z=z, f=f, e=e, v=v)


def materialestimation(length, nozzle, unit=0):
    """
    Estimates the amount of material to be used, relevant for 3DCP
    Parameters
    ----------
    length: float
        length of the print path(s) in mm
    nozzle: float
        diameter of the printing nozzle
    returns: float
        volume in litres
    """
    from math import pi
    r = nozzle/2
    area = pi * r**2  # in mm2
    vol = area * length  # in mm3
    l = vol/1000000  # vol in litres
    return l


def slice_brep_uniform(brep, layer_height, max_deviation=0.1, max_slices=1000):
    """
    Uniformly slice a Brep with horizontal planes, returning polylines approximating the intersection curves.

    Parameters:
        brep (GUID): Rhino Brep object to slice.
        layer_height (float): Distance between slicing planes.
        max_deviation (float): Maximum deviation when converting curves to polylines.
        max_slices (int): Maximum number of allowable slices to prevent overload.

    Returns:
        list of PolylineCurve: Each element is a Rhino polyline curve.
    """
    if not rs.IsBrep(brep):
        raise ValueError("Input is not a valid Brep")

    io.validate_scalar("Layer height", layer_height, min_value=0, allow_zero=False)

    bbox = rs.BoundingBox(brep)
    if not bbox:
        raise ValueError("Failed to compute bounding box")

    z_min = bbox[0].Z
    z_max = bbox[4].Z
    height = z_max - z_min
    num_slices = int(height / layer_height)

    if num_slices > max_slices:
        raise ValueError(f"Too many slices ({num_slices}). Reduce the number by increasing the layer height.")

    start_pt = [0, 0, z_min]
    end_pt = [0, 0, z_max]

    contours = rs.AddSrfContourCrvs(brep, [start_pt, end_pt], layer_height)
    if not contours:
        return []

    slices = []
    for crv in contours:
        poly = rs.ConvertCurveToPolyline(crv, angle_tolerance=1.0, tolerance=max_deviation)
        if poly:
            slices.append(poly)
        rs.DeleteObject(crv)

    return slices


def stack_curves_by_pattern(crvA, crvB, pattern="AB", layer_height=1.0, total_height=10.0):
    """
    Creates a vertical stack of curve copies based on a repeating pattern until a given height is reached.

    Parameters:
        crvA (GUID): First base curve.
        crvB (GUID): Second base curve.
        pattern (str): A string pattern defining the sequence (e.g. "AABB", "AB").
        layer_height (float): The vertical distance between each layer.
        total_height (float): The total height to reach.

    Returns:
        list: A list of curve GUIDs forming the stacked pattern.
    """

    if not crvA or not crvB:
        raise ValueError("Both crvA and crvB must be valid curve objects.")
    io.validate_scalar("layer_height", layer_height, min_value=0, allow_zero=False)

    curves = []
    z = 0.0
    pattern_index = 0
    pattern = pattern.upper()
    if not pattern or any(key not in "AB" for key in pattern):
        # A key that never matches would otherwise loop forever (z never advances).
        raise ValueError("pattern must be a non-empty string made of 'A' and 'B' only.")

    base_curves = {'A': crvA, 'B': crvB}

    while z < total_height:
        key = pattern[pattern_index % len(pattern)]
        base_crv = base_curves.get(key)

        if base_crv:
            move_vec = rs.VectorCreate((0, 0, z), (0, 0, 0))
            new_crv = rs.CopyObject(base_crv, move_vec)
            curves.append(new_crv)
            z += layer_height

        pattern_index += 1

    return curves

# ---------------------------------------------------------------------------
# Bead footprint (Rhino adapter for beadlib)
# ---------------------------------------------------------------------------

def bead_footprint(curves, bead_width=None, corner_style="round", end_cap="round",
                   corner_blend=20.0, corner_bulge=0.25, corner_mitre=2.0, fuse_gap=0.0,
                   union=True, tolerance=0.1, plane=None, machine=None, with_islands=False,
                   component=None):
    """
    Planar surfaces of the printed filament for the toolpath curves of one layer.

    The geometry is computed by beadlib (disc sweep of the whole polyline, so
    self-intersecting offsets and corner artefacts cannot occur); this function
    converts Rhino curves to 2D points in the layer plane and the resulting
    regions back to planar Breps.

    Parameters
    ----------
    curves: list
        toolpath centrelines (GUIDs or Rhino.Geometry.Curve), all in one plane.
        Polylines keep their exact vertices; other curves are discretised
        within `tolerance`.
    bead_width: float or list, optional
        one value, or one per curve. When None, the profile's "bead_width"
        is used (see `machine`).
    corner_style: str
        beadlib corner type: round | mitre | bevel | blend | bulge, or any
        type registered with beadlib.register_corner.
    end_cap: str
        end of open curves: round | flat | square.
    corner_blend: float
        fillet radius of the centreline for corner_style "blend".
    corner_bulge: float
        extra width at a 180 deg turn, as a fraction of bead_width ("bulge").
    corner_mitre: float
        mitre limit as a multiple of bead_width/2 ("mitre").
    fuse_gap: float
        beads closer than this are fused into one region (0 = off).
    union: bool
        True: merged regions in one branch. False: one branch per curve.
    tolerance: float
        curve discretisation and arc accuracy.
    plane: Rhino.Geometry.Plane, optional
        layer plane. Default: world XY at the height of the first curve.
    machine: str, optional
        machine id or .json path (gcodelib.load_machine_properties), used
        only to read "bead_width" when `bead_width` is None.
    with_islands: bool
        also build the islands: planar surfaces of the regions enclosed by
        the bead (the holes of the footprint), cut where another bead lies
        inside them.
    component: optional
        ghenv.Component, for warnings via iolib.report_issue.

    Returns
    -------
    tuple (surfaces, outlines, areas, islands, options)
        surfaces, outlines, areas and islands are lists of branches (lists):
        planar Breps, closed boundary curves (outer and holes), region areas
        and island Breps (empty branches unless with_islands). options is the
        beadlib.BeadOptions that was used.

    Requires shapely >= 2.0 in Rhino's Python (line '# r: shapely>=2.0' in the
    calling script). beadlib is imported here, not at module level, so other
    printlib functions keep working where shapely is not installed.
    """
    import Rhino
    import Rhino.Geometry as rg
    import System
    import beadlib as bl

    curves = [rs.coercecurve(c, -1, True) for c in io.require_nonempty_list("curves", curves) if c is not None]
    if not curves:
        raise io.ValidationError("bead_footprint: no valid curves.")

    # bead width: explicit value(s), else the machine profile
    if bead_width is None or (isinstance(bead_width, (list, tuple)) and len(bead_width) == 0):
        if not machine:
            raise io.ValidationError("bead_footprint: give a bead_width or a machine profile with 'bead_width'.")
        profile = gcl.load_machine_properties(machine)
        if "bead_width" not in profile:
            raise io.ValidationError("bead_footprint: machine profile '{}' has no 'bead_width'.".format(machine))
        bead_width = profile["bead_width"]
    widths = list(bead_width) if isinstance(bead_width, (list, tuple)) else [bead_width]
    widths = [io.validate_scalar("bead_width", w, min_value=0, allow_zero=False) for w in widths]
    if len(widths) not in (1, len(curves)):
        raise io.ValidationError("bead_footprint: bead_width needs one value, or one per curve "
                                 "({} curves, {} widths).".format(len(curves), len(widths)))

    opt = bl.BeadOptions(bead_width=widths[0], corner_style=corner_style, end_cap=end_cap,
                         corner_mitre=corner_mitre, corner_blend=corner_blend,
                         corner_bulge=corner_bulge, fuse_gap=fuse_gap, union=union,
                         tolerance=tolerance)

    # layer plane and transforms between world and plane coordinates
    if plane is None or not plane.IsValid:
        plane = rg.Plane(rg.Point3d(0, 0, curves[0].PointAtStart.Z), rg.Vector3d.ZAxis)
    to_local = rg.Transform.PlaneToPlane(plane, rg.Plane.WorldXY)
    to_world = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, plane)
    doc = Rhino.RhinoDoc.ActiveDoc
    doc_tol = doc.ModelAbsoluteTolerance if doc else 0.001

    paths, off_plane = [], 0.0
    for i, crv in enumerate(curves):
        local = crv.DuplicateCurve()
        local.Transform(to_local)
        ok, pl = local.TryGetPolyline()
        if not ok:
            ok, pl = local.ToPolyline(tolerance, 0.1, 0.0, 0.0).TryGetPolyline()
            if not ok:
                raise io.ValidationError("bead_footprint: could not discretise curve {}.".format(i))
        off_plane = max([off_plane] + [abs(p.Z) for p in pl])
        w = widths[i] if len(widths) > 1 else widths[0]
        paths.append(([(p.X, p.Y) for p in pl], local.IsClosed, w))
    if off_plane > 10 * tolerance:
        io.report_issue("bead_footprint: curves are up to {:.3g} off the layer plane and are "
                        "projected onto it.".format(off_plane), "warning", component)

    def ring_curve(coords):
        pts = System.Collections.Generic.List[rg.Point3d]()
        for x, y in coords:
            pts.Add(rg.Point3d(x, y, 0.0))
        pts.Add(pts[0])
        ring = rg.PolylineCurve(pts)
        ring.Transform(to_world)
        return ring

    def planar_breps(poly, what):
        """One shapely polygon (with holes) -> (planar Breps, boundary curves)."""
        outer, holes = bl.rings(poly)
        loops = [ring_curve(outer)] + [ring_curve(h) for h in holes]
        loop_list = System.Collections.Generic.List[rg.Curve]()
        for lp in loops:
            loop_list.Add(lp)
        breps = rg.Brep.CreatePlanarBreps(loop_list, doc_tol)
        if not breps:
            io.report_issue("bead_footprint: a {} could not be turned into a planar surface."
                            .format(what), "warning", component)
        return (list(breps) if breps else []), loops

    surfaces, outlines, areas, islands = [], [], [], []
    for polys in bl.footprint(paths, opt):
        s_branch, o_branch, a_branch, i_branch = [], [], [], []
        for poly in polys:
            breps, loops = planar_breps(poly, "region")
            s_branch.extend(breps)
            o_branch.extend(loops)
            a_branch.append(poly.area)
        if with_islands:
            for island in bl.islands(polys):
                i_branch.extend(planar_breps(island, "island")[0])
        surfaces.append(s_branch)
        outlines.append(o_branch)
        areas.append(a_branch)
        islands.append(i_branch)
    return surfaces, outlines, areas, islands, opt
