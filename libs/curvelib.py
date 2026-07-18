import rhinoscriptsyntax as rs
import geometrylib as gl
import iolib as io
import math


def tween2crv(crv0, crv1):
    midcrv = rs.MeanCurve(crv0, crv1)
    return(midcrv)


def midpt(pt0, pt1):
    """ Returns the midpoint from two given points"""
    x = (pt0.X + pt1.X) / 2
    y = (pt0.Y + pt1.Y) / 2
    z = (pt0.Z + pt1.Z) / 2
    return(rs.AddPoint(x, y, z))


def divcrvlen(crv, dist):
    len = rs.CurveLength(crv)
    divlen = round(len / dist)
    curve = rs.DivideCurve(crv, divlen)
    return(curve)


def interpolatept(pt1, pt2, factor):
    x = gl.lerp(pt1[0], pt2[0], factor)
    y = gl.lerp(pt1[1], pt2[1], factor)
    z = gl.lerp(pt1[2], pt2[2], factor)

    return([x, y, z])


def createcurve(nodes, isperiodic):
    deg = 3
    # nodes.pop()
    # nodes = nodes[1:-1]
    if isperiodic:
        # add three overlapping points for periodic curve
        nodes.append(nodes[0])
        nodes.append(nodes[1])
        nodes.append(nodes[2])
        knots = []
        pts = len(nodes)
        for i in range(pts+2):
            knots.append(i)
    else:
        knots = []
        pts = len(nodes)
        knots.append(0)  # adds two ctrlpts at the start
        knots.append(0)
        for i in range(pts-3):  # a third in i = 0
            knots.append(i)
        knots.append(pts-3)  # and three ctrlpts at the end
        knots.append(pts-3)
        knots.append(pts-3)

    curve = rs.AddNurbsCurve(nodes, knots, deg)
    return (curve)


def linesdl(start, dir, len):
    """
    Creates a line based on start point, direction and length
    Arguments
        start: start point
        dir: direction vector
        len: length
    Returns
        A rhino line with the specified start point, direction, and length
    """
    vector = rs.VectorScale(rs.VectorUnitize(dir), len)
    pt2 = rs.CopyObject(start, vector)
    return(rs.AddLine(start, pt2))


def vectorsdl(start, dir, length):
    """
    Creates a vector with a specific direction and length
    Arguments
        start: Start point of the vector
        dir: vector specifying the direction
        length: desired length of the vector
    Returns
        a vector with the specified direction and length
    """
    vector = rs.VectorUnitize(dir)
    vector = rs.VectorScale(vector, length)
    return (vector)


def centercrv(start, end, length):
    """
    Creates a line with a specific length in the direction vector specified by two points
    The line is always centered between the two points
    Arguments
        start: start point of the direction vector
        end: end point of the direction vector
        length: Length of the desired line
    Returns
        a line with the desired orientation and length
    """
    line = rs.AddLine(start, end)
    mid = rs.CurveMidPoint(line)
    vect = vectorsdl(mid, end, length/2)  # creates a vector to half the size
    newstart = rs.CopyObject(mid, -vect)  # first half
    newend = rs.CopyObject(mid, vect)  # second half
    result = rs.AddLine(newstart, newend)
    return(result)

def centercrv_dir(mid, dir, length):
    """
    Creates a line from a mid point, direction and length
    
    Arguments
        mid: mid point of the line
        dir: direction vector
        length: Length of the desired line
    Returns
        line
    """
    vect = vectorsdl(mid, dir, length/2)  # creates a vector to half the size
    startpt = rs.CopyObject(mid, -vect)  # first half
    endpt = rs.CopyObject(mid, vect)  # second half
    result = rs.AddLine(startpt, endpt)
    return(result)


def remap2dpointdomain(pt, idom, odom):
    """Remaps a point from one 2D domain to another
    Needs remap function
    """
    newptx = gl.remap(idom[0], idom[1], odom[0], odom[1], pt[0])
    newpty = gl.remap(idom[0], idom[1], odom[0], odom[1], pt[1])
    newptz = pt[2]
    return (newptx, newpty, newptz)


def bboxplanedomain(obj, plane="xy"):
    """Creates a bounding box and returns the domain for a certain plane"""
    bounds = gl.bbox_bounds(obj)
    if not bounds:
        print("unable to get object dimensions")
        return
    minx, miny, minz, maxx, maxy, maxz = bounds
    plane = plane.lower()
    if plane == "xy":
        domx = (minx, maxx)
        domy = (miny, maxy)
        return(domx, domy)
    elif plane == "yz":
        domy = (maxy, miny)
        domz = (minz, maxz)
        return(domy, domz)
    elif plane == "xz":
        domx = (minx, maxx)
        domz = (minz, maxz)
        return (domx, domz)
    else:
        print("Plane not recognised. Use xy, yz, or xz")
        return

def curveselfintersection(crv, *params):
    """returns a list of self intersection points for a single curve
    """
    if crv:
        intpoints = []
        intparams = []
        intersections = rs.CurveCurveIntersection(crv)
        if intersections == None:
            print("No self-intersections found")
            print(intersections)
        else:
            for i, intpt in enumerate(intersections):
                if intpt[0] == 1:
                    intpoints.append(intpt[1])
                    intparams.append(intpt[5])
                else:
                    print("curve overlap")
        if params:
            return (intparams)
        else:
            return (intpoints)

def curveselfintersection2(crv, params=False, direction=0):
    """returns a list of self intersection points for a single curve
    """
    if crv:
        intpoints = []
        intparams = []
        intersections = rs.CurveCurveIntersection(crv)
        if intersections == None:
            print("No self-intersections found")
            print(intersections)
        else:
            for i, intpt in enumerate(intersections):
                if intpt[0] == 1:
                    if direction == 0:
                        # intersection point and param on the first curve
                        intpoints.append(intpt[1])
                        intparams.append(intpt[5])
                    elif direction == 1 :
                        # intersection point and param on the second curve
                        intpoints.append(intpt[3])
                        intparams.append(intpt[7])
                    else:
                        print("direction parameter must be either 0 or 1 for first or second curve respecively")
                else:
                    print("curve overlap")
        if params:
            return(intparams)
        else:
            return(intpoints)

def sort_curves_z(crvs):
    """sorts a list of curves according to their z value
    """
    return sorted(crvs, key=lambda curve: rs.CurvePlane(curve)[0][2])

def spiralise(points , height):
    """transforms a list of points to a continously rising spiral with vertical (+z) displacement
    it requires even-spaced points
    """
    newpts = []
    for i, pt in enumerate(points):
        disp = (0,0,(i/len(points) * height))
        point = rs.PointCoordinates(rs.CopyObject(pt , disp))
        newpts.append(point)
    return newpts

def divide_crv_equal(crv, target_distance, create_points=False, return_points=True):
    """
    Divides a curve into equal-length segments based on a target segment length.

    This function calculates how many divisions are needed to approximate the 
    target segment length and divides the curve accordingly.

    Parameters:
        crv: Curve to divide (GUID).
        target_distance: Target length for each segment (float). Must be > 0.
        create_points: If True, division points are added to the document (bool).
        return_points: If True, return points instead of parameters (bool).

    Returns:
        List of points or parameters along the curve, depending on return_points.

    Raises:
        iolib.ValidationError: if crv is not a valid curve or target_distance
            is not positive.
    """
    if not rs.IsCurve(crv):
        raise io.ValidationError("crv is not a valid curve.")
    io.validate_scalar("target_distance", target_distance, min_value=0, allow_zero=False)

    curve_length = rs.CurveLength(crv)
    divs = max(1, int(round(curve_length / target_distance)))
    return rs.DivideCurve(crv, divs, create_points, return_points)

def orient_cross_sections(frames, cross_section):
    """
    Orient curves into the plane frames.
    
    Parameters:
    frames (list): A list of planes to orient the cross sections to.
    cross_section (GUID): The identifier of the cross section curve to be oriented.
    
    Returns:
    list: A list of oriented cross section curves.
    """
    oriented_sections = []
    for frame in frames:
        # Orient the cross section to the frame
        oriented_section = rs.CopyObject(cross_section)
        rs.OrientObject(oriented_section, rs.WorldXYPlane(), frame)
        oriented_sections.append(oriented_section)
    
    return oriented_sections

def rounded_rectangle(width, height, subdivisions, return_polyline=False):
    io.validate_scalar("subdivisions", subdivisions, min_value=0, allow_zero=True)

    subdivisions = int(subdivisions)

    # sharp rectangle for subdivisions = 0
    if subdivisions == 0:
        half_w = width / 2.0
        half_h = height / 2.0
        points = [
            ( half_w,  half_h, 0),
            (-half_w,  half_h, 0),
            (-half_w, -half_h, 0),
            ( half_w, -half_h, 0),
            ( half_w,  half_h, 0)  # close the polyline
        ]
        return rs.AddPolyline(points)

    radius = height / 2.0  # use half height as corner radius
    arc_divs = subdivisions

    half_w = width / 2.0 - radius
    half_h = height / 2.0 - radius

    # rectangle corner centers (arc centers)
    centers = [
        ( half_w,  half_h, 0),  # top right
        (-half_w,  half_h, 0),  # top left
        (-half_w, -half_h, 0),  # bottom left
        ( half_w, -half_h, 0)   # bottom right
    ]

    # arc angles per corner (in degrees)
    arc_bases = [
        (0, 90),    # top right
        (90, 180),  # top left
        (180, 270), # bottom left
        (270, 360)  # bottom right
    ]

    points = []

    for i in range(4):
        center = centers[i]
        angle_start, angle_end = arc_bases[i]
        for j in range(arc_divs + 1):
            t = j / arc_divs
            angle = math.radians((1 - t) * angle_start + t * angle_end)
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            points.append((x, y, 0))

    points.append(points[0])  # close the polyline
    polyline = rs.AddPolyline(points)
    return polyline

def offset_crv_both_sides(crv, offset_distance, style=1, cap_style=1):
    """Offsets a curve on both sides by a specified distance.
    
    Parameters:
        crv: The curve to offset (GUID).
        offset_distance: The distance to offset the curve (float).
        style: Offset style (int), default is 1 = sharp.
        0 = None
        1 = Sharp
        2 = Round
        3 = Smooth
        4 = Chamfer
        cap_style: Cap style for open curves (int), default is 0 = flat.
        0 = Flat
        1 = Round
        2 = Square
    Returns:
        A tuple containing the two offset curves (left, right) or None if offset fails.
    """
    left_offset = rs.OffsetCurve(crv, (0,0,1), offset_distance, style=style)
    right_offset = rs.OffsetCurve(crv, (0,0,1), -offset_distance, style=style)

    if rs.IsCurveClosed(crv):
        cap_style = 0  # no caps for closed curves

    if cap_style == 0:  # flat cap
        start_cap = rs.AddLine(rs.CurveStartPoint(left_offset), rs.CurveStartPoint(right_offset))
        end_cap= rs.AddLine(rs.CurveEndPoint(left_offset), rs.CurveEndPoint(right_offset))
    elif cap_style == 1:  # round cap
        start_tangent = rs.CurveTangent(crv, rs.CurveParameter(crv, 0))
        end_tangent = rs.CurveTangent(crv, rs.CurveParameter(crv, 1))
        cap_pt_start = rs.CopyObject(rs.CurveStartPoint(crv), -rs.VectorUnitize(start_tangent)*offset_distance)
        cap_pt_end = rs.CopyObject(rs.CurveEndPoint(crv), rs.VectorUnitize(end_tangent)*offset_distance)
        start_cap = rs.AddArc3Pt(rs.CurveStartPoint(left_offset), rs.CurveStartPoint(right_offset),cap_pt_start)
        end_cap = rs.AddArc3Pt(rs.CurveEndPoint(right_offset), rs.CurveEndPoint(left_offset),cap_pt_end)
    elif cap_style == 2:  # rectangular cap
        start_tangent = rs.CurveTangent(crv, rs.CurveParameter(crv, 0))
        end_tangent = rs.CurveTangent(crv, rs.CurveParameter(crv, 1))
        cap_line_start = rs.CopyObject(rs.AddLine(rs.CurveStartPoint(left_offset), rs.CurveStartPoint(right_offset)), -rs.VectorUnitize(start_tangent)*offset_distance)
        cap_line_end = rs.CopyObject(rs.AddLine(rs.CurveEndPoint(left_offset), rs.CurveEndPoint(right_offset)), rs.VectorUnitize(end_tangent)*offset_distance)
        start_cap = rs.AddPolyline([rs.CurveStartPoint(left_offset), rs.CurveStartPoint(cap_line_start), rs.CurveEndPoint(cap_line_start), rs.CurveStartPoint(right_offset)])
        end_cap = rs.AddPolyline([rs.CurveEndPoint(left_offset), rs.CurveStartPoint(cap_line_end), rs.CurveEndPoint(cap_line_end), rs.CurveEndPoint(right_offset)])
    
    if left_offset and right_offset:
        return (rs.JoinCurves([left_offset, end_cap, right_offset, start_cap]))
    else:
        print("Offset failed on one or both sides.")
        return None