#! python3

"""Grasshopper Script

Converts a list of planar polylines into a continuous spiral
    Inputs:
        crvs: list of polylines (their existing control points are used directly)
        delta: layer height (mm) used for the last curve in the stack, where it
            can't be derived from the next curve's plane; optional, defaults to
            10 mm with a warning if not supplied
        fade_out: if True, repeats the last curve once more at the spiral's
            final height (a level loop, no further rise) after the spiral ends,
            so the path closes back on the same point where the spiral ended.
            Defaults to False.
    Output:
        a: The a output variable
        d: point count of the fade-out lap (0 when fade_out is False) - add a
           matching output param on the component to view it
"""
__author__ = "jose hernandez vargas"
__version__ = "2024-08-05"

import System
import Rhino
import Grasshopper
import rhinoscriptsyntax as rs
import geometrylib as gl
import iolib as io

DEFAULT_DELTA = 10  # fallback layer height (mm) when delta is not supplied

spiralpts = []
dampedpts = []

if delta is None:
    io.report_issue(
        f"'delta' input not supplied; falling back to {DEFAULT_DELTA} mm.",
        level="warning", component=ghenv.Component)
    delta = DEFAULT_DELTA

try:
    if fade_out is None:
        fade_out = False
except NameError:
    fade_out = False

def spiralise(points , height, direction=(0, 0, 1)):
    """Progressively displaces a list of points along `direction`.

    The offset at each point is proportional to the cumulative segment
    length up to that point (not its index), so this works correctly on
    polyline control points that aren't evenly spaced. `direction` should
    be a unit vector, typically the input curve's own plane normal.
    """
    newpts = []
    seg_lengths = [rs.Distance(points[i], points[i + 1]) for i in range(len(points) - 1)]
    total_length = sum(seg_lengths)
    cumulative = 0.0
    for i, pt in enumerate(points):
        if i > 0:
            cumulative += seg_lengths[i - 1]
        t = (cumulative / total_length) if total_length else 0
        offset = t * height
        newpts.append((
            pt[0] + direction[0] * offset,
            pt[1] + direction[1] * offset,
            pt[2] + direction[2] * offset,
        ))
    return newpts

def getcrvplane(crv):
    if rs.IsCurvePlanar(crv):
        plane = rs.CurvePlane(crv)
    else:
        # HACK: get plane from curve start point
        stpt = rs.CurveStartPoint(crv)
        plane = rs.PlaneFromPoints(stpt, rs.CopyObject(stpt, (1,0,0)) , rs.CopyObject(stpt, (0,1,0)))
    # keep the normal pointing "up" so the spiral direction matches delta's sign
    # regardless of the curve's seam direction/winding
    if plane.Normal.Z < 0:
        plane = rs.RotatePlane(plane, 180, plane.XAxis)
    return plane

def getcrvheight(crv):
    plane = getcrvplane(crv)
    return plane[0][2]  # 0 plane origin - 2 coordinate Z



# arbitrary threshold for determining the periodicity of a curve
threshold = 10
# crvs = crvs[:-3]

for i, crv in enumerate(crvs):

    if i < len(crvs)-1:
        delta = getcrvheight(crvs[i+1]) - getcrvheight(crvs[i])
    

    damped = []
    t = max( 0 , (1 - i / 2)) # progressively increase spiral until second layer
    #dheight defines the height of the spiral
    # if i == 0:
    #     dheight = delta / 4
    # elif i==1:
    #     dheight = delta / 2
    # else:
    #     delta = dheight

    # use the polyline's existing control points, in their original 3D position,
    # instead of resampling, so the spiral follows the input geometry's own
    # segment spacing; displace along the curve's own plane normal rather than
    # assuming a flat, world-XY-aligned curve
    plane = getcrvplane(crv)
    normal = rs.VectorUnitize(plane.Normal)
    crvpts = rs.CurvePoints(crv)
    points = [(cpt[0], cpt[1], cpt[2]) for cpt in crvpts]
    spiral = spiralise( points , delta, normal)


    for i, pt in enumerate(points):
        newpt = gl.lerppts( spiral[i], points[i], t)
        damped.append(newpt)
    dampedpts.append(damped)
    spiralpts.append(spiral)

fade_out_points = []
if fade_out:
    # repeat the last curve once more as a level loop (no further rise) at the
    # spiral's final height; if that curve is closed, tracing it again brings
    # the path right back to the point where the spiral ended
    fade_out_points = [(p[0] + normal[0] * delta, p[1] + normal[1] * delta, p[2] + normal[2] * delta) for p in points]
    spiralpts.append(fade_out_points)

distance = rs.Distance(rs.CurveStartPoint(crvs[0]) , rs.CurveEndPoint(crvs[0]))

# if distance between start and endpoint is below a threshold (nozzle) spiralise the curves
# add _spiral to the name
#if distance < threshold:
#    a = rs.AddPolyline(flattenlist(spiralpts))
#    name = n + "_spiral"
#else:
#    a= crvs
#    name = n

a = gl.flattenlist(spiralpts)
b = rs.AddPolyline(a)
c = gl.flattenlist(dampedpts)
d = len(fade_out_points)  # debug: point count of the fade-out lap (0 when fade_out is False)