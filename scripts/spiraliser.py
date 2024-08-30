#! python3

"""Grasshopper Script

Converts a list of planar curves into a continuous spiral
    Inputs:
        crvs: list of crvs
        delta: distance for curve division
    Output:
        a: The a output variable
"""
__author__ = "jose hernandez vargas"
__version__ = "2024-08-05"

import System
import Rhino
import Grasshopper
import rhinoscriptsyntax as rs
import curvelib as cl
import geometrylib as gl

spiralpts = []
dampedpts = []

def spiralise( points , height):
    newpts = []
    for i, pt in enumerate(points):
        disp = (0,0,(i/len(points) * height))
        point = rs.PointCoordinates(rs.CopyObject(pt , disp))
        newpts.append(point)
    return newpts

def getcrvheight(crv):
    if rs.IsCurvePlanar(crv):
        plane = rs.CurvePlane(crv)
        height = plane[0][2]  # 0 plane origin - 2 coordinate Z
        return height
    else:
        # HACK: get plane from curve start point
        stpt = rs.CurveStartPoint(crv)
        # ptx = rs.CopyObject()
        plane = rs.PlaneFromPoints(stpt, rs.CopyObject(stpt, (1,0,0)) , rs.CopyObject(stpt, (0,1,0)))
        height = plane[0][2]
        # print "Error! the curve is not planar"



# arbitrary threshold for determining the periodicity of a curve
threshold = 50
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
    # HACK: rebuild planar curves 
    crvpts = rs.CurvePoints(crv)
    for i, cpt in enumerate(crvpts):
        crvpts[i] = (cpt[0], cpt[1], 0)
    newcrv = rs.AddPolyline(crvpts)

    # points = rs.DivideCurveLength(crv , div_length)
    points = cl.divide_crv_equal(crv, div_length)
    spiral = spiralise( points , delta)


    for i, pt in enumerate(points):
        newpt = gl.lerppts( spiral[i], points[i], t)
        damped.append(newpt)
    dampedpts.append(damped)
    spiralpts.append(spiral)

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