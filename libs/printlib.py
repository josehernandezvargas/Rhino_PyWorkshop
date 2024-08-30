import rhinoscriptsyntax as rs
from curvelib import centercrv
import math


def selfclosestpt2(pts, i, diam):
    """
    calculates the distance between a point in a list and its closest non-consecutive point
    i.e. the maximum printing width to avoid self collision
    Inputs:
        pts:list of points (presliced)
        i: index of the point in the list to evaluate
        diam: max distance to ignore expressed in number of points ( = mm when curve is sliced at regular 1 mm distance )
    Output:
        distance
    """

    dist = []
    low = max(i-diam, 0)  # some range before and after the point
    high = min(i+diam, len(pts)-1)
    list1 = pts[0:low]  # first slice before the point
    list2 = pts[high:-1]  # second slice after
    newlist = list1+list2  # new list some points contiguous to the point
    pt = pts[i]
    closestpt = newlist[rs.PointArrayClosestPoint(newlist, pt)]
    # returns a single distance
    return(rs.Distance(pts[i], closestpt))


def testline(object, length, distance):
    # gets the lower (min y) side on the x axis
    end = rs.BoundingBox(object)[1]
    start = rs.BoundingBox(object)[0]
    offvect = (0, -distance, 0)
    line = centercrv(end, start, length)
    line = rs.MoveObject(line, offvect)
    return(line)


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
            return(intparams)
        else:
            return(intpoints)


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

    bbox = rs.BoundingBox(input)
    minx = bbox[0][0]
    miny = bbox[0][1]
    minz = bbox[0][2]
    maxx = bbox[6][0]
    maxy = bbox[6][1]
    maxz = bbox[6][2]
    disp = height - minz
    vector = rs.CreateVector(0, 0, disp)
    if type(input) is list:
        output = []
        for i, crv in enumerate(input):
            output.append(rs.MoveObject(crv, vector))
    else:
        output = rs.MoveObject(input, vector)
    return(output)


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

    bbox = rs.BoundingBox(input)
    minx = bbox[0][0]
    miny = bbox[0][1]
    minz = bbox[0][2]
    maxx = bbox[6][0]
    maxy = bbox[6][1]
    maxz = bbox[6][2]
    centrex = (minx + maxx) / 2 # part midpoint in x and y
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
    Creates a line of gcode from a variable set of keyword arguments
    pt: it can be used to define the x,y,z coordinates by a tuple
    x,y,z values will override the values on pt if present
    """
    if pt and len(pt)==3:
        x = pt[0]
        y = pt[1]
        z = pt[2]
    g = "G{}".format(int(g))
    x = " X{:.1f}".format(float(x)) if x is not None else ""
    y = " Y{:.1f}".format(float(y)) if y is not None else ""
    z = " Z{:.1f}".format(float(z)) if z is not None else ""
    e = " E{:.1f}".format(float(e)) if e is not None else ""
    f = " F{}".format(int(f)) if f else ""
    v = " V{:.1f}".format(float(v)) if v else ""
    gline =  g + x + y + z + v + e + f 
    return(gline)

def caluclate_flow(nozzle, layerheight, filament):
    narea = (((nozzle / 2) ** 2) * math.pi) # nozzle area
    filarea = (((filament / 2) ** 2) * math.pi) # filament area
    flow = (nozzle * layerheight) / filarea * 10
    return(flow)

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
    area = pi * r**2 # in mm2
    vol  = area * length # in mm3
    l = vol/1000000 # vol in litres
    m = length/1000 # length in metres
    return l