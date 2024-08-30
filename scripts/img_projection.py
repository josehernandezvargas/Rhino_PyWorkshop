#! python3

"""Applies variable width printing to a series of curves
Projecting from a reference surface
    Inputs:
        pts:        list of points
        srf:        surface
        image_path: path of the projected file
    Output:
        speeds:     list of float

"""

__author__ = "joseh"
__version__ = "2024.07.16"


import math
import rhinoscriptsyntax as rs
from System.Drawing import Bitmap
import geometrylib as gl
import time

t0 = time.time()

basespeed = 125
intspeed = 250
maxspeed = 250
minspeed = 50
layerheight = 10
dispmin = 0
dispmax = 20
period = 3



points = []
speeds = []
vectors = []
points = []
sizes = []
out = []

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

print("1 intial commands: {:.4f} seconds".format(time.time() - t0))
t1 = time.time()

if srf1:
    # determine the closest surface
    srf = closest_srf(pt, srf0, srf1)[0]

    # image parameters
    if closest_srf(pt, srf0, srf1)[1]:
        img = Bitmap(image_path1)
    else:
        img = Bitmap(image_path0)
else:
    srf = srf0
    img = Bitmap(image_path0)

srfdomx = rs.SurfaceDomain(srf,0)
srfdomy = rs.SurfaceDomain(srf,1)
imgdomx = (0, img.Width-1)
imgdomy = (img.Height-1, 0)
    
for i, pt in enumerate(pts[:10000]):

    srfparam = rs.SurfaceClosestPoint(srf, pt)
    # ptparam = crvparam[i]
    paramx = gl.remap(srfdomx[0],srfdomx[1],imgdomx[0],imgdomx[1],srfparam[0])
    paramy = gl.remap(srfdomy[0],srfdomy[1],imgdomy[0],imgdomy[1],srfparam[1])

    # print("3 srf params: {:.4f} seconds".format(time.time() - t1))
    # t2 = time.time()
#        print(paramx,paramy)
    colour = img.GetPixel(int(round(paramx)),int(round(paramy)))
#        print(colour)
    out.append(colour)
    colour = rs.ColorRGBToHLS(colour)
    size = colour[2] # black is 0 and white is 1
    speed = gl.remap(0, 1, minspeed, maxspeed, size) # reverse omin and omax to acount for reversed colour scale
#        disp = gf.remap(0,1, 7.5 , 12.5, size)

    # print("4 colour sampling: {:.4f} seconds".format(time.time() - t2))
    # t3 = time.time()

    # porous mode overrides the compensation
    # applies a zigzag motion controlled by the image mapping
    if porous:
        sine_vector = math.sin(2 * math.pi * i / period) # period needs to be defined as a constant
        # square_vector = 1 if (index // 2) % 2 == 0 else -1
        # print("square" , square_vector)
        disp = gl.remap(1, 0, dispmin , dispmax, size * sine_vector)
    else:
        disp = gl.remap(1, 0, dispmin , dispmax, size) # reverse order to displace thicker lines the most
    # print(size, disp)

    # ptparam = rs.CurveClosestPoint(crv, pt)
#        if i < 2:
#            disp = 0
    # Replace this vector from the curve with the normal from the surface
    # dispvect = rs.CurvePerpFrame(crv,ptparam).XAxis * disp
    dispvect = rs.SurfaceNormal(srf,[paramx, paramy]) * disp
    disppt = rs.CopyObject(rs.AddPoint(pt), dispvect)
    # print("5 porosity: {:.4f} seconds".format(time.time() - t2))
    # t4 = time.time()

    vectors.append(dispvect)
    speeds.append(speed)
    sizes.append(size)
#        points.append(rs.AddPoint(pt))
    points.append(disppt)

print("main loop: {:.4f} seconds".format(time.time() - t0))
t1 = time.time()

a = points
# speeds for robot
b = speeds 
c = [gl.remap(125, 250, 10, 5, s) for s in speeds]
# speeds for ultimaker
d = [gl.remap(50, 250, 0.5 , 1.5, s) for s in speeds]
# d = vectors