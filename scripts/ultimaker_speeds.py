#! python

"""Grasshopper Script
Exports gcode for Ultimaker taking a list of points and a list of velocities as an input

TODO: 
-find a better name
-implement gcode Class
-integrate with main script
"""

__author__ = "jose hernandez vargas"
__version__ = "2024-06-26"

import System
import Rhino
import rhinoscriptsyntax as rs # type: ignore
import Grasshopper as gh
import os
import time
import math
from itertools import chain
import printlib as pl
import gcodelib as gcl


ghenv.Component.Name = "Ultimaker 2 exporter"

header = []
ini = []
gcode = []

previewpts = []
previewflow = []

# Global variables

filament = 2.85
zero = nozzle / 2

timestamp = time.strftime("%Y%m%d")  # adds a timestamp with the date
hourstamp = " at " + time.strftime("%X")  # a timestamp with the hour


toolpath = pl.centerobject(toolpath)
ctoolpath = pl.leveltoplatform(toolpath)
# ctoolpath = pl.centerobject(ctoolpath)

# HACK: disabled mesh
# if mesh:
#     mesh = leveltoplatform2(mesh)
#     mesh = centerobject2(mesh)

bbox = rs.BoundingBox(ctoolpath)

minx = bbox[0][0]
miny = bbox[0][1]
minz = bbox[0][2]
maxx = bbox[6][0]
maxy = bbox[6][1]
maxz = bbox[6][2]

# if minz >= 1:
#    warning = "Flying model! (not attached to the buildplate)"
#    ghenv.Component.AddRuntimeMessage(gh.Kernel.GH_RuntimeMessageLevel.Warning, warning)




# if printer == "2+" or "3+":
#     maxheight = 305
# else:
#     maxheight = 205

maxheight = 305


if minx < 0 or miny < 0 or maxx > 223 or maxy > 223:
    warning = "Out of the buildplate!"
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Error, warning)

if maxz > maxheight:
    warning = "MAX HEIGHT EXCEDDED"
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Error, warning)

header.append(";FLAVOR:UltiGCode")
header.append(";MATERIAL:1")
header.append(";MATERIAL2:0")
header.append(";TARGET_MACHINE.NAME:Ultimaker 2+")
header.append(";NOZZLE_DIAMETER:" + str(nozzle))
header.append(";MINX:" + str(minx))
header.append(";MINY:" + str(miny))
header.append(";MINZ:" + str(minz))
header.append(";MAXX:" + str(maxx))
header.append(";MAXY:" + str(maxy))
header.append(";MAXZ:" + str(maxz))
header.append(";Generated with Python / GH")
header.append(";File created " + timestamp )
header.append(";" + hourstamp)
header.append(";OVERFLOW: " + str(flow))
header.append("M82 ;absolute extrusion mode")
header.append(";END_OF_HEADER")


def caluclateflow(nozzle, layerheight, filament):
    narea = (((nozzle / 2) ** 2) * math.pi) # nozzle area
    filarea = (((filament / 2) ** 2) * math.pi) # filament area
    flow = (nozzle * layerheight) / filarea * 10 # flow rate
    return(flow)


materialflow = caluclateflow(nozzle, layerheight, filament)

# Feedrates

F0 = 3600
F1 = 600

ext = 0.0
tool = 0


ini.append("G92 E0")
ini.append("M109 S205")
ini.append("G0 F12000 X5 Y5 Z20")
ini.append("G280")
ini.append("G10")

# code generation
first = True

# HACK: disabled mesh
# Evaluate the colour in a reference mesh
# if mesh:
#     meshcol = rs.MeshVertexColors(mesh)
#     meshvert = rs.MeshVertices(mesh)

# This version takes a list of points as an input


for i, pt in enumerate(PTS):
    if pt[2]>= 2:  # if printing height >= 2 mm start the fans
        gcode.append("M106; Turn fans on") # turn on the fans after first layer
    # Variable flow by distance
    # varflow = pl.selfclosestpt2(points, index, 4) / nozzle
    # Variable flow from list of speeds
    varflow = VEL[i]
    ext = varflow * materialflow + ext
    previewpts.append(pt)
    previewflow.append(varflow)
    if i == 0:
        # first point
        if first:  # first point in the first curve only
            # gline = gcline(0, F0, pt)
            gline = pl.gcodeline(0,pt,f=F0)
            gcode.append("G11") # unretract
            first = False
        else:  # first point of subsequent curves
            # gline = gcline(1, F1, pt)
            gline = pl.gcodeline(1, pt, f=F1)
        gcode.append(gline)
    else:
        # gline = gcline(1, F1, pt, ext)
        gline = pl.gcodeline(0, pt, f=F1, e=ext)
        gcode.append(gline)
    # print(pt, gline)
# gcode.append("G10") # retract

footer = []

footer.append("G10")
footer.append("M107; turn fans off")
footer.append(";M82 ;absolute extrusion mode")
footer.append(";End of Gcode")

preview = rs.AddPolyline(previewpts)

# time estimation
est = rs.CurveLength(preview) / F1 * 60
header.insert(1, ";TIME:{:.0f}".format(est))

gcodelines = chain(header, ini, gcode, footer)
lines = [line for line in gcodelines]


file = os.path.dirname(os.path.realpath(ghdoc.Path))


extension = ".gcode"
#if "." not in ext: ext = "." + ext
# else: pass

# if os.path.exists(file) == False: # Test if file already exists; if it doesn't, proceed
file += '\\' + timestamp + "_" + filename + \
    extension  # Set file name and extension
# if os.path.exists(file) == True: # If it does exists, follow the next steps
#    file_count = len([f for f in os.walk(".").next()[2] if f[-4:] == ext]) # Find all files with the same extension
#    file = timestamp + name + "_" + str(file_count) + ext # Add the number to the new file name as a differentiator


if save:
    with open(file, "w") as filePath:  # Open the file
        for line in lines:  # Iterate through lines
            filePath.write(line + "\n")  # Write separate lines

    # print the filepath and a timestamp with the hour
    print('File Saved  ' + file + hourstamp)
else:
    msg = "Set 'write' to True."
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Warning, msg)
