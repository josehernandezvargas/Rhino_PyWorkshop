#! python

"""Grasshopper Script
Exports gcode for Ultimaker taking a list of points and a list of velocities as an input
"""

__author__ = "jose hernandez vargas"
__version__ = "2024-06-26"

import System
import rhinoscriptsyntax as rs # type: ignore
import Grasshopper as gh
import os
from itertools import chain
import printlib as pl
import gcodelib as gcl
import geometrylib as gl


ghenv.Component.Name = "Ultimaker 2 exporter"

header = []
ini = []
gcode = []

previewpts = []
previewflow = []

# Global variables

filament = 2.85
zero = nozzle / 2

timestamp = gl.timestamp()  # adds a timestamp with the date
hourstamp = " at " + gl.timestamp(format=3)  # a timestamp with the hour

machine = gcl.load_machine_properties("ultimaker2")
build_x = machine["build_volume"]["x"]
build_y = machine["build_volume"]["y"]
build_z = machine["build_volume"]["z"]

toolpath = pl.centerobject(toolpath)
ctoolpath = pl.leveltoplatform(toolpath)

bbox = rs.BoundingBox(ctoolpath)

minx = bbox[0][0]
miny = bbox[0][1]
minz = bbox[0][2]
maxx = bbox[6][0]
maxy = bbox[6][1]
maxz = bbox[6][2]

if minx < 0 or miny < 0 or maxx > build_x or maxy > build_y:
    warning = "Out of the buildplate!"
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Error, warning)

if maxz > build_z:
    warning = "MAX HEIGHT EXCEDDED"
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Error, warning)

header = gcl.build_header(
    machine, nozzle, flow, (minx, miny, minz, maxx, maxy, maxz),
    timestamp=timestamp, hourstamp=hourstamp,
)

materialflow = gcl.calculate_flow(nozzle, layerheight, filament)

# Feedrates

F0 = 3600
F1 = 600

ext = 0.0
tool = 0


ini.append("G92 E0")
ini.append("M109 S205")
ini.append("G0 F12000 X5 Y5 Z20")
ini.append("G280")
ini.append(gcl.retract())

# code generation
first = True

# This version takes a list of points as an input

for i, pt in enumerate(PTS):
    if pt[2]>= 2:  # if printing height >= 2 mm start the fans
        gcode.append("M106; Turn fans on") # turn on the fans after first layer
    # Variable flow from list of speeds
    varflow = VEL[i]
    ext = varflow * materialflow + ext
    previewpts.append(pt)
    previewflow.append(varflow)
    if i == 0:
        # first point
        if first:  # first point in the first curve only
            gline = gcl.gcodeline(0, pt, f=F0)
            gcode.append(gcl.unretract())
            first = False
        else:  # first point of subsequent curves
            gline = gcl.gcodeline(1, pt, f=F1)
        gcode.append(gline)
    else:
        gline = gcl.gcodeline(0, pt, f=F1, e=ext)
        gcode.append(gline)

footer = []

footer.append(gcl.retract())
footer.append("M107; turn fans off")
footer.append(";M82 ;absolute extrusion mode")
footer.append(";End of Gcode")

preview = rs.AddPolyline(previewpts)

# time estimation
est = rs.CurveLength(preview) / F1 * 60
header.insert(1, ";TIME:{:.0f}".format(est))

commands = list(chain(ini, gcode, footer))

base_dir = os.path.dirname(os.path.realpath(ghdoc.Path))

if save:
    file = gcl.save_gcode_file(base_dir, filename, header, commands, timestamp=timestamp)

    # print the filepath and a timestamp with the hour
    print('File Saved  ' + file + hourstamp)
else:
    msg = "Set 'write' to True."
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Warning, msg)
