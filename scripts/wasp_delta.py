#! python

"""Grasshopper Script
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
from gcodelib import GCodeLib

start_time = time.time()

ghenv.Component.Name = "Wasp gcode exporter"

# initialise gcode class

gcode = GCodeLib(filename, 'wasp')

polyline = rs.AddPolyline(PTS)

gcode.get_part_dims(polyline)
gcode.add_header(nozzle, flow)

print("0 initialise class / add header: {:.4f} seconds".format(time.time() - start_time))

previewpts = []
previewflow = []

# Global variables

filament = 2.85
zero = nozzle / 2

timestamp = time.strftime("%Y%m%d")  # adds a timestamp with the date
hourstamp = " at " + time.strftime("%X")  # a timestamp with the hour

# get the radius from printer parameters
delta_radius = gcode.machine['build_volume']['r']

# Check if both lists have the same length
if len(PTS) != len(VEL):
    raise ValueError("The PTS and VEL lists have different lengths")

print("1 get delta radius: {:.4f} seconds".format(time.time() - start_time))


# create a polyline from point list
toolpath = rs.AddPolyline(PTS)

# adjust the print to the centre of the buildplate
toolpath = pl.centerobject(toolpath, (-delta_radius, -delta_radius), delta=True)
toolpath = pl.leveltoplatform(toolpath)

# turn back into point list
PTS = rs.CurvePoints(toolpath)

print("2 PTS to polyline and back: {:.4f} seconds".format(time.time() - start_time))


materialflow = pl.caluclate_flow(nozzle, layerheight, filament) / 10 * flow
# print(nozzle, layerheight, filament, materialflow)

print("3 material flow calc: {:.4f} seconds".format(time.time() - start_time))



tool = 0

# initial commands
gcode.commands.append("G21 ;metric values")
gcode.commands.append("G90 ;absolute positioning")
gcode.commands.append("M82 ;set extruder to absolute mode")
gcode.commands.append("M107 ;start with the fan off")
gcode.commands.append("G28 X0 Y0 ;move X/Y to min endstops")
gcode.commands.append("G28 Z0 ;move Z to min endstops")
gcode.commands.append("T0")
gcode.commands.append("G92 E0 ;zero the extruded length")
gcode.commands.append("M140 S40; print bed") # sets up the bed temperature without waiting
gcode.commands.append("M109 S200; print head") # sets up the printhead temperatire and waits
gcode.commands.append("G0 F9000 X0 Y0 Z20")
# gcode.commands.append("G1 F200 E-{retraction_dual_amount}")



# Feedrates

F0 = 3600
F1 = 600 # start speed F1200 after 2 mm

# code generation
first = True
ext = 0
adhesion_layer = 1
fans_off = True

print("4 intial commands: {:.4f} seconds".format(time.time() - start_time))


for i, pt in enumerate(PTS):
    # start slower, with extra extrusion and and without fans
    if pt[2]>= 2 and fans_off:  # if printing height >= 2 mm start the fans
        gcode.commands.append("M106; Turn fans on") # turn on the fans after first layer
        adhesion_layer = 0
        fans_off = False
        F1 = 900
    # Variable flow by distance
    varflow = pl.selfclosestpt2(PTS, i, 4) / nozzle
    # end variable flow
    if i < len(PTS) - 1:  # Ensure there is a next point
        next_pt = PTS[i + 1]
        dist = rs.Distance(pt, next_pt)
    ext = materialflow * VEL[i] * dist + ext + (materialflow * dist * adhesion_layer)
    previewpts.append(pt)
    previewflow.append(VEL[i] * dist)
    if i == 0:
        # first point
        if first:  # first point in the first curve only
            # gline = gcline(0, F0, pt)
            gline = pl.gcodeline(0,pt,f=F1)
            gcode.commands.append("G11") # unretract
            first = False
        else:  # first point of subsequent curves
            # gline = gcline(1, F1, pt)
            gline = pl.gcodeline(1, pt, f=F1)
        gcode.commands.append(gline)
    else:
        # gline = gcline(1, F1, pt, ext)
        gline = pl.gcodeline(0, pt, f=F1, e=ext)
        gcode.commands.append(gline)
    # print(pt, gline)
    # gcode.commands.append("G10") # retract

print("5 main loop: {:.4f} seconds".format(time.time() - start_time))


gcode.commands.append("G10")
gcode.commands.append(";M82 ;absolute extrusion mode")
gcode.commands.append("")
gcode.commands.append(";End of Gcode")

gcode.commands.append("M104 S0  ;extruder heater off")
gcode.commands.append("M140 S0  ;heated bed heater off")
gcode.commands.append("G91  ;relative positioning")
gcode.commands.append("G1 E-1 F300   ;retract the filament a bit")
gcode.commands.append("G1 Z+1 E-5 F9000 ;move Z up a bit and retract filament even more")
gcode.commands.append("M107  ;fan off")
gcode.commands.append("G28 ;move X/Y to min endstops, so the head is out of the way")
gcode.commands.append(";M84 ;steppers off")
gcode.commands.append("G90 ;absolute positioning")

preview = rs.AddPolyline(previewpts)

# time estimation
secs = int(rs.CurveLength(preview) / F1 * 60)
gcode.header.insert(1, f"; TIME: {secs//3600:02}:{(secs%3600)//60:02}:{secs%60:02}")


file = os.path.dirname(os.path.realpath(ghdoc.Path))
extension = ".gcode"

file += '\\' + timestamp + "_" + filename + \
    extension  # Set file name and extension


print("6 end and compile gcode: {:.4f} seconds".format(time.time() - start_time))


if save:
    # gcode.save() FIXME: this function is not working
    with open(file, "w") as filePath:  # Open the file
        for line in gcode.header:  # Iterate through lines
            filePath.write(line + "\n")  # Write separate lines
        for line in gcode.commands:  # Iterate through lines
            filePath.write(line + "\n")  # Write separate lines

    # print the filepath and a timestamp with the hour
    print('File Saved ' + file + hourstamp)
else:
    msg = "Set 'write' to True."
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Warning, msg)

print("7 save: {:.4f} seconds".format(time.time() - start_time))
