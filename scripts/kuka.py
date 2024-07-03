#! python 3

"""Grasshopper script
    Inputs:
        PTS: list of 3dpoint
        VEL: list of int
        name: str
        startpt: 3dpoint
        save: bool
    Output:
        a: The a output variable"""

__author__ = "joseh"
__version__ = "2024.06.28"

import rhinoscriptsyntax as rs
import kukalib as kl
import Grasshopper as gh
import os
#import generalfunctions as gf
import time


timestamp = time.strftime("%y%m%d")  # adds a timestamp with the date
hourstamp = " at " + time.strftime("%X")  # a timestamp with the hour

name = name + "_" + timestamp
krl = kl.KukaKRL(name)

previewpts = []


krl.set_tool(6)
krl.set_base(1)
krl.krl_header(startpos)

# Starting points outside of the build plate
firstpt = (startpt[0] , startpt[1], 50.0) #HACK: hardcoded 50 mm height for first point

#secondpt = (startpt[0] , startpt[1], startpt[2])
##print firstpt
#firstpt = (100.0 ,1600.0, 100.0)
#secondpt = (100.0 ,1450.0, 10.0)


# Main loop

for i, pt in enumerate(PTS):
    vel = round(VEL[i]/1000, 3) # velocity in m/s
    pt = rs.coerce3dpoint(pt)
    if i == 0:
        krl.code.append(";FOLD LIN SPEED IS {} m/sec, INTERPOLATION SETTINGS IN FOLD".format(vel))
        krl.code.append("$VEL.CP={}".format(vel))
        krl.code.append("$ADVANCE=3")
        krl.code.append(";ENDFOLD")
        krl.code.append("$OUT[3]=FALSE")
        krl.lin(firstpt[0],firstpt[1],firstpt[2], 0.0, 0.0, 0.0, 0, 0)
#        krl.LIN(secondpt[0],secondpt[1],secondpt[2], 0, 0, 0, 0, 0)
        previewpts.append(firstpt)
#        previewpts.append(secondpt)
        lastvel = vel
    if vel != lastvel:
        krl.setVelocity(vel)
#    print vel
    krl.lin(pt.X, pt.Y, pt.Z, 0.0, 0.0, 0.0, 0, 0)
    previewpts.append(pt)
    lastvel = vel
    lastpt = pt
krl.code.append("$OUT[3]=TRUE")

# Rise the nozzle quickly after the last point
krl.setVelocity(0.25)
krl.lin(firstpt[0], firstpt[1], lastpt[2]+50, 0.0, 0.0, 0.0, 0, 0)
previewpts.append(rs.AddPoint(firstpt[0], firstpt[1], lastpt[2]+50))
# Add a last exit point after the print
krl.lin(firstpt[0], firstpt[1], lastpt[2], 0.0, 0.0, 0.0, 0, 0)
previewpts.append(rs.AddPoint(firstpt[0], firstpt[1], lastpt[2]))
krl.code.append("$OUT[3]=FALSE")

krl.code.append("END")

a = krl.code

file = os.path.dirname(os.path.realpath(ghdoc.Path))
extension = ".src"
file += '\\'+ name + extension

previewpath = rs.AddPolyline(previewpts)


if save:
    with open(file, "w") as filePath:  # Open the file
        for line in krl.code:  # Iterate through lines
            filePath.write(line + "\n")  # Write separate lines

    # print the filepath and a timestamp with the hour
    print('File Saved  ' + file + hourstamp)
else:
    msg = "Set 'write' to True."
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Warning, msg)