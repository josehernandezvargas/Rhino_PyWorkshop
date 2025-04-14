#! python 3

"""Grasshopper script
    Inputs:
        PTS: list of 3dpoint
        VEL: list of int
        startpos: list with 6 angle position (int or float)
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

zero = 5 # height correction


krl.set_tool(6)
krl.set_base(1)

tool = 6
base = 1

# ADD HEADER

# krl.krl_header(startpos)
# print(krl.code)

# print(type(startpos), len(list))

# ADD HEADER
try:
    krl.krl_header(startpos)  # Using krl_header to add the header to the code
except Exception as e:
    print(f"Error adding header: {e}")

if len(startpos) == 6 and all(isinstance(i, (int, float)) for i in startpos):
    A1, A2, A3, A4, A5, A6 = startpos
else:
    raise Exception(
        "Start position should be a tuple with angles for each robot axis")

# # header from template
# krl.code.append("&ACCESS RVP")
# krl.code.append("&PARAM TEMPLATE = C:\KRC\Roboter\Template\\vorgabe")
# krl.code.append("&PARAM EDITMASK = *")

# # add some initial setup stuff
# krl.code.append("DEF "+str(name)+" ( )")
# krl.code.append(";FOLD INI")
# krl.code.append(";FOLD BASISTECH INI")
# krl.code.append(
#     "GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )")

# """
#     INTERRUPT

#     Description Executes one of the following actions:
#         - Activates an interrupt.
#         - Deactivates an interrupt.
#         - Disables an interrupt.
#         - Enables an interrupt.
#     Up to 16 interrupts may be active at any one time
    
# """
# krl.code.append("INTERRUPT ON 3")
# krl.code.append("BAS (#INITMOV,0 )")
# krl.code.append(";ENDFOLD (BASISTECH INI)")
# krl.code.append(";ENDFOLD (INI)")

# krl.code.append(";FOLD STARTPOSITION - BASE IS {}, TOOL IS {}, SPEED IS 100%, POSITION IS A1 {},A2 {},A3 {},A4 {},A5 {},A6 {},E1 0,E2 0,E3 0,E4 0".format(
#     base, tool, A1, A2, A3, A4, A5, A6))
# krl.code.append("$BWDSTART = FALSE")
# krl.code.append("PDAT_ACT = {VEL 100,ACC 20,APO_DIST 50}")
# krl.code.append(
#     "FDAT_ACT = {{TOOL_NO {},BASE_NO {},IPO_FRAME #BASE}}".format(tool, base))
# krl.code.append("BAS (#PTP_PARAMS,100)")
# krl.code.append("PTP  {{A1 {},A2 {},A3 {},A4 {},A5 {},A6 {},E1 0,E2 0,E3 0,E4 0}}".format(
#     A1, A2, A3, A4, A5, A6))
# krl.code.append(";ENDFOLD")

# # self.code.append("$APO.CDIS = 0.5000")
# # self.code.append("BAS (#INITMOV,0)")
# # self.code.append("BAS (#VEL_PTP,20)")
# # self.code.append("BAS (#ACC_PTP,20)")
# # self.code.append("")

# """
#     Advance run
#     The advance run is the maximum number of motion blocks that the robot controller calculates and plans in advance during program execution. The actual
#     number is dependent on the capacity of the computer.
#     The advance run refers to the current position of the block pointer. It is set via
#     the system variable $ADVANCE:
#         - Default value: 3
#         - Maximum value: 5
#     The advance run is required, for example, in order to be able to calculate approximate positioning motions. If $ADVANCE = 0 is set, approximate positioning is not possible.
#     Certain statements trigger an advance run stop. These include statements
#     that influence the periphery, e.g. OUT statements
# """
# krl.code.append("$ADVANCE=3")

# Starting points outside of the build plate
firstpt = (startpt[0] , startpt[1], 50.0) #HACK: hardcoded 50 mm height for first point

#secondpt = (startpt[0] , startpt[1], startpt[2])
##print firstpt
#firstpt = (100.0 ,1600.0, 100.0)
#secondpt = (100.0 ,1450.0, 10.0)


# Check if both lists have the same length
if len(PTS) != len(VEL):
    print("The lists have different lengths")

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
        plane = (firstpt[0],firstpt[1],firstpt[2]+ zero, 0, 0, 0 )
        krl.lin(plane)
#        krl.LIN(secondpt[0],secondpt[1],secondpt[2], 0, 0, 0, 0, 0)
        previewpts.append(firstpt)
#        previewpts.append(secondpt)
        lastvel = vel
    if vel != lastvel:
        krl.set_velocity(vel)
#    print vel
    krl.lin([pt.X, pt.Y, pt.Z+zero, 0, 0, 0])
    previewpts.append(pt)
    lastvel = vel
    lastpt = pt
krl.code.append("$OUT[3]=TRUE")

# Rise the nozzle quickly after the last point
krl.set_velocity(0.25)
krl.lin([lastpt[0], lastpt[1], lastpt[2]+50, 0, 0, 0])
previewpts.append(rs.AddPoint(lastpt[0], lastpt[1], lastpt[2]+50))
# Move the nozzle back over the start point
krl.set_velocity(0.25)
krl.lin([firstpt[0], firstpt[1], lastpt[2]+50, 0, 0, 0])
previewpts.append(rs.AddPoint(firstpt[0], firstpt[1], lastpt[2]+50))
# Add a last exit point after the print
krl.lin([firstpt[0], firstpt[1], lastpt[2], 0, 0, 0])
previewpts.append(rs.AddPoint(firstpt[0], firstpt[1], lastpt[2]))
krl.code.append("$OUT[3]=FALSE")



file = os.path.dirname(os.path.realpath(ghdoc.Path))
extension = ".src"
file += '\\'+ name + extension

previewpath = rs.AddPolyline(previewpts)


if save:
    krl.write_file(file)

    # print the filepath and a timestamp with the hour
    print('File Saved  ' + file + hourstamp)
else:
    msg = "Set 'write' to True."
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Warning, msg)

a = krl.code