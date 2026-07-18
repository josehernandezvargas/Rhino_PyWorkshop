#! python

"""Grasshopper Script
"""

__author__ = "jose hernandez vargas"
__version__ = "2026-01-15"

import System
import Rhino
import rhinoscriptsyntax as rs # type: ignore
import Grasshopper as gh
import os
import time
import math
import json
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

def _is_number(value):
    return isinstance(value, (int, float))

def _add_runtime_message(level, message):
    ghenv.Component.AddRuntimeMessage(level, message)

def _load_machine_properties(machine_id):
    current_dir = os.path.dirname(os.path.abspath(gcl.__file__))
    parent_dir = os.path.dirname(current_dir)
    machine_file = os.path.join(parent_dir, "machine_settings", f"{machine_id}.json")
    try:
        with open(machine_file, "r") as file_handle:
            return json.load(file_handle)
    except (IOError, ValueError) as exc:
        raise ValueError(f"Error loading machine properties from {machine_file}: {exc}")

def _get_curve_plane_z(curve):
    crv_obj = rs.coercecurve(curve)
    if crv_obj:
        success, plane = crv_obj.TryGetPlane()
        if success:
            return plane.Origin.Z, True
    bbox = rs.BoundingBox(curve)
    if bbox:
        return bbox[0][2], False
    return 0.0, False

def _build_volume_preview(build_x, build_y, build_z):
    preview_items = []
    base = [
        (0, 0, 0),
        (build_x, 0, 0),
        (build_x, build_y, 0),
        (0, build_y, 0),
        (0, 0, 0),
    ]
    top = [
        (0, 0, build_z),
        (build_x, 0, build_z),
        (build_x, build_y, build_z),
        (0, build_y, build_z),
        (0, 0, build_z),
    ]
    preview_items.append(rs.AddPolyline(base))
    preview_items.append(rs.AddPolyline(top))
    preview_items.append(rs.AddLine(base[0], top[0]))
    preview_items.append(rs.AddLine(base[1], top[1]))
    preview_items.append(rs.AddLine(base[2], top[2]))
    preview_items.append(rs.AddLine(base[3], top[3]))
    return preview_items

errors = []
warnings = []

machine_id = "ultimaker2"

try:
    test_line_enabled = bool(test_line_enabled)
except NameError:
    test_line_enabled = True
    warnings.append("test_line_enabled not provided; defaulting to True.")

try:
    test_line_length = float(test_line_length)
except NameError:
    test_line_length = 100.0
    warnings.append("test_line_length not provided; defaulting to 100.0 mm.")

try:
    test_line_offset = float(test_line_offset)
except NameError:
    test_line_offset = 20
    warnings.append("test_line_offset not provided; defaulting to 20 mm.")

try:
    machine = _load_machine_properties(machine_id)
except ValueError as exc:
    errors.append(str(exc))
    machine = None

if machine:
    build_x = machine["build_volume"]["x"]
    build_y = machine["build_volume"]["y"]
    build_z = machine["build_volume"]["z"]
else:
    build_x = build_y = build_z = 0

try:
    flow = float(flow)
except NameError:
    flow = 1.0
    warnings.append("flow not provided; defaulting to 1.0.")
except (TypeError, ValueError):
    errors.append("flow must be a number when provided.")

zero = nozzle / 2 if _is_number(nozzle) else 0.0

timestamp = time.strftime("%Y%m%d")  # adds a timestamp with the date
hourstamp = " at " + time.strftime("%X")  # a timestamp with the hour

if toolpath is None:
    errors.append("toolpath is required.")
else:
    if isinstance(toolpath, (list, tuple, System.Array)):
        toolpath_list = list(toolpath)
    else:
        toolpath_list = [toolpath]
    if not toolpath_list:
        errors.append("toolpath is empty.")
    else:
        invalid_curves = [crv for crv in toolpath_list if not rs.coercecurve(crv)]
        if invalid_curves:
            errors.append("toolpath must contain only curve inputs.")

if not _is_number(nozzle) or nozzle <= 0:
    errors.append("nozzle must be a positive number.")
if not _is_number(layerheight) or layerheight <= 0:
    errors.append("layerheight must be a positive number.")
if not _is_number(filament) or filament <= 0:
    errors.append("filament must be a positive number.")
if not isinstance(filename, str) or not filename.strip():
    errors.append("filename must be a non-empty string.")
if not isinstance(save, bool):
    errors.append("save must be a boolean.")
# if not isinstance(variableflow, bool):
#     errors.append("variableflow must be a boolean.")
if "F" in globals():
    if not _is_number(F) or F <= 0:
        errors.append("F must be a positive number when provided.")
if test_line_length <= 0:
    errors.append("test_line_length must be greater than 0.")
if test_line_offset < 0:
    errors.append("test_line_offset must be 0 or greater.")

for message in warnings:
    _add_runtime_message(gh.Kernel.GH_RuntimeMessageLevel.Warning, message)

if errors:
    for message in errors:
        _add_runtime_message(gh.Kernel.GH_RuntimeMessageLevel.Error, message)

can_generate = not errors
preview_objects = []

if machine:
    preview_objects.extend(_build_volume_preview(build_x, build_y, build_z))

if can_generate:
    toolpath = pl.centerobject(toolpath_list, buildplate=(build_x, build_y))
    toolpath = pl.leveltoplatform(toolpath)
# ctoolpath = pl.centerobject(ctoolpath)

# HACK: disabled mesh
# if mesh:
#     mesh = leveltoplatform2(mesh)
#     mesh = centerobject2(mesh)

    bbox = rs.BoundingBox(toolpath)

    minx = bbox[0][0]
    miny = bbox[0][1]
    minz = bbox[0][2]
    maxx = bbox[6][0]
    maxy = bbox[6][1]
    maxz = bbox[6][2]

    z_values = []
    non_planar = False
    for crv in toolpath:
        z_value, is_planar = _get_curve_plane_z(crv)
        z_values.append(z_value)
        if not is_planar:
            non_planar = True
    if non_planar:
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Warning,
            "Non-planar curve detected; using bounding box Z for ordering.",
        )
    if len(z_values) > 1:
        is_increasing = all(z_values[i] <= z_values[i + 1] for i in range(len(z_values) - 1))
        if not is_increasing:
            toolpath = [crv for _, crv in sorted(zip(z_values, toolpath), key=lambda item: item[0])]
            _add_runtime_message(
                gh.Kernel.GH_RuntimeMessageLevel.Warning,
                "Toolpath order adjusted to increase by curve plane Z.",
            )


# if minz >= 1:
#    warning = "Flying model! (not attached to the buildplate)"
#    ghenv.Component.AddRuntimeMessage(gh.Kernel.GH_RuntimeMessageLevel.Warning, warning)

# G code utilities


# ###

    fits_buildplate = True
    if minx < 0 or miny < 0 or maxx > build_x or maxy > build_y:
        warning = "Buildplate dimensions exceeded"
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Error, warning)
        fits_buildplate = False

    if maxz > build_z:
        warning = "Max height exceeded"
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Error, warning)
        fits_buildplate = False

    if test_line_enabled:
        if miny < test_line_offset:
            warning = "Test line offset reduces usable build volume. Increase min Y or reduce test_line_offset."
            _add_runtime_message(
                gh.Kernel.GH_RuntimeMessageLevel.Error, warning)
            fits_buildplate = False
        line_center_x = (minx + maxx) / 2.0
        half_len = test_line_length / 2.0
        if (line_center_x - half_len) < 0 or (line_center_x + half_len) > build_x:
            warning = "Test line length exceeds available build volume width."
            _add_runtime_message(
                gh.Kernel.GH_RuntimeMessageLevel.Error, warning)
            fits_buildplate = False

    if not fits_buildplate:
        can_generate = False
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Warning,
            "Build volume check failed; preview only, no gcode will be generated.",
        )

if can_generate:
    header.append(f";FLAVOR:{machine['gcode_flavour']}")
    header.append(";MATERIAL:1")
    header.append(";MATERIAL2:0")
    header.append(f";TARGET_MACHINE.NAME:{machine['machine_name']}")
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

# TODO: move to printlib
def calculate_flow(nozzle, layerheight, filament):
    narea = (((nozzle / 2) ** 2) * math.pi) # nozzle area
    filarea = (((filament / 2) ** 2) * math.pi) # filament area
    flow = (nozzle * layerheight) / filarea * 10 # flow rate
    return(flow)



if can_generate:
    materialflow = calculate_flow(nozzle, layerheight, filament)

# Feedrates

F0 = 6000
# Use default feedrate if F is not set
if can_generate:
    if "F" in globals():
        try:
            F1 = float(F)
        except:
            F1 = 600
    else:
        F1 = 600

if can_generate:
    ext = 0.0
    tool = 0
    first = 0

    ini.append("G92 E0")
    ini.append("M109 S205")
    ini.append("G0 F12000 X5 Y5 Z20")
    ini.append("G280")
    ini.append("G10")

    if test_line_enabled:
        line_y = miny - test_line_offset
        line_center_x = (minx + maxx) / 2.0
        half_len = test_line_length / 2.0
        line_start = (line_center_x - half_len, line_y, 0)
        line_end = (line_center_x + half_len, line_y, 0)
        preview_objects.append(rs.AddLine(line_start, line_end))
        gcode.append("; TEST_LINE START")
        gcode.append(pl.gcodeline(0, pt=line_start, f=F0))
        gcode.append(pl.gcodeline(1, pt=line_end, e=test_line_length * materialflow, f=F1))
        gcode.append(pl.gcodeline(0, pt=line_start, f=F0))
        gcode.append("; TEST_LINE END")
        gcode.append("G92 E0")

# HACK: disabled mesh
# Evaluate the colour in a reference mesh
# if mesh:
#     meshcol = rs.MeshVertexColors(mesh)
#     meshvert = rs.MeshVertices(mesh)


if can_generate:
    tol = rs.UnitAbsoluteTolerance()
    tol2 = 1 # 1 mm tolerance for closing loops
    for i, crv in enumerate(toolpath):
        # Efficient polyline conversion (obs. not compatible with variable flow)
        polyline_crv = rs.ConvertCurveToPolyline(crv, angle_tolerance=5.0, tolerance=1.0, delete_input=False, min_edge_length =1.0)
        points = rs.PolylineVertices(polyline_crv)
        # reverse every other curve if the curve is not closed
        if i != 0:
            if _get_curve_plane_z(toolpath[i]) == _get_curve_plane_z(toolpath[i-1]):
                dist_last_pt = rs.Distance(rs.CurveEndPoint(toolpath[i]), rs.CurveEndPoint(toolpath[i-1]))
                if not rs.IsCurveClosed(crv) and dist_last_pt >= tol2 and len(points) > 1:
                    points.reverse()
        # points = rs.DivideCurveLength(crv, 1)  # Divide the curve in 1mm segments
        # reverse every other curve if the curve is not closed
        # if len(points) > 1:
        #     first_pt = points[0]
        #     last_pt = points[-1]
        #     dx = first_pt[0] - last_pt[0]
        #     dy = first_pt[1] - last_pt[1]
        #     dz = first_pt[2] - last_pt[2]
        #     if (dx * dx) + (dy * dy) + (dz * dz) > tol2 and i % 2 == 1:
        #         points.reverse()
        
        gcode.append(";TYPE:WALL-OUTER")
        gcode.append(";LAYER_COUNT:" + str(len(toolpath)))
        gcode.append(";LAYER:" + str(i))
        for j, pt in enumerate(points):
            if pt[2]>= 2:  # if printing height >= 2 mm start the fans
                gcode.append("M106; Turn fans on") # turn on the fans after first layer
            # DISABLED: Variable flow by distance
            # varflow = pl.selfclosestpt2(points, i, 4) / nozzle
            # Variable flow by density map
            # HACK: disabled mesh
            # if mesh:
            #     meshindex = rs.PointArrayClosestPoint(meshvert, pt)
            #     # FIXME: uses just the red channel for now
            #     colour = meshcol[meshindex].R / 255
            #     # FIXME: to function: map variation from 0,2 to 1
            #     varflow = (colour*.5) + 0.5
            # end variable flow
            # if variableflow:
            #     ext = varflow * materialflow + ext
            # else:

            previewpts.append(pt)
            previewflow.append(materialflow)
            if j == 0:
                # first point
                if first == 0:  # first point in the first curve only
                    # gline = gcline(0, F0, pt)
                    gline = pl.gcodeline(0,pt,f=F1)
                    gcode.append("G11") # unretract
                    first = 1
                else:  # first point of subsequent curves
                    # gline = gcline(1, F1, pt)
                    gline = pl.gcodeline(1, pt, f=F0)
                gcode.append(gline)
            else:
                # gline = gcline(1, F1, pt, ext)
                ext += materialflow * rs.Distance(points[j], points[j - 1])
                gline = pl.gcodeline(1, pt, f=F1, e=ext)
                gcode.append(gline)
            # print(pt, gline)
        # gcode.append("G10") # retract

footer = []

footer.append("G10")
footer.append("M107; turn fans off")
footer.append(";M82 ;absolute extrusion mode")
footer.append(";End of Gcode")

if previewpts:
    preview_objects.insert(0, rs.AddPolyline(previewpts))
preview = preview_objects

if can_generate and previewpts:
    # time estimation
    est = rs.CurveLength(preview_objects[0]) / F1 * 60
    header.insert(1, ";TIME:{:.0f}".format(est))

    gcodelines = chain(header, ini, gcode, footer)
    lines = [line for line in gcodelines]

    # saves file in a /gcode subfolder. It will be created if it doesn't exist
    base_dir = os.path.dirname(os.path.realpath(ghdoc.Path))
    gcode_dir = os.path.join(base_dir, "gcode")
    if not os.path.isdir(gcode_dir):
        os.makedirs(gcode_dir)

    extension = ".gcode"
    #if "." not in ext: ext = "." + ext
    # else: pass

    # if os.path.exists(file) == False: # Test if file already exists; if it doesn't, proceed
    file = os.path.join(gcode_dir, timestamp + "_" + filename + extension)
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
