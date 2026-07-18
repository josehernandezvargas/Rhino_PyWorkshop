#! python

"""Grasshopper Script

Experimental alpha self-contained Ultimaker 2+ G-code exporter for Grasshopper.

Required inputs
- toolpath: Curve or list of planar slicing curves to export.
- nozzle: Nozzle diameter in mm.
- layerheight: Layer height in mm.
- filename: Output file name without extension.
- save: Boolean. When True, writes the G-code file to a local ``gcode`` folder.

Optional inputs
- F: Printing feed rate in mm/min. Defaults to 600.
- flow: Flow multiplier stored in the header only. Defaults to 1.0.
- test_line_enabled: Boolean to generate a priming/test line. Defaults to True.
- test_line_length: Test line length in mm. Defaults to 100.0.
- test_line_offset: Distance from model min Y to the test line in mm. Defaults to 20.0.

Outputs
- preview: Build volume, optional test line, and print path preview geometry.
- previewpts: Ordered print points used to generate the extrusion path.
- previewflow: Flow values paired with preview points.

Notes
- This script is self-contained and does not require repository libraries.
- Machine settings are embedded for the Ultimaker 2+ profile.
- Status: experimental alpha. Validate output on the target machine before production use.
"""

__author__ = "jose hernandez vargas"
__version__ = "2026-03-24-alpha"

import System
import rhinoscriptsyntax as rs  # type: ignore
import Grasshopper as gh
import os
import time
import math
from itertools import chain


ghenv.Component.Name = "Ultimaker 2 Exporter Contained"
ghenv.Component.NickName = "UM2 Contained"
ghenv.Component.Message = "experimental alpha 2026-03-24"
ghenv.Component.Description = (
    "Self-contained experimental alpha exporter for Ultimaker 2+ G-code."
)

header = []
ini = []
gcode = []

previewpts = []
previewflow = []

filament = 2.85

MACHINE_PROFILES = {
    "ultimaker2": {
        "machine_name": "Ultimaker 2+",
        "gcode_flavour": "UltiGCode",
        "type": "cartesian",
        "build_volume": {
            "x": 223,
            "y": 223,
            "z": 205,
        },
        "nozzle_diameter": 0.4,
    }
}


def _is_number(value):
    return isinstance(value, (int, float))


def _add_runtime_message(level, message):
    ghenv.Component.AddRuntimeMessage(level, message)


def _load_machine_properties(machine_id):
    try:
        return MACHINE_PROFILES[machine_id]
    except KeyError:
        supported_ids = ", ".join(sorted(MACHINE_PROFILES))
        raise ValueError(
            "Unknown machine_id '{}'. Supported machine ids: {}.".format(
                machine_id, supported_ids
            )
        )


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


def _centerobject(geometry, buildplate=(223, 223), delta=False):
    bbox = rs.BoundingBox(geometry)
    minx = bbox[0][0]
    miny = bbox[0][1]
    maxx = bbox[6][0]
    maxy = bbox[6][1]
    centrex = (minx + maxx) / 2.0
    centrey = (miny + maxy) / 2.0
    if delta:
        dispx = -centrex
        dispy = -centrey
    else:
        dispx = buildplate[0] / 2.0 - centrex
        dispy = buildplate[1] / 2.0 - centrey
    dispvector = rs.CreateVector(dispx, dispy, 0)
    if isinstance(geometry, list):
        return [rs.CopyObject(item, dispvector) for item in geometry]
    return rs.CopyObject(geometry, dispvector)


def _leveltoplatform(geometry, height=0):
    bbox = rs.BoundingBox(geometry)
    minz = bbox[0][2]
    disp = height - minz
    vector = rs.CreateVector(0, 0, disp)
    if isinstance(geometry, list):
        return [rs.MoveObject(item, vector) for item in geometry]
    return rs.MoveObject(geometry, vector)


def _gcodeline(g, pt=None, x=None, y=None, z=None, f=None, e=None, v=None):
    if pt and len(pt) == 3:
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
    return g + x + y + z + v + e + f


def calculate_flow(nozzle, layerheight, filament):
    filarea = (((filament / 2) ** 2) * math.pi)
    return (nozzle * layerheight) / filarea * 10


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
    test_line_offset = 20.0
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

timestamp = time.strftime("%Y%m%d")
hourstamp = " at " + time.strftime("%X")

toolpath_list = []
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
    toolpath = _centerobject(toolpath_list, buildplate=(build_x, build_y))
    toolpath = _leveltoplatform(toolpath)

    bbox = rs.BoundingBox(toolpath)

    minx = bbox[0][0]
    miny = bbox[0][1]
    minz = bbox[0][2]
    maxx = bbox[6][0]
    maxy = bbox[6][1]
    maxz = bbox[6][2]

    z_data = []
    non_planar = False
    for crv in toolpath:
        z_value, is_planar = _get_curve_plane_z(crv)
        z_data.append((z_value, is_planar))
        if not is_planar:
            non_planar = True
    if non_planar:
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Warning,
            "Non-planar curve detected; using bounding box Z for ordering.",
        )
    z_values = [item[0] for item in z_data]
    if len(z_values) > 1:
        is_increasing = all(
            z_values[i] <= z_values[i + 1] for i in range(len(z_values) - 1)
        )
        if not is_increasing:
            toolpath = [
                crv for _, crv in sorted(zip(z_values, toolpath), key=lambda item: item[0])
            ]
            z_values = sorted(z_values)
            _add_runtime_message(
                gh.Kernel.GH_RuntimeMessageLevel.Warning,
                "Toolpath order adjusted to increase by curve plane Z.",
            )

    fits_buildplate = True
    if minx < 0 or miny < 0 or maxx > build_x or maxy > build_y:
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Error, "Buildplate dimensions exceeded"
        )
        fits_buildplate = False

    if maxz > build_z:
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Error, "Max height exceeded"
        )
        fits_buildplate = False

    if test_line_enabled:
        if miny < test_line_offset:
            _add_runtime_message(
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                "Test line offset reduces usable build volume. Increase min Y or reduce test_line_offset.",
            )
            fits_buildplate = False
        line_center_x = (minx + maxx) / 2.0
        half_len = test_line_length / 2.0
        if (line_center_x - half_len) < 0 or (line_center_x + half_len) > build_x:
            _add_runtime_message(
                gh.Kernel.GH_RuntimeMessageLevel.Error,
                "Test line length exceeds available build volume width.",
            )
            fits_buildplate = False

    if not fits_buildplate:
        can_generate = False
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Warning,
            "Build volume check failed; preview only, no gcode will be generated.",
        )

if can_generate:
    header.append(";FLAVOR:{}".format(machine["gcode_flavour"]))
    header.append(";MATERIAL:1")
    header.append(";MATERIAL2:0")
    header.append(";TARGET_MACHINE.NAME:{}".format(machine["machine_name"]))
    header.append(";NOZZLE_DIAMETER:{}".format(nozzle))
    header.append(";MINX:{}".format(minx))
    header.append(";MINY:{}".format(miny))
    header.append(";MINZ:{}".format(minz))
    header.append(";MAXX:{}".format(maxx))
    header.append(";MAXY:{}".format(maxy))
    header.append(";MAXZ:{}".format(maxz))
    header.append(";Generated with Python / GH")
    header.append(";File created {}".format(timestamp))
    header.append(";{}".format(hourstamp))
    header.append(";OVERFLOW: {}".format(flow))
    header.append("M82 ;absolute extrusion mode")
    header.append(";END_OF_HEADER")

if can_generate:
    materialflow = calculate_flow(nozzle, layerheight, filament)

F0 = 6000
if can_generate:
    if "F" in globals():
        try:
            F1 = float(F)
        except Exception:
            F1 = 600
    else:
        F1 = 600

if can_generate:
    ext = 0.0
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
        gcode.append(_gcodeline(0, pt=line_start, f=F0))
        gcode.append(_gcodeline(1, pt=line_end, e=test_line_length * materialflow, f=F1))
        gcode.append(_gcodeline(0, pt=line_start, f=F0))
        gcode.append("; TEST_LINE END")
        gcode.append("G92 E0")

if can_generate:
    tol2 = 1
    for i, crv in enumerate(toolpath):
        polyline_crv = rs.ConvertCurveToPolyline(
            crv,
            angle_tolerance=5.0,
            tolerance=1.0,
            delete_input=False,
            min_edge_length=1.0,
        )
        if not polyline_crv:
            _add_runtime_message(
                gh.Kernel.GH_RuntimeMessageLevel.Warning,
                "Curve {} could not be converted to polyline and was skipped.".format(i),
            )
            continue
        points = rs.PolylineVertices(polyline_crv)
        if not points or len(points) < 2:
            _add_runtime_message(
                gh.Kernel.GH_RuntimeMessageLevel.Warning,
                "Curve {} produced insufficient polyline points and was skipped.".format(i),
            )
            continue
        if i != 0 and z_values[i] == z_values[i - 1]:
            dist_last_pt = rs.Distance(
                rs.CurveEndPoint(toolpath[i]), rs.CurveEndPoint(toolpath[i - 1])
            )
            if not rs.IsCurveClosed(crv) and dist_last_pt >= tol2 and len(points) > 1:
                points.reverse()

        gcode.append(";TYPE:WALL-OUTER")
        gcode.append(";LAYER_COUNT:{}".format(len(toolpath)))
        gcode.append(";LAYER:{}".format(i))
        for j, pt in enumerate(points):
            if pt[2] >= 2:
                gcode.append("M106; Turn fans on")

            previewpts.append(pt)
            previewflow.append(materialflow)
            if j == 0:
                if first == 0:
                    gline = _gcodeline(0, pt=pt, f=F1)
                    gcode.append("G11")
                    first = 1
                else:
                    gline = _gcodeline(1, pt=pt, f=F0)
                gcode.append(gline)
            else:
                ext += materialflow * rs.Distance(points[j], points[j - 1])
                gline = _gcodeline(1, pt=pt, f=F1, e=ext)
                gcode.append(gline)

footer = []
footer.append("G10")
footer.append("M107; turn fans off")
footer.append(";M82 ;absolute extrusion mode")
footer.append(";End of Gcode")

if previewpts:
    preview_objects.insert(0, rs.AddPolyline(previewpts))
preview = preview_objects

if can_generate and previewpts:
    est = rs.CurveLength(preview_objects[0]) / F1 * 60
    header.insert(1, ";TIME:{:.0f}".format(est))

    gcodelines = chain(header, ini, gcode, footer)
    lines = [line for line in gcodelines]

    base_dir = os.path.dirname(os.path.realpath(ghdoc.Path))
    gcode_dir = os.path.join(base_dir, "gcode")
    if not os.path.isdir(gcode_dir):
        os.makedirs(gcode_dir)

    extension = ".gcode"
    file = os.path.join(gcode_dir, timestamp + "_" + filename + extension)

    if save:
        with open(file, "w") as filePath:
            for line in lines:
                filePath.write(line + "\n")

        print("File Saved  " + file + hourstamp)
    else:
        _add_runtime_message(
            gh.Kernel.GH_RuntimeMessageLevel.Warning, "Set 'save' to True."
        )
