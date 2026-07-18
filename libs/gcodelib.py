# libs/gcodelib.py
"""Shared, machine-agnostic G-code generation helpers.

Provides a machine-profile abstraction (covering both rectangular and
radial/delta build volumes), a canonical flow calculation, motion/retraction
line builders, a header assembler, a curve-to-toolpath preprocessor, and a
file-save helper. Individual export scripts (ultimaker.py, wasp_delta.py,
generic_gcode.py, ultimaker_speeds.py) call into this module instead of each
re-implementing the same logic.
"""
import json
import math
import os

import rhinoscriptsyntax as rs

import geometrylib as gl
import iolib as io


def load_machine_properties(machine_file):
    """Load machine properties from a JSON file.

    machine_file may be an absolute/relative path to a .json file, or a bare
    machine id (e.g. "ultimaker2"), in which case it is resolved against the
    repo's machine_settings/ folder.
    """
    if not machine_file.lower().endswith(".json"):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        parent_dir = os.path.dirname(current_dir)
        machine_file = os.path.join(parent_dir, "machine_settings", f"{machine_file}.json")

    try:
        with open(machine_file, "r") as file_handle:
            return json.load(file_handle)
    except (IOError, OSError, ValueError) as exc:
        raise io.ValidationError(f"Error loading machine properties from {machine_file}: {exc}")


def is_within_build_volume(profile, x, y, z):
    """Return True when point (x, y, z) fits within the machine profile's build volume.

    Supports both rectangular ("build_volume": {"x", "y", "z"}) and
    radial/delta ("build_volume": {"r", "h"}) profile shapes.
    """
    build_volume = profile["build_volume"]
    if profile.get("type") == "delta":
        return (x ** 2 + y ** 2) <= build_volume["r"] ** 2 and 0 <= z <= build_volume["h"]
    return (
        0 <= x <= build_volume["x"]
        and 0 <= y <= build_volume["y"]
        and 0 <= z <= build_volume["z"]
    )


def calculate_flow(nozzle, layerheight, filament):
    """Canonical FFF/FDM extrusion flow multiplier.

    The single source of truth for the formula previously duplicated (with a
    typo in one copy) across printlib.py and several ultimaker scripts.
    """
    narea = ((nozzle / 2) ** 2) * math.pi  # nozzle area
    filarea = ((filament / 2) ** 2) * math.pi  # filament area
    return (nozzle * layerheight) / filarea * 10


def gcodeline(g, pt=None, x=None, y=None, z=None, f=None, e=None, v=None):
    """
    Creates a line of gcode from a variable set of keyword arguments.
    pt: it can be used to define the x,y,z coordinates by a tuple
    x,y,z values will override the values on pt if present
    """
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


def travel_move(pt, feedrate=None):
    """Non-extruding G0 move to pt."""
    return gcodeline(0, pt=pt, f=feedrate)


def print_move(pt, e, feedrate=None):
    """Extruding G1 move to pt with cumulative extrusion e."""
    return gcodeline(1, pt=pt, e=e, f=feedrate)


def retract():
    """G-code line to retract the filament."""
    return "G10"


def unretract():
    """G-code line to unretract (prime) the filament."""
    return "G11"


def purge_duplicate_points(pts, tol=1e-6):
    """Remove consecutive points closer together than tol."""
    unique, last = [], None
    for p in pts:
        if last is None or rs.Distance(p, last) > tol:
            unique.append(p)
            last = p
    return unique


def purge_collinear_points(pts, tol=1e-6):
    """Remove interior points that are collinear with their neighbors."""
    if len(pts) < 3:
        return pts
    cleaned = [pts[0]]
    for i in range(1, len(pts) - 1):
        prev, curr, nxt = pts[i - 1], pts[i], pts[i + 1]
        v1 = rs.VectorCreate(curr, prev)
        v2 = rs.VectorCreate(nxt, curr)
        if rs.VectorLength(rs.VectorCrossProduct(v1, v2)) > tol:
            cleaned.append(curr)
    cleaned.append(pts[-1])
    return cleaned


def center_points_to_origin(point_groups):
    """
    Shift point groups so their combined XY bounding box is centered at the
    origin and the group rests on Z=0 (min Z becomes 0).
    """
    pts = [pt for group in point_groups for pt in group]
    if not pts:
        return point_groups
    min_x = min(pt[0] for pt in pts)
    max_x = max(pt[0] for pt in pts)
    min_y = min(pt[1] for pt in pts)
    max_y = max(pt[1] for pt in pts)
    min_z = min(pt[2] for pt in pts)
    shift = rs.CreateVector(-((min_x + max_x) / 2.0), -((min_y + max_y) / 2.0), -min_z)
    return [[rs.PointAdd(pt, shift) for pt in group] for group in point_groups]


def curve_to_polyline_points(curve, angle_tolerance=5.0, tolerance=1.0, min_edge_length=1.0):
    """Tessellate a curve into ordered vertices via rs.ConvertCurveToPolyline."""
    polyline_crv = rs.ConvertCurveToPolyline(
        curve,
        angle_tolerance=angle_tolerance,
        tolerance=tolerance,
        delete_input=False,
        min_edge_length=min_edge_length,
    )
    if not polyline_crv:
        return None
    return rs.PolylineVertices(polyline_crv)


def curve_to_points(curve, target_distance):
    """
    Convert a curve into points: native vertices when it's already a
    polyline, else evenly-divided points at approximately target_distance
    spacing.
    """
    if rs.IsPolyline(curve):
        return rs.PolylineVertices(curve)
    length = rs.CurveLength(curve)
    segments = max(int(round(length / target_distance)), 1)
    return rs.DivideCurve(curve, segments)


def build_header(profile, nozzle, flow, bounds, timestamp=None, hourstamp=None):
    """
    Build the UltiGCode-style comment header lines shared by the
    Ultimaker/Wasp export scripts.

    bounds: (minx, miny, minz, maxx, maxy, maxz)
    timestamp, hourstamp: optional pre-computed date/time strings; pass these
        through when the caller also reuses them elsewhere (e.g. the output
        filename or a "file saved" message), so they never disagree.
    """
    minx, miny, minz, maxx, maxy, maxz = bounds
    if timestamp is None:
        timestamp = gl.timestamp()
    if hourstamp is None:
        hourstamp = " at " + gl.timestamp(format=3)
    return [
        f";FLAVOR:{profile['gcode_flavour']}",
        ";MATERIAL:1",
        ";MATERIAL2:0",
        f";TARGET_MACHINE.NAME:{profile['machine_name']}",
        ";NOZZLE_DIAMETER:" + str(nozzle),
        ";MINX:" + str(minx),
        ";MINY:" + str(miny),
        ";MINZ:" + str(minz),
        ";MAXX:" + str(maxx),
        ";MAXY:" + str(maxy),
        ";MAXZ:" + str(maxz),
        ";Generated with Python / GH",
        ";File created " + timestamp,
        ";" + hourstamp,
        ";OVERFLOW: " + str(flow),
        "M82 ;absolute extrusion mode",
        ";END_OF_HEADER",
    ]


def save_gcode_file(base_dir, filename, header, commands, extension=".gcode", timestamp=None):
    """
    Write header+commands to <base_dir>/<timestamp>_<filename><extension>,
    creating base_dir if needed. filename is caller-supplied (typically a
    Grasshopper component input) and is never invented here.

    Returns the full file path written.
    """
    if timestamp is None:
        timestamp = gl.timestamp()
    if not os.path.isdir(base_dir):
        os.makedirs(base_dir)
    file_path = os.path.join(base_dir, f"{timestamp}_{filename}{extension}")
    with open(file_path, "w") as file_out:
        for line in header:
            file_out.write(f"{line}\n")
        for command in commands:
            file_out.write(f"{command}\n")
    return file_path


class GCodeLib:
    """Stateful helper bundling machine profile, part bounds, header and
    commands for a single export run."""

    def __init__(self, filename, machine_file):
        """
        Parameters:
        filename (str): The base name of the Gcode file to write.
        machine_file (str): Machine configuration file name (without
            extension) resolved against machine_settings/, or a path to a
            JSON file.
        """
        self.filename = filename
        self.machine = load_machine_properties(machine_file)
        self.header = []
        self.commands = []
        self.part = None
        self.minx = self.miny = self.minz = self.maxx = self.maxy = self.maxz = None

    def add_comment(self, comment):
        """Add a comment to the Gcode command list."""
        self.commands.append(f"; {comment}")

    def get_part_dims(self, geometry):
        """Compute and store the min/max coordinates for the part."""
        self.part = geometry
        bounds = gl.bbox_bounds(geometry)
        if bounds:
            self.minx, self.miny, self.minz, self.maxx, self.maxy, self.maxz = bounds

    def check_print(self, reporter=None):
        """
        Validate the part against the machine's build volume, reporting any
        issues via `reporter` (a GH component) if given, else printing them.
        """
        if self.part is None:
            raise io.ValidationError(
                "Part dimensions not set. Please run get_part_dims() before checking the print."
            )

        build_volume = self.machine["build_volume"]
        if self.machine.get("type") == "delta":
            radius = build_volume["r"]
            max_height = build_volume["h"]
            corners = (
                (self.minx, self.miny),
                (self.minx, self.maxy),
                (self.maxx, self.miny),
                (self.maxx, self.maxy),
            )
            fits_plan = all((x ** 2 + y ** 2) <= radius ** 2 for x, y in corners)
        else:
            max_x = build_volume["x"]
            max_y = build_volume["y"]
            max_height = build_volume["z"]
            fits_plan = (
                self.minx >= 0 and self.miny >= 0 and self.maxx <= max_x and self.maxy <= max_y
            )

        if not fits_plan:
            io.report_issue("Out of the buildplate!", level="error", component=reporter)

        if self.maxz > max_height:
            io.report_issue("Max height exceeded", level="error", component=reporter)

        if self.minz >= 1:
            io.report_issue(
                "Flying model! (not attached to the buildplate)",
                level="warning",
                component=reporter,
            )

    def add_header(self, nozzle, flow):
        """Build and store the comment header for the current part."""
        if self.part is None:
            raise io.ValidationError(
                "Part dimensions not set. Please run get_part_dims() before adding a header."
            )
        bounds = (self.minx, self.miny, self.minz, self.maxx, self.maxy, self.maxz)
        self.header = build_header(self.machine, nozzle, flow, bounds)

    def save(self, base_dir):
        """Save the accumulated header+commands to base_dir. Returns the file path."""
        return save_gcode_file(base_dir, self.filename, self.header, self.commands)
