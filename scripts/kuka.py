#! python 3

"""Grasshopper script to generate KUKA KRL code from point paths.

Inputs:
    PTS: list of 3d points (Rhino/Grasshopper points). Alternative to CRVS -
         provide exactly one of the two.
    CRVS: list of curves, alternative to PTS. Each curve is treated as a
         separate deposited line: the extruder stops for the travel move
         between curves and resumes at the start of the next one. Polylines
         are read via their control points; other curves are first converted
         to a polyline (5 mm max deviation, 10 mm min segment length). When
         CRVS is used, VEL must be a single number (applied uniformly).
    VEL: list of velocities in mm/s (same length as PTS) or a single number.
         If a single number is provided, it is used for all points.
         Values > 1 are interpreted as mm/s, values <= 1 as m/s. Hardcoded
         safety cap: clamped to 250 mm/s (see MAX_LIN_SPEED_MM_S).
    startpos: list/tuple with 6 joint angles (int or float).
    name: base program name (str).
    startpt: 3d point for initial approach.
    save: whether to write the KRL file (bool).
    PTP: optional PTP speed percentage. Hardcoded safety cap: warn above 10%,
         hard-clamped to 25% (see PTP_WARN_PERCENT/PTP_MAX_PERCENT). Defaults
         to 20% if not supplied.
    PTP_ACC: optional PTP acceleration percentage (0-100).
    PTP_APO: optional PTP APO distance in mm (>=0).
    MACHINE: optional machine/material profile id or path (str). Resolved
             against machine_settings/ (e.g. "kuka" -> machine_settings/kuka.json).
             Must provide 'bead_width', 'layer_height' (mm), 'Density' (kg/m3)
             and 'Water ratio' (%) for the material estimate; 'flow_factor'
             and 'Waste per batch (kg)' are optional (default 1.0 / 0.0).
             Defaults to "kuka" if not supplied.
Output:
    a: the generated KRL code (list of strings).
    material: text summary of the material estimate (str) - add a matching
              output param on the component (alongside previewpts/previewpath)
              to view it in a Panel.
    previewpts: the full toolpath points, for previewing in Rhino.
    previewpath: a polyline through previewpts.
"""

__author__ = "joseh"
__version__ = "2024.06.28"

import rhinoscriptsyntax as rs
import kukalib as kl
import gcodelib as gcl
import Grasshopper as gh
import os
#import generalfunctions as gf
import time


timestamp = time.strftime("%y%m%d")  # adds a timestamp with the date
hourstamp = " at " + time.strftime("%X")  # a timestamp with the hour

MAX_LIN_SPEED_MM_S = 250.0  # hardcoded safety cap: LIN moves must never exceed this

def _is_number(value):
    """Return True when value is an int or float (not a bool)."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _coerce_number(value):
    """Coerce common Grasshopper quirks (text panels, 1-item tree branches,
    GH_Number wrappers) into a plain float. Returns None if not coercible."""
    if isinstance(value, (list, tuple)) and len(value) == 1:
        value = value[0]
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.strip())
        except ValueError:
            return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None

def _warn(message):
    """Emit a Grasshopper warning if available, otherwise print."""
    try:
        ghenv.Component.AddRuntimeMessage(
            gh.Kernel.GH_RuntimeMessageLevel.Warning, message
        )
    except Exception:
        print(message)


def _require_list(name_label, value):
    """Ensure value is a list-like (list/tuple) and return it."""
    if value is None or not isinstance(value, (list, tuple)):
        raise Exception("{} should be a list or tuple.".format(name_label))
    return value


def _require_nonempty_list(name_label, value):
    """Ensure value is a non-empty list/tuple and return it."""
    value = _require_list(name_label, value)
    if len(value) == 0:
        raise Exception("{} should not be empty.".format(name_label))
    return value


def _coerce_point(name_label, value):
    """Coerce a Rhino/Grasshopper point to a 3D point."""
    pt = rs.coerce3dpoint(value)
    if not pt:
        raise Exception("{} should be a valid 3d point.".format(name_label))
    return pt


def _coerce_curve(name_label, value):
    """Coerce a Rhino/Grasshopper curve."""
    crv = rs.coercecurve(value)
    if not crv:
        raise Exception("{} should be a valid curve.".format(name_label))
    return crv


def _curve_to_points(name_label, curve):
    """Extract ordered points from a curve for deposition.

    Polylines use their own control points. Any other curve is first
    approximated as a polyline (5 mm max deviation, 10 mm min segment
    length) and the same control points are then used.
    """
    if rs.IsPolyline(curve):
        pts = rs.PolylineVertices(curve)
    else:
        pts = gcl.curve_to_polyline_points(curve, tolerance=5.0, min_edge_length=10.0)
    if not pts:
        raise Exception("{} could not be converted to points.".format(name_label))
    return list(pts)


def _normalize_velocities(vel_value, count):
    """Normalize VEL input to a list of m/s values with validation."""
    if _is_number(vel_value):
        vel_list = [vel_value] * count
    elif isinstance(vel_value, (list, tuple)):
        if len(vel_value) == 1:
            vel_list = [vel_value[0]] * count
        elif len(vel_value) == count:
            vel_list = list(vel_value)
        else:
            raise Exception("VEL should be a single value or match PTS length.")
    else:
        raise Exception("VEL should be a number or a list/tuple of numbers.")

    normalized = []
    for i, v in enumerate(vel_list):
        if not _is_number(v):
            raise Exception("VEL[{}] should be a number.".format(i))
        if v <= 0:
            raise Exception("VEL[{}] should be greater than 0.".format(i))
        if v > 1:
            # Interpret as mm/s
            v_mps = v / 1000.0
        else:
            # Interpret as m/s
            v_mps = v

        if v_mps < 0.005:
            _warn("VEL[{}] is below 5 mm/s; please double-check.".format(i))
        if v_mps * 1000.0 > MAX_LIN_SPEED_MM_S:
            _warn(
                "VEL[{}] ({:.1f} mm/s) exceeds the hardcoded {:.0f} mm/s safety "
                "cap; clamping.".format(i, v_mps * 1000.0, MAX_LIN_SPEED_MM_S)
            )
            v_mps = MAX_LIN_SPEED_MM_S / 1000.0
        normalized.append(round(v_mps, 3))

    return normalized


PTP_WARN_PERCENT = 10.0  # hardcoded safety threshold: warn above this PTP speed
PTP_MAX_PERCENT = 25.0  # hardcoded safety cap: PTP speed must never exceed this


def _validate_ptp_percent(ptp_value):
    """Validate PTP speed percentage; hardcoded safety cap (warn/clamp thresholds above)."""
    if ptp_value is None:
        ptp_value = 20.0  # a safe, sub-cap default when PTP isn't supplied
    if not _is_number(ptp_value):
        raise Exception("PTP should be a number (percentage).")
    if ptp_value <= 0:
        raise Exception("PTP should be greater than 0.")
    if ptp_value > PTP_WARN_PERCENT:
        _warn("PTP speed is above {:.0f}%; please double-check.".format(PTP_WARN_PERCENT))
    if ptp_value > PTP_MAX_PERCENT:
        _warn("PTP speed above {:.0f}% is not allowed; capping to {:.0f}%.".format(
            PTP_MAX_PERCENT, PTP_MAX_PERCENT))
        return PTP_MAX_PERCENT
    return float(ptp_value)


def _validate_ptp_acc(ptp_acc_value):
    """Validate optional PTP acceleration percentage."""
    if ptp_acc_value is None:
        return 20.0
    if not _is_number(ptp_acc_value) or ptp_acc_value <= 0 or ptp_acc_value > 100:
        raise Exception("PTP_ACC should be a number in the range (0, 100].")
    return float(ptp_acc_value)


def _validate_ptp_apo(ptp_apo_value):
    """Validate optional PTP APO distance in mm."""
    if ptp_apo_value is None:
        return 50.0
    if not _is_number(ptp_apo_value) or ptp_apo_value < 0:
        raise Exception("PTP_APO should be a number >= 0.")
    return float(ptp_apo_value)


def _validate_common_inputs(
    startpos,
    program_name,
    startpt_value,
    save_flag,
    ptp_value,
    ptp_acc_value,
    ptp_apo_value,
):
    """Validate the Grasshopper inputs shared by both PTS and CRVS modes."""
    startpos = _require_list("startpos", startpos)
    if len(startpos) != 6:
        raise Exception(
            "startpos should have exactly 6 joint angles, got {}.".format(len(startpos))
        )
    coerced_startpos = []
    for i, v in enumerate(startpos):
        num = _coerce_number(v)
        if num is None:
            raise Exception(
                "startpos[{}] ({!r}) should be a numeric joint angle.".format(i, v)
            )
        coerced_startpos.append(num)
    startpos = coerced_startpos

    if not isinstance(program_name, str) or not program_name.strip():
        raise Exception("name should be a non-empty string.")

    startpt_value = _coerce_point("startpt", startpt_value)

    if not isinstance(save_flag, bool):
        raise Exception("save should be a boolean.")

    ptp_value = _validate_ptp_percent(ptp_value)
    ptp_acc_value = _validate_ptp_acc(ptp_acc_value)
    ptp_apo_value = _validate_ptp_apo(ptp_apo_value)

    return (
        startpos,
        program_name,
        startpt_value,
        save_flag,
        ptp_value,
        ptp_acc_value,
        ptp_apo_value,
    )


def _validate_inputs(
    pts,
    vel,
    startpos,
    program_name,
    startpt_value,
    save_flag,
    ptp_value,
    ptp_acc_value,
    ptp_apo_value,
):
    """Validate Grasshopper inputs (PTS mode) and return normalized values."""
    pts = _require_nonempty_list("PTS", pts)
    vel = _normalize_velocities(vel, len(pts))

    (
        startpos,
        program_name,
        startpt_value,
        save_flag,
        ptp_value,
        ptp_acc_value,
        ptp_apo_value,
    ) = _validate_common_inputs(
        startpos, program_name, startpt_value, save_flag, ptp_value, ptp_acc_value, ptp_apo_value
    )

    return (
        pts,
        vel,
        startpos,
        program_name,
        startpt_value,
        save_flag,
        ptp_value,
        ptp_acc_value,
        ptp_apo_value,
    )


def _validate_uniform_velocity(vel_value):
    """Validate VEL as a single scalar velocity (m/s), required for CRVS mode."""
    if isinstance(vel_value, (list, tuple)):
        if len(vel_value) != 1:
            raise Exception("VEL should be a single number when using CRVS.")
        vel_value = vel_value[0]
    if not _is_number(vel_value):
        raise Exception("VEL should be a number when using CRVS.")
    return _normalize_velocities(vel_value, 1)[0]


def _validate_curve_inputs(crvs_value):
    """Validate CRVS and convert each curve into an ordered point list."""
    crvs_value = _require_nonempty_list("CRVS", crvs_value)
    point_groups = []
    for i, crv in enumerate(crvs_value):
        curve = _coerce_curve("CRVS[{}]".format(i), crv)
        point_groups.append(_curve_to_points("CRVS[{}]".format(i), curve))
    return point_groups

def estimate_print_volume(print_length, bead_width, layer_height, flow_factor=1.0,):
    """
    Estimate the deposited material volume from toolpath length.

    Parameters
    ----------
    print_length : float
        Total deposited toolpath length (mm).
    bead_width : float
        Average deposited bead width (mm).
    layer_height : float
        Layer height (mm).
    flow_factor : float, optional
        Extrusion multiplier (default = 1.0).

    Returns
    -------
    dict
        {
            "volume_mm3": float,
            "volume_l": float,
            "volume_m3": float,
        }
    """
    volume_mm3 = (print_length * bead_width * layer_height * flow_factor)

    return {
        "volume_mm3": volume_mm3,
        "volume_l": volume_mm3 / 1_000_000,
        "volume_m3": volume_mm3 / 1_000_000_000,
    }

def _add_krl_header(krl_obj, start_position, ptp_value, ptp_acc_value, ptp_apo_value):
    """Add KRL header with backward-compatible kukalib call signatures."""
    try:
        # New kukalib signature with configurable PTP parameters.
        krl_obj.krl_header(
            start_position,
            ptp_velocity_percent=ptp_value,
            ptp_acc_percent=ptp_acc_value,
            ptp_apo_dist=ptp_apo_value,
        )
    except TypeError:
        # Backward compatibility if an older kukalib is loaded/cached by Rhino.
        krl_obj.krl_header(start_position)
        krl_obj.code.append(
            "PDAT_ACT = {{VEL {vel:.1f},ACC {acc:.1f},APO_DIST {apo:.1f}}}".format(
                vel=ptp_value, acc=ptp_acc_value, apo=ptp_apo_value
            )
        )
        krl_obj.code.append("BAS (#PTP_PARAMS,{:.1f})".format(ptp_value))

def estimate_material_mass(volume_l, density_kg_m3, water_ratio, waste_kg=0.0):
    """
    Convert a deposited volume into dry-mix (premix) and water mass, in grams.

    Parameters
    ----------
    volume_l : float
        Deposited volume (litres).
    density_kg_m3 : float
        Density of the fresh/mixed material (kg/m3).
    water_ratio : float
        Water-to-dry-mix ratio as a fraction (e.g. 0.145 for 14.5%).
    waste_kg : float, optional
        Fixed dry-mix allowance added per batch for mixer/pump waste (kg).

    Returns
    -------
    dict
        {"dry_mix_g": float, "water_g": float, "total_g": float}
    """
    wet_mass_kg = (volume_l / 1000.0) * density_kg_m3
    dry_mix_kg = wet_mass_kg / (1.0 + water_ratio) + waste_kg
    water_kg = dry_mix_kg * water_ratio

    return {
        "dry_mix_g": dry_mix_kg * 1000.0,
        "water_g": water_kg * 1000.0,
        "total_g": (dry_mix_kg + water_kg) * 1000.0,
    }


def _profile_field(profile, machine_id, key, required=True, default=None):
    """Look up a machine_settings field, tolerant of case variants."""
    for candidate in (key, key.lower(), key.upper()):
        if candidate in profile:
            return profile[candidate]
    if required:
        raise Exception(
            "Machine profile '{}' is missing '{}' - add it to the JSON to enable "
            "the material estimate.".format(machine_id, key)
        )
    return default


def _resolve_machine_profile(machine_value):
    """Load bead/layer geometry and material properties from a machine_settings profile."""
    machine_id = machine_value if machine_value else "kuka"
    try:
        profile = gcl.load_machine_properties(machine_id)
    except Exception as exc:
        raise Exception("Could not load machine profile '{}': {}".format(machine_id, exc))

    return {
        "bead_width": float(_profile_field(profile, machine_id, "bead_width")),
        "layer_height": float(_profile_field(profile, machine_id, "layer_height")),
        "flow_factor": float(_profile_field(profile, machine_id, "flow_factor", required=False, default=1.0)),
        "density_kg_m3": float(_profile_field(profile, machine_id, "Density")),
        "water_ratio": float(_profile_field(profile, machine_id, "Water ratio")) / 100.0,
        "waste_kg": float(_profile_field(profile, machine_id, "Waste per batch (kg)", required=False, default=0.0)),
    }


PTP = globals().get("PTP", None)
PTP_ACC = globals().get("PTP_ACC", None)
PTP_APO = globals().get("PTP_APO", None)
MACHINE = globals().get("MACHINE", None)
PTS = globals().get("PTS", None)
CRVS = globals().get("CRVS", None)

using_curves = isinstance(CRVS, (list, tuple)) and len(CRVS) > 0

if using_curves:
    if isinstance(PTS, (list, tuple)) and len(PTS) > 0:
        raise Exception("Provide either PTS or CRVS, not both.")
    point_groups = _validate_curve_inputs(CRVS)
    vel_scalar = _validate_uniform_velocity(VEL)
    (
        startpos,
        name,
        startpt,
        save,
        PTP,
        PTP_ACC,
        PTP_APO,
    ) = _validate_common_inputs(startpos, name, startpt, save, PTP, PTP_ACC, PTP_APO)
else:
    PTS, VEL, startpos, name, startpt, save, PTP, PTP_ACC, PTP_APO = _validate_inputs(
        PTS, VEL, startpos, name, startpt, save, PTP, PTP_ACC, PTP_APO
    )
    point_groups = [PTS]
    vel_scalar = None


def _point_velocity(point_index):
    """Return the velocity (m/s) to use for a given point index."""
    return vel_scalar if using_curves else VEL[point_index]


machine_profile = _resolve_machine_profile(MACHINE)
bead_width = machine_profile["bead_width"]
layer_height = machine_profile["layer_height"]
flow_factor = machine_profile["flow_factor"]

name = name + "_" + timestamp
krl = kl.KukaKRL(name)

previewpts = []

zero = 9 # height correction for first layer


krl.set_tool(6)
krl.set_base(1)

tool = 6
base = 1

# ADD HEADER

# krl.krl_header(startpos)
# print(krl.code)

# print(type(startpos), len(list))

# ADD HEADER
_add_krl_header(krl, startpos, PTP, PTP_ACC, PTP_APO)

material_comment_index = len(krl.code)  # material estimate comments are spliced in here later

A1, A2, A3, A4, A5, A6 = startpos

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


# Main loop
point_index = 0
for g, group in enumerate(point_groups):
    label = "CRVS[{}] point {{}}".format(g) if using_curves else "PTS[{}]"
    group_pts = [_coerce_point(label.format(k), p) for k, p in enumerate(group)]

    if g == 0:
        vel = _point_velocity(point_index)
        krl.code.append(";FOLD LIN SPEED IS {} m/sec, INTERPOLATION SETTINGS IN FOLD".format(vel))
        krl.code.append("$VEL.CP={}".format(vel))
        krl.code.append("$ADVANCE=3")
        krl.code.append(";ENDFOLD")
        krl.code.append("$OUT[3]=FALSE") # Start the extruder
        plane = (firstpt[0],firstpt[1],firstpt[2]+ zero, 0, 0, 0 )
        krl.lin(plane)
#        krl.LIN(secondpt[0],secondpt[1],secondpt[2], 0, 0, 0, 0, 0)
        previewpts.append(firstpt)
#        previewpts.append(secondpt)
        lastvel = vel
    else:
        # Travel move to the next curve: stop the extruder, move to its
        # start point, then resume before depositing along it.
        krl.code.append("$OUT[3]=TRUE") # Stop the extruder for the travel move
        travel_pt = group_pts[0]
        krl.lin([travel_pt.X, travel_pt.Y, travel_pt.Z+zero, 0, 0, 0])
        previewpts.append(travel_pt)
        krl.code.append("$OUT[3]=FALSE") # Resume the extruder for this line
        lastpt = travel_pt
        group_pts = group_pts[1:] # already reached via the travel move above

    for pt in group_pts:
        vel = _point_velocity(point_index)
        if vel != lastvel:
            krl.set_velocity(vel)
#    print vel
        krl.lin([pt.X, pt.Y, pt.Z+zero, 0, 0, 0])
        previewpts.append(pt)
        lastvel = vel
        lastpt = pt
        point_index += 1
krl.code.append("$OUT[3]=TRUE") # Stop the extruder

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


# saves file in a /krl subfolder. It will be created if it doesn't exist
base_dir = os.path.dirname(os.path.realpath(ghdoc.Path))
krl_dir = os.path.join(base_dir, "krl")
if not os.path.isdir(krl_dir):
    os.makedirs(krl_dir)

extension = ".src"
file = os.path.join(krl_dir, name + extension)

# file = os.path.dirname(os.path.realpath(ghdoc.Path))
# extension = ".src"
# file += '\\'+ name + extension

previewpath = rs.AddPolyline(previewpts)

# Material estimate, from the previewpath length (includes lead-in/lead-out travel)
path_length_mm = rs.CurveLength(previewpath)
if path_length_mm is None:
    raise Exception("Could not measure previewpath length for the material estimate.")
volume_info = estimate_print_volume(path_length_mm, bead_width, layer_height, flow_factor)
mass_info = estimate_material_mass(
    volume_info["volume_l"],
    machine_profile["density_kg_m3"],
    machine_profile["water_ratio"],
    machine_profile["waste_kg"],
)

# Material estimate summary, shared by the "material" text output, the
# Python print, and the KRL comment block below.
material_summary_lines = [
    "Path length: {:.2f} m".format(path_length_mm / 1000.0),
    "Bead width x layer height: {:.1f} x {:.1f} mm (flow factor {:.2f})".format(
        bead_width, layer_height, flow_factor),
    "Estimated volume: {:.3f} L".format(volume_info["volume_l"]),
    "Estimated dry mix: {:.1f} g, water: {:.1f} g, total: {:.1f} g".format(
        mass_info["dry_mix_g"], mass_info["water_g"], mass_info["total_g"]),
]
material = "\n".join(material_summary_lines)
print(material)

material_comment_lines = [";FOLD MATERIAL ESTIMATE"]
material_comment_lines += [";" + line for line in material_summary_lines]
material_comment_lines.append(";ENDFOLD")
krl.code[material_comment_index:material_comment_index] = material_comment_lines


if save:
    krl.write_file(file)

    # print the filepath and a timestamp with the hour
    print('File Saved  ' + file + hourstamp)
else:
    msg = "Set 'write' to True."
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Warning, msg)

a = krl.code
