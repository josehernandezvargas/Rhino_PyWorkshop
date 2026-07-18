#! python 3

"""Grasshopper script to generate KUKA KRL code from point paths.

Inputs:
    PTS: list of 3d points (Rhino/Grasshopper points).
    VEL: list of velocities in mm/s (same length as PTS) or a single number.
         If a single number is provided, it is used for all points.
         Values > 1 are interpreted as mm/s (max 1000 mm/s = 1 m/s).
         Values <= 1 are interpreted as m/s.
    startpos: list/tuple with 6 joint angles (int or float).
    name: base program name (str).
    startpt: 3d point for initial approach.
    save: whether to write the KRL file (bool).
    PTP: optional PTP speed percentage (warn > 20%, cap at 50%).
    PTP_ACC: optional PTP acceleration percentage (0-100).
    PTP_APO: optional PTP APO distance in mm (>=0).
Output:
    a: the generated KRL code (list of strings).
"""

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

def _is_number(value):
    """Return True when value is an int or float (not a bool)."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)

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
            # Interpret as mm/s, cap at 1000 mm/s (1 m/s)
            if v > 1000:
                raise Exception("VEL[{}] exceeds 1 m/s. Max is 1000 mm/s.".format(i))
            v_mps = v / 1000.0
        else:
            # Interpret as m/s
            v_mps = v

        if v_mps < 0.005:
            _warn("VEL[{}] is below 5 mm/s; please double-check.".format(i))
        normalized.append(round(v_mps, 3))

    return normalized


def _validate_ptp_percent(ptp_value):
    """Validate PTP speed percentage; preserve legacy default when omitted."""
    if ptp_value is None:
        # Keep legacy header behavior unless user overrides PTP explicitly.
        return 100.0
    if not _is_number(ptp_value):
        raise Exception("PTP should be a number (percentage).")
    if ptp_value > 20:
        _warn("PTP speed is above 20%; please double-check.")
    if ptp_value > 50:
        _warn("PTP speed above 50% is not allowed; capping to 50%.")
        return 50
    if ptp_value <= 0:
        raise Exception("PTP should be greater than 0.")
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
    """Validate Grasshopper inputs and return normalized values."""
    pts = _require_nonempty_list("PTS", pts)
    vel = _normalize_velocities(vel, len(pts))

    startpos = _require_list("startpos", startpos)
    if len(startpos) != 6 or not all(_is_number(v) for v in startpos):
        raise Exception("startpos should be a list/tuple with 6 numeric joint angles.")

    if not isinstance(program_name, str) or not program_name.strip():
        raise Exception("name should be a non-empty string.")

    startpt_value = _coerce_point("startpt", startpt_value)

    if not isinstance(save_flag, bool):
        raise Exception("save should be a boolean.")

    ptp_value = _validate_ptp_percent(ptp_value)
    ptp_acc_value = _validate_ptp_acc(ptp_acc_value)
    ptp_apo_value = _validate_ptp_apo(ptp_apo_value)

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

def estimate_material_requirements(
    volume_m3,
    material_json,
):
    """
    Convert a target print volume into required premix and water.

    Parameters
    ----------
    volume_m3 : float
        Target deposited volume (m³).
    material_json : str
        Path to a material JSON file.

    Returns
    -------
    dict
        {
            "total_mass_kg": float,
            "premix_kg": float,
            "water_kg": float,
        }
    """

    with open(material_json, "r") as file:
        material = json.load(file)

    density = material["density_kg_m3"]
    water_ratio = material["water"]["water_premix_ratio"]

    total_mass = volume_m3 * density
    premix_mass = total_mass / (1.0 + water_ratio)
    water_mass = premix_mass * water_ratio

    return {
        "total_mass_kg": total_mass,
        "premix_kg": premix_mass,
        "water_kg": water_mass,
    }


PTP = globals().get("PTP", None)
PTP_ACC = globals().get("PTP_ACC", None)
PTP_APO = globals().get("PTP_APO", None)

PTS, VEL, startpos, name, startpt, save, PTP, PTP_ACC, PTP_APO = _validate_inputs(
    PTS, VEL, startpos, name, startpt, save, PTP, PTP_ACC, PTP_APO
)

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
_add_krl_header(krl, startpos, PTP, PTP_ACC, PTP_APO)

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
for i, pt in enumerate(PTS):
    vel = VEL[i] # velocity in m/s
    pt = _coerce_point("PTS[{}]".format(i), pt)
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


if save:
    krl.write_file(file)

    # print the filepath and a timestamp with the hour
    print('File Saved  ' + file + hourstamp)
else:
    msg = "Set 'write' to True."
    ghenv.Component.AddRuntimeMessage(
        gh.Kernel.GH_RuntimeMessageLevel.Warning, msg)

a = krl.code
