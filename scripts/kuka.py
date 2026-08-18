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
    STANDBY_IN: optional digital input number that requests standby (default 11).
    RESUME_IN: optional digital input number that leaves standby (default 12).
    REDO_IN: optional digital input number read on leaving standby; when TRUE the
             layer that just finished is printed again (default 13). Ignored when
             the layer was aborted mid-way, since that one always gets repeated.
    SAFE_Z: optional clearance in mm used to lift clear of the part before
            travelling to the park position, and to approach a layer on
            re-entry (default 50).
    LAYER_TOL: optional Z tolerance in mm for grouping curves into layers
               (default: half the profile's layer_height, minimum 0.1).
    FADE_OUT: optional non-negative integer (default 0). The last FADE_OUT
              points of the toolpath are travelled with the extruder off
              ($OUT[3]=TRUE), for a trailing loop (e.g. spiraliser.py's
              fade_out lap) that closes the seam without depositing material.
Output:
    a: the generated KRL code (list of strings), including the local subprograms.
    material: text summary of the material estimate (str) - add a matching
              output param on the component (alongside previewpts/previewpath)
              to view it in a Panel.
    layers: text summary of the detected layers and their subprogram names (str) -
            also needs a matching output param on the component.
    previewpts: the full toolpath points, for previewing in Rhino.
    previewpath: a polyline through previewpts.

Standby / layer recovery
------------------------
The deposition path is emitted as one local KRL subprogram per layer (L001,
L002, ...) plus a dispatcher loop in the main program, so any single layer can
be re-run without restarting the print:

    REPEAT
      SWITCH LAYER_N ... L00n ( ) ... ENDSWITCH
      <standby / repeat / advance decision>
    UNTIL LAYER_N > LAYER_COUNT

Switching to standby works two ways, both driven by $IN[STANDBY_IN]:

  * mid-layer - a GLOBAL INTERRUPT aborts the move in progress (BRAKE/RESUME),
    which drops execution back into the dispatcher. The aborted layer is always
    repeated, because it is by definition incomplete.
  * between layers - the dispatcher checks the same input after each layer
    returns normally. $IN[REDO_IN] then decides repeat vs. continue.

STANDBY ( ) lifts SAFE_Z straight up from wherever the robot stopped, travels
over the start point at that height, and runs the extruder there so material
keeps moving while the operator clears the failed layer. It holds until
$IN[STANDBY_IN] goes FALSE and $IN[RESUME_IN] goes TRUE, then sets the re-entry
flag so the next layer approaches from SAFE_Z above its own start point instead
of driving straight through the part.

Layers are detected by grouping consecutive CRVS curves whose average Z is
within LAYER_TOL. In PTS mode the whole path is a single layer, so use CRVS if
you want layer-level recovery.
"""

__author__ = "joseh"
__version__ = "2026.07.28"

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

# Extruder control. NOTE the inverted logic: $OUT[3]=FALSE runs the extruder.
EXTRUDER_OUT = 3
EXTRUDER_ON = False
EXTRUDER_OFF = True

# Standby / layer-recovery wiring
STANDBY_INT = 20  # KRL interrupt number (also its priority) for the standby request
FLAG_ABORT = 1  # $FLAG[1]: the layer in progress was aborted by the interrupt
FLAG_REENTRY = 2  # $FLAG[2]: the next layer must be approached from a safe height
DEFAULT_STANDBY_IN = 11
DEFAULT_RESUME_IN = 12
DEFAULT_REDO_IN = 13
DEFAULT_SAFE_Z = 50.0  # mm of clearance for lift/park/re-entry moves
TRAVEL_VEL = 0.25  # m/s for standby and re-entry travel moves


def _krl_bool(state):
    """KRL literal for a Python bool."""
    return "TRUE" if state else "FALSE"


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

def _validate_digital_input(name_label, value, default):
    """Validate an optional digital-input number, falling back to a default."""
    if value is None:
        return default
    num = _coerce_number(value)
    if num is None or num != int(num) or int(num) < 1 or int(num) > 4096:
        raise Exception(
            "{} should be an integer digital input number between 1 and 4096.".format(name_label)
        )
    return int(num)


def _validate_safe_z(value):
    """Validate the optional SAFE_Z clearance in mm."""
    if value is None:
        return DEFAULT_SAFE_Z
    num = _coerce_number(value)
    if num is None or num <= 0:
        raise Exception("SAFE_Z should be a number greater than 0 (mm).")
    return num


def _validate_layer_tol(value, profile_layer_height):
    """Validate the optional layer-grouping Z tolerance in mm."""
    if value is None:
        return max(0.1, profile_layer_height * 0.5)
    num = _coerce_number(value)
    if num is None or num <= 0:
        raise Exception("LAYER_TOL should be a number greater than 0 (mm).")
    return num


def _validate_fade_out(value, total_points):
    """Validate the optional FADE_OUT tail point count (extruder off near the end)."""
    if value is None:
        return 0
    num = _coerce_number(value)
    if num is None or num != int(num) or int(num) < 0:
        raise Exception("FADE_OUT should be a non-negative integer point count.")
    num = int(num)
    if num > total_points:
        raise Exception(
            "FADE_OUT ({}) should not exceed the total point count ({}).".format(num, total_points)
        )
    return num


def _average_z(pts):
    """Average Z of a point list, used as a layer's reference height."""
    return sum(p.Z for p in pts) / float(len(pts))


def _group_into_layers(groups, tol):
    """Group consecutive point lists into layers by their average Z.

    Curves are assumed to arrive in slicer order (bottom-up): a new layer
    starts as soon as a curve's average Z differs from the current layer's
    reference Z by more than tol. Curves that share a height but are not
    consecutive therefore end up in separate layers, which keeps the emitted
    order identical to the input order.
    """
    layers = []
    current = []
    reference_z = None
    for pts in groups:
        z = _average_z(pts)
        if reference_z is None:
            reference_z = z
            current = [pts]
        elif abs(z - reference_z) <= tol:
            current.append(pts)
        else:
            layers.append(current)
            current = [pts]
            reference_z = z
    if current:
        layers.append(current)
    return layers


def _require_kukalib_features(krl_obj):
    """Fail loudly when Rhino has an older kukalib cached in memory.

    The layer/standby structure needs the subprogram + declaration API added to
    kukalib; without it the script would emit a program with no END and no
    layer subprograms.
    """
    missing = [
        attr
        for attr in ("add_declaration", "start_subprogram", "end_subprogram",
                     "full_program", "add_line", "lin_rel", "wait_for_input")
        if not hasattr(krl_obj, attr)
    ]
    if missing:
        raise Exception(
            "kukalib is out of date (missing {}). Restart Rhino so the updated "
            "libs/kukalib.py is reloaded.".format(", ".join(missing))
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
STANDBY_IN = globals().get("STANDBY_IN", None)
RESUME_IN = globals().get("RESUME_IN", None)
REDO_IN = globals().get("REDO_IN", None)
SAFE_Z = globals().get("SAFE_Z", None)
LAYER_TOL = globals().get("LAYER_TOL", None)
FADE_OUT = globals().get("FADE_OUT", None)

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

STANDBY_IN = _validate_digital_input("STANDBY_IN", STANDBY_IN, DEFAULT_STANDBY_IN)
RESUME_IN = _validate_digital_input("RESUME_IN", RESUME_IN, DEFAULT_RESUME_IN)
REDO_IN = _validate_digital_input("REDO_IN", REDO_IN, DEFAULT_REDO_IN)
if len({STANDBY_IN, RESUME_IN, REDO_IN}) != 3:
    raise Exception("STANDBY_IN, RESUME_IN and REDO_IN must be three different inputs.")
SAFE_Z = _validate_safe_z(SAFE_Z)
LAYER_TOL = _validate_layer_tol(LAYER_TOL, layer_height)

name = name + "_" + timestamp
krl = kl.KukaKRL(name)
_require_kukalib_features(krl)

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


# Coerce every curve/point group up front so layers can be detected before any
# KRL is emitted - the dispatcher needs to know the layer count.
coerced_groups = []
for g, group in enumerate(point_groups):
    label = "CRVS[{}] point {{}}".format(g) if using_curves else "PTS[{}]"
    coerced_groups.append([_coerce_point(label.format(k), p) for k, p in enumerate(group)])

print_layers = _group_into_layers(coerced_groups, LAYER_TOL)
layer_count = len(print_layers)

# Total deposited points that will actually be counted by point_index below:
# every group's first point is a travel move when it isn't the very first
# group overall, so it isn't counted there (see the emission loop).
_total_groups = sum(len(layer) for layer in print_layers)
_total_raw_points = sum(len(gp) for layer in print_layers for gp in layer)
total_points = _total_raw_points - max(0, _total_groups - 1)
FADE_OUT = _validate_fade_out(FADE_OUT, total_points)
fade_out_start_index = total_points - FADE_OUT

# Standby / layer-recovery declarations. These have to sit in the declaration
# block right after DEF, so they are spliced in rather than appended.
krl.add_declaration("DECL INT LAYER_N")
krl.add_declaration("DECL INT LAYER_COUNT")
krl.add_declaration(
    "GLOBAL INTERRUPT DECL {} WHEN $IN[{}]==TRUE DO IR_STANDBY ( )".format(
        STANDBY_INT, STANDBY_IN)
)

# Captured after the declarations above, since those are spliced into the code
# list and would otherwise shift this index.
material_comment_index = len(krl.code)  # material estimate comments are spliced in here later

krl.open_fold(
    "PRINT SETUP - {} LAYER(S), STANDBY $IN[{}], RESUME $IN[{}], REDO $IN[{}]".format(
        layer_count, STANDBY_IN, RESUME_IN, REDO_IN)
)
krl.add_line("LAYER_COUNT = {}".format(layer_count))
krl.add_line("$FLAG[{}]=FALSE".format(FLAG_ABORT))
krl.add_line("$FLAG[{}]=FALSE".format(FLAG_REENTRY))
krl.close_fold()

# Lead-in: approach the start point with the extruder already running, exactly
# as before. The layer subprograms take over from here.
lead_in_vel = _point_velocity(0)
krl.code.append(";FOLD LIN SPEED IS {} m/sec, INTERPOLATION SETTINGS IN FOLD".format(lead_in_vel))
krl.code.append("$VEL.CP={}".format(lead_in_vel))
krl.code.append("$ADVANCE=3")
krl.code.append(";ENDFOLD")
krl.set_output(EXTRUDER_OUT, EXTRUDER_ON) # Start the extruder
plane = (firstpt[0],firstpt[1],firstpt[2]+ zero, 0, 0, 0 )
krl.lin(plane)
previewpts.append(firstpt)

# Layer dispatcher. Each pass runs one layer, then decides whether to park in
# standby, repeat the layer, or move on to the next one.
krl.open_fold("LAYER LOOP")
krl.add_line("LAYER_N = 1")
krl.add_line("REPEAT")
krl.add_line("  $FLAG[{}]=FALSE".format(FLAG_ABORT))
krl.add_line("  INTERRUPT ON {}".format(STANDBY_INT))
krl.add_line("  SWITCH LAYER_N")
for layer_index in range(layer_count):
    krl.add_line("  CASE {}".format(layer_index + 1))
    krl.add_line("    L{:03d} ( )".format(layer_index + 1))
krl.add_line("  ENDSWITCH")
krl.add_line("  INTERRUPT OFF {}".format(STANDBY_INT))
krl.add_line("  IF $FLAG[{}]==TRUE THEN".format(FLAG_ABORT))
krl.add_comment("    aborted part-way through: park, then print this layer again")
krl.add_line("    STANDBY ( )")
krl.add_line("  ELSE")
krl.add_line("    IF $IN[{}]==TRUE THEN".format(STANDBY_IN))
krl.add_comment("    standby asked for after a completed layer")
krl.add_line("      STANDBY ( )")
krl.add_line("      IF $IN[{}]==FALSE THEN".format(REDO_IN))
krl.add_line("        LAYER_N = LAYER_N + 1")
krl.add_line("      ENDIF")
krl.add_line("    ELSE")
krl.add_line("      LAYER_N = LAYER_N + 1")
krl.add_line("    ENDIF")
krl.add_line("  ENDIF")
krl.add_line("UNTIL LAYER_N > LAYER_COUNT")
krl.close_fold()

krl.set_output(EXTRUDER_OUT, EXTRUDER_OFF) # Stop the extruder

# --- local subprograms ------------------------------------------------------

krl.start_subprogram("IR_STANDBY")
krl.add_comment("Standby request while a layer is running: stop the extruder, abort")
krl.add_comment("the move in progress and drop back to the layer loop in the main")
krl.add_comment("program. RESUME returns to the level the interrupt was declared in.")
krl.set_output(EXTRUDER_OUT, EXTRUDER_OFF)
krl.add_line("$FLAG[{}]=TRUE".format(FLAG_ABORT))
krl.add_line("BRAKE")
krl.add_line("RESUME")
krl.end_subprogram()

krl.start_subprogram("STANDBY")
krl.add_line("DECL E6POS PPARK")
krl.add_comment("Lift clear of the part, park over the start point and keep the")
krl.add_comment("extruder running there so material keeps moving while the operator")
krl.add_comment("clears the failed layer.")
krl.set_output(EXTRUDER_OUT, EXTRUDER_OFF)
krl.set_velocity(TRAVEL_VEL)
krl.lin_rel(z=SAFE_Z)
krl.add_comment("keep the height reached above, only move over the start point")
krl.add_line("PPARK = $POS_ACT")
krl.add_line("PPARK.X = {:.1f}".format(firstpt[0]))
krl.add_line("PPARK.Y = {:.1f}".format(firstpt[1]))
krl.add_line("LIN PPARK")
krl.add_comment("purge here until the operator clears the standby request")
krl.set_output(EXTRUDER_OUT, EXTRUDER_ON)
krl.wait_for_input(STANDBY_IN, False)
krl.wait_for_input(RESUME_IN, True)
krl.set_output(EXTRUDER_OUT, EXTRUDER_OFF)
krl.add_line("$FLAG[{}]=FALSE".format(FLAG_ABORT))
krl.add_comment("the next layer must come in from above instead of straight across")
krl.add_line("$FLAG[{}]=TRUE".format(FLAG_REENTRY))
krl.end_subprogram()

point_index = 0
lastpt = None
layer_infos = []
extruder_state = EXTRUDER_ON  # matches the lead-in state (line ~796)


def _in_fade_out_zone(index):
    return FADE_OUT > 0 and index >= fade_out_start_index


def _set_extruder(state):
    """Emit $OUT[3] only when the extruder state actually changes."""
    global extruder_state
    if state != extruder_state:
        krl.set_output(EXTRUDER_OUT, state)
        extruder_state = state

for layer_index, layer_groups in enumerate(print_layers):
    sub_name = "L{:03d}".format(layer_index + 1)
    layer_start = layer_groups[0][0]
    layer_z = layer_start.Z + zero
    layer_point_count = sum(len(gp) for gp in layer_groups)

    krl.start_subprogram(sub_name)
    krl.open_fold("LAYER {} OF {} - Z {:.1f} - {} SEGMENT(S), {} POINT(S)".format(
        layer_index + 1, layer_count, layer_z, len(layer_groups), layer_point_count))

    # Re-entry approach, skipped entirely on the normal sequential path.
    krl.add_line("IF $FLAG[{}]==TRUE THEN".format(FLAG_REENTRY))
    krl.add_comment("  coming back from standby: drop in from above this layer's start")
    krl.add_line("  $OUT[{}]={}".format(EXTRUDER_OUT, _krl_bool(EXTRUDER_OFF)))
    krl.add_line("  $VEL.CP={}".format(TRAVEL_VEL))
    krl.add_line("  LIN {{X {:.1f}, Y {:.1f}, Z {:.1f}, A 0.00, B 0.00, C 0.00, E1 0, E2 0}}".format(
        layer_start.X, layer_start.Y, layer_z + SAFE_Z))
    krl.add_line("  $FLAG[{}]=FALSE".format(FLAG_REENTRY))
    if layer_index == 0:
        # The first layer is normally entered from the lead-in with the
        # extruder already running, so match that state here.
        krl.add_line("  $OUT[{}]={}".format(EXTRUDER_OUT, _krl_bool(EXTRUDER_ON)))
    krl.add_line("ENDIF")

    # Reset per layer so every layer re-states its own feedrate; a layer that
    # inherited $VEL.CP from its predecessor would run at the wrong speed when
    # re-entered on its own.
    lastvel = None

    for group_index, group_pts in enumerate(layer_groups):
        if layer_index > 0 or group_index > 0:
            # Travel move to the next curve: stop the extruder, move to its
            # start point, then resume before depositing along it - unless
            # we're already in the fade-out tail, which stays dry.
            _set_extruder(EXTRUDER_OFF) # Stop the extruder for the travel move
            travel_pt = group_pts[0]
            krl.lin([travel_pt.X, travel_pt.Y, travel_pt.Z+zero, 0, 0, 0])
            previewpts.append(travel_pt)
            _set_extruder(EXTRUDER_OFF if _in_fade_out_zone(point_index) else EXTRUDER_ON)
            lastpt = travel_pt
            group_pts = group_pts[1:] # already reached via the travel move above

        for pt in group_pts:
            vel = _point_velocity(point_index)
            if vel != lastvel:
                krl.set_velocity(vel)
            _set_extruder(EXTRUDER_OFF if _in_fade_out_zone(point_index) else EXTRUDER_ON)
            krl.lin([pt.X, pt.Y, pt.Z+zero, 0, 0, 0])
            previewpts.append(pt)
            lastvel = vel
            lastpt = pt
            point_index += 1

    krl.close_fold()
    krl.end_subprogram()

    layer_infos.append({
        "name": sub_name,
        "z": layer_z,
        "segments": len(layer_groups),
        "points": layer_point_count,
    })

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

# Layer map, so the layer number on the pendant can be matched to a subprogram
# and a height.
layer_summary_lines = [
    "{} layer(s), grouped by Z within {:.2f} mm".format(layer_count, LAYER_TOL),
    "standby $IN[{}], resume $IN[{}], redo $IN[{}], clearance {:.1f} mm".format(
        STANDBY_IN, RESUME_IN, REDO_IN, SAFE_Z),
]
for i, info in enumerate(layer_infos):
    layer_summary_lines.append(
        "LAYER_N {:>4}  {}  Z {:8.1f}  {} segment(s)  {} point(s)".format(
            i + 1, info["name"], info["z"], info["segments"], info["points"])
    )
layers = "\n".join(layer_summary_lines)


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
print(layers)

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

# full_program() closes the main program with END and appends the local
# subprograms (IR_STANDBY, STANDBY, L001...), so the panel shows what gets saved.
a = krl.full_program()
