#! python

"""KUKA KRL program assembly for Grasshopper / Rhino scripts.

The module has no Rhino import at module level, so it also runs (and is tested) in plain CPython.
Only object ids (GUIDs) need Rhino; they are coerced with rhinoscriptsyntax when one is passed.

Motions (KukaKRL methods). Cartesian targets are 6 numbers (X, Y, Z in mm, A, B, C in degrees):

    ptp(axes)                     PTP to joint angles A1..A6
    ptp_pose(pose, status, turn)  PTP to a Cartesian pose, with optional Status / Turn
    ptp_rel(a1=.., a6=..)         PTP_REL, joint offsets
    lin(pose)                     LIN
    lin_rel(x, y, z, a=, b=, c=)  LIN_REL in #BASE or #TOOL
    circ(aux, end, ca)            CIRC through an auxiliary point, optional circular angle CA
    circ_rel(aux, end, ca)        CIRC_REL, offsets from the current position
    start_spline() spl() slin() scirc() end_spline()   SPLINE block (KR C4 / KSS 8.x only)

Motion settings: set_velocity ($VEL.CP), set_acceleration ($ACC.CP), set_approximation ($APO),
set_orientation_mode ($ORI_TYPE / $CIRC_TYPE), set_ptp_velocity, set_ptp_acceleration.

Rhino geometry to KRL tuples (module functions):

    pose(item)        Rhino Plane, Point3d, point object id, (x, y, z) or (x, y, z, a, b, c)
                      -> (X, Y, Z, A, B, C)
    poses(items)      the same for a list
    arc_poses(arc)    Rhino Arc, arc-shaped curve or three points -> (start, aux, end) for circ()
    abc_from_axes(x, y) / axes_from_abc(a, b, c)   KUKA A, B, C <-> frame axes

Example in a Grasshopper Script component (planes P, an arc ARC):

    import kukalib as kl
    krl = kl.KukaKRL("demo")
    krl.set_tool(6)
    krl.set_base(1)
    krl.krl_header((0, -90, 90, 0, 0, 0), ptp_velocity_percent=10)
    krl.set_velocity(0.05)
    for target in kl.poses(P):
        krl.lin(target)
    start, aux, end = kl.arc_poses(ARC)
    krl.lin(start)
    krl.circ(aux, end)
    a = krl.full_program()
"""

__author__ = "jose hernandez vargas"
__version__ = "2026-11-19"

import math
import os
import re
import warnings

import iolib as io

KRL_NAME_MAX = 24  # KRL names (DEF, module = file name without extension) are at most 24 characters
MAX_IO_NUMBER = 4096  # conservative digital I/O range, the same as scripts/kuka.py checks
CP_APPROX = ("C_DIS", "C_VEL", "C_ORI")  # approximation of path motions (LIN, CIRC)
PTP_APPROX = ("C_PTP",) + CP_APPROX  # PTP needs C_PTP; a path criterion may follow for a next LIN/CIRC
SPLINE_APPROX = ("C_SPL",)
ORI_TYPES = ("VAR", "CONSTANT", "JOINT")  # $ORI_TYPE
CIRC_TYPES = ("BASE", "PATH")  # $CIRC_TYPE
_POSE_HINT = " Convert Rhino points and planes with kukalib.pose()."
_GUID = re.compile(r"^\{?[0-9A-Fa-f]{8}(-[0-9A-Fa-f]{4}){3}-[0-9A-Fa-f]{12}\}?$")


def krl_name(name):
    """Return `name` as a valid KRL identifier: ASCII letters, digits and underscores, not starting
    with a digit, at most 24 characters. The controller rejects a program whose DEF name breaks
    these rules or differs from its file name."""
    text = re.sub(r"[^A-Za-z0-9_]", "_", str(name).strip())
    if not text:
        text = "PROGRAM"
    if text[0].isdigit():
        text = "P" + text
    return text[:KRL_NAME_MAX]


def _krl_real(value):
    """A REAL literal in plain decimal notation. Python's str() switches to exponent notation for
    small values (5e-05), which KRL does not read; this keeps 0.1 -> '0.1' and 2 -> '2'."""
    text = "{:.6f}".format(float(value)).rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def _krl_num(value):
    """A number as str() writes it (so existing programs stay byte-identical), except that exponent
    notation such as 1e-05 is written as a plain decimal."""
    text = str(value)
    return _krl_real(value) if "e" in text.lower() else text


def _finite(value, label):
    """Return value as a float, or raise when it is not a finite number (bool is not a number)."""
    if not io.is_number(value) or not math.isfinite(value):
        raise io.ValidationError(f"{label} must be a finite number, got {value!r}.")
    return float(value)


def _numbers(values, count, label, names, hint=""):
    """Return `values` as a tuple of `count` finite numbers. The original objects are kept, so a
    caller that formats them with str() writes exactly what it wrote before."""
    try:
        items = None if isinstance(values, str) else tuple(values)
    except TypeError:
        items = None
    if items is None or len(items) != count or not all(io.is_number(v) and math.isfinite(v) for v in items):
        raise io.ValidationError(f"{label} must be {count} numbers {names}, got {values!r}.{hint}")
    return items


def _pose6(position, label):
    """A Cartesian target: 6 finite numbers as floats."""
    return tuple(float(v) for v in _numbers(position, 6, label, "(X, Y, Z, A, B, C)", _POSE_HINT))


def _whole(value, label, lowest, highest):
    """A whole number in [lowest, highest] (Grasshopper numbers such as 6.0 are accepted)."""
    number = io.validate_scalar(label, value, min_value=lowest, max_value=highest, allow_zero=True)
    if number != int(number):
        raise io.ValidationError(f"{label} must be a whole number, got {value}.")
    return int(number)


def _approx_text(approx, allowed, motion, required=None):
    """The approximation suffix of a motion (' C_DIS'), or '' for exact positioning."""
    if not approx:
        return ""
    tokens = str(approx).upper().split()
    if (any(t not in allowed for t in tokens) or len(set(tokens)) != len(tokens)
            or (required and required not in tokens)):
        need = f", including {required}" if required else ""
        raise io.ValidationError(
            f"{motion} approximation must be one or more of {', '.join(allowed)}{need}; got {approx!r}."
        )
    return " " + " ".join(tokens)


def _enum(value, allowed, label):
    """A KRL ENUM value such as #CONSTANT; accepts 'constant', 'CONSTANT' or '#CONSTANT'."""
    text = str(value).strip().lstrip("#").upper()
    if text not in allowed:
        raise io.ValidationError(f"{label} must be one of {', '.join('#' + a for a in allowed)}; got {value!r}.")
    return "#" + text


def _status_number(value, label, highest):
    """Status (0..7) or Turn (0..63): an integer, or a binary string such as 'B110' / "'B110'"."""
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip().strip("'").upper()
        text = text[1:] if text.startswith("B") else text
        if not text or any(ch not in "01" for ch in text):
            raise io.ValidationError(f"{label} must be an integer or a binary string such as 'B110'; got {value!r}.")
        value = int(text, 2)
    return _whole(value, label, 0, highest)


# Rhino geometry to KRL tuples ----------------------------------------------------------------------
# Duck-typed, so the module needs no Rhino import: a Plane has Origin / XAxis / YAxis, a Point3d or
# Vector3d has X / Y / Z, a Rhino.Geometry.Point has Location, Grasshopper goo has Value.

def _is_sequence(item):
    return not isinstance(item, str) and hasattr(item, "__len__") and hasattr(item, "__getitem__")


def _is_plane(item):
    return all(hasattr(item, k) for k in ("Origin", "XAxis", "YAxis"))


def _is_point(item):
    return hasattr(item, "Location") or all(hasattr(item, k) for k in ("X", "Y", "Z"))


def _is_guid(item):
    if type(item).__name__ == "Guid":  # System.Guid
        return True
    return isinstance(item, str) and bool(_GUID.match(item.strip()))


def _unwrap(item):
    """Grasshopper goo (GH_Point, GH_Plane, GH_Arc, ...) carries its geometry in .Value."""
    if item is not None and hasattr(item, "Value") and not _is_plane(item) and not _is_point(item):
        return item.Value
    return item


def _rhinoscript():
    try:
        import rhinoscriptsyntax as rs
    except ImportError:
        raise io.ValidationError("Object ids can only be converted inside Rhino; pass the geometry instead.")
    return rs


def _vector(item, label):
    """(x, y, z) of a Point3d / Vector3d or a 3-number sequence, as floats."""
    if all(hasattr(item, k) for k in ("X", "Y", "Z")):
        return tuple(_finite(v, label) for v in (item.X, item.Y, item.Z))
    return tuple(float(v) for v in _numbers(item, 3, label, "(x, y, z)"))


def _point(item, label):
    """(x, y, z) of a Point3d, a Rhino.Geometry.Point, a point object id or a 3-number sequence."""
    if hasattr(item, "Location"):
        item = item.Location
    if _is_guid(item):
        point = _rhinoscript().coerce3dpoint(item)
        if point is None:
            raise io.ValidationError(f"{label}: object {item} is not a point.")
        item = point
    return _vector(item, label)


def _sub(p, q):
    return (p[0] - q[0], p[1] - q[1], p[2] - q[2])


def _cross(u, v):
    return (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])


def _norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(v, label):
    length = _norm(v)
    if length < 1e-12:
        raise io.ValidationError(f"{label} has zero length.")
    return (v[0] / length, v[1] / length, v[2] / length)


def _clean(value, digits=9):
    """Round away float noise and turn -0.0 into 0.0, so outputs read 0.00 and not -0.00."""
    return round(value, digits) + 0.0


def _angle(degrees):
    """An angle in degrees, cleaned and in (-180, 180]."""
    degrees = _clean(degrees)
    return degrees + 360.0 if degrees <= -180.0 else degrees


def _abc(x, y, z):
    """KUKA A, B, C (degrees) of the rotation whose columns are the unit axes x, y, z.

    KUKA uses R = Rz(A) * Ry(B) * Rx(C): A about Z, then B about the new Y, then C about the new X.
    B stays in [-90, 90]; at B = +-90 (tool X axis vertical) C is set to 0.
    """
    cos_b = math.hypot(x[0], x[1])
    b = math.atan2(-x[2], cos_b)
    if cos_b > 1e-7:
        a = math.atan2(x[1], x[0])
        c = math.atan2(y[2], z[2])
    else:
        a = math.atan2(-y[0], y[1])
        c = 0.0
    return tuple(_angle(math.degrees(v)) for v in (a, b, c))


def abc_from_axes(xaxis, yaxis):
    """KUKA A, B, C (degrees) of a frame given by its X and Y axes (Vector3d or 3 numbers).

    The axes need not be unit length or exactly perpendicular: Z = X x Y and Y = Z x X, as a Rhino
    Plane does.
    """
    x = _unit(_vector(xaxis, "X axis"), "X axis")
    z = _unit(_cross(x, _vector(yaxis, "Y axis")), "X x Y (the axes are parallel or one)")
    return _abc(x, _cross(z, x), z)


def axes_from_abc(a, b, c):
    """The unit X, Y and Z axes of KUKA angles A, B, C (degrees), as three (x, y, z) tuples."""
    a, b, c = (math.radians(_finite(v, "Angle")) for v in (a, b, c))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    x = (ca * cb, sa * cb, -sb)
    y = (ca * sb * sc - sa * cc, sa * sb * sc + ca * cc, cb * sc)
    z = (ca * sb * cc + sa * sc, sa * sb * cc - ca * sc, cb * cc)
    return x, y, z


def _flipped(abc):
    """A, B, C turned 180 degrees about their own X axis (Y and Z reversed)."""
    x, y, z = axes_from_abc(*abc)
    return _abc(x, (-y[0], -y[1], -y[2]), (-z[0], -z[1], -z[2]))


def pose(item, orientation=(0.0, 0.0, 0.0), flip=False, scale=1.0):
    """A KRL Cartesian target (X, Y, Z, A, B, C) from Rhino geometry, ready for lin(), circ() etc.

    item:
        Plane     position = origin; the tool axes follow the plane axes (plane X -> tool X,
                  plane Y -> tool Y, plane Z -> tool Z), so WorldXY gives A 0, B 0, C 0.
        Point3d, Rhino.Geometry.Point, a point object id, or (x, y, z)
                  position = the point, orientation = `orientation`.
        (x, y, z, a, b, c)   returned as floats (flip and scale still apply).
    orientation: A, B, C in degrees for point input (default 0, 0, 0: the tool frame parallel to
        the base frame, which is what scripts/kuka.py writes).
    flip: turn the orientation 180 degrees about its own X axis (Y and Z reversed). Use it when
        the tool's Z axis must point into the part while the planes' Z points out of it:
        WorldXY gives A 0, B 0, C 180.
    scale: multiplies X, Y, Z, e.g. 1000 for a Rhino model in metres (KRL is in mm).

    Positions are in the coordinates of the Rhino model; they must already be expressed in the
    KUKA base used by the program (set_base), as kuka.py's points are.
    """
    scale = _finite(scale, "scale")
    if scale <= 0:
        raise io.ValidationError(f"scale must be > 0, got {scale}.")
    item = _unwrap(item)
    if item is None:
        raise io.ValidationError("pose() needs a plane, a point or numbers, got None.")
    if _is_plane(item):
        xyz = _point(item.Origin, "Plane origin")
        abc = abc_from_axes(item.XAxis, item.YAxis)
    elif _is_sequence(item) and len(item) == 6:
        values = _pose6(item, "Pose")
        xyz, abc = values[:3], values[3:]
    else:
        xyz = _point(item, "Point")
        abc = tuple(float(v) for v in _numbers(orientation, 3, "orientation", "(A, B, C)"))
    if flip:
        abc = _flipped(abc)
    return tuple(_clean(v * scale) for v in xyz) + tuple(abc)


def poses(items, orientation=(0.0, 0.0, 0.0), flip=False, scale=1.0):
    """pose() for every item of a list (planes, points, ids, number tuples or a Polyline)."""
    items = _unwrap(items)
    if items is None:
        raise io.ValidationError("poses() needs a list of planes or points, got None.")
    if _is_plane(items) or _is_point(items) or _is_guid(items):
        return [pose(items, orientation, flip, scale)]
    if _is_sequence(items) and len(items) and all(io.is_number(v) for v in items):
        raise io.ValidationError("poses() takes a list of positions; use pose() for a single one.")
    result = []
    for index, item in enumerate(items):
        try:
            result.append(pose(item, orientation, flip, scale))
        except io.ValidationError as error:
            raise io.ValidationError(f"Item {index}: {error}")
    return result


def arc_poses(arc, orientation=(0.0, 0.0, 0.0), flip=False, scale=1.0, tolerance=0.01):
    """(start, aux, end) poses of an arc, for: krl.lin(start); krl.circ(aux, end).

    arc: a Rhino.Geometry.Arc, a curve that is an arc within `tolerance` (model units; ArcCurve,
        NURBS arc, curve id), or three points (start, a point on the arc, end). The auxiliary point
        is the arc's midpoint. The tool keeps `orientation` (see pose()) at all three points.
    A closed circle is refused: use circ(aux, end, ca=360) with two other points on the circle, or
    split it into two arcs.
    """
    arc = _unwrap(arc)
    if _is_guid(arc):
        curve = _rhinoscript().coercecurve(arc)
        if curve is None:
            raise io.ValidationError(f"Object {arc} is not a curve.")
        arc = curve
    if all(hasattr(arc, k) for k in ("StartPoint", "MidPoint", "EndPoint")):  # Rhino.Geometry.Arc
        points = (arc.StartPoint, arc.MidPoint, arc.EndPoint)
    elif hasattr(arc, "IsArc") and hasattr(arc, "PointAtNormalizedLength"):  # a Rhino curve
        if not arc.IsArc(float(tolerance)):
            raise io.ValidationError(f"The curve is not an arc within a tolerance of {tolerance}.")
        points = (arc.PointAtStart, arc.PointAtNormalizedLength(0.5), arc.PointAtEnd)
    elif _is_sequence(arc) and len(arc) == 3:
        points = tuple(arc)
    else:
        raise io.ValidationError(f"arc_poses() needs an arc, an arc curve or three points, got {arc!r}.")
    start, aux, end = (pose(p, orientation, flip, scale) for p in points)
    if _norm(_sub(end[:3], start[:3])) < 1e-6:
        raise io.ValidationError(
            "The arc is closed (start = end). Use circ(aux, end, ca=360) with two other points on "
            "the circle, or split the circle into two arcs."
        )
    u, v = _sub(aux[:3], start[:3]), _sub(end[:3], start[:3])
    if _norm(_cross(u, v)) <= 1e-9 * max(1.0, _norm(u) * _norm(v)):
        raise io.ValidationError("The three arc points lie on one line, so they define no circle.")
    return start, aux, end


class KukaKRL:

    MAX_LIN_VELOCITY_MPS = 0.25  # hardcoded safety cap: 250 mm/s, LIN moves must never exceed this
    PTP_WARN_PERCENT = 10.0  # hardcoded safety threshold: warn above this PTP speed
    PTP_MAX_PERCENT = 25.0  # hardcoded safety cap: PTP speed must never exceed this
    XYZ_DECIMALS = 1  # decimals of X, Y, Z in Cartesian targets (mm)
    ABC_DECIMALS = 2  # decimals of A, B, C in Cartesian targets (degrees)

    def __init__(self, name):
        clean = krl_name(name)
        if clean != str(name):
            warnings.warn(
                f'Program name "{name}" is not a valid KRL name; using "{clean}".', UserWarning
            )
        self.name = clean
        self.base = None
        self.tool = None
        self.code = []
        # Local subprograms, each a full "DEF x ( ) ... END" block. They are
        # written after the main program's END - see full_program().
        self.subprograms = []
        self._target = None  # open subprogram line list, or None for the main program
        self._decl_index = None  # where KRL declarations get spliced (set by krl_header)
        self._spline_open = False  # between start_spline() and end_spline()
        self._spline_segments = 0

    def _emit(self, line: str):
        """Append a line to the open subprogram, or to the main program."""
        if self._target is None:
            self.code.append(line)
        else:
            self._target.append(line)

    def _no_spline(self, what):
        """A SPLINE block may only hold SPL / SLIN / SCIRC segments."""
        if self._spline_open:
            raise Exception(f"{what} is not allowed inside a SPLINE block; call end_spline() first.")

    def _pose_text(self, pose, status=None, turn=None, external=True):
        """A Cartesian target aggregate: {X 1.0, Y 2.0, Z 3.0, A 0.00, B 0.00, C 0.00, E1 0, E2 0}."""
        x, y, z, a, b, c = pose
        xd, ad = self.XYZ_DECIMALS, self.ABC_DECIMALS
        text = f"{{X {x:.{xd}f}, Y {y:.{xd}f}, Z {z:.{xd}f}, A {a:.{ad}f}, B {b:.{ad}f}, C {c:.{ad}f}"
        if status is not None:
            text += f", S {status}"
        if turn is not None:
            text += f", T {turn}"
        if external:
            text += ", E1 0, E2 0"
        return text + "}"

    def _xyz_text(self, xyz):
        """An auxiliary point: {X 1.0, Y 2.0, Z 3.0}. The controller ignores its orientation."""
        xd = self.XYZ_DECIMALS
        return f"{{X {xyz[0]:.{xd}f}, Y {xyz[1]:.{xd}f}, Z {xyz[2]:.{xd}f}}}"

    def _rel_text(self, values, label):
        """A relative target with its non-zero components: {X 10.0, C 5.00}."""
        names = "XYZABC"[:len(values)]
        parts = []
        for name, value in zip(names, values):
            value = _finite(value, f"{label} {name}")
            if value:
                decimals = self.XYZ_DECIMALS if name in "XYZ" else self.ABC_DECIMALS
                parts.append(f"{name} {value:.{decimals}f}")
        if not parts:
            raise ValueError(f"{label} needs at least one non-zero offset.")
        return "{" + ", ".join(parts) + "}"

    @staticmethod
    def _ca_text(ca):
        """', CA 90' for a circular angle in degrees (sign = direction), or ''."""
        if ca is None:
            return ""
        ca = _finite(ca, "Circular angle CA")
        if ca == 0:
            raise io.ValidationError("Circular angle CA must not be 0.")
        return f", CA {_krl_real(ca)}"

    def _circle_points(self, aux, target, motion):
        """Validated auxiliary point (3 or 6 numbers, only X, Y, Z used) and end pose of a circle."""
        values = _numbers(aux, len(aux) if _is_sequence(aux) and len(aux) in (3, 6) else 3,
                          f"{motion} auxiliary point", "(X, Y, Z) or (X, Y, Z, A, B, C)", _POSE_HINT)
        aux_xyz = tuple(float(v) for v in values[:3])
        end = _pose6(target, f"{motion} end point")
        if self._xyz_text(aux_xyz) == self._xyz_text(end[:3]):
            raise io.ValidationError(f"{motion} auxiliary point and end point are the same point.")
        return aux_xyz, end

    def add_line(self, line: str):
        """Append a raw KRL line to the current target (main program or subprogram)."""
        self._emit(line)

    def add_declaration(self, line: str):
        """Insert a declaration into the main program's declaration block.

        KRL requires every declaration (DECL, INTERRUPT DECL) to appear before
        the first executable instruction, so these are spliced in right after
        the "DEF <name> ( )" line rather than appended. Declarations are kept
        in call order.
        """
        if self._decl_index is None:
            raise Exception("Call krl_header() before adding declarations.")
        self.code.insert(self._decl_index, line)
        self._decl_index += 1

    def start_subprogram(self, name: str):
        """Open a local subprogram; every emitter call is routed into it until
        end_subprogram() is called."""
        self._no_spline("A subprogram")
        if self._target is not None:
            raise Exception("A subprogram is already open; call end_subprogram() first.")
        if not isinstance(name, str) or not name.strip():
            raise Exception("Subprogram name should be a non-empty string.")
        if krl_name(name) != name:
            # Not renamed silently: the calls to it are written elsewhere with this exact name.
            raise Exception(
                f'Subprogram name "{name}" is not a valid KRL name (letters, digits, "_", '
                f"not starting with a digit, at most {KRL_NAME_MAX} characters)."
            )
        self._target = [f"DEF {name} ( )"]
        self.subprograms.append(self._target)

    def end_subprogram(self):
        """Close the open subprogram with its END and go back to the main program."""
        if self._target is None:
            raise Exception("No subprogram is open.")
        self._no_spline("END")
        self._target.append("END")
        self._target = None

    def full_program(self):
        """Assemble main program + END + local subprograms.

        Does not mutate self.code, so it is safe to call more than once.
        """
        if self._target is not None:
            raise Exception("A subprogram is still open; call end_subprogram() first.")
        self._no_spline("END")
        lines = list(self.code)
        lines.append("END")
        for block in self.subprograms:
            lines.append("")
            lines.extend(block)
        return lines

    def open_fold(self, comment: str):
        """Opens a fold with a comment."""
        self._emit(f";FOLD {comment}")

    def close_fold(self):
        """Closes a fold."""
        self._emit(";ENDFOLD")

    def add_comment(self, text: str):
        """Add a comment to the code."""
        self._emit(f";{text}")

    # motion settings ---------------------------------------------------------------------------
    def set_velocity(self, velocity: float):
        """Set the LIN velocity in m/s. Hard-capped at MAX_LIN_VELOCITY_MPS for safety."""
        self._no_spline("$VEL.CP")
        velocity = io.validate_scalar("LIN velocity", velocity, min_value=0, allow_zero=False)
        if velocity > self.MAX_LIN_VELOCITY_MPS:
            warnings.warn(
                f"LIN velocity {velocity} m/s exceeds the hardcoded "
                f"{self.MAX_LIN_VELOCITY_MPS * 1000:.0f} mm/s safety cap; clamping.",
                UserWarning,
            )
            velocity = self.MAX_LIN_VELOCITY_MPS
        self._emit(f"$VEL.CP={_krl_real(velocity)}")

    def set_acceleration(self, acceleration: float):
        """Set the path acceleration of LIN / CIRC / spline motions in m/s^2 ($ACC.CP).

        The controller stops with an error above the robot's maximum ($ACC_MA.CP).
        """
        self._no_spline("$ACC.CP")
        acceleration = io.validate_scalar("Path acceleration", acceleration, min_value=0, allow_zero=False)
        self._emit(f"$ACC.CP={_krl_real(acceleration)}")

    def set_approximation(self, distance=None, velocity=None, orientation=None, ptp=None):
        """Set the approximation criteria used by C_DIS / C_VEL / C_ORI / C_PTP motions.

        distance: $APO.CDIS in mm, for C_DIS.
        velocity: $APO.CVEL in percent (1-100), for C_VEL.
        orientation: $APO.CORI in degrees, for C_ORI.
        ptp: $APO.CPTP in percent (1-100), for C_PTP.
        """
        self._no_spline("$APO")
        lines = []
        if distance is not None:
            distance = io.validate_scalar("Approximation distance", distance, min_value=0, allow_zero=True)
            lines.append(f"$APO.CDIS={_krl_real(distance)}")
        if velocity is not None:
            lines.append(f"$APO.CVEL={_whole(velocity, 'Approximation velocity percent', 1, 100)}")
        if orientation is not None:
            orientation = io.validate_scalar("Approximation orientation", orientation, min_value=0, allow_zero=True)
            lines.append(f"$APO.CORI={_krl_real(orientation)}")
        if ptp is not None:
            lines.append(f"$APO.CPTP={_whole(ptp, 'PTP approximation percent', 1, 100)}")
        if not lines:
            raise ValueError("set_approximation needs at least one criterion.")
        for line in lines:
            self._emit(line)

    def set_orientation_mode(self, ori_type=None, circ_type=None):
        """Set how the tool orientation is guided along path motions.

        ori_type ($ORI_TYPE): "VAR" (default on the robot: orientation blends from start to end),
            "CONSTANT" (keeps the start orientation), "JOINT" (wrist-axis interpolation).
        circ_type ($CIRC_TYPE, CIRC only): "BASE" (orientation relative to the base) or "PATH"
            (orientation relative to the circle, e.g. a tool that must stay tangent).
        """
        self._no_spline("$ORI_TYPE / $CIRC_TYPE")
        if ori_type is None and circ_type is None:
            raise ValueError("set_orientation_mode needs ori_type and/or circ_type.")
        if ori_type is not None:
            self._emit(f"$ORI_TYPE={_enum(ori_type, ORI_TYPES, 'ori_type')}")
        if circ_type is not None:
            self._emit(f"$CIRC_TYPE={_enum(circ_type, CIRC_TYPES, 'circ_type')}")

    def set_ptp_velocity(self, percent: float):
        """Set the PTP axis velocity in percent, with the same safety caps as krl_header()."""
        self._no_spline("BAS (#VEL_PTP)")
        percent = self._sanitize_ptp_velocity_percent(percent)
        self._emit("BAS (#VEL_PTP,{:.1f})".format(percent))

    def set_ptp_acceleration(self, percent: float):
        """Set the PTP axis acceleration in percent (0-100]."""
        self._no_spline("BAS (#ACC_PTP)")
        percent = self._validate_percent(percent, "PTP acceleration percent", min_value=0, max_value=100)
        self._emit("BAS (#ACC_PTP,{:.1f})".format(percent))

    def _validate_percent(self, value: float, label: str, min_value: float = 0, max_value: float = 100):
        """Validate percent-like inputs and return a float."""
        return io.validate_scalar(label, value, min_value=min_value, max_value=max_value, allow_zero=False)

    def _sanitize_ptp_velocity_percent(self, value: float):
        """
        Validate PTP speed percent for safer motion.

        Warn above PTP_WARN_PERCENT and hard-cap at PTP_MAX_PERCENT.
        """
        value = self._validate_percent(value, "PTP velocity percent", min_value=0, max_value=100)
        if value > self.PTP_WARN_PERCENT:
            warnings.warn(
                f"PTP velocity is {value}%; values above {self.PTP_WARN_PERCENT:.0f}% "
                "should be double-checked.",
                UserWarning,
            )
        if value > self.PTP_MAX_PERCENT:
            warnings.warn(
                f"PTP velocity {value}% exceeds the hardcoded safety cap; "
                f"clamping to {self.PTP_MAX_PERCENT:.0f}%.",
                UserWarning,
            )
            value = self.PTP_MAX_PERCENT
        return value

    def _validate_tool_or_base_number(self, number, label):
        """Validate a tool/base number is a whole number in the KUKA-supported 1-16 range.

        TOOL_NO / BASE_NO are INT in KRL: a Grasshopper number such as 6.0 would be written as
        "TOOL_NO 6.0" and stop the program from compiling, so the value is returned as an int.
        """
        value = io.validate_scalar(label, number, min_value=1, max_value=16, allow_zero=True)
        if value != int(value):
            raise io.ValidationError(f"{label} must be a whole number, got {number}.")
        return int(value)

    @staticmethod
    def _validate_io_number(number, label):
        """Validate a digital I/O index: a whole number from 1 to MAX_IO_NUMBER."""
        value = io.validate_scalar(label, number, min_value=1, max_value=MAX_IO_NUMBER, allow_zero=True)
        if value != int(value):
            raise io.ValidationError(f"{label} must be a whole number, got {number}.")
        return int(value)

    def set_tool(self, number):
        """Defines the tool number. Accepts an integer between 1-16."""
        self.tool = self._validate_tool_or_base_number(number, "Tool number")

    def set_base(self, number):
        """Defines the base number. Accepts an integer between 1-16."""
        self.base = self._validate_tool_or_base_number(number, "Base number")

    def krl_header(
        self,
        startposition,
        ptp_velocity_percent: float = 20,
        ptp_acc_percent: float = 20,
        ptp_apo_dist: float = 50,
    ):
        """
        Writes a KRL header into self.code
        requires self.tool and self.base to be defined

        Args:
            startposition: Iterable with A1..A6 start joint angles.
            ptp_velocity_percent: PTP speed (%) for PDAT and BAS PTP params.
            ptp_acc_percent: PTP acceleration (%) for PDAT_ACT.
            ptp_apo_dist: Approximation distance for PDAT_ACT.
        """
        if not self.tool or not self.base:
            raise Exception(
                'You need to define a tool and a base first.')
        if self._decl_index is not None:
            raise Exception("krl_header() was already called; a program has exactly one header.")


        if len(startposition) == 6 and all(isinstance(i, (int, float)) for i in startposition):
            A1, A2, A3, A4, A5, A6 = (_krl_num(i) for i in startposition)
        else:
            raise Exception(
                "Start position should be a tuple with angles for each robot axis")
        # An Array that will contain all of the commands
        base = self.base
        tool = self.tool
        ptp_velocity_percent = self._sanitize_ptp_velocity_percent(ptp_velocity_percent)
        ptp_acc_percent = self._validate_percent(
            ptp_acc_percent, "PTP acceleration percent", min_value=0, max_value=100
        )
        if not isinstance(ptp_apo_dist, (int, float)) or ptp_apo_dist < 0:
            raise ValueError("PTP APO distance must be a numeric value >= 0.")

        # header from template
        self.code.append("&ACCESS RVP")
        self.code.append("&REL 1")
        self.code.append("&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe")
        self.code.append("&PARAM EDITMASK = *")

        # add some initial setup stuff
        self.code.append("DEF "+str(self.name)+" ( )")
        # KRL wants all declarations before the first instruction; add_declaration()
        # splices them in here.
        self._decl_index = len(self.code)
        self.code.append(";FOLD INI")
        self.code.append(";FOLD BASISTECH INI")
        self.code.append(
            "GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )")

        """
            INTERRUPT

            Description Executes one of the following actions:
                - Activates an interrupt.
                - Deactivates an interrupt.
                - Disables an interrupt.
                - Enables an interrupt.
            Up to 16 interrupts may be active at any one time

        """
        self.code.append("INTERRUPT ON 3")
        self.code.append("BAS (#INITMOV,0 )")
        self.code.append(";ENDFOLD (BASISTECH INI)")
        self.code.append(";ENDFOLD (INI)")

        self.code.append(";FOLD STARTPOSITION - BASE IS {}, TOOL IS {}, SPEED IS {:.1f}%, POSITION IS A1 {},A2 {},A3 {},A4 {},A5 {},A6 {},E1 0,E2 0,E3 0,E4 0".format(
            base, tool, ptp_velocity_percent, A1, A2, A3, A4, A5, A6))
        self.code.append("$BWDSTART = FALSE")
        self.code.append(
            "PDAT_ACT = {{VEL {vel:.1f},ACC {acc:.1f},APO_DIST {apo:.1f}}}".format(
                vel=ptp_velocity_percent, acc=ptp_acc_percent, apo=ptp_apo_dist
            )
        )
        self.code.append(
            "FDAT_ACT = {{TOOL_NO {},BASE_NO {},IPO_FRAME #BASE}}".format(tool, base))
        self.code.append("BAS (#PTP_PARAMS,{:.1f})".format(ptp_velocity_percent))
        self.code.append("PTP  {{A1 {},A2 {},A3 {},A4 {},A5 {},A6 {},E1 0,E2 0,E3 0,E4 0}}".format(
            A1, A2, A3, A4, A5, A6))
        self.code.append(";ENDFOLD")

        # self.code.append("$APO.CDIS = 0.5000")
        # self.code.append("BAS (#INITMOV,0)")
        # self.code.append("BAS (#VEL_PTP,20)")
        # self.code.append("BAS (#ACC_PTP,20)")
        # self.code.append("")

        """
            Advance run
            The advance run is the maximum number of motion blocks that the robot controller calculates and plans in advance during program execution. The actual
            number is dependent on the capacity of the computer.
            The advance run refers to the current position of the block pointer. It is set via
            the system variable $ADVANCE:
                - Default value: 3
                - Maximum value: 5
            The advance run is required, for example, in order to be able to calculate approximate positioning motions. If $ADVANCE = 0 is set, approximate positioning is not possible.
            Certain statements trigger an advance run stop. These include statements
            that influence the periphery, e.g. OUT statements
        """
        self.code.append("$ADVANCE=3")

    def set_output(self, output_number: int, state: bool):
        """Set the state of a specified output. KRL wants TRUE/FALSE, not Python's
        True/False."""
        self._no_spline("$OUT")
        output_number = self._validate_io_number(output_number, "Output number")
        self._emit(f"$OUT[{output_number}]={'TRUE' if state else 'FALSE'}")

    def wait(self, wait_time: float):
        """Wait for a specified amount of time in seconds (>= 0)."""
        self._no_spline("WAIT SEC")
        wait_time = io.validate_scalar("Wait time", wait_time, min_value=0, allow_zero=True)
        self._emit(f"WAIT SEC {_krl_real(wait_time)}")

    def wait_for_input(self, input_number: int, state: bool = True):
        """Hold the program until a digital input reaches the given state."""
        self._no_spline("WAIT FOR")
        input_number = self._validate_io_number(input_number, "Input number")
        self._emit(f"WAIT FOR $IN[{input_number}]=={'TRUE' if state else 'FALSE'}")

    # motions -----------------------------------------------------------------------------------
    def ptp(self, position: tuple, e1: float = 0, e2: float = 0, approx: str = None):
        """Point-to-point motion to a joint position (A1..A6 axis angles in degrees),
        with optional external axes E1/E2.

        approx: None for exact positioning (default), or "C_PTP" to blend into the next motion
        (needs $APO.CPTP, see set_approximation).
        """
        self._no_spline("PTP")
        axes = _numbers(position, 6, "Position", "(A1, A2, A3, A4, A5, A6)")
        e1, e2 = _numbers((e1, e2), 2, "External axes", "(E1, E2)")
        a1, a2, a3, a4, a5, a6 = (_krl_num(v) for v in axes)
        self._emit(
            f"PTP {{A1 {a1}, A2 {a2}, A3 {a3}, A4 {a4}, A5 {a5}, A6 {a6}, E1 {_krl_num(e1)}, E2 {_krl_num(e2)}, E3 0, E4 0, E5 0, E6 0}}"
            + _approx_text(approx, PTP_APPROX, "PTP", required="C_PTP")
        )

    def ptp_pose(self, pose: tuple, status=None, turn=None, approx: str = None):
        """Point-to-point motion to a Cartesian pose (X, Y, Z in mm, A, B, C in degrees).

        status (S, 0-7) and turn (T, 0-63) pick one of the arm configurations that reach the pose;
        pass them (as integers or 'B110'-style strings) whenever the configuration matters. Without
        them the controller keeps the current configuration, so the result depends on where the
        robot comes from. The first motion of a program must not rely on that: krl_header() starts
        with a PTP to joint angles.
        approx: None (exact) or "C_PTP".
        """
        self._no_spline("PTP")
        values = _pose6(pose, "PTP pose")
        status = _status_number(status, "Status", 7)
        turn = _status_number(turn, "Turn", 63)
        self._emit(f"PTP {self._pose_text(values, status, turn)}"
                   + _approx_text(approx, PTP_APPROX, "PTP", required="C_PTP"))

    def ptp_rel(self, a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0, a6=0.0, approx: str = None):
        """Point-to-point motion by joint offsets in degrees from the current axis position
        (PTP_REL {A1 10, A6 -5}). Only non-zero offsets are written."""
        self._no_spline("PTP_REL")
        parts = []
        for label, value in zip(("A1", "A2", "A3", "A4", "A5", "A6"), (a1, a2, a3, a4, a5, a6)):
            value = _finite(value, f"PTP_REL {label}")
            if value:
                parts.append(f"{label} {_krl_real(value)}")
        if not parts:
            raise ValueError("ptp_rel needs at least one non-zero axis offset.")
        self._emit("PTP_REL {" + ", ".join(parts) + "}" + _approx_text(approx, PTP_APPROX, "PTP_REL", "C_PTP"))

    def lin(self, position: tuple, approx: str = "C_DIS"):
        """Linear motion to a Cartesian position (X, Y, Z in mm, A, B, C in degrees).
        External axes E1/E2 are always written as 0. Rhino planes and points: see pose().

        approx is the approximation suffix ("C_DIS", "C_VEL" or "C_ORI"); pass None/"" for exact
        positioning (required, for instance, on the first motion after a RESUME).
        """
        self._no_spline("LIN")
        values = _pose6(position, "Position")
        self._emit(f"LIN {self._pose_text(values)}" + _approx_text(approx, CP_APPROX, "LIN"))

    def lin_rel(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, frame: str = "#BASE",
                a: float = 0.0, b: float = 0.0, c: float = 0.0):
        """Relative linear motion from wherever the robot currently stands.

        x, y, z in mm and a, b, c in degrees; only non-zero values are written. frame "#BASE"
        moves along the base axes, "#TOOL" along the tool axes (e.g. lin_rel(z=-5, frame="#TOOL")
        to plunge along the tool).

        Used when the position is not known at generation time (e.g. lifting
        clear of the part after an interrupt aborted a move mid-path). Always
        exact positioning - LIN_REL is not approximated here.
        """
        self._no_spline("LIN_REL")
        components = self._rel_text((x, y, z, a, b, c), "lin_rel")[1:-1]
        line = f"LIN_REL {{{components}}}"
        if frame:
            line += " " + _enum(frame, ("BASE", "TOOL"), "lin_rel frame")
        self._emit(line)

    def circ(self, aux, target, ca=None, approx: str = "C_DIS"):
        """Circular motion from the current position through `aux` to `target`.

        aux: a point on the circle, (X, Y, Z) or a 6-number pose (its A, B, C are ignored by the
            controller). Start, aux and end must be distinct and not on one line; arc_poses()
            checks this for Rhino arcs.
        target: the end pose (X, Y, Z, A, B, C). With the default $ORI_TYPE #VAR the orientation
            blends from the start to this pose.
        ca: optional circular angle in degrees. The motion then runs CA degrees along the circle
            from the start (negative = the other way round) and only the orientation of `target`
            is used, e.g. ca=360 for a full circle.
        approx: "C_DIS" (default), "C_VEL", "C_ORI", or None for exact positioning.
        """
        self._no_spline("CIRC")
        aux_xyz, end = self._circle_points(aux, target, "CIRC")
        self._emit(f"CIRC {self._xyz_text(aux_xyz)}, {self._pose_text(end)}{self._ca_text(ca)}"
                   + _approx_text(approx, CP_APPROX, "CIRC"))

    def circ_rel(self, aux, target, ca=None, approx: str = None):
        """Circular motion with the auxiliary and end point given as offsets from the current
        position, in the base frame: (x, y, z) or (x, y, z, a, b, c). Exact positioning by default."""
        self._no_spline("CIRC_REL")
        for label, values in (("circ_rel auxiliary point", aux), ("circ_rel end point", target)):
            if not (_is_sequence(values) and len(values) in (3, 6)):
                raise io.ValidationError(f"{label} must be 3 or 6 offsets, got {values!r}.")
        aux_text = self._rel_text(tuple(aux)[:3], "circ_rel auxiliary point")
        end_text = self._rel_text(tuple(target), "circ_rel end point")
        if aux_text == end_text:
            raise io.ValidationError("circ_rel auxiliary point and end point are the same point.")
        self._emit(f"CIRC_REL {aux_text}, {end_text}{self._ca_text(ca)}"
                   + _approx_text(approx, CP_APPROX, "CIRC_REL"))

    # spline blocks (KR C4, KSS 8.x) ------------------------------------------------------------
    def start_spline(self):
        """Open a SPLINE block: SPL / SLIN / SCIRC segments that the controller plans as one
        smooth path at constant $VEL.CP, set before the block (set_velocity). Inside the block no
        other statement is allowed (no $OUT, WAIT, LIN ...). KR C4 / KSS 8.x only: a KR C2 has
        no spline motions."""
        self._no_spline("A nested SPLINE")
        self._emit("SPLINE")
        self._spline_open = True
        self._spline_segments = 0

    def _segment(self, line):
        if not self._spline_open:
            raise Exception("Spline segments need start_spline() first.")
        self._emit("  " + line)
        self._spline_segments += 1

    def spl(self, pose: tuple):
        """A spline point: the path passes through it smoothly (SPL)."""
        self._segment(f"SPL {self._pose_text(_pose6(pose, 'SPL point'))}")

    def slin(self, pose: tuple):
        """A straight segment inside a spline block (SLIN)."""
        self._segment(f"SLIN {self._pose_text(_pose6(pose, 'SLIN point'))}")

    def scirc(self, aux, target, ca=None):
        """A circular segment inside a spline block (SCIRC), arguments as circ()."""
        aux_xyz, end = self._circle_points(aux, target, "SCIRC")
        self._segment(f"SCIRC {self._xyz_text(aux_xyz)}, {self._pose_text(end)}{self._ca_text(ca)}")

    def end_spline(self, approx: str = None):
        """Close the SPLINE block. approx: None (exact stop at the end) or "C_SPL" to blend into
        the next motion."""
        if not self._spline_open:
            raise Exception("No SPLINE block is open.")
        if self._spline_segments == 0:
            raise Exception("A SPLINE block needs at least one segment.")
        self._spline_open = False
        self._emit("ENDSPLINE" + _approx_text(approx, SPLINE_APPROX, "ENDSPLINE"))

    def write_file(self, filename):
        """Write the program to filename (.src) and return the path actually written.

        KRC controllers require the ``DEF <name>`` inside the file to match the
        file name, and both to be a valid KRL name of at most 24 characters. The
        file name is therefore cleaned and cut with krl_name(), the DEF line is set
        to that same name, and when the name had to change the original file name
        is kept as a comment right after the DEF line. self.code is not modified,
        so full_program() keeps returning the program as built.
        """
        folder, base_name = os.path.split(str(filename))
        root, ext = os.path.splitext(base_name)
        module = krl_name(root)
        if module != root:
            warnings.warn(
                f'File name "{base_name}" is not a valid KRL module name; writing "{module}{ext or ".src"}".',
                UserWarning,
            )
        path = os.path.join(folder, module + (ext or ".src"))

        # full_program() closes the main program with END and appends any local
        # subprograms after it.
        lines = self.full_program()

        def_line = f"DEF {self.name} ( )"
        if def_line in lines:
            index = lines.index(def_line)
            lines[index] = f"DEF {module} ( )"
            if module != root:
                lines.insert(index + 1, f";FULLNAME: {base_name}")

        # Write each line of the KUKA src program to the specified file
        with open(path, "w") as fileOut:
            fileOut.write("\n".join(lines))
        return path
