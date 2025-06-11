#! python3

import Rhino.Geometry as rg
import scriptcontext as sc
import rhinoscriptsyntax as rs
import math

# GH Python inputs:
curve: rg.Curve               # rail for sweep
width: float                  # filament width
height: float                 # filament height
subdivs: float                # corner subdivisions (will be forced to int)
section_tolerance: float      # max deviation tolerance for curve sampling
closed: bool                  # close loft
caps: int                     # 0 = none, 1 = flat end caps

# Output:
# a: rg.Mesh (filament sweep mesh)

# --------------------- MeshLoft (inlined) ---------------------
class MeshLoft:
    def __init__(self, polylines, close_loft=False, cap_loft=0):
        self.polylines = polylines
        self.close_loft = close_loft
        self.cap_loft = cap_loft
        self.mesh = self._build_mesh()

    def _build_mesh(self):
        # flatten vertices
        verts = [pt for pl in self.polylines for pt in pl]
        # build faces
        faces = []
        count = len(self.polylines[0])
        num = len(self.polylines)
        for i in range(num - 1 + int(self.close_loft)):
            i_next = (i + 1) % num
            for j in range(count - 1):
                v1 = i * count + j
                v2 = i * count + j + 1
                v3 = i_next * count + j + 1
                v4 = i_next * count + j
                faces.append((v1, v2, v3, v4))
        mesh = rg.Mesh()
        for v in verts:
            mesh.Vertices.Add(v)
        for f in faces:
            mesh.Faces.AddFace(*f)
        if self.cap_loft:
            # flat caps
            for pl in (self.polylines[0], self.polylines[-1]):
                cap_mesh = rg.Mesh.CreateFromClosedPolyline(pl)
                mesh.Append(cap_mesh)
        mesh.Normals.ComputeNormals()
        mesh.Compact()
        return mesh

# --------------------- Sweep Functions ---------------------

def create_rounded_rectangle(width, height, subdivs):
    """
    Port of the rhinoscriptsyntax rounded_rectangle: uses only RhinoCommon.
    subdivisions: number of segments per corner fillet.
    If subdivisions == 0, returns a sharp rectangle.
    Returns: rg.Polyline
    """
    if subdivs < 0:
        raise ValueError("subdivs cannot be negative")
    sub = int(subdivs)
    half_w = width * 0.5
    half_h = height * 0.5
    pts = []
    # sharp rectangle
    if sub == 0:
        rect = [
            rg.Point3d( half_w,  half_h, 0),
            rg.Point3d(-half_w,  half_h, 0),
            rg.Point3d(-half_w, -half_h, 0),
            rg.Point3d( half_w, -half_h, 0),
        ]
        pts.extend(rect)
        pts.append(pts[0])
        return rg.Polyline(pts)
    # fillet radius = half height
    radius = half_h
    # compute corner centers offset by radius
    centers = [
        ( half_w - radius,  half_h - radius, 0),  # top right
        (-half_w + radius,  half_h - radius, 0),  # top left
        (-half_w + radius, -half_h + radius, 0),  # bottom left
        ( half_w - radius, -half_h + radius, 0),  # bottom right
    ]
    # arc angles in radians
    arc_bases = [
        (0,   math.pi/2),    # top right
        (math.pi/2, math.pi), # top left
        (math.pi, 3*math.pi/2),# bottom left
        (3*math.pi/2, 2*math.pi) # bottom right
    ]
    # sample each corner arc
    for idx in range(4):
        cx, cy, cz = centers[idx]
        a_start, a_end = arc_bases[idx]
        for j in range(sub + 1):
            t = j / float(sub)
            ang = (1 - t) * a_start + t * a_end
            x = cx + radius * math.cos(ang)
            y = cy + radius * math.sin(ang)
            pts.append(rg.Point3d(x, y, cz))
    pts.append(pts[0])
    return rg.Polyline(pts)

    # filleted rectangle
    radius = min(half_w, half_h)
    # corner centers and their arc angles
    corner_data = [
        ( half_w - radius,  half_h - radius,  0.0,      math.pi/2),
        ( half_w - radius, -half_h + radius, -math.pi/2, 0.0),
        (-half_w + radius, -half_h + radius, math.pi,   -math.pi/2),
        (-half_w + radius,  half_h - radius, math.pi/2,  math.pi)
    ]
    for cx, cy, a0, a1 in corner_data:
        # sample the arc between a0 and a1
        for i in range(subdivs + 1):
            t = i / float(subdivs)
            ang = a0 + (a1 - a0) * t
            x = cx + math.cos(ang) * radius
            y = cy + math.sin(ang) * radius
            pts.append(rg.Point3d(x, y, 0))
    pts.append(pts[0])
    return rg.Polyline(pts)
    # filleted rectangle
    radius = min(half_w, half_h)
    # define corner centers and start/end angles for arcs
    corner_data = [
        # center_x, center_y, start_angle, end_angle
        ( half_w - radius,  half_h - radius,  0.0,      math.pi/2),
        ( half_w - radius, -half_h + radius, -math.pi/2, 0.0),
        (-half_w + radius, -half_h + radius, math.pi,   -math.pi/2),
        (-half_w + radius,  half_h - radius, math.pi/2,  math.pi)
    ]
    for cx, cy, a0, a1 in corner_data:
        for i in range(subdivs + 1):
            t = i / subdivs
            ang = a0 + (a1 - a0) * t
            x = cx + math.cos(ang) * radius
            y = cy + math.sin(ang) * radius
            pts.append(rg.Point3d(x, y, 0))
    pts.append(pts[0])
    return rg.Polyline(pts)
    radius = min(half_w, half_h)
    # generate fillet arcs and straight segments
    corners = [(half_w - radius,  half_h - radius,  0.0, 0.0),
               ( half_w - radius, -half_h + radius,  0.0, -1.0),
               (-half_w + radius, -half_h + radius, -1.0,  0.0),
               (-half_w + radius,  half_h - radius,  0.0,  1.0)]
    for cx, cy, dx, dy in corners:
        # center of fillet arc
        for i in range(subdivs + 1):
            angle = math.atan2(dy, dx) + (math.pi/2) * (i / subdivs)
            x = cx + math.cos(angle) * radius
            y = cy + math.sin(angle) * radius
            pts.append(rg.Point3d(x, y, 0))
    pts.append(pts[0])
    return rg.Polyline(pts)


def sample_frames(crv, tol):
    """Divide curve by length/tolerance and return list of perpendicular frames."""
    length = crv.GetLength()
    count = max(int(round(length / tol)), 1)
    points = rs.DivideCurve(crv, count)
    frames = []
    for pt in points:
        ok, param = crv.ClosestPoint(pt)
        _, frame = crv.PerpendicularFrameAt(param)
        frames.append(frame)
    return frames

# --------------------- Main ---------------------

def main():
    crv = rs.coercecurve(curve)
    if not crv:
        raise ValueError("Invalid curve input")
    fillet_count = int(subdivs)
    profile = create_rounded_rectangle(width, height, fillet_count)
    frames = sample_frames(crv, section_tolerance)
    # orient sections
    sections = []
    for frame in frames:
        pl = rg.Polyline(profile)
        pl.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, frame))
        sections.append(pl)
    # mesh loft
    loft = MeshLoft(sections, close_loft=closed, cap_loft=caps)
    return loft.mesh

# execute and output
a = main()
