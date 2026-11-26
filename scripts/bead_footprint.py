#! python3
# r: shapely>=2.0

"""Grasshopper script: toolpath curves -> planar surfaces of the printed filament (bead footprint).

All geometry comes from libs/ (printlib.bead_footprint -> beadlib). This file
only reads the component inputs, calls the library and writes the outputs.
The '# r: shapely>=2.0' line above makes Rhino install shapely, which beadlib
needs, on the first run.

Inputs:
    curve: toolpath centrelines of one layer (Curve, list access). Planar,
           all in the same plane. Polylines keep their exact vertices.
    bead_width: bead width (float, list access). One value for all curves, or
                one per curve. Optional when machine is given (its 'bead_width'
                is used); defaults to 40 when neither is connected.
    corner_style: corner type (str), default "round":
        round: exact disc sweep, outer corners are arcs of radius bead_width/2.
        mitre: sharp outer corners, cut where the spike exceeds corner_mitre * bead_width/2.
        bevel: outer corners cut straight across.
        blend: centreline filleted with radius corner_blend first (robot zone), then round.
        bulge: wider bead where the path turns, bead_width * (1 + corner_bulge * turn / pi).
    end_cap: end of open curves (str): round | flat | square. Default round.
             Ignored by corner_style "bulge" (always round).
    corner_blend: fillet radius for corner_style "blend" (float). Default 20.
    corner_bulge: extra width at a 180 deg turn, as a fraction of bead_width,
                  for corner_style "bulge" (float). Default 0.25.
    corner_mitre: mitre limit for corner_style "mitre" (float). Default 2.
    fuse_gap: beads closer than this are fused into one region (float). 0 = off. Default 0.
    union: True = merged regions, False = one footprint per curve (bool). Default True.
    tolerance: curve discretisation and arc accuracy (float). Default 0.1.
    plane: layer plane (Plane, optional). Default: world XY at the first curve's height.
    machine: optional machine profile id or .json path (e.g. "kuka"), resolved
             against machine_settings/. Only used for 'bead_width' when
             bead_width is not connected.
Output:
    surfaces: footprint surfaces (Brep tree; one branch, or one branch per
              curve if union is False).
    outlines: outline curves (closed polylines, outer boundaries and holes), same tree.
    areas: region areas, same tree.
    islands: optional. Planar surfaces of the regions enclosed by the bead
             (the holes cut inside the footprint), same tree. Only computed
             when the component has an output named "islands".
    info: settings used and a summary of the result.

New corner types are added in libs/beadlib.py with @register_corner("name");
the name then works as a corner_style value here without changing this script.
"""

from ghpythonlib import treehelpers as th

import iolib as io
import printlib as pl


curve = curve if 'curve' in globals() and curve else []
bead_width = bead_width if 'bead_width' in globals() and bead_width else None
corner_style = corner_style if 'corner_style' in globals() and corner_style else "round"
end_cap = end_cap if 'end_cap' in globals() and end_cap else "round"
corner_blend = corner_blend if 'corner_blend' in globals() and corner_blend is not None else 20.0
corner_bulge = corner_bulge if 'corner_bulge' in globals() and corner_bulge is not None else 0.25
corner_mitre = corner_mitre if 'corner_mitre' in globals() and corner_mitre is not None else 2.0
fuse_gap = fuse_gap if 'fuse_gap' in globals() and fuse_gap is not None else 0.0
union = union if 'union' in globals() and union is not None else True
tolerance = tolerance if 'tolerance' in globals() and tolerance is not None else 0.1
plane = plane if 'plane' in globals() else None
machine = machine if 'machine' in globals() and machine else None


def _has_output(name):
    """True when the component has an output parameter called `name`."""
    try:
        return any(p.NickName == name or p.Name == name for p in ghenv.Component.Params.Output)
    except Exception:
        return True


surfaces = outlines = areas = islands = None
info = ""

if not curve:
    info = "Connect toolpath curves to curve."
else:
    if bead_width is None and machine is None:
        bead_width = 40.0
    try:
        surf_list, outline_list, area_list, island_list, opt = pl.bead_footprint(
            curve, bead_width=bead_width, corner_style=corner_style, end_cap=end_cap,
            corner_blend=corner_blend, corner_bulge=corner_bulge, corner_mitre=corner_mitre,
            fuse_gap=fuse_gap, union=union, tolerance=tolerance, plane=plane,
            machine=machine, with_islands=_has_output("islands"),
            component=ghenv.Component)
        surfaces = th.list_to_tree(surf_list)
        outlines = th.list_to_tree(outline_list)
        areas = th.list_to_tree(area_list)
        islands = th.list_to_tree(island_list)
        if bead_width is None:
            width_txt = "{} (from {})".format(opt.bead_width, machine)
        elif isinstance(bead_width, (list, tuple)) and len(bead_width) > 1:
            width_txt = "per curve"
        else:
            width_txt = opt.bead_width
        info = "corner_style={} end_cap={} bead_width={} | {} curves -> {} surface(s), {} island(s), area {:.1f}".format(
            opt.corner_style, opt.end_cap, width_txt, len(curve),
            sum(len(b) for b in surf_list), sum(len(b) for b in island_list), sum(map(sum, area_list)))
    except Exception as err:
        io.report_issue(str(err), "error", ghenv.Component)
        info = "Error: {}".format(err)
