# coding: utf-8
"""Headless tests for libs/beadlib.py (plain CPython, no Rhino).

Run from the repo root:  python tests/test_beadlib.py   (or: python -m pytest -q tests)
Needs shapely >= 2.0 (pip install shapely).
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'libs'))
import beadlib as bc  # noqa: E402
import iolib as io  # noqa: E402

W = 40.0


def zigzag(n=4, seg=60.0, ang=20.0):
    h = seg / (2 * math.tan(math.radians(ang / 2)))
    return [(i * seg, 0.0 if i % 2 == 0 else h) for i in range(n)]


def tight_sine(a=60.0, p=90.0, n=200):
    return [(x, a * math.sin(2 * math.pi * x / p)) for x in (i * 2 * p / n for i in range(n + 1))]


def figure8(r=50.0, n=120):
    return [(r * math.sin(2 * t), r * math.sin(t)) for t in (2 * math.pi * i / n for i in range(n))]


CASES = {
    'zigzag': [(zigzag(), False)],
    'tight_sine': [(tight_sine(), False)],
    'figure8': [(figure8(), True)],
    'hairpin': [([(0, 0), (150, 0), (150, 30), (0, 30)], False)],
    'tangent': [([(0, 0), (200, 0)], False), ([(0, W), (200, W)], False)],
    'square_loop': [([(0, 0), (300, 0), (300, 200), (0, 200)], True)],
}


def test_all_corners_and_caps_valid():
    for corner in sorted(bc.CORNERS):
        for cap in bc.CAPS:
            for name, paths in CASES.items():
                out = bc.footprint(paths, bc.BeadOptions(bead_width=W, corner_style=corner, end_cap=cap))
                assert len(out) == 1 and out[0], (corner, cap, name)
                for poly in out[0]:
                    assert poly.is_valid and poly.area > 0, (corner, cap, name)


def test_round_matches_exact_sweep_area():
    # straight bead, round caps: rectangle + disc
    out = bc.footprint([([(0, 0), (100, 0)], False)], bc.BeadOptions(bead_width=W, tolerance=0.01))
    exact = 100 * W + math.pi * (W / 2) ** 2
    assert abs(out[0][0].area / exact - 1) < 1e-3


def test_corner_ordering_on_square_loop():
    # outer corners: bevel < round < mitre ; inner corners identical
    a = {c: bc.footprint(CASES['square_loop'], bc.BeadOptions(bead_width=W, corner_style=c,
                                                              corner_mitre=10))[0][0].area
         for c in ('bevel', 'round', 'mitre')}
    assert a['bevel'] < a['round'] < a['mitre']
    # mitre on a square loop = full rectangle frame
    frame = (300 + W) * (200 + W) - (300 - W) * (200 - W)
    assert abs(a['mitre'] - frame) < 1e-6 * frame


def test_blend_fillet_is_tangent_arc():
    pts = [(0, 0), (100, 0), (100, 100)]
    out = bc.fillet_polyline(pts, False, 30.0, 0.01)
    # arc points lie on the circle centred at (70, 30) with r = 30
    arc = [p for p in out if 70 - 1e-6 <= p[0] <= 100 + 1e-6 and -1e-6 <= p[1] <= 30 + 1e-6]
    assert len(arc) > 5
    assert all(abs(math.hypot(x - 70, y - 30) - 30) < 1e-6 for x, y in arc)
    assert out[0] == (0.0, 0.0) and out[-1] == (100.0, 100.0)


def test_blend_radius_is_clamped():
    pts = [(0, 0), (10, 0), (10, 10)]          # radius larger than the legs allow
    out = bc.fillet_polyline(pts, False, 100.0, 0.01)
    assert all(-1e-9 <= x <= 10 + 1e-9 and -1e-9 <= y <= 10 + 1e-9 for x, y in out)


def test_bulge_wider_than_round_only_at_corners():
    paths = CASES['zigzag']
    r = bc.footprint(paths, bc.BeadOptions(bead_width=W, corner_style='round'))[0][0]
    b = bc.footprint(paths, bc.BeadOptions(bead_width=W, corner_style='bulge', corner_bulge=0.25))[0][0]
    assert b.area > r.area
    assert b.buffer(1e-6).contains(r.buffer(-0.5))


def test_bulge_independent_of_discretisation():
    # the same 90 deg arc (r = 15 < w) sampled with 4 and with 64 segments
    def arc(n):
        return [(-100, 0)] + [(15 * math.sin(math.pi / 2 * k / n), 15 - 15 * math.cos(math.pi / 2 * k / n))
                              for k in range(n + 1)] + [(15, 115)]
    a = [bc.footprint([(arc(n), False)], bc.BeadOptions(bead_width=W, corner_style='bulge', corner_bulge=0.5))[0][0].area
         for n in (4, 64)]
    r = bc.footprint([(arc(64), False)], bc.BeadOptions(bead_width=W))[0][0].area
    assert a[1] > r * 1.01                      # a smooth bend does widen
    assert abs(a[0] / a[1] - 1) < 0.01          # and the result does not depend on sampling


def test_bulge_is_local():
    paths = [([(0, 0), (400, 0), (400, 400)], False)]
    b = bc.footprint(paths, bc.BeadOptions(bead_width=W, corner_style='bulge', corner_bulge=0.5))[0][0]
    # 150 mm before the corner the bead has its nominal half width
    from shapely.geometry import LineString
    cut = b.intersection(LineString([(250, -100), (250, 100)]))
    assert abs(cut.length - W) < 0.05


def test_union_false_gives_one_branch_per_path():
    out = bc.footprint(CASES['tangent'], bc.BeadOptions(bead_width=W, union=False))
    assert len(out) == 2 and all(len(br) == 1 for br in out)


def test_fuse_gap_closes_near_tangent_beads():
    paths = [([(0, 0), (200, 0)], False), ([(0, W + 0.05), (200, W + 0.05)], False)]
    assert len(bc.footprint(paths, bc.BeadOptions(bead_width=W))[0]) == 2
    assert len(bc.footprint(paths, bc.BeadOptions(bead_width=W, fuse_gap=1.0))[0]) == 1


def test_per_path_width_and_rings_orientation():
    out = bc.footprint([([(0, 0), (100, 0)], False, 20.0)], bc.BeadOptions(bead_width=W))
    poly = out[0][0]
    assert abs(poly.bounds[3] - 10.0) < 1e-6           # used the path width 20, not 40
    outer, holes = bc.rings(bc.footprint(CASES['square_loop'], bc.BeadOptions(bead_width=W))[0][0])
    area2 = sum(outer[i - 1][0] * outer[i][1] - outer[i][0] * outer[i - 1][1]
                for i in range(len(outer)))
    assert area2 > 0 and len(holes) == 1                # outer CCW, one hole


def test_custom_corner_registration():
    @bc.register_corner('test_square')
    def _sq(pts, closed, width, opt):
        return bc.sweep_constant(pts, closed, width, 'mitre', 'square', 10.0, opt.tolerance)
    out = bc.footprint([([(0, 0), (100, 0)], False)], bc.BeadOptions(bead_width=W, corner_style='test_square'))
    assert abs(out[0][0].area - (100 + W) * W) < 1e-6
    del bc.CORNERS['test_square']


def test_unknown_corner_message():
    try:
        bc.footprint(CASES['zigzag'], bc.BeadOptions(corner_style='nope'))
    except io.ValidationError as e:
        assert 'round' in str(e)
    else:
        raise AssertionError('expected ValueError')


def test_options_are_validated():
    for bad in (dict(bead_width=0), dict(bead_width=-5), dict(end_cap='pointy'), dict(tolerance=0),
                dict(fuse_gap=-1), dict(bead_width='40'), dict(corner_mitre=0.5)):
        try:
            bc.BeadOptions(**bad)
        except io.ValidationError:
            continue
        raise AssertionError('accepted invalid options {}'.format(bad))


def test_islands_of_a_closed_loop():
    polys = bc.footprint(CASES['square_loop'], bc.BeadOptions(bead_width=W))[0]
    isl = bc.islands(polys)
    assert len(isl) == 1
    assert abs(isl[0].area - (300 - W) * (200 - W)) < 1e-6
    assert not isl[0].interiors


def test_islands_cut_by_inner_bead():
    # a closed bead inside the loop's cavity: the big island gets a hole where that
    # bead is, and the inner bead's own cavity (100-40 x 80-40) is a second island
    inner_loop = ([(100, 60), (200, 60), (200, 140), (100, 140)], True)
    polys = bc.footprint(CASES['square_loop'] + [inner_loop], bc.BeadOptions(bead_width=W))[0]
    isl = sorted(bc.islands(polys), key=lambda g: g.area)
    assert len(isl) == 2
    assert abs(isl[0].area - 60 * 40) < 1e-6 and not isl[0].interiors
    inner_bead = bc.footprint([inner_loop], bc.BeadOptions(bead_width=W))[0][0]
    outer_cavity = (300 - W) * (200 - W)
    assert len(isl[1].interiors) == 1
    assert abs(isl[1].area - (outer_cavity - inner_bead.area - 60 * 40)) < 1e-3


def test_islands_of_fused_infill_and_open_path():
    zig = [(0, 0), (70, 150), (140, 0), (210, 150)]
    polys = bc.footprint([(zig, False), ([(0, -40), (210, -40)], False)],
                         bc.BeadOptions(bead_width=W, fuse_gap=1.0))[0]
    assert len(bc.islands(polys)) == 1
    assert bc.islands(bc.footprint([(zig, False)], bc.BeadOptions(bead_width=W))[0]) == []
    assert bc.islands(polys, min_area=1e9) == []


def test_width_from_flow():
    # rounded profile: A = (w - h) h + pi h^2 / 4, here w = 40, h = 10, v = 100
    q = ((40 - 10) * 10 + math.pi * 100 / 4) * 100
    assert abs(bc.width_from_flow(q, 100, 10) - 40) < 1e-9
    assert abs(bc.width_from_flow(4000, 100, 10, profile='rect') - 4) < 1e-9


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in fns:
        f()
        print('ok ', f.__name__)
    print('{} tests passed'.format(len(fns)))
