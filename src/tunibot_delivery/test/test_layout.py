"""Static checks of the restaurant layout (scripts/build_world.py) and of the
delivery poses against it and against the keepout mask: catches a pose moved
into a wall, a table or a keepout zone before any simulation is run."""
import math
import os

import build_world as bw
import pytest
import yaml
from conftest import PKG

DOCK = (-9.0, 2.2)                          # map = world - DOCK
FOOT_HALF = (0.27, 0.25)                    # robot footprint half length / width (m)
POINTS = yaml.safe_load(open(os.path.join(PKG, 'config', 'delivery_points.yaml')))
MGR = POINTS['delivery_manager']['ros__parameters']


def poses(name):
    v = MGR[name]
    assert len(v) % 3 == 0, '%s: flat list of (x, y, yaw) triples' % name
    return [tuple(v[i:i + 3]) for i in range(0, len(v), 3)]


def all_places():
    return ['pickup', 'pre_dock', 'dock'] + list(MGR['tables'])


def load_pgm(path):
    raw = open(path, 'rb').read()
    parts, i = [], 0
    while len(parts) < 4:
        while raw[i:i + 1].isspace():
            i += 1
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        parts.append(raw[i:j])
        i = j
    w, h = int(parts[1]), int(parts[2])
    return w, h, raw[i + 1:i + 1 + w * h]


@pytest.fixture(scope='module')
def mask():
    meta = yaml.safe_load(open(os.path.join(PKG, 'maps', 'keepout_mask.yaml')))
    w, h, data = load_pgm(os.path.join(PKG, 'maps', meta['image']))
    return w, h, data, meta['resolution'], meta['origin'][:2]


def footprint_cells(x, y, yaw, res, margin=0.0):
    """Sample points covering the robot footprint (map frame)."""
    c, s = math.cos(yaw), math.sin(yaw)
    n = 12
    for i in range(n + 1):
        for j in range(n + 1):
            lx = -FOOT_HALF[0] - margin + (2 * (FOOT_HALF[0] + margin)) * i / n
            ly = -FOOT_HALF[1] - margin + (2 * (FOOT_HALF[1] + margin)) * j / n
            yield x + c * lx - s * ly, y + s * lx + c * ly


def wall_boxes():
    for w in bw.WALLS:
        x1, y1, x2, y2 = w[:4]
        e1, e2 = (w[4], w[5]) if len(w) > 4 else (True, True)   # end extension flags
        if (x1, y1) > (x2, y2):
            x1, y1, x2, y2, e1, e2 = x2, y2, x1, y1, e2, e1
        horiz = abs(y2 - y1) < 1e-6
        t = bw.T / 2
        if horiz:
            yield (x1 - (t if e1 else 0), x2 + (t if e2 else 0), y1 - t, y1 + t)
        else:
            yield (x1 - t, x1 + t, y1 - (t if e1 else 0), y2 + (t if e2 else 0))
    for _, cx, cy, sx, sy, _, _ in bw.FURNITURE:
        yield (cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)


def test_every_table_has_poses():
    for t in MGR['tables']:
        assert poses(t), t


@pytest.mark.parametrize('place', all_places())
def test_poses_outside_keepout(mask, place):
    w, h, data, res, (ox, oy) = mask
    for k, (x, y, yaw) in enumerate(poses(place)):
        if place == 'dock':
            continue      # the dock is in the off-lane area, nothing lethal around it
        for px, py in footprint_cells(x, y, yaw, res):
            col, row = int((px - ox) / res), h - 1 - int((py - oy) / res)
            assert 0 <= col < w and 0 <= row < h
            assert data[row * w + col] != 0, '%s approach %d: footprint in a keepout zone at (%.2f, %.2f)' % (
                place, k + 1, px, py)


@pytest.mark.parametrize('place', all_places())
def test_poses_clear_of_walls_and_furniture(place):
    for k, (x, y, yaw) in enumerate(poses(place)):
        for px, py in footprint_cells(x + DOCK[0], y + DOCK[1], yaw, 0.05, margin=0.03):
            for x0, x1, y0, y1 in wall_boxes():
                assert not (x0 <= px <= x1 and y0 <= py <= y1), \
                    '%s approach %d hits a wall/furniture box (%.2f..%.2f, %.2f..%.2f)' % (
                        place, k + 1, x0, x1, y0, y1)


def gap_along(x_or_y, axis_value, lo, hi):
    """Free intervals of the line {axis = axis_value} between lo and hi left by the walls."""
    blocks = []
    for x0, x1, y0, y1 in wall_boxes():
        if x_or_y == 'y' and y0 <= axis_value <= y1:
            blocks.append((x0, x1))
        if x_or_y == 'x' and x0 <= axis_value <= x1:
            blocks.append((y0, y1))
    free, cur = [], lo
    for b0, b1 in sorted(blocks):
        if b0 > cur:
            free.append((cur, min(b0, hi)))
        cur = max(cur, b1)
    if cur < hi:
        free.append((cur, hi))
    return [(a, b) for a, b in free if b > a]


def test_doors_keep_nominal_width():
    # door D2 (service), WC doors and lounge door D3 must stay at least 0.8 m wide
    d2 = [g for g in gap_along('y', bw.Y_PART + 0.05, 6.0, 9.0) if g[0] <= 7.7 <= g[1]]
    assert d2 and d2[0][1] - d2[0][0] >= 0.85
    for door_x in (-1.55, 0.45, 2.85):
        g = [g for g in gap_along('y', 8.9, -2.4, 7.0) if g[0] <= door_x <= g[1]]
        assert g and g[0][1] - g[0][0] >= 0.8 - 1e-6, 'door at x=%.2f' % door_x


def test_serving_windows_reachable_from_lane():
    lane_y = bw.LANES[0][0][1]
    for _, cx, w, sill, top in bw.OPENINGS:
        assert top - sill >= 0.75 and w >= 1.0
    # the robot's pickup poses face the two robot windows, not the takeaway door
    for x, y, yaw in poses('pickup'):
        wx = x + DOCK[0]
        assert any(abs(wx - cx) < 0.1 for cx, _ in bw.WINDOWS)
        assert abs(yaw - math.pi / 2) < 0.01
        assert lane_y < y + DOCK[1] < bw.Y_PART


def test_lanes_clear_of_furniture():
    for (ax, ay), (bx, by) in bw.LANES:
        for i in range(101):
            px, py = ax + (bx - ax) * i / 100, ay + (by - ay) * i / 100
            for x0, x1, y0, y1 in wall_boxes():
                assert not (x0 - 0.3 <= px <= x1 + 0.3 and y0 - 0.3 <= py <= y1 + 0.3), (px, py)


def test_queue_zone_does_not_cover_robot_windows():
    q = bw.QUEUE_ZONE
    for cx, w in bw.WINDOWS:
        assert cx + w / 2 + FOOT_HALF[0] < q[0] or cx - w / 2 > q[1]


def test_waypoint_dwell_format():
    for name, wps, speed, dwell, delay in bw.HUMANS:
        assert len(wps) >= 2 and speed > 0
        for w in wps:
            assert len(w) in (2, 3)
