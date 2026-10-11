#!/usr/bin/env python3
"""Build the Nav2 keepout mask: cafe_table tops + staff-only zones.

The 2D lidar (18 cm high) only sees the thin central column of each cafe_table,
but the 0.91 x 0.91 m tabletop (75 cm high) overhangs it and the robot is 1.1 m
tall. The table positions come from the Gazebo world, the mask is expressed in
the SLAM map frame (= the robot spawn/dock pose), and Nav2's KeepoutFilter marks
those cells as lethal. Gazebo models are left unchanged.

Staff-only and customer-only zones (the kitchen: the robot is served through the
windows and never goes in; the takeaway queue; the toilets) are keepout rectangles.

Robot lanes: in the dining room every cell off the painted robot lanes gets a
small, non-lethal cost (the mask is written in `scale` mode; Humble's
KeepoutFilter copies intermediate mask values into the costmap as costs), so the
planner follows the lanes when it can and still reaches the tables when it must.

usage: make_keepout_mask.py <world> <out_dir> [--dock-x -9.0 --dock-y 2.2]
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_world import LANE_HALF_WIDTH, LANES, QUEUE_ZONE, Y_PART  # noqa: E402

RES = 0.05            # m/pixel
TABLE_SIZE = 0.913    # cafe_table top edge length (m)
MARGIN = 0.12         # extra clearance around the tabletop (m): footprint discretization + overhang
# staff-only rectangles, world frame: (xmin, xmax, ymin, ymax)
STAFF_ONLY = [(-10.5, -2.4, 3.05, 10.5),    # kitchen
              (8.35, 9.6, 3.05, 7.0),      # customer toilet
              (QUEUE_ZONE[0] - 0.05, QUEUE_ZONE[1] + 0.05, QUEUE_ZONE[2] - 0.05, 3.05)]   # takeaway queue
OFF_LANE_PIXEL = 205  # off-lane cost: occupancy (255 - 205) / 255 = 0.2 -> cost ~50 of 254
DINING = (-10.5, 9.6, -10.5, Y_PART)       # where the lane preference applies


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('out_dir')
    ap.add_argument('--dock-x', type=float, default=-9.0)
    ap.add_argument('--dock-y', type=float, default=2.2)
    ap.add_argument('--half-extent', type=float, default=10.5,
                    help='half size of the square area covered, world frame (m)')
    ap.add_argument('--no-lanes', action='store_true', help='keepout zones only (trinary mask)')
    a = ap.parse_args()

    world = ET.parse(a.world).getroot().find('world')
    tables = []
    for m in world.findall('model'):
        if m.get('name', '').startswith('cafe_table'):
            x, y, _, _, _, yaw = map(float, m.find('pose').text.split())
            tables.append((x - a.dock_x, y - a.dock_y, yaw))   # world -> map frame

    # map-frame bounds of the restaurant
    ox, oy = -a.half_extent - a.dock_x, -a.half_extent - a.dock_y
    n = int(round(2 * a.half_extent / RES))
    img = bytearray([254]) * (n * n)          # free everywhere

    def lane_dist(wx, wy):
        best = 1e9
        for (ax, ay), (bx, by) in LANES:
            dx, dy = bx - ax, by - ay
            t = max(0.0, min(1.0, ((wx - ax) * dx + (wy - ay) * dy) / (dx * dx + dy * dy)))
            best = min(best, math.hypot(wx - ax - t * dx, wy - ay - t * dy))
        return best

    if not a.no_lanes:
        for py in range(n):
            wy = oy + (py + 0.5) * RES + a.dock_y
            if not DINING[2] <= wy <= DINING[3]:
                continue
            for px in range(n):
                wx = ox + (px + 0.5) * RES + a.dock_x
                if DINING[0] <= wx <= DINING[1] and lane_dist(wx, wy) > LANE_HALF_WIDTH:
                    img[(n - 1 - py) * n + px] = OFF_LANE_PIXEL
    half = TABLE_SIZE / 2 + MARGIN
    for tx, ty, yaw in tables:
        c, s = math.cos(yaw), math.sin(yaw)
        r = half * math.sqrt(2)
        for py in range(int((ty - r - oy) / RES), int((ty + r - oy) / RES) + 1):
            for px in range(int((tx - r - ox) / RES), int((tx + r - ox) / RES) + 1):
                if not (0 <= px < n and 0 <= py < n):
                    continue
                wx, wy = ox + (px + 0.5) * RES - tx, oy + (py + 0.5) * RES - ty
                lx, ly = c * wx + s * wy, -s * wx + c * wy
                if abs(lx) <= half and abs(ly) <= half:
                    img[(n - 1 - py) * n + px] = 0   # PGM rows go top -> bottom

    for x0, x1, y0, y1 in STAFF_ONLY:
        for py in range(n):
            wy = oy + (py + 0.5) * RES + a.dock_y
            if not y0 <= wy <= y1:
                continue
            for px in range(n):
                wx = ox + (px + 0.5) * RES + a.dock_x
                if x0 <= wx <= x1:
                    img[(n - 1 - py) * n + px] = 0

    os.makedirs(a.out_dir, exist_ok=True)
    with open(os.path.join(a.out_dir, 'keepout_mask.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (n, n))
        f.write(bytes(img))
    with open(os.path.join(a.out_dir, 'keepout_mask.yaml'), 'w') as f:
        # scale mode: free_thresh below the off-lane occupancy (0.2), so it stays graded
        f.write('image: keepout_mask.pgm\nmode: %s\nresolution: %.3f\n'
                'origin: [%.3f, %.3f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n'
                'free_thresh: %s\n' % ('trinary' if a.no_lanes else 'scale', RES, ox, oy,
                                       '0.25' if a.no_lanes else '0.05'))
    print('%d tables written to %s' % (len(tables), a.out_dir))


if __name__ == '__main__':
    main()
