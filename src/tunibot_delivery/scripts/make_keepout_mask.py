#!/usr/bin/env python3
"""Build a Nav2 keepout mask for the cafe_table tops.

The 2D lidar (18 cm high) only sees the thin central column of each cafe_table,
but the 0.91 x 0.91 m tabletop (75 cm high) overhangs it and the robot is 1.1 m
tall. The table positions come from the Gazebo world, the mask is expressed in
the SLAM map frame (= the robot spawn/dock pose), and Nav2's KeepoutFilter marks
those cells as lethal. Gazebo models are left unchanged.

usage: make_keepout_mask.py <world> <out_dir> [--dock-x -9.0 --dock-y 0.0]
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

RES = 0.05            # m/pixel
TABLE_SIZE = 0.913    # cafe_table top edge length (m)
MARGIN = 0.05         # extra clearance around the tabletop (m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('out_dir')
    ap.add_argument('--dock-x', type=float, default=-9.0)
    ap.add_argument('--dock-y', type=float, default=0.0)
    ap.add_argument('--half-extent', type=float, default=10.5,
                    help='half size of the square area covered, world frame (m)')
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

    os.makedirs(a.out_dir, exist_ok=True)
    with open(os.path.join(a.out_dir, 'keepout_mask.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (n, n))
        f.write(bytes(img))
    with open(os.path.join(a.out_dir, 'keepout_mask.yaml'), 'w') as f:
        f.write('image: keepout_mask.pgm\nmode: trinary\nresolution: %.3f\n'
                'origin: [%.3f, %.3f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n'
                'free_thresh: 0.25\n' % (RES, ox, oy))
    print('%d tables written to %s' % (len(tables), a.out_dir))


if __name__ == '__main__':
    main()
