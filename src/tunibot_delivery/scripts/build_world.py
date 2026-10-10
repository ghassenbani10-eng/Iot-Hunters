#!/usr/bin/env python3
"""Phase 2 layout generator for worlds/restaurant.world.

Adds to the Phase 1 dining room (idempotent: re-running replaces the generated
block between the GENERATED markers):

  * a full-height partition wall at y = 3.0 with two 0.9 m doorways
      - D1 (x = -4.0): dining room -> kitchen
      - D2 (x = 7.7) : dining room -> service corridor
  * a kitchen (north-west) with the pickup pass counter, a stove island,
    fridges and shelves
  * a 1.05 m wide service corridor going north, an L-turn into a 1.36 m wide
    corridor going west, and a third 0.9 m doorway D3 into a VIP lounge
    with two tables (table_8, table_9)
  * walking people: Gazebo actors driven by libtunibot_human_actor.so, each
    with an invisible collision body so the lidar/bumper perceive them
  * the gazebo_ros_state plugin (ground-truth poses for the test metrics)
  * the open exterior door in the south wall is closed

usage: build_world.py [world_file] [--no-humans OUT]
  default world_file: ../worlds/restaurant.world
  --no-humans OUT also writes a copy without the walking people (used for SLAM
  mapping, so that the saved map only contains the static environment)
"""
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

BEGIN = '<!-- BEGIN GENERATED (scripts/build_world.py) -->'
END = '<!-- END GENERATED (scripts/build_world.py) -->'

T = 0.15        # wall thickness (m)
H = 2.5         # wall height (m)
X_MIN, X_MAX = -10.415, 9.535     # inner faces of the west / east walls
Y_MAX = 10.34                      # inner face of the north wall
Y_PART = 3.0                       # partition wall centre line

# Wall segments as (x1, y1, x2, y2) centre lines, world frame.
WALLS = [
    # partition, doorways D1 [-4.45, -3.55] and D2 [7.25, 8.15]
    (X_MIN, Y_PART, -4.45, Y_PART),
    (-3.55, Y_PART, 7.25, Y_PART),
    (8.15, Y_PART, X_MAX, Y_PART),
    # kitchen east wall
    (-2.5, Y_PART, -2.5, Y_MAX),
    # service corridor: west wall (lounge side) and east wall (storage side)
    (7.1, Y_PART, 7.1, 8.9),
    (8.3, Y_PART, 8.3, Y_MAX),
    # lounge north wall = south side of the west corridor, doorway D3 [1.0, 1.9]
    (-2.5, 8.9, 1.0, 8.9),
    (1.9, 8.9, 7.1, 8.9),
    # close the exterior door of the south wall
    (7.55, -9.686, 8.6, -9.686),
]

# Static furniture: (name, cx, cy, sx, sy, sz, material)
FURNITURE = [
    ('pickup_counter', -4.0, 6.5, 1.8, 0.6, 0.95, 'Gazebo/Wood'),
    ('stove_island', -7.5, 6.5, 3.0, 1.0, 0.95, 'Gazebo/Grey'),
    ('fridge_1', -10.05, 4.2, 0.7, 0.8, 2.0, 'Gazebo/White'),
    ('fridge_2', -10.05, 5.1, 0.7, 0.8, 2.0, 'Gazebo/White'),
    ('kitchen_shelf', -6.5, 9.95, 6.0, 0.6, 1.8, 'Gazebo/DarkGrey'),
    ('storage_rack', 8.9, 6.5, 1.0, 5.0, 1.8, 'Gazebo/DarkGrey'),
    ('lounge_sofa', 3.0, 3.45, 3.0, 0.7, 0.8, 'Gazebo/Red'),
]

# VIP lounge tables (cafe_table copies; the keepout mask picks every cafe_table up)
VIP_TABLES = [('cafe_table_vip_1', 0.5, 6.0), ('cafe_table_vip_2', 4.5, 6.0)]

# Walking people: name, waypoints (world frame), speed, dwell, start delay
HUMANS = [
    ('human_waiter', [(-7.5, 0.9), (6.0, 0.9)], 0.75, 3.0, 5.0),
    ('human_guest', [(2.4, -4.2), (2.4, 1.8)], 0.6, 4.0, 12.0),
    ('human_cook', [(-8.2, 5.0), (-5.6, 5.0)], 0.5, 5.0, 3.0),
    ('human_vip', [(-1.5, 7.6), (6.2, 7.6)], 0.55, 6.0, 20.0),
]


def box_link(name, cx, cy, cz, sx, sy, sz, yaw, material):
    geo = '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>' % (sx, sy, sz)
    return f"""
      <link name='{name}'>
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.4f}</pose>
        <collision name='{name}_collision'>{geo}</collision>
        <visual name='{name}_visual'>{geo}
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script></material>
        </visual>
      </link>"""


def layout_model():
    links = []
    for i, (x1, y1, x2, y2) in enumerate(WALLS):
        length = math.hypot(x2 - x1, y2 - y1) + T  # overlap the corners
        yaw = math.atan2(y2 - y1, x2 - x1)
        links.append(box_link('wall_%d' % i, (x1 + x2) / 2, (y1 + y2) / 2, H / 2,
                              length, T, H, yaw, 'Gazebo/Grey'))
    for name, cx, cy, sx, sy, sz, mat in FURNITURE:
        links.append(box_link(name, cx, cy, sz / 2, sx, sy, sz, 0.0, mat))
    # visual-only floor markings: charging pad, pickup spot, doorways
    marks = [('mark_dock', -9.0, 0.0, 0.7, 0.7, 'Gazebo/Green'),
             ('mark_pickup', -4.0, 5.3, 0.6, 0.6, 'Gazebo/Yellow')]
    for name, cx, cy, sx, sy, mat in marks:
        links.append(f"""
      <link name='{name}'>
        <pose>{cx} {cy} 0.002 0 0 0</pose>
        <visual name='{name}_visual'><geometry><box><size>{sx} {sy} 0.004</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{mat}</name></script></material>
        </visual>
      </link>""")
    return "\n    <model name='tunibot_layout'>\n      <static>1</static>%s\n    </model>" % ''.join(links)


def human_models():
    out = []
    for name, wps, speed, dwell, delay in HUMANS:
        x0, y0 = wps[0]
        wp_xml = ''.join('\n          <waypoint>%.2f %.2f</waypoint>' % w for w in wps)
        # Invisible collision body (lidar + bumper), moved by the actor plugin.
        out.append(f"""
    <model name='{name}'>
      <pose>{x0} {y0} 0 0 0 0</pose>
      <link name='body'>
        <gravity>0</gravity>
        <kinematic>1</kinematic>
        <inertial><mass>70</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name='body_collision'>
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>1.7</length></cylinder></geometry>
        </collision>
      </link>
    </model>
    <actor name='{name}_actor'>
      <pose>{x0} {y0} 1.0 1.5708 0 0</pose>
      <skin><filename>walk.dae</filename><scale>1.0</scale></skin>
      <animation name='walking'><filename>walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <plugin name='{name}_walker' filename='libtunibot_human_actor.so'>{wp_xml}
          <speed>{speed}</speed>
          <dwell>{dwell}</dwell>
          <start_delay>{delay}</start_delay>
          <body>{name}</body>
          <robot>waiter_robot</robot>
          <yield_distance>0.9</yield_distance>
          <max_yield_time>6.0</max_yield_time>
      </plugin>
    </actor>""")
    return ''.join(out)


def vip_tables(src_text):
    m = re.search(r"(\n\s*<model name='cafe_table'>.*?</model>)", src_text, re.S)
    template = m.group(1)
    out = []
    for name, x, y in VIP_TABLES:
        t = template.replace("<model name='cafe_table'>", "<model name='%s'>" % name)
        t = re.sub(r"(\n\s*<pose>)[^<]*(</pose>\s*</model>)", r"\g<1>%s %s 0 0 -0 0\g<2>" % (x, y), t)
        out.append(t)
    return ''.join(out)


STATE_PLUGIN = """
    <plugin name='gazebo_ros_state' filename='libgazebo_ros_state.so'>
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>"""


def remove_old_partition(text):
    """Drop the Phase 1 windowed partition (link Wall_16) from model and state."""
    return re.sub(r"\n\s*<link name='Wall_16'>.*?</link>", '', text, flags=re.S)


def generate(base, humans=True):
    text = re.sub(r'\n\s*' + re.escape(BEGIN) + '.*?' + re.escape(END), '', base, flags=re.S)
    text = remove_old_partition(text)
    block = '\n    %s%s%s%s%s\n    %s' % (BEGIN, layout_model(), vip_tables(text),
                                         human_models() if humans else '', STATE_PLUGIN, END)
    idx = text.rfind('</world>')
    text = text[:idx].rstrip() + block + '\n  ' + text[idx:]
    ET.fromstring(text)  # sanity check: well-formed XML
    return text


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    args = sys.argv[1:]
    no_humans = None
    if '--no-humans' in args:
        i = args.index('--no-humans')
        no_humans = args[i + 1]
        del args[i:i + 2]
    path = args[0] if args else os.path.join(here, '..', 'worlds', 'restaurant.world')
    base = open(path).read()
    open(path, 'w').write(generate(base))
    print('wrote', path)
    if no_humans:
        open(no_humans, 'w').write(generate(base, humans=False))
        print('wrote', no_humans)


if __name__ == '__main__':
    main()
