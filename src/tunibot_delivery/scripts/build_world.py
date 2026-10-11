#!/usr/bin/env python3
"""Phase 2 layout generator for worlds/restaurant.world.

The restaurant is divided in two by a partition wall at y = 3.0:
  * north-west: the KITCHEN (staff only). Cooks hand the food to the robot
    through two SERVING WINDOWS in the partition (counter at 0.95 m, opening
    up to 1.75 m). The robot never enters the kitchen (staff-only keepout zone);
    staff reach the dining room through a staff door and the service corridor.
  * south: the DINING ROOM (Phase 1 tables); the charging dock is in its
    north-west corner, next to the serving windows.
  * north-east (customer side): a VIP lounge reached from the dining room
    through door D2 (0.9 m), a 1.05 m service corridor, an L-turn into a
    1.36 m corridor and door D3 (0.9 m) - the tight-passage part of Phase 2.
  * east, behind the service corridor: the customer TOILET, entered straight
    from the dining room through its own door next to D2 (customers never
    use the robot's corridor), and a closed storage room.

Also generated (idempotent: re-running replaces the block between the
GENERATED markers): kitchen furniture, VIP tables, walking people (Gazebo
actors driven by libtunibot_human_actor.so, each with an invisible collision
body so the lidar/bumper perceive them), the gazebo_ros_state plugin
(ground-truth poses for the tests), the closed exterior door of the south wall,
and the docking station pose.

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

# Docking station (world frame). The robot spawns on it, facing +x: SLAM map origin.
DOCK_STATION = (-9.75, 2.2)
ROBOT_DOCK = (-9.0, 2.2)

# Openings in the kitchen/dining partition: (name, centre x, width, sill z, top z).
#  - two serving windows for the robot (trays at 0.95 m)
#  - the takeaway door: a half door with a counter (1.0 m), where takeaway customers
#    queue and collect their meal; for the robot it is a wall (it never enters the kitchen)
OPENINGS = [('window_1', -6.5, 1.0, 0.95, 1.75), ('window_2', -5.0, 1.0, 0.95, 1.75),
            ('takeaway', -3.3, 1.2, 1.0, 2.1)]
WINDOWS = [(cx, w) for n, cx, w, _, _ in OPENINGS if n.startswith('window')]

# Customer toilet, entered from the dining room through the partition (world frame)
TOILET_DOOR = (9.0, 0.9)            # centre x, width
TOILET_Y_MAX = 5.6                  # centre line of its north wall

# Full-height wall segments: (x1, y1, x2, y2[, extend_start, extend_end]), centre
# lines, world frame. Ends are extended by half the thickness to close corners,
# except at door edges (extend flag False) so that doors keep their nominal width.
WALLS = [
    # partition, customer part: door D2 [7.25, 8.15] (0.9 m) into the service corridor,
    # toilet door [8.55, 9.45] (customers go straight from the dining room to the toilet,
    # never through the robot's corridor)
    (-2.5, Y_PART, 7.25, Y_PART, True, False),
    (8.15, Y_PART, 8.55, Y_PART, False, False),
    (9.45, Y_PART, X_MAX, Y_PART, False, True),
    # kitchen east wall, staff door [9.1, 10.0] onto the back corridor
    (-2.5, Y_PART, -2.5, 9.1, True, False),
    (-2.5, 10.0, -2.5, Y_MAX, False, True),
    # service corridor: west wall (lounge side) and east wall (storage side), 1.05 m clear.
    # The east wall has two recessed storage doors (niches, see NICHES): a long corridor
    # with two flat parallel walls gives a lidar nothing to measure along its axis.
    (7.1, Y_PART, 7.1, 8.9),
    (8.3, Y_PART, 8.3, 4.4, True, False),
    (8.3, 5.2, 8.3, 7.1, False, False),
    (8.3, 8.3, 8.3, Y_MAX, False, True),
    # lounge north wall = south side of the back corridor, doorway D3 [1.0, 1.9] (0.9 m)
    (-2.5, 8.9, 1.0, 8.9, True, False),
    (1.9, 8.9, 7.1, 8.9, False, True),
    # customer toilet (south end of the former storage room, east of the service
    # corridor): north wall; the storage room behind it stays closed
    (8.3, TOILET_Y_MAX, X_MAX, TOILET_Y_MAX),
]

# Backs of the corridor niches: (x, y_from, y_to), 0.15 m behind the corridor wall.
# Different widths and irregular spacing on purpose: identical, evenly spaced
# features make a corridor look the same at two places (perceptual aliasing).
NICHES = [(8.45, 4.4, 5.2), (8.45, 7.1, 8.3)]

# Static furniture with collision: (name, cx, cy, sx, sy, sz, material)
FURNITURE = [
    # kitchen: pass counters behind the serving windows and the takeaway door
    ('pass_counter_1', -6.5, 3.35, 1.4, 0.55, 0.95, 'Gazebo/Wood'),
    ('pass_counter_2', -5.0, 3.35, 1.3, 0.55, 0.95, 'Gazebo/Wood'),
    ('takeaway_counter', -3.3, 3.4, 1.5, 0.6, 1.0, 'Gazebo/Wood'),
    # cooking line along the north wall (under the extraction hood)
    ('range_1', -9.4, 9.95, 0.9, 0.75, 0.9, 'Gazebo/DarkGrey'),
    ('range_2', -8.45, 9.95, 0.9, 0.75, 0.9, 'Gazebo/DarkGrey'),
    ('fryer', -7.6, 9.95, 0.7, 0.75, 0.9, 'Gazebo/Black'),
    ('griddle', -6.8, 9.95, 0.8, 0.75, 0.9, 'Gazebo/DarkGrey'),
    ('oven', -5.85, 9.95, 1.0, 0.75, 1.5, 'Gazebo/Black'),
    ('salamander_counter', -4.6, 9.95, 1.4, 0.75, 0.9, 'Gazebo/Grey'),
    # cold room + fridges + dry storage on the west wall
    ('cold_room', -9.75, 4.3, 1.3, 2.3, 2.3, 'Gazebo/White'),
    ('fridge_1', -10.05, 6.1, 0.7, 0.8, 2.0, 'Gazebo/White'),
    ('fridge_2', -10.05, 7.0, 0.7, 0.8, 2.0, 'Gazebo/White'),
    ('dry_storage', -10.1, 8.4, 0.6, 1.6, 1.8, 'Gazebo/Wood'),
    # stainless prep tables (island)
    ('prep_table_1', -7.4, 6.9, 2.4, 0.9, 0.9, 'Gazebo/Grey'),
    ('prep_table_2', -4.8, 6.9, 1.8, 0.9, 0.9, 'Gazebo/Grey'),
    # dish area on the east wall
    ('dish_sink', -2.95, 5.6, 0.75, 1.3, 0.9, 'Gazebo/Grey'),
    ('dishwasher', -2.95, 6.7, 0.75, 0.75, 0.9, 'Gazebo/White'),
    ('plate_rack', -2.95, 7.7, 0.75, 1.0, 1.6, 'Gazebo/DarkGrey'),
    # customer toilet: WC against the north wall, wash basin on the east wall
    ('wc_toilet', 9.15, 5.15, 0.4, 0.65, 0.45, 'Gazebo/White'),
    ('wc_sink', 9.3, 3.9, 0.45, 0.5, 0.85, 'Gazebo/White'),
    # VIP lounge sofa
    ('lounge_sofa', 3.0, 3.45, 3.0, 0.7, 0.8, 'Gazebo/Red'),
    # storage room behind the service corridor (north of the toilet)
    ('storage_rack', 8.9, 7.7, 1.0, 2.8, 1.8, 'Gazebo/DarkGrey'),
    # main entrance (south-east): closed glass door
    ('entrance_door', 8.075, -9.686, 1.05, 0.06, 2.2, 'Gazebo/BlueTransparent'),
]

# Queue for the takeaway door: stanchion posts (lidar-visible) with ropes (visual only).
# The posts mark the queue lane x in [-4.0, 1.0], y in [1.8, 2.9] - a keepout zone for the robot.
STANCHIONS = [(x, 1.8) for x in (-4.0, -3.0, -2.0, -1.0, 0.0, 1.0)]   # queue entrance: east end
QUEUE_ZONE = (-4.0, 1.0, 1.8, 2.92)

# Robot lanes painted on the floor (centre lines, world frame). Nav2 prefers them
# (graded cost elsewhere in the dining room, see make_keepout_mask.py).
LANES = [
    ((-8.6, 1.0), (7.7, 1.0)),      # service lane: dock -> serving windows -> service door D2
    ((7.7, 1.0), (7.7, 2.9)),       # into door D2
    ((-4.4, 1.0), (-4.4, -8.6)),    # west aisle
    ((5.8, 1.0), (5.8, -8.6)),      # east aisle
    ((-8.6, -4.4), (6.4, -4.4)),    # middle aisle between the single tables and the groups
    ((-8.6, -8.6), (6.4, -8.6)),    # south aisle
]
LANE_HALF_WIDTH = 0.4

# VIP lounge tables (cafe_table copies; the keepout mask picks every cafe_table up)
VIP_TABLES = [('cafe_table_vip_1', 0.5, 6.0), ('cafe_table_vip_2', 4.5, 6.0)]

# Walking people: name, waypoints (world frame, (x, y) or (x, y, dwell)), speed,
# default dwell, start delay. Each one also listens on /<name>/cmd
# (std_msgs/Float64MultiArray) to be given a new walk at runtime. human_extra_1/2 wait,
# parked in the closed storage room, until a test scenario or a demo sends them.
HUMANS = [
    # staff
    ('human_waiter', [(-7.5, 0.2), (6.0, 0.2)], 0.75, 3.0, 5.0),
    ('human_cook', [(-8.6, 8.9), (-4.6, 8.9)], 0.5, 6.0, 3.0),
    ('human_cook_2', [(-8.4, 4.3), (-3.9, 4.3)], 0.5, 5.0, 8.0),
    # customers
    ('human_guest', [(2.4, -4.2), (2.4, 0.6)], 0.6, 4.0, 12.0),
    ('human_vip', [(6.3, 4.1), (6.3, 8.2), (2.8, 8.2)], 0.55, 6.0, 20.0),
    # a guest going to the toilet (60 s inside) through its own door, next to door D2
    ('human_wc', [(4.0, -1.0, 60.0), (8.95, 1.7), (9.0, 2.7), (8.85, 4.4, 60.0)], 0.7, 1.0, 40.0),
    # takeaway queue: three people waiting (shuffling), one walking in from the entrance
    # along the east wall to the end of the queue and back out with the meal
    ('human_queue_1', [(-3.3, 2.35, 25.0), (-3.0, 2.35, 25.0)], 0.3, 25.0, 0.0),
    ('human_queue_2', [(-2.4, 2.35, 30.0), (-2.1, 2.35, 30.0)], 0.3, 30.0, 6.0),
    ('human_queue_3', [(-1.5, 2.35, 35.0), (-1.2, 2.35, 35.0)], 0.3, 35.0, 11.0),
    ('human_takeaway', [(7.9, -8.9, 10.0), (7.0, -7.8), (7.0, 0.4), (1.6, 2.35), (-0.3, 2.35, 30.0)],
     0.75, 1.0, 15.0),
    # parked for scripted test scenarios
    ('human_extra_1', [(8.95, 9.7), (8.95, 9.72)], 0.5, 1000.0, 0.0),
    ('human_extra_2', [(8.95, 5.95), (8.95, 5.97)], 0.5, 1000.0, 0.0),
]


def visual_link(name, cx, cy, cz, sx, sy, sz, yaw, material):
    geo = '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>' % (sx, sy, sz)
    return f"""
      <link name='{name}'>
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.4f}</pose>
        <visual name='{name}_visual'>{geo}
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script></material>
        </visual>
      </link>"""


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
    for i, w in enumerate(WALLS):
        x1, y1, x2, y2 = w[:4]
        ext1, ext2 = (w[4], w[5]) if len(w) > 4 else (True, True)
        yaw = math.atan2(y2 - y1, x2 - x1)
        ux, uy = math.cos(yaw), math.sin(yaw)
        if ext1:
            x1, y1 = x1 - ux * T / 2, y1 - uy * T / 2
        if ext2:
            x2, y2 = x2 + ux * T / 2, y2 + uy * T / 2
        links.append(box_link('wall_%d' % i, (x1 + x2) / 2, (y1 + y2) / 2, H / 2,
                              math.hypot(x2 - x1, y2 - y1), T, H, yaw, 'Gazebo/Grey'))
    for i, (x, y0, y1) in enumerate(NICHES):
        links.append(box_link('niche_%d' % i, x, (y0 + y1) / 2, H / 2, T, y1 - y0 + T, H, 0.0, 'Gazebo/Wood'))
    # kitchen/dining partition: below the sill and above each opening the wall is
    # continuous; between the openings it is full height
    x = X_MIN
    for i, (name, wx, ww, sill, top) in enumerate(sorted(OPENINGS, key=lambda o: o[1]) + [('end', -2.0, 1.0, 0, 0)]):
        x0, x1 = wx - ww / 2, wx + ww / 2
        if x0 > x:   # full-height pier
            links.append(box_link('partition_pier_%d' % i, (x + x0) / 2, Y_PART, H / 2,
                                  x0 - x + T / 2, T, H, 0.0, 'Gazebo/Grey'))
        if name != 'end':
            links.append(box_link('%s_sill' % name, wx, Y_PART, sill / 2, ww, T, sill, 0.0, 'Gazebo/Grey'))
            links.append(box_link('%s_lintel' % name, wx, Y_PART, (top + H) / 2, ww, T, H - top, 0.0, 'Gazebo/Grey'))
            # pass-through ledge on the kitchen side only (flush with the dining-room
            # face): nothing sticks out at counter height where a 2D lidar is blind
            links.append(box_link('%s_ledge' % name, wx, Y_PART + T / 2, sill + 0.015, ww, T + 0.15, 0.03,
                                  0.0, 'Gazebo/Wood'))
        x = x1
    for name, cx, cy, sx, sy, sz, mat in FURNITURE:
        links.append(box_link(name, cx, cy, sz / 2, sx, sy, sz, 0.0, mat))
    # queue stanchions: posts (collision) + ropes (visual)
    for i, (px, py) in enumerate(STANCHIONS):
        links.append(f"""
      <link name='stanchion_{i}'>
        <pose>{px} {py} 0.5 0 0 0</pose>
        <collision name='stanchion_{i}_collision'><geometry><cylinder><radius>0.03</radius><length>1.0</length></cylinder></geometry></collision>
        <visual name='stanchion_{i}_visual'><geometry><cylinder><radius>0.03</radius><length>1.0</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Gold</name></script></material>
        </visual>
      </link>""")
    for i, ((ax, ay), (bx, by)) in enumerate(zip(STANCHIONS, STANCHIONS[1:])):
        if abs(ax - bx) > 1.5 or abs(ay - by) > 1.5:
            continue
        links.append(visual_link('rope_%d' % i, (ax + bx) / 2, (ay + by) / 2, 0.9, math.hypot(bx - ax, by - ay), 0.03,
                                 0.03, math.atan2(by - ay, bx - ax), 'Gazebo/Red'))
    # extraction hood above the cooking line, signs (visual only)
    links.append(visual_link('hood', -7.2, 9.95, 2.1, 5.2, 0.9, 0.35, 0.0, 'Gazebo/Grey'))
    links.append(visual_link('sign_takeaway', -3.3, Y_PART - 0.09, 2.3, 1.2, 0.02, 0.25, 0.0, 'Gazebo/Orange'))
    links.append(visual_link('sign_wc', TOILET_DOOR[0], Y_PART - 0.09, 2.2, 0.3, 0.02, 0.3, 0.0, 'Gazebo/Blue'))
    links.append(visual_link('sign_vip', 1.45, 8.9 + 0.09, 2.3, 0.9, 0.02, 0.2, 0.0, 'Gazebo/Gold'))
    # floor markings: charging pad, window stops, entrance mat, robot lanes (two edge lines)
    marks = [('mark_dock', ROBOT_DOCK[0], ROBOT_DOCK[1], 0.7, 0.7, 0.0, 'Gazebo/Green'),
             ('mark_entrance', 8.075, -9.1, 1.4, 0.9, 0.0, 'Gazebo/DarkGrey')]
    marks += [('mark_%s' % n, cx, Y_PART - 0.55, 0.6, 0.6, 0.0, 'Gazebo/Yellow') for n, cx, _, _, _ in OPENINGS
              if n.startswith('window')]
    for i, ((ax, ay), (bx, by)) in enumerate(LANES):
        yaw = math.atan2(by - ay, bx - ax)
        length = math.hypot(bx - ax, by - ay)
        nx, ny = -math.sin(yaw), math.cos(yaw)
        for side in (-1, 1):
            marks.append(('lane_%d_%s' % (i, 'l' if side < 0 else 'r'),
                          (ax + bx) / 2 + side * LANE_HALF_WIDTH * nx, (ay + by) / 2 + side * LANE_HALF_WIDTH * ny,
                          length, 0.05, yaw, 'Gazebo/Yellow'))
    for name, cx, cy, sx, sy, yaw, mat in marks:
        links.append(visual_link(name, cx, cy, 0.002, sx, sy, 0.004, yaw, mat))
    return "\n    <model name='tunibot_layout'>\n      <static>1</static>%s\n    </model>" % ''.join(links)


def human_models():
    out = []
    for name, wps, speed, dwell, delay in HUMANS:
        x0, y0 = wps[0][:2]
        wp_xml = ''.join('\n          <waypoint>%s</waypoint>' % ' '.join('%.2f' % v for v in w) for w in wps)
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
      <plugin name='{name}_walker' filename='libtunibot_human_actor.so'>
          <ros><namespace>/{name}</namespace></ros>{wp_xml}
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


# Default gzclient camera: whole venue seen from the south, ~45 deg down (good for videos)
GUI_CAMERA_POSE = '0.0 -17.5 19.0 0 0.86 1.5708'


def generate(base, humans=True):
    text = re.sub(r'\n\s*' + re.escape(BEGIN) + '.*?' + re.escape(END), '', base, flags=re.S)
    text = re.sub(r"(<camera name='user_camera'>\s*<pose>)[^<]*(</pose>)",
                  r'\g<1>%s\g<2>' % GUI_CAMERA_POSE, text)
    # no shadows: on machines without a GPU driver gzclient renders in software and
    # shadows alone cost several CPU cores (starving AMCL/Nav2 during live demos)
    text = re.sub(r'<shadows>1</shadows>', '<shadows>0</shadows>', text)
    text = re.sub(r'(<model name="docking_station">\s*<static>1</static>\s*<pose>)[^<]*(</pose>)',
                  r'\g<1>%s %s 0 0 0 0\g<2>' % DOCK_STATION, text)
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
