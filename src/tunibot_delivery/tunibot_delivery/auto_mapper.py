"""Autonomous mapping tour: drives the robot through the whole venue while
Cartographer builds the map, returns to the dock and saves the map.

Run with auto_mapping.launch.py (Cartographer + Nav2 without AMCL/map_server).
Nav2 plans on the growing SLAM map (unknown space allowed), so the tour only
needs a coarse list of waypoints; each one that cannot be reached is skipped.

Parameters
  map_path   output path without extension (default: ~/tunibot_map)
  waypoints  flat list [x0, y0, x1, y1, ...] in the map frame (default: TOUR)
"""
import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

DOCK_WORLD = (-9.0, 0.0)   # map = world - dock

# Tour of the restaurant, world frame: dining room, kitchen, dining room loop,
# service corridor, west corridor, VIP lounge, back to the dock.
TOUR_WORLD = [
    # dining room north aisle, kitchen door D1, kitchen loop
    (-6.0, 1.0), (-4.0, 1.8), (-4.0, 4.4), (-6.0, 4.4), (-8.6, 4.4), (-8.6, 8.2),
    (-5.0, 8.2), (-3.3, 8.2), (-3.3, 4.4), (-4.0, 4.0), (-4.0, 1.8),
    # dining room loop (east, south, west, middle aisle)
    (0.0, 1.0), (5.5, 1.0), (6.4, -1.0), (6.4, -4.2), (6.4, -8.6), (0.5, -8.6),
    (-8.2, -8.6), (-8.6, -4.2), (-4.5, -4.2), (1.0, -4.2), (5.5, -4.0),
    # door D2, service corridor, L-turn, west corridor
    (7.7, 1.8), (7.7, 4.5), (7.7, 7.6), (7.7, 9.0), (7.2, 9.65), (4.0, 9.65), (2.6, 9.65),
    # look into the lounge through door D3 before entering it
    (1.45, 9.65), (1.45, 8.3), (1.45, 7.0), (-1.4, 7.0), (-1.4, 4.5), (2.5, 4.6), (6.3, 4.6),
    (6.3, 7.0), (2.6, 7.4), (1.45, 8.3), (1.45, 9.65),
    # west end of the corridor and back
    (-1.8, 9.65), (1.45, 9.65), (4.0, 9.65), (7.2, 9.65), (7.7, 9.0), (7.7, 4.5), (7.7, 1.8),
    (0.0, 0.5), (-7.8, 0.0),
]


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator('auto_mapper')
    nav.declare_parameter('map_path', os.path.expanduser('~/tunibot_map'))
    flat = [c for x, y in TOUR_WORLD for c in (x - DOCK_WORLD[0], y - DOCK_WORLD[1])]
    nav.declare_parameter('waypoints', flat)
    map_path = nav.get_parameter('map_path').value
    wp = list(nav.get_parameter('waypoints').value)
    points = list(zip(wp[0::2], wp[1::2]))

    # Cartographer provides map->odom, there is no AMCL: wait for bt_navigator only.
    nav.waitUntilNav2Active(localizer='bt_navigator')
    t0 = time.time()
    reached = 0
    prev = (0.0, 0.0)
    for i, (x, y) in enumerate(points):
        # face the next waypoint: no overshoot towards a wall at corners
        nx, ny = points[i + 1] if i + 1 < len(points) else (0.0, 0.0)
        yaw = math.atan2(ny - y, nx - x) if (nx, ny) != (x, y) else math.atan2(y - prev[1], x - prev[0])
        nav.goToPose(make_pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            time.sleep(0.2)
        ok = nav.getResult() == TaskResult.SUCCEEDED
        reached += ok
        nav.get_logger().info('[%d/%d] (%.1f, %.1f) %s  t=%.0fs' % (
            i + 1, len(points), x, y, 'reached' if ok else 'SKIPPED', time.time() - t0))
        prev = (x, y)

    # Final straight approach onto the dock (map origin), facing +x.
    nav.goToPose(make_pose(nav, 0.0, 0.0, 0.0))
    while not nav.isTaskComplete():
        time.sleep(0.2)
    time.sleep(3.0)
    nav.get_logger().info('Tour done: %d/%d waypoints. Saving map to %s' % (reached, len(points), map_path))
    subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path,
                    '--ros-args', '-p', 'use_sim_time:=true'], check=False)
    nav.get_logger().info('Map saved.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
