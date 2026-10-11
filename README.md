# Iot-Hunters — TuniBot, autonomous restaurant delivery robot (CSTAM 3.0, challenge CSTAM-TUNIBOT)

ROS 2 Humble workspace: a simulated waiter robot that maps a restaurant, takes orders, collects
the food at the kitchen's serving windows and delivers it among walking people through doorways
and narrow corridors. It manages its battery and returns to its charging dock early enough to
never run out.

| | |
|---|---|
| **Package documentation** | [src/tunibot_delivery/README.md](src/tunibot_delivery/README.md) |
| **Phase 2 design** (dynamic obstacles, complex layout, battery & state) | [src/tunibot_delivery/docs/PHASE2.md](src/tunibot_delivery/docs/PHASE2.md) |
| **Test cases and results** | [src/tunibot_delivery/docs/TEST_CASES.md](src/tunibot_delivery/docs/TEST_CASES.md) |
| **Architecture diagram** | [src/tunibot_delivery/docs/architecture.png](src/tunibot_delivery/docs/architecture.png) |

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch tunibot_delivery demo.launch.py          # Gazebo + RViz + Nav2 + delivery, UI on http://localhost:8080
```

Packages: `src/tunibot_delivery` (nodes, launch, config, Gazebo world, map, web UI, tests) and
`src/tunibot_gazebo_plugins` (walking-person actor plugin visible to the lidar).

Stack: ROS 2 Humble · Gazebo Classic 11 · Cartographer · Nav2 · Python (rclpy) · C++ (Gazebo plugin).
