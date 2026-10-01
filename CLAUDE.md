# CSTAM-TUNIBOT — Phase 1 (deadline: 1 Oct 2026, late = -5 pts)

Challenge spec: `/home/bani/CSTAM-BOOK.pdf` pages 10-12 (TuniBot "Autonomous Service Robot for Indoor Delivery").
Phase 1 (50 pts): SLAM 15 · Autonomous navigation 15 · Delivery task management 10 · Auto-docking 10.
Deliverables: source repo (ROS 2 ws, nodes, UI backend), high-level architecture diagram, short video of movement + mapping.

## Stack
ROS 2 Humble, Gazebo Classic, Cartographer, Nav2, Python (ament_python). Package: `src/tunibot_delivery` (see its README.md).
User environment: Ubuntu, no passwordless sudo — the user must run `sudo` commands themselves.

## State (as of handoff)
- Package fully written but NEVER BUILT OR RUN. colcon was not installed (`sudo apt install python3-colcon-common-extensions`).
- Build: `cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash`
- `maps/restaurant_map.*` at the workspace root is the OLD, broken map (drift). A new map must be built and saved to
  `src/tunibot_delivery/maps/restaurant_map.{pgm,yaml}` (navigation.launch.py default). Keepout mask already generated there.
- Old package backup: `~/tunibot_delivery_backup_*`.

## Fixes already applied (root causes of the broken first map)
1. Lidar self-occlusion: lidar saw the robot's own rear column → FOV limited to ±145° (rear excluded).
2. No caster wheels → added frictionless front/back casters + wheel friction.
3. Lidar range 3.5 m in a 20x20 m room → 12 m; Cartographer max_range 12.
4. Cardboard boxes were dynamic → made static in `worlds/restaurant.world`; docking station model added at world (-9.75, 0).
5. Cartographer tracking_frame = base_footprint → the old `imu_link` static TF is NOT needed anymore.
6. cafe_table tops invisible to 2D lidar → Nav2 KeepoutFilter, mask from `scripts/make_keepout_mask.py`.

## Frames convention
Robot spawns on the dock at world (-9.0, 0.0, yaw 0) → SLAM map origin = dock. map = world + (9.0, 0.0).
Named poses in `config/delivery_points.yaml` (map frame).

## Run order
1. Mapping: `sim.launch.py` → `slam.launch.py` → teleop (`--ros-args -p speed:=0.15 -p turn:=0.4`) → drive loop, return to dock →
   `ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/tunibot_delivery/maps/restaurant_map --ros-args -p use_sim_time:=true` → rebuild.
2. Delivery: `sim.launch.py` → `navigation.launch.py` → `delivery.launch.py` → http://localhost:8080
   or `ros2 topic pub --once /delivery/order std_msgs/String '{data: "table_3"}'`.

## Remaining TODO
- Build, fix any launch/param errors, remap, test navigation + a full delivery + docking.
- Nav2 params generated from Humble defaults; untested: RotationShim+RPP params, keepout filter, AMCL initial pose.
- git init + push to GitHub (ask user for repo / confirm before pushing).
- User records the video (Ctrl+Shift+Alt+R).
