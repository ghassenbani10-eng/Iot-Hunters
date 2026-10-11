"""Autonomous SLAM mapping: Cartographer + Nav2 (no AMCL / map_server) + a
waypoint tour of the venue, then the map is saved.

    ros2 launch tunibot_delivery sim.launch.py world:=<pkg>/worlds/restaurant_mapping.world
    ros2 launch tunibot_delivery auto_mapping.launch.py map_path:=<pkg_src>/maps/restaurant_map

The tabletop keepout filter is active during the tour, so the robot never drives
under a table top that the 2D lidar cannot see.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('tunibot_delivery')
    bringup = get_package_share_directory('nav2_bringup')

    # Cartographer's occupancy grid is published as volatile, and the planner
    # must be allowed to plan through not-yet-mapped space.
    params = RewrittenYaml(
        source_file=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        param_rewrites={'map_subscribe_transient_local': 'False', 'allow_unknown': 'True',
                        'desired_linear_vel': '0.25',
                        'default_nav_to_pose_bt_xml': os.path.join(pkg, 'config', 'navigate_tunibot.xml')},
        convert_types=True)

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items())
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': params, 'use_sim_time': 'true', 'autostart': 'true'}.items())
    keepout = [
        Node(package='nav2_map_server', executable='map_server', name='filter_mask_server',
             parameters=[params, {'yaml_filename': os.path.join(pkg, 'maps', 'keepout_mask.yaml')}]),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_costmap_filters',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['filter_mask_server', 'costmap_filter_info_server']}]),
    ]
    mapper = Node(package='tunibot_delivery', executable='auto_mapper', output='screen',
                  parameters=[{'use_sim_time': True, 'map_path': LaunchConfiguration('map_path')}])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map_path', default_value=os.path.expanduser('~/tunibot_map')),
        slam, nav2, *keepout, mapper,
    ])
