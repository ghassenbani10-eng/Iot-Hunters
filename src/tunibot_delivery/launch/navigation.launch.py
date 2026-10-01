"""Nav2 (AMCL localization + navigation) on the saved SLAM map, with the
tabletop keepout filter. Run after sim.launch.py (Cartographer must NOT run)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tunibot_delivery')
    bringup = get_package_share_directory('nav2_bringup')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': params,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )

    keepout_nodes = [
        Node(
            package='nav2_map_server', executable='map_server', name='filter_mask_server',
            parameters=[params, {'yaml_filename': LaunchConfiguration('keepout_mask'),
                                 'use_sim_time': use_sim_time}],
            output='screen'),
        Node(
            package='nav2_map_server', executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            parameters=[params, {'use_sim_time': use_sim_time}], output='screen'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['filter_mask_server', 'costmap_filter_info_server']}],
            output='screen'),
    ]

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'restaurant_map.yaml')),
        DeclareLaunchArgument('keepout_mask', default_value=os.path.join(pkg, 'maps', 'keepout_mask.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        *keepout_nodes,
        nav2,
        rviz,
    ])
