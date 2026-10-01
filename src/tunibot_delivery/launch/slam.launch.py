"""Cartographer SLAM (mapping) + RViz. Run after sim.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tunibot_delivery')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_cartographer'), 'rviz', 'tb3_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', os.path.join(pkg, 'config'),
                       '-configuration_basename', 'cartographer_2d.lua'],
            output='screen',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
