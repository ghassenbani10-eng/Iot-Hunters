"""Gazebo restaurant world + waiter robot, spawned on its docking station."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Dock pose in the Gazebo world frame. The robot starts here, so the SLAM map
# origin (0, 0, 0) is the dock. delivery_points.yaml uses the same convention.
DOCK_X, DOCK_Y, DOCK_YAW = '-9.0', '0.0', '0.0'


def generate_launch_description():
    pkg = get_package_share_directory('tunibot_delivery')
    world = os.path.join(pkg, 'worlds', 'restaurant.world')
    xacro_file = os.path.join(pkg, 'urdf', 'waiter_robot.urdf.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'waiter_robot', '-topic', 'robot_description',
                   '-x', DOCK_X, '-y', DOCK_Y, '-z', '0.01', '-Y', DOCK_YAW],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn,
    ])
