"""Delivery task manager + battery simulator + web UI backend.
Run after navigation.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    points = os.path.join(get_package_share_directory('tunibot_delivery'),
                          'config', 'delivery_points.yaml')
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('time_acceleration', default_value='25.0',
                              description='battery energy-flow multiplier (demo speed-up)'),
        DeclareLaunchArgument('initial_soc', default_value='100.0'),
        Node(package='tunibot_delivery', executable='battery_simulator', output='screen',
             parameters=[points, sim, {'time_acceleration': LaunchConfiguration('time_acceleration'),
                                       'initial_soc': LaunchConfiguration('initial_soc')}]),
        Node(package='tunibot_delivery', executable='delivery_manager', output='screen',
             parameters=[points, sim]),
        Node(package='tunibot_delivery', executable='web_api', output='screen',
             parameters=[sim, {'port': LaunchConfiguration('port')}]),
    ])
