"""Delivery task manager + web UI backend. Run after navigation.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    points = os.path.join(get_package_share_directory('tunibot_delivery'),
                          'config', 'delivery_points.yaml')
    return LaunchDescription([
        Node(package='tunibot_delivery', executable='delivery_manager',
             parameters=[points], output='screen'),
        Node(package='tunibot_delivery', executable='web_api',
             parameters=[{'port': 8080}], output='screen'),
    ])
