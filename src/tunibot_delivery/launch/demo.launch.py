"""One-command demo: Gazebo (GUI) + Nav2 (RViz) + delivery stack (+ an optional
scripted scenario), started in the right order.

    ros2 launch tunibot_delivery demo.launch.py                       # then use http://localhost:8080
    ros2 launch tunibot_delivery demo.launch.py scenario:=tc03        # person crossing in front of the robot
    ros2 launch tunibot_delivery demo.launch.py scenario:=tc09        # battery failing mid-mission

Scenarios: see tunibot_delivery/scenario_runner.py and docs/TEST_CASES.md.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tunibot_delivery')
    launch_dir = os.path.join(pkg, 'launch')

    def include(name, **args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
                                        launch_arguments={k: v for k, v in args.items()}.items())

    scenario = LaunchConfiguration('scenario')
    runner = Node(
        package='tunibot_delivery', executable='scenario_runner', output='screen',
        parameters=[{'use_sim_time': True, 'scenario': scenario,
                     'out_dir': LaunchConfiguration('out_dir')}],
        condition=IfCondition(PythonExpression(["'", scenario, "' != ''"])))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='', description='tc01 ... tc12, empty = none'),
        DeclareLaunchArgument('out_dir', default_value=os.path.expanduser('~/tunibot_test_results')),
        DeclareLaunchArgument('time_acceleration', default_value='25.0'),
        include('sim.launch.py', gui=LaunchConfiguration('gui')),
        # explicit paths: inside a TimerAction the included file's own defaults were not
        # applied to the rewritten parameters (map_server: 'yaml_filename' not initialized)
        TimerAction(period=15.0, actions=[include(
            'navigation.launch.py', rviz=LaunchConfiguration('rviz'),
            map=os.path.join(pkg, 'maps', 'restaurant_map.yaml'),
            keepout_mask=os.path.join(pkg, 'maps', 'keepout_mask.yaml'),
            params_file=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            controller='mppi', use_sim_time='true', log_level='info')]),
        TimerAction(period=35.0, actions=[include('delivery.launch.py',
                                                  time_acceleration=LaunchConfiguration('time_acceleration'))]),
        TimerAction(period=45.0, actions=[runner]),
    ])
