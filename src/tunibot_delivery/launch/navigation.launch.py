"""TuniBot navigation stack (Phase 2): AMCL localization on the saved SLAM map +
Nav2 + safety and dynamic-obstacle layers. Run after sim.launch.py.

Velocity command chain (every motion command goes through the collision monitor):

  controller_server --cmd_vel_nav--> velocity_smoother --cmd_vel_raw--> collision_monitor --cmd_vel--> robot
  behavior_server (spin / backup / wait) -----------------cmd_vel_raw--^

Costmap filters:
  keepout_filter : static mask of the table tops the 2D lidar cannot see
  people_filter  : dynamic cost mask of where tracked people will be (people_tracker)
Speed limit (/speed_limit, nav2_msgs/SpeedLimit): people_tracker = social speed
governor (slower near people, in doorways and corridors).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

LIFECYCLE_NAV = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                 'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor']


def generate_launch_description():
    pkg = get_package_share_directory('tunibot_delivery')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'use_sim_time': use_sim_time,
                        'yaml_filename': LaunchConfiguration('map'),
                        'default_nav_to_pose_bt_xml': os.path.join(pkg, 'config', 'navigate_tunibot.xml')},
        convert_types=True)
    tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def nav_node(package, executable, name=None, remaps=(), extra=()):
        return Node(package=package, executable=executable, name=name or executable, output='screen',
                    parameters=[params, *extra], remappings=tf_remap + list(remaps),
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')])

    # controller:=mppi -> MPPI overlay (config/nav2_mppi.yaml) on top of nav2_params.yaml
    mppi = PythonExpression(["'", os.path.join(pkg, 'config', 'nav2_mppi.yaml'), "' if '",
                             LaunchConfiguration('controller'), "' == 'mppi' else '",
                             os.path.join(pkg, 'config', 'nav2_controller_default.yaml'), "'"])

    localization = [
        nav_node('nav2_map_server', 'map_server'),
        nav_node('nav2_amcl', 'amcl'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ]
    navigation = [
        nav_node('nav2_controller', 'controller_server', remaps=[('cmd_vel', 'cmd_vel_nav')], extra=[mppi]),
        nav_node('nav2_smoother', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', remaps=[('cmd_vel', 'cmd_vel_raw')]),
        nav_node('nav2_bt_navigator', 'bt_navigator'),
        nav_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother',
                 remaps=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_raw')]),
        nav_node('nav2_collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': LIFECYCLE_NAV}]),
    ]
    filters = [
        Node(package='nav2_map_server', executable='map_server', name='filter_mask_server',
             parameters=[params, {'yaml_filename': LaunchConfiguration('keepout_mask')}], output='screen'),
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='costmap_filter_info_server', parameters=[params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_costmap_filters', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['filter_mask_server', 'costmap_filter_info_server']}]),
        Node(package='tunibot_delivery', executable='people_tracker', name='people_tracker',
             parameters=[params], output='screen'),
    ]
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'tunibot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('controller', default_value='rpp', description='rpp | mppi'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'restaurant_map.yaml')),
        DeclareLaunchArgument('keepout_mask', default_value=os.path.join(pkg, 'maps', 'keepout_mask.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        *localization, *navigation, *filters, rviz,
    ])
