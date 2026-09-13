"""SLAM mapping of the three-room world: slam_toolbox + RViz, nothing else.

Deliberately does NOT start Nav2. While mapping there is nothing to plan, and
the full stack competes for CPU with the gz_ros2_control update loop, which is
what starved the controllers in P-32. Navigation is a separate launch.

The base is driven either by `mapping_tour` (repeatable, scripted) or by hand
with teleop_twist_keyboard on /cmd_vel. Both rely on the cmd_vel_relay that
sim.launch.py already starts.

Usage:
    ros2 launch pas_dual_arm_bringup sim.launch.py
    ros2 launch pas_dual_arm_bringup mapping.launch.py
    ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2   # before driving!
    ros2 run pas_dual_arm_scripts mapping_tour
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('pas_dual_arm_bringup')
    slam_params = os.path.join(pkg_bringup, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'mapping.rviz')

    # Force Fast DDS locally (see sim.launch.py note on the rmw_zenoh override).
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz with the mapping view.')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    features = Node(
        package='pas_dual_arm_scripts', executable='feature_registry',
        output='screen', parameters=[{'use_sim_time': True}],
    )
    detach_box = ExecuteProcess(
        cmd=['ign', 'topic', '-t', '/aruco_box/detach',
             '-m', 'ignition.msgs.Empty', '-p', 'unused: true'],
        output='screen',
    )

    return LaunchDescription([rmw_env, zenoh_env, rviz_arg, detach_box, slam, features, rviz])
