"""Scenario 3: the mission only, on the map that ships with the repository.

    ros2 launch pas_dual_arm_bringup scenario_mission.launch.py

This is the demo: no mapping, no SLAM. The robot spawns at the origin, which is
where the checked-in map expects it, AMCL localises against that map, and the
mission node waits for the button.

The other two scenarios build a map first:
    scenario_manual_map.launch.py   drive by hand, then run the mission
    scenario_auto_map.launch.py     explore on its own, then run the mission
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('pas_dual_arm_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('quiet', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(bringup, 'worlds', 'seminar_world.sdf')),
        DeclareLaunchArgument('debug_truth', default_value='false'),
        DeclareLaunchArgument('laser_odometry', default_value='false'),
        DeclareLaunchArgument('pick_room', default_value='blue'),
        DeclareLaunchArgument('place_room', default_value='red'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(bringup, 'maps', 'seminar_map.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'mission.launch.py')),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'quiet': LaunchConfiguration('quiet'),
                'gui': LaunchConfiguration('gui'),
                'open_rviz': LaunchConfiguration('open_rviz'),
                'world': LaunchConfiguration('world'),
                'debug_truth': LaunchConfiguration('debug_truth'),
                'laser_odometry': LaunchConfiguration('laser_odometry'),
                'pick_room': LaunchConfiguration('pick_room'),
                'place_room': LaunchConfiguration('place_room'),
                'map': LaunchConfiguration('map'),
            }.items()),
    ])
