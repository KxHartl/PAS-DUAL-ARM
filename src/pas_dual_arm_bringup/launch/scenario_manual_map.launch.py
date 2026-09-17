"""Scenario 2: map the world by hand, then run the mission on that map.

    ros2 launch pas_dual_arm_bringup scenario_manual_map.launch.py

The robot comes up in the driving posture and SLAM starts, but nothing moves on
its own. Drive it through the rooms:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

or send goals with RViz's "2D Goal Pose" - Nav2 is running and will plan on the
part of the map that exists so far. Watch the map fill in; the driving rules
(stop, turn in place, go straight, take doorways square) are in docs/MAPPING.md.

When the map looks complete, press **MAPIRANJE GOTOVO** in the panel. The map is
saved, SLAM is swapped for localisation on it, and the mission node comes up and
waits for **MISIJA: po kutiju**. No rebuild in between.
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
        DeclareLaunchArgument('pick_room', default_value='blue'),
        DeclareLaunchArgument('place_room', default_value='red'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'map_then_mission.launch.py')),
            launch_arguments={
                'explore': 'false',
                'headless': LaunchConfiguration('headless'),
                'quiet': LaunchConfiguration('quiet'),
                'gui': LaunchConfiguration('gui'),
                'open_rviz': LaunchConfiguration('open_rviz'),
                'world': LaunchConfiguration('world'),
                'pick_room': LaunchConfiguration('pick_room'),
                'place_room': LaunchConfiguration('place_room'),
            }.items()),
    ])
