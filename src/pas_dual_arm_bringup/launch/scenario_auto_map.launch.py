"""Scenario 1: explore and map on its own, then run the mission on that map.

    ros2 launch pas_dual_arm_bringup scenario_auto_map.launch.py

The robot comes up in the driving posture and starts frontier exploration: it
drives to the boundary between the mapped and the unknown, and repeats until
there is none left. Doorways are handled as doorways - detected in the map,
approached square and driven through in a straight line - because a 1.0 m
opening does not forgive a diagonal entry from a robot 0.85 m wide.

You can still steer it: an RViz "2D Goal Pose" is planned like any other goal,
which is the way to force a corner the search skipped.

When the map is good enough, press **MAPIRANJE GOTOVO**. The map is saved, SLAM
is swapped for localisation on it, and the mission node waits for
**MISIJA: po kutiju**. Exploration also stops by itself when no frontier is
left, but saving stays the user's decision.

Exploration needs Nav2 and SLAM at once, which is heavier than either alone. If
the Real Time Factor suffers, run it with headless:=true and watch RViz instead.
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
        DeclareLaunchArgument(
            'explore_mode', default_value='sweep',
            description='sweep (default) or frontier - see map_then_mission.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'map_then_mission.launch.py')),
            launch_arguments={
                'explore': 'true',
                'explore_mode': LaunchConfiguration('explore_mode'),
                'headless': LaunchConfiguration('headless'),
                'quiet': LaunchConfiguration('quiet'),
                'gui': LaunchConfiguration('gui'),
                'open_rviz': LaunchConfiguration('open_rviz'),
                'world': LaunchConfiguration('world'),
                'pick_room': LaunchConfiguration('pick_room'),
                'place_room': LaunchConfiguration('place_room'),
            }.items()),
    ])
