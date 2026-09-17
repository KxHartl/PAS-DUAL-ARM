"""The whole assignment in one command (user, 16. 9.).

The robot spawns in the home room with its arms spread (the bare spawn posture),
the saved map comes up with AMCL, and the mission node folds the arms into
DRIVE_V4 and then WAITS. Press "MISIJA: po kutiju" in the navigation panel (or
publish on /mission/start) and it drives to the blue room's table, picks the cube
up, carries it through the doorway and places it in the red room.

    ros2 launch pas_dual_arm_bringup mission.launch.py

Mapping is a separate job (mapping.launch.py); this one navigates on the saved
map from run 60. Procedure and what to watch: notes/00_run/00_testing/misija.md.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # sim.launch.py reads the arm spawn profile from its own command line or from
    # the environment, never from launch arguments, so an include has to use the
    # environment (grasp_stage.launch.py does the same). carry_arms=false is what
    # spawns the arms spread: the mission node folds them into DRIVE_V4 itself,
    # in view, before anything drives.
    os.environ['PAS_SIM_TABLE_ARMS'] = 'false'
    os.environ['PAS_SIM_CARRY_ARMS'] = 'false'

    bringup = get_package_share_directory('pas_dual_arm_bringup')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'quiet': LaunchConfiguration('quiet'),
            # One RViz for the run, started by this launch (see `rviz` below).
            # Two views cost frames the controllers need (P-32).
            'rviz': 'false',
            # AMCL's initial pose is (0, 0, 0) in nav2_params.yaml, so spawning
            # there is what lines the robot up with the saved map without anyone
            # setting a pose by hand.
            'robot_spawn_x': '0.0',
            'robot_spawn_y': '0.0',
            'robot_spawn_yaw': '0.0',
            'world': LaunchConfiguration('world'),
        }.items())
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'mode': 'localization',
            'map': LaunchConfiguration('map'),
            'quiet': LaunchConfiguration('quiet'),
            # This launch starts RViz itself (below). Handing the job down to
            # nav2.launch.py left the run with no RViz at all on 16. 9. - not one
            # mention of it in launch.log - so the mission owns it directly.
            'rviz': 'false',
            'gui': LaunchConfiguration('gui'),
        }.items())
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        # NOT called 'rviz': sim.launch.py and nav2.launch.py both declare an
        # argument by that name and this launch passes 'false' to both of them.
        # Sharing the name is why no RViz came up on 16. 9. - two runs, not one
        # rviz2 process in the logs, while the same RViz starts fine when
        # nav2.launch.py is run on its own.
        condition=IfCondition(LaunchConfiguration('open_rviz')),
        output='log',
    )
    task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'task.launch.py')),
        launch_arguments={
            'auto_start': 'true',
            'mission': 'true',
            'quiet': LaunchConfiguration('quiet'),
            'pick_room': LaunchConfiguration('pick_room'),
            'place_room': LaunchConfiguration('place_room'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('quiet', default_value='true',
                              description='Suppress noisy background node output from terminal (logs still saved)'),
        DeclareLaunchArgument('open_rviz', default_value='true',
                              description='Open RViz for the run'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(bringup, 'maps', 'seminar_map.yaml')),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(bringup, 'worlds', 'seminar_world.sdf'),
            description='SDF world; a generated layout needs its own map too'),
        DeclareLaunchArgument('pick_room', default_value='blue'),
        DeclareLaunchArgument('place_room', default_value='red'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(bringup, 'rviz', 'nav2.rviz'),
            description='RViz view for the run; rviz/cube.rviz shows the grasp poses'),
        sim,
        # Nav2 once the controllers are up, then the task layer (move_group, the
        # ArUco detectors and the mission node, which waits for the user anyway).
        # These were 12 s and 22 s: dead time before anything happens, and the
        # run already feels slow enough (user, 16. 9.).
        TimerAction(period=6.0, actions=[nav2]),
        TimerAction(period=8.0, actions=[rviz]),
        TimerAction(period=10.0, actions=[task]),
    ])
