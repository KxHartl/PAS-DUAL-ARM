"""Build a map, then run the mission on it - in one command.

Used by both mapping scenarios; ``explore:=true`` is the only difference between
them. Not usually launched directly:

    scenario_manual_map.launch.py   drive by hand   (explore:=false)
    scenario_auto_map.launch.py     frontier search (explore:=true)

**Why the handover is built the way it is.** SLAM and AMCL both publish
``map -> odom``. Running them at once puts two publishers on one transform and
the tree starts flickering between them, so the switch has to be a real switch:
slam_toolbox is stopped *before* map_server and AMCL come up. A launch file can
only shut down a process it started itself, which is why this file owns the
slam_toolbox node instead of letting ``nav2.launch.py`` start it
(``mode:=none``).

The sequence is driven by ``map_handoff`` exiting:

    mapping ... user presses the button ... map_handoff saves the map and exits
        -> slam_toolbox is shut down
        -> map_server + AMCL come up on the map just written
        -> seed_pose tells AMCL where mapping ended
        -> the mission node starts and waits for "MISIJA: po kutiju"

The map is written to a runtime directory, not into ``src/``, so no rebuild is
needed between mapping and driving: Nav2 loads its default map out of
``install/.../share``, but it will load any absolute path it is handed.

If map_handoff exits non-zero (nothing to save, or the save failed) the mission
is not started. Driving on a map that was never written is worse than stopping.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

RUNTIME_DIR = os.path.expanduser('~/.ros/pas_dual_arm')
MAP_NAME = 'live_map'


def generate_launch_description():
    bringup = get_package_share_directory('pas_dual_arm_bringup')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(bringup, 'config', 'nav2_params.yaml')

    # Mapping is driven with the arms already tucked in: in the spread spawn
    # posture the robot is wider than the opening and catches on the frame.
    os.environ['PAS_SIM_TABLE_ARMS'] = 'false'
    os.environ['PAS_SIM_CARRY_ARMS'] = 'true'

    live_map = os.path.join(RUNTIME_DIR, f'{MAP_NAME}.yaml')
    live_pose = os.path.join(RUNTIME_DIR, f'{MAP_NAME}.pose.yaml')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'quiet': LaunchConfiguration('quiet'),
            'rviz': 'false',
            'robot_spawn_x': '0.0',
            'robot_spawn_y': '0.0',
            'robot_spawn_yaw': '0.0',
            'world': LaunchConfiguration('world'),
        }.items())

    # This launch owns the SLAM node so it can stop it at the handover.
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='log',
        parameters=[os.path.join(bringup, 'config', 'slam_params.yaml'),
                    {'use_sim_time': True}],
    )

    # Nav2 without a map source. Exploration needs a planner, and so does the
    # user sending a goal by hand to fill in a corner the tour missed.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'mode': 'none',
            'quiet': LaunchConfiguration('quiet'),
            'rviz': 'false',
            'gui': LaunchConfiguration('gui'),
            # Zones are derived FROM the map, so they cannot be built WHILE the
            # map is being built. On 18 Sep, half way through mapping, nav_zones
            # reported "1 room(s) ['home'], 1 door(s), 5 table(s)" - five tables
            # in a world that has two, found in wall fragments the scan had not
            # joined up yet - and put a keepout halo around each. The pose in
            # front of the doorway landed inside one, so the controller spent
            # its full 30 s allowance failing to move and aborted, twice.
            #
            # They come up after the handover instead, computed once on the
            # finished map, which is the only map they were ever valid for.
            'zones': 'false',
        }.items())

    # Started with the zones, because it drives off their graph.
    zones_nodes = [
        Node(package='pas_dual_arm_scripts', executable='nav_zones',
             name='nav_zones', output='log', parameters=[{'use_sim_time': True}]),
        Node(package='pas_dual_arm_scripts', executable='room_navigator',
             name='room_navigator', output='both', parameters=[{'use_sim_time': True}]),
    ]

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(bringup, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('open_rviz')),
        output='log',
    )

    # Two ways of deciding where to drive while mapping, and the default is the
    # second one. Frontier search steers at the centroid of the boundary between
    # mapped and unknown, which is the right idea when the unknown is *beyond*
    # sensor range; here the lidar outranges the room by 4x, so the boundary is
    # a ring of occlusion shadows around the robot and its centroid is the robot
    # (measured 17 Sep: one cluster of 4127 cells, goal 0.83 m away, 90 s spent
    # on it). Coverage decides the whole route before moving, and on a mecanum
    # base it runs without a single turn in place.
    explore_mode = LaunchConfiguration('explore_mode')
    sweeping = PythonExpression(["'", LaunchConfiguration('explore'),
                                 "'=='true' and '", explore_mode, "'=='sweep'"])
    frontier = PythonExpression(["'", LaunchConfiguration('explore'),
                                 "'=='true' and '", explore_mode, "'=='frontier'"])
    sweeper = Node(
        package='pas_dual_arm_scripts', executable='room_sweeper',
        name='room_sweeper', output='both',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(sweeping),
    )
    explorer = Node(
        package='pas_dual_arm_scripts', executable='frontier_explorer',
        name='frontier_explorer', output='both',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(frontier),
    )

    handoff = Node(
        package='pas_dual_arm_scripts', executable='map_handoff',
        name='map_handoff', output='both',
        parameters=[{'use_sim_time': True,
                     'map_dir': RUNTIME_DIR,
                     'map_name': MAP_NAME}],
    )

    # --- what happens after the map is saved ------------------------------
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'map': live_map,
                          'params_file': params_file}.items())
    seed = Node(
        package='pas_dual_arm_scripts', executable='seed_pose',
        name='seed_pose', output='both',
        parameters=[{'use_sim_time': True, 'pose_file': live_pose}],
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

    on_map_saved = RegisterEventHandler(OnProcessExit(
        target_action=handoff,
        # Only on a clean exit. A failed save must not be followed by a mission.
        on_exit=lambda event, context: (
            [LogInfo(msg='Map saved. Stopping SLAM and switching to localisation.'),
             EmitEvent(event=ShutdownProcess(process_matcher=matches_action(slam))),
             # Give slam_toolbox a moment to drop map -> odom before AMCL claims it.
             TimerAction(period=4.0, actions=[localization, seed]),
             # After map_server is up, so the first /map they see is the saved
             # one rather than the last SLAM frame.
             TimerAction(period=7.0, actions=zones_nodes),
             # MoveIt and the mission node last: by then the robot is localised.
             TimerAction(period=10.0, actions=[task])]
            if event.returncode == 0 else
            [LogInfo(msg='Map handoff failed; the mission will not be started. '
                         'The simulation stays up so the map can be inspected.')]),
    ))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('quiet', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument('explore', default_value='false',
                              description='true: the robot maps on its own'),
        DeclareLaunchArgument(
            'explore_mode', default_value='sweep',
            description="sweep: cover each room lane by lane, then step through "
                        "its doorways (default). frontier: the earlier "
                        "frontier-centroid search, kept for comparison."),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(bringup, 'worlds', 'seminar_world.sdf'),
            description='SDF world to map and then run the mission in'),
        DeclareLaunchArgument('pick_room', default_value='blue'),
        DeclareLaunchArgument('place_room', default_value='red'),
        sim,
        TimerAction(period=6.0, actions=[slam, nav2]),
        TimerAction(period=8.0, actions=[rviz]),
        TimerAction(period=12.0, actions=[handoff, sweeper, explorer]),
        on_map_saved,
    ])
