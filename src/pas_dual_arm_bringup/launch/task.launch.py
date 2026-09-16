"""Full-task layer: MoveIt2 move_group + Aruco detector + the orchestration node.

Assumes the simulation and Nav2 are already up:
  ros2 launch pas_dual_arm_bringup sim.launch.py
  ros2 launch pas_dual_arm_bringup nav2.launch.py
  ros2 launch pas_dual_arm_bringup task.launch.py

The orchestration node (main_task) is started after a short delay so move_group
and the Aruco detector have time to come up and advertise their actions/TF.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Force Fast DDS locally (see sim.launch.py note on rmw_zenoh override).
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    auto_start = LaunchConfiguration('auto_start')
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description=(
            'Run the legacy main_task orchestration node. Disabled by default '
            'so grasp_cube.py is the only motion owner.'))
    navigate_region = LaunchConfiguration('navigate_region')
    region_x = LaunchConfiguration('region_x')
    region_y = LaunchConfiguration('region_y')
    region_yaw = LaunchConfiguration('region_yaw')
    # Mission mode: wait for the user, then drive through room_navigator and run
    # the whole [MAIL] sequence. mission.launch.py is what turns it on.
    mission = LaunchConfiguration('mission')
    pick_room = LaunchConfiguration('pick_room')
    place_room = LaunchConfiguration('place_room')

    quiet = LaunchConfiguration('quiet', default='false')
    quiet_arg = DeclareLaunchArgument(
        'quiet', default_value='false',
        description='Log background nodes to file instead of screen')

    moveit_dir = get_package_share_directory('pas_dual_arm_moveit_config')
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_dir, 'launch', 'move_group.launch.py')),
        launch_arguments={'quiet': quiet}.items())

    aruco_dir = get_package_share_directory('pas_dual_arm_bringup')
    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aruco_dir, 'launch', 'aruco.launch.py')),
        launch_arguments={'quiet': quiet}.items())

    main_task = Node(
        package='pas_dual_arm_scripts',
        executable='main_task',
        # 'both', not 'screen': after the 16. 9. run the only question that
        # mattered - did it see the place marker? - could not be answered,
        # because the mission's output existed solely in the terminal and the
        # run directory held nothing but launch.log.
        output='both',
        parameters=[{'use_sim_time': True,
                     'navigate_region': navigate_region,
                     'region_x': region_x,
                     'region_y': region_y,
                     'region_yaw': region_yaw,
                     'mission': mission,
                     'pick_room': pick_room,
                     'place_room': place_room}],
        condition=IfCondition(auto_start),
    )
    # Give move_group + aruco a few seconds to advertise before orchestrating.
    # main_task waits for every server it uses anyway, so this only has to cover
    # the gap before the action servers exist; 12 s here was dead time.
    delayed_task = TimerAction(period=3.0, actions=[main_task])

    return LaunchDescription([
        rmw_env,
        zenoh_env,
        auto_start_arg,
        quiet_arg,
        DeclareLaunchArgument('navigate_region', default_value='false'),
        DeclareLaunchArgument('mission', default_value='false'),
        DeclareLaunchArgument('pick_room', default_value='blue'),
        DeclareLaunchArgument('place_room', default_value='red'),
        DeclareLaunchArgument('region_x', default_value='0.0'),
        DeclareLaunchArgument('region_y', default_value='-4.5'),
        DeclareLaunchArgument('region_yaw', default_value='-1.57079632679'),
        move_group,
        aruco,
        delayed_task,
    ])
