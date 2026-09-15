"""Robot in front of the cube table, both hands in the side-scanning pre-grasp.

One command for designing grasp poses by hand. The robot spawns where the
mission leaves it - at the table's dock pose, 0.87 m from the cube - in
ARM_CARRY_V2, the posture it maps, localises and drives in. MoveIt and the
ArUco detectors come up, and `grasp_stage` raises the carriages, brings both
open hands into the side-scanning pose, drives up to 0.62 m and checks that
each wrist camera reads the marker on its face. Poses are then dialled in with
`scripts/joint_gui.py --sim`.

    bash scripts/run_cube_isolated.sh ros2 launch pas_dual_arm_bringup grasp_stage.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The table's dock pose (nav_zones.py, hvat_kocke.md): the cube is 0.87 m ahead.
# ARM_CARRY_V2 must start here, not closer: at 0.62 m its hands would be inside
# the tabletop (measured 0.0 cm); here they clear it by 15 cm.
DOCK_Y = -5.479


def generate_launch_description():
    # sim.launch.py picks the arm spawn profile from its command line or the
    # environment, not from launch arguments, so an include has to use the
    # environment: the drive posture, no table staging.
    os.environ['PAS_SIM_TABLE_ARMS'] = 'false'
    os.environ['PAS_SIM_CARRY_ARMS'] = 'true'

    bringup = get_package_share_directory('pas_dual_arm_bringup')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('rviz'),
            'robot_spawn_x': '0.0',
            'robot_spawn_y': LaunchConfiguration('robot_spawn_y'),
            'robot_spawn_yaw': '-1.5708',
        }.items())
    # MoveIt + ArUco, without the mission node: grasp_stage and then the person
    # at joint_gui are the only ones moving the arms.
    task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'task.launch.py')),
        launch_arguments={'auto_start': 'false'}.items())
    stage = Node(
        package='pas_dual_arm_scripts',
        executable='grasp_stage',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('robot_spawn_y', default_value=f'{DOCK_Y:.3f}'),
        sim,
        TimerAction(period=10.0, actions=[task]),
        # grasp_stage waits for the controllers and for the carry fold itself.
        TimerAction(period=20.0, actions=[stage]),
    ])
