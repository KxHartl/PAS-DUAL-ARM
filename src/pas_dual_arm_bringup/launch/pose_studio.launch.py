"""Simulation for moving the robot by hand, and nothing that moves it on its own.

Opens Gazebo, RViz, MoveIt and the joint window (`scripts/joint_gui.py --sim`,
with its saved-poses window). The robot spawns at the table's dock pose in
DRIVE_V4 (the drive posture, arms tucked) and stays there until you command it. The only automatic action is
releasing the cube from the left wrist once, because the DetachableJoint starts
attached. Drive the base yourself with teleop_twist_keyboard.

    bash scripts/run_cube_isolated.sh ros2 launch pas_dual_arm_bringup pose_studio.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

# The table's dock pose, where the mission stands: the cube is 0.87 m ahead.
# DRIVE_V4 reaches only 0.27 m ahead, 36 cm clear of the table here.
SPAWN_Y = '-5.479'


def generate_launch_description():
    # sim.launch.py reads these from the environment when it is included.
    os.environ['PAS_SIM_TABLE_ARMS'] = 'false'
    os.environ['PAS_SIM_CARRY_ARMS'] = 'true'
    os.environ['PAS_SIM_AUTO_POSTURE'] = 'false'
    root = os.environ.get('PAS_DUAL_ARM_ROOT', os.getcwd())

    bringup = get_package_share_directory('pas_dual_arm_bringup')
    moveit = get_package_share_directory('pas_dual_arm_moveit_config')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': 'false', 'rviz': 'true', 'robot_spawn_x': '0.0',
                          'robot_spawn_y': SPAWN_Y, 'robot_spawn_yaw': '-1.5708'}.items())
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit, 'launch', 'move_group.launch.py')))
    # Once the robot exists (its joint states are published), release the cube.
    release_cube = ExecuteProcess(
        cmd=['bash', '-c',
             'until timeout 5 ros2 topic echo --once /joint_states >/dev/null 2>&1; do sleep 2; done; '
             'for i in 1 2 3; do ign topic -t /aruco_box/detach -m ignition.msgs.Empty -p "unused: true"; '
             'sleep 2; done; echo "kocka otpustena sa zapesca"'],
        output='screen')
    joint_window = ExecuteProcess(
        cmd=['python3', os.path.join(root, 'scripts', 'joint_gui.py'), '--sim'],
        output='screen')
    return LaunchDescription([
        sim,
        TimerAction(period=10.0, actions=[move_group, release_cube]),
        TimerAction(period=15.0, actions=[joint_window]),
    ])
