"""Experiment: hold the box by friction instead of a rigid joint.

    ros2 launch pas_dual_arm_bringup scenario_friction_grasp.launch.py

This is **not** how the delivered mission works, and it is kept separate for that
reason. The mission drives every joint on the `position` interface and holds the
box with a rigid joint engaged only after contact on both hands is confirmed.
This scenario switches the arms and the carriages to the `effort` interface,
removes the rigid joint entirely, and tries to carry the box on pad friction
alone.

Why it is an experiment and not the answer:

  * a position-driven joint in Gazebo follows its command with no force limit at
    all, so a squeeze can develop a torque the real Gen3 could never produce.
    The effort profile exists to stop pretending otherwise.
  * there is no independent reference for the contact force. In Fortress the
    contact message carries contact *points* with empty wrenches - in one
    measurement, 49 035 records with zero force values - so the wrist
    force-torque sensors are the only reference, and they are not what a real
    Gen3 has either.

Running it produces data, not a demonstration. Judge it with
`scripts/qualify_force_log.py`, and expect a negative result to be a result: a
measured "friction does not hold it" is worth more than an unmeasured claim
that it does.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('pas_dual_arm_bringup')
    # sim.launch.py reads this profile off its own command line, so the flag has
    # to be on argv; an include cannot pass it as a launch argument.
    os.environ['PAS_SIM_TABLE_ARMS'] = 'true'
    os.environ['PAS_SIM_CARRY_ARMS'] = 'false'

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('open_rviz'),
            'force_grasp': 'true',
            'robot_spawn_y': '-5.479',      # already at the table; no driving here
        }.items())
    estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'force_estimation.launch.py')),
        launch_arguments={'record': LaunchConfiguration('record'),
                          'output': LaunchConfiguration('output')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument('record', default_value='true',
                              description='log wrench estimates and contacts for qualification'),
        DeclareLaunchArgument('output', default_value='log/grasp-force.jsonl'),
        sim,
        TimerAction(period=8.0, actions=[estimation]),
    ])
