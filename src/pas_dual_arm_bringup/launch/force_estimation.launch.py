"""Sensor-realistic estimator; evaluation recorder is a separate opt-in node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('output', default_value='log/grasp-force.jsonl'),
        DeclareLaunchArgument('effort_sign', default_value='1.0'),
        Node(package='pas_dual_arm_scripts', executable='wrench_estimator',
             parameters=[{'use_sim_time': True, 'effort_sign': ParameterValue(
                 LaunchConfiguration('effort_sign'), value_type=float)}], output='screen'),
        Node(package='pas_dual_arm_scripts', executable='grasp_force_diagnostics',
             condition=IfCondition(LaunchConfiguration('record')),
             parameters=[{'use_sim_time': True, 'output': LaunchConfiguration('output')}],
             output='screen'),
    ])
