import os
import re
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = 'pas_dual_arm_moveit_config'


def load_yaml(relative_path):
    path = os.path.join(get_package_share_directory(PKG), relative_path)
    with open(path) as f:
        return yaml.safe_load(f)


def build_robot_description():
    """Expand the bringup xacro exactly like sim.launch.py does (incl. stripping the
    negative mirror scales that crash DART) so MoveIt and Gazebo share one model."""
    bringup = get_package_share_directory('pas_dual_arm_bringup')
    urdf_file = os.path.join(bringup, 'urdf', 'robot.urdf.xacro')
    xml = subprocess.check_output(['xacro', urdf_file, 'sim_ignition:=true']).decode('utf-8')
    return re.sub(r'(scale="[0-9. ]*)-([0-9])', r'\1\2', xml)


def generate_launch_description():
    # Force Fast DDS locally (see pas_dual_arm_bringup/sim.launch.py for rationale).
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    quiet = LaunchConfiguration('quiet', default='false')
    quiet_arg = DeclareLaunchArgument(
        'quiet', default_value='false',
        description='Log move_group to file instead of screen')
    move_output = PythonExpression(["'log' if '", quiet, "'=='true' else 'screen'"])

    share = get_package_share_directory(PKG)
    with open(os.path.join(share, 'config', 'pas_dual_arm.srdf')) as f:
        srdf = f.read()

    robot_description = {'robot_description': build_robot_description()}
    robot_description_semantic = {'robot_description_semantic': srdf}
    kinematics = {'robot_description_kinematics': load_yaml('config/kinematics.yaml')}
    joint_limits = {'robot_description_planning': load_yaml('config/joint_limits.yaml')}

    ompl_pipeline = load_yaml('config/ompl_planning.yaml')
    planning_pipeline = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_pipeline,
    }

    moveit_controllers = load_yaml('config/moveit_controllers.yaml')

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 2.0,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output=move_output,
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            planning_pipeline,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([rmw_env, zenoh_env, quiet_arg, move_group_node])
