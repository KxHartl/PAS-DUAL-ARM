import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

PKG = 'pas_dual_arm_moveit_config'


def load_yaml(relative_path):
    path = os.path.join(get_package_share_directory(PKG), relative_path)
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    # Reuse the robot_description builder from move_group.launch.py
    import importlib.util
    share = get_package_share_directory(PKG)
    spec = importlib.util.spec_from_file_location(
        'mg', os.path.join(share, 'launch', 'move_group.launch.py'))
    mg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mg)

    with open(os.path.join(share, 'config', 'pas_dual_arm.srdf')) as f:
        srdf = f.read()

    rviz_config = os.path.join(share, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': mg.build_robot_description()},
            {'robot_description_semantic': srdf},
            {'robot_description_kinematics': load_yaml('config/kinematics.yaml')},
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([rmw_env, zenoh_env, rviz_node])
