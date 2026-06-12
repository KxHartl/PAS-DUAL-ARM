import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('pas_dual_arm_bringup')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Force Fast DDS locally (user's ~/.bashrc sets rmw_zenoh_cpp + an external
    # router override that is unreachable locally and crashes every node).
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                      description='Absolute path to robot urdf file')
    
    robot_description = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rmw_env,
        zenoh_env,
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
