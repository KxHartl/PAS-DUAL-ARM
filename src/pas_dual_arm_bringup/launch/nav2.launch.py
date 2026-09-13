import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_bringup = get_package_share_directory('pas_dual_arm_bringup')
    params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    # Force Fast DDS locally (see sim.launch.py note on rmw_zenoh override).
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    # SLAM (slam_toolbox): builds the map on the fly and provides map->odom.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
        }.items(),
    )

    # Nav2 stack (planner, controller, behaviors, bt_navigator...).
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
        }.items(),
    )

    # Nav2 publishes /cmd_vel, the base controller listens on its own topic
    # inside the gz controller_manager - the relay bridges the two.
    cmd_vel_relay = Node(
        package='pas_dual_arm_scripts',
        executable='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Delay the Nav2 stack so slam_toolbox has time to publish map->odom first.
    # Otherwise local_costmap activates while the TF tree is still split
    # (odom and base_link in unconnected trees) and logs a startup-race error.
    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])

    return LaunchDescription([
        rmw_env,
        zenoh_env,
        slam_launch,
        cmd_vel_relay,
        delayed_nav2,
    ])
