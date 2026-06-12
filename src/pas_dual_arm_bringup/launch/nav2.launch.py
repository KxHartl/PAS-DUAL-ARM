import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Force Fast DDS locally (see sim.launch.py note on rmw_zenoh override).
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    # We use slam=True to map on the fly, since we don't have a pre-saved map.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'slam_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([
        rmw_env,
        zenoh_env,
        slam_launch,
        nav2_launch
    ])
