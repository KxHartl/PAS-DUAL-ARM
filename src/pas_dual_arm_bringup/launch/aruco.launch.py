from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Force Fast DDS locally (see sim.launch.py note on rmw_zenoh override).
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    # Custom OpenCV-based detector: the bundled aruco_ros library has no OpenCV
    # DICT_4X4_50 dictionary, so it can never decode the marker chosen for this
    # assignment (see pas_dual_arm_scripts/aruco_detector.py).
    detector = Node(
        package='pas_dual_arm_scripts',
        executable='aruco_detector',
        output='screen',
        parameters=[{
            'marker_id': 0,
            'marker_size': 0.195,  # 0.26 m sign panel, marker fills 75% -> 0.195 m
            'image_topic': '/camera/image',
            'camera_info_topic': '/camera/camera_info',
            'marker_frame': 'aruco_marker_frame',
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([rmw_env, zenoh_env, detector])
