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
            'marker_size': 0.165,  # 0.22 m plate on the box, marker fills 75% -> 0.165 m
            'image_topic': '/camera/image',
            'camera_info_topic': '/camera/camera_info',
            'marker_frame': 'aruco_marker_frame',
            'use_sim_time': True,
        }],
    )

    # One detector per wrist camera. The head marker (id 0, 0.165 m on the
    # front face) puts the hands at the pre-grasp pose; these read the smaller
    # markers on the two pressed faces - id 1 for the left hand, id 2 for the
    # right - so the last stretch is steered by what the hand itself sees
    # rather than by dead reckoning from a measurement taken a metre away.
    wrist_detectors = [
        Node(
            package='pas_dual_arm_scripts',
            executable='aruco_detector',
            name=f'aruco_detector_{side}_wrist',
            output='screen',
            parameters=[{
                'marker_id': marker_id,
                # 0.12 m plate on the face, marker fills 75% -> 0.09 m.
                'marker_size': 0.09,
                'image_topic': f'/wrist_{side}/image',
                'camera_info_topic': f'/wrist_{side}/camera_info',
                'marker_frame': f'{side}_grasp_marker_frame',
                'pose_topic': f'/wrist_{side}/marker_pose',
                'use_sim_time': True,
            }],
        )
        for side, marker_id in (('left', 1), ('right', 2))
    ]

    # The place target in the red room (user, 16. 9.): a 0.30 m marker lying flat
    # on the table, read by the HEAD camera from the dock. Its centre is where the
    # centre of the cube's bottom face has to end up. Read from the dock rather
    # than up close - a marker seen at a shallow angle from 20 cm foreshortens
    # (P-19), and from the dock the head camera looks down on it at ~40 deg.
    place_detector = Node(
        package='pas_dual_arm_scripts',
        executable='aruco_detector',
        name='aruco_detector_place',
        output='screen',
        parameters=[{
            'marker_id': 3,
            # 0.40 m plate with a 5% quiet zone -> 0.36 m of marker. Bigger than
            # the box markers' 75% on purpose: the 0.30 m cube lands on top of
            # this one, so a 0.30 m marker would end up completely hidden and
            # 3 cm of marker has to stick out past the cube to be seen at all.
            'marker_size': 0.36,
            'image_topic': '/camera/image',
            'camera_info_topic': '/camera/camera_info',
            'marker_frame': 'place_marker_frame',
            'pose_topic': '/place_marker/pose',
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([rmw_env, zenoh_env, detector, *wrist_detectors,
                              place_detector])
