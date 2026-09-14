import os
import re
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_bringup = get_package_share_directory('pas_dual_arm_bringup')

    # Force Fast DDS for local sim. The user's ~/.bashrc globally sets rmw_zenoh_cpp
    # with a ZENOH_CONFIG_OVERRIDE pointing at an external router (192.168.0.14:7447)
    # that is not available locally; without this override every node aborts on startup.
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    # Let Ignition resolve package:// mesh URIs. Kinova meshes are emitted as absolute
    # file:// paths, but the omni_base, torso and pan_tilt meshes stay as package://,
    # which Fortress cannot resolve unless every package's share dir is on the resource
    # path. Without this, the base/torso/pan-tilt are invisible (only the arms render).
    share_dirs = [os.path.join(p, 'share')
                  for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':') if p]
    resource_path = ':'.join(share_dirs)
    existing_res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing_res:
        resource_path = resource_path + ':' + existing_res
    ign_resource_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path)

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # headless:=true runs the Gazebo server only (no GUI). The GUI renderer is
    # CPU-hungry and, on a loaded machine, starves the gz_ros2_control update
    # loop enough that controller activation times out; headless is reliable for
    # automated runs.
    headless = LaunchConfiguration('headless', default='false')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    carry_arms_arg = DeclareLaunchArgument(
        'carry_arms', default_value='false',
        description='Spawn robot with arms folded in carry posture.')

    world_file = os.path.join(pkg_bringup, 'worlds', 'seminar_world.sdf')
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'robot.urdf.xacro')

    # 1. Gazebo Ignition Server. PythonExpression picks server-only (-s) args when
    # headless, full (GUI) args otherwise.
    gz_args = PythonExpression(
        ["('-s -r ' if '", headless, "'=='true' else '-r ') + '", world_file, "'"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )
    
    # 2. Robot State Publisher
    # Expand the xacro here (at launch time) and post-process it. The PAL omni_base
    # mirrors its left-side parts (antennas, suspensions, wheels) with NEGATIVE mesh
    # scales (e.g. scale="1 -1 1"); DART asserts (scale > 0) on collision meshes and
    # aborts the whole Gazebo server. Strip the minus signs from every scale attribute
    # (visually negligible) so physics can build the collision shapes.
    carry_arms = os.environ.get('PAS_SIM_CARRY_ARMS', '').strip().lower()
    for arg in sys.argv:
        if arg.startswith('carry_arms:='):
            carry_arms = arg.split(':=', 1)[1].strip().lower()
    if not carry_arms:
        carry_arms = 'false'
    if carry_arms not in ('true', 'false'):
        raise ValueError('carry_arms must be true or false')
    robot_xml = subprocess.check_output(
        ['xacro', urdf_file, 'sim_ignition:=true',
         f'carry_arms:={carry_arms}']).decode('utf-8')
    robot_xml = re.sub(r'(scale="[0-9. ]*)-([0-9])', r'\1\2', robot_xml)

    # Do not inject position_proportional_gain into joint interfaces here:
    # the installed Humble plugin ignores that tag and retains its 0.1 default.
    robot_description = {'robot_description': robot_xml}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # 3. Spawn Robot in Gazebo
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'dual_arm_robot',
                   '-z', '0.0'],
        output='screen'
    )
    
    # 4. ROS-GZ Bridge (Clock, Joint States, Cmd Vel, Odom, TF)
    # We will expand this bridge config as needed.
    bridge_config = os.path.join(pkg_bringup, 'config', 'bridge.yaml')
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # 5. Controller Spawners
    # Each spawner blocks until controller_manager (loaded by the ign_ros2_control
    # plugin inside Gazebo) is available, so ordering relative to the GZ server is safe.
    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            # Large CM timeout: Gazebo Fortress needs ~50-60 s to load this big model,
            # so a short default makes spawners retry and double-load ("already loaded").
            arguments=[name, '--controller-manager-timeout', '120'],
            output='screen',
        )

    controller_names = [
        'joint_state_broadcaster',
        'base_controller',
        'left_arm_controller',
        'right_arm_controller',
        'torso_controller',
        'pan_tilt_controller',
        'left_gripper_controller',
        'right_gripper_controller',
    ]
    controller_spawners = [spawner(n) for n in controller_names]

    # Nav2/teleop publish Twist on /cmd_vel; the diff_drive controller listens on
    # its namespaced topic, so relay between them.
    cmd_vel_relay = Node(
        package='pas_dual_arm_scripts',
        executable='cmd_vel_relay',
        output='screen',
    )
    scan_filter = Node(
        package='pas_dual_arm_scripts',
        executable='scan_filter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    # Publishes the robot's real outline to both costmaps' footprint topics.
    # The YAML footprint is one arm posture frozen in a file; the arms move and
    # sag (P-37), so without this every layer that reasons about space is
    # reasoning about a robot that is not there. Describes only - gates nothing.
    footprint_publisher = Node(
        package='pas_dual_arm_scripts',
        executable='footprint_publisher',
        name='footprint_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 6. DetachableJoint initial release ("Normally Open" enforcement).
    # Fortress's DetachableJoint hardcodes attachRequested=true at startup,
    # so we explicitly detach on spawn exit and via a delayed fallback.
    detach_box_on_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_spawn_entity,
            on_exit=[
                ExecuteProcess(
                    cmd=['ign', 'topic', '-t', '/aruco_box/detach',
                         '-m', 'ignition.msgs.Empty', '-p', 'unused: true'],
                    output='screen',
                )
            ]
        )
    )
    delayed_detach = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ign', 'topic', '-t', '/aruco_box/detach',
                     '-m', 'ignition.msgs.Empty', '-p', 'unused: true'],
                output='screen',
            )
        ]
    )

    # 7. Auto-fold arms into carry posture if carry_arms is true.
    extra_actions = []
    if carry_arms == 'true':
        auto_carry = Node(
            package='pas_dual_arm_scripts',
            executable='set_posture',
            arguments=['ARM_CARRY_V2'],
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
        carry_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_spawners[-1],
                on_exit=[auto_carry]
            )
        )
        extra_actions.append(carry_handler)

    return LaunchDescription([
        headless_arg,
        carry_arms_arg,
        rmw_env,
        zenoh_env,
        ign_resource_env,
        gz_sim,
        node_robot_state_publisher,
        node_spawn_entity,
        node_ros_gz_bridge,
        detach_box_on_spawn,
        delayed_detach,
        cmd_vel_relay,
        scan_filter,
        footprint_publisher,
        *controller_spawners,
        *extra_actions,
    ])
