import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition
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
    # Default TRUE: the robot spawns with every arm joint at zero, which is
    # 2.28 m wide - the arms stick straight out from the side-mounted carriages,
    # nothing can drive anywhere, and it looks wrong in the GUI. Folding into
    # the 0.854 m carry posture right after the controllers come up is the only
    # state the robot is ever actually useful in.
    carry_arms_arg = DeclareLaunchArgument(
        'carry_arms', default_value='true',
        description='Spawn robot with arms folded in carry posture.')
    # Ground truth is a simulator privilege. It is bridged only on request, so a
    # run that has to behave like the physical robot simply does not ask for it.
    debug_truth = LaunchConfiguration('debug_truth')
    debug_truth_arg = DeclareLaunchArgument(
        'debug_truth', default_value='false',
        description='Bridge Gazebo ground-truth poses on /debug/gz_dynamic_pose '
                    '(diagnostics only - no control node may subscribe).')
    rviz = LaunchConfiguration('rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Open RViz with the cube-grasp view (robot, marker TF, '
                    'computed grasp poses, depth cloud).')
    table_arms_arg = DeclareLaunchArgument(
        'table_arms', default_value='false',
        description='Spawn with the wrists above the 0.75 m tabletop, then '
                    'command the carriages up (cube-table staging).')
    spawn_args = [
        DeclareLaunchArgument('robot_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_yaw', default_value='0.0'),
    ]

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
    # The arm spawn profile is read from the command line and, for a launch
    # file that includes this one (and so cannot put it on sys.argv), from
    # PAS_SIM_TABLE_ARMS / PAS_SIM_CARRY_ARMS. Table staging is decided first
    # because it changes the carry default: with carry_arms defaulting to true,
    # a plain `table_arms:=true` used to raise "cannot both be true".
    table_arms = os.environ.get('PAS_SIM_TABLE_ARMS', '').strip().lower() or 'false'
    for arg in sys.argv:
        if arg.startswith('table_arms:='):
            table_arms = arg.split(':=', 1)[1].strip().lower()
    if table_arms not in ('true', 'false'):
        raise ValueError('table_arms must be true or false')
    carry_arms = os.environ.get('PAS_SIM_CARRY_ARMS', '').strip().lower()
    for arg in sys.argv:
        if arg.startswith('carry_arms:='):
            carry_arms = arg.split(':=', 1)[1].strip().lower()
    if not carry_arms:
        carry_arms = 'false' if table_arms == 'true' else 'true'
    if carry_arms not in ('true', 'false'):
        raise ValueError('carry_arms must be true or false')
    if table_arms == 'true' and carry_arms == 'true':
        raise ValueError('table_arms and carry_arms cannot both be true')
    robot_xml = subprocess.check_output(
        ['xacro', urdf_file, 'sim_ignition:=true',
         f'carry_arms:={carry_arms}',
         f'table_arms:={table_arms}']).decode('utf-8')
    robot_xml = re.sub(r'(scale="[0-9. ]*)-([0-9])', r'\1\2', robot_xml)

    # Do not inject position_proportional_gain into joint interfaces here:
    # the installed Humble plugin ignores that tag and retains its 0.1 default.
    # Explicit experimental profile: preserve the default model/controllers.
    force_grasp = any(arg == 'force_grasp:=true' for arg in sys.argv)
    if force_grasp:
        model = ET.fromstring(robot_xml)
        limits = {j.get('name'): j.find('limit') for j in model.findall('joint')}
        for control in model.findall('ros2_control'):
            for joint in control.findall('joint'):
                name = joint.get('name')
                if re.fullmatch(r'(left|right)_joint_[1-7]|torso_(left|right)_carriage_joint', name):
                    command = joint.find('command_interface')
                    command.clear()
                    command.set('name', 'effort')
                    effort = float(limits[name].get('effort'))
                    ET.SubElement(command, 'param', name='min').text = str(-effort)
                    ET.SubElement(command, 'param', name='max').text = str(effort)
                    if not any(i.get('name') == 'effort' for i in joint.findall('state_interface')):
                        ET.SubElement(joint, 'state_interface', name='effort')
        for gazebo in model.findall('gazebo'):
            for plugin in list(gazebo.findall('plugin')):
                if 'DetachableJoint' in plugin.get('name', ''):
                    gazebo.remove(plugin)
                for parameters in plugin.findall('parameters'):
                    if parameters.text.endswith('/controllers.yaml'):
                        parameters.text = os.path.join(pkg_bringup, 'config', 'force_controllers.yaml')
        robot_xml = ET.tostring(model, encoding='unicode')
    robot_description = {'robot_description': robot_xml}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # 3. Spawn Robot in Gazebo
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'dual_arm_robot',
                   '-x', LaunchConfiguration('robot_spawn_x'),
                   '-y', LaunchConfiguration('robot_spawn_y'),
                   '-Y', LaunchConfiguration('robot_spawn_yaw'),
                   '-z', '0.0'],
        output='both'
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
        output='both'
    )

    # 4b. Ground-truth bridge, off unless debug_truth:=true. Feeds `loc_error`,
    # which measures |AMCL - actual| and prints it; nothing else reads it.
    bridge_debug_config = os.path.join(pkg_bringup, 'config', 'bridge_debug.yaml')
    node_ros_gz_bridge_debug = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge_debug',
        parameters=[{'config_file': bridge_debug_config}],
        condition=IfCondition(debug_truth),
        output='both'
    )
    node_loc_error = Node(
        package='pas_dual_arm_scripts',
        executable='loc_error',
        condition=IfCondition(debug_truth),
        parameters=[{'use_sim_time': True}],
        output='both',
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
            output='both',
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
        output='both',
    )
    scan_filter = Node(
        package='pas_dual_arm_scripts',
        executable='scan_filter',
        output='both',
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
        output='both',
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
                    output='both',
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
                output='both',
            )
        ]
    )

    # 7. Auto-fold arms into carry posture if carry_arms is true.
    extra_actions = []
    # PAS_SIM_AUTO_POSTURE=false leaves the arms alone after spawn: they already
    # spawn in ARM_CARRY_V2 (xacro initial values), and a launch meant for moving
    # the robot by hand must not start any node that commands it.
    auto_posture = os.environ.get('PAS_SIM_AUTO_POSTURE', 'true').strip().lower() != 'false'
    if carry_arms == 'true' and auto_posture:
        auto_carry = Node(
            package='pas_dual_arm_scripts',
            executable='set_posture',
            arguments=['DRIVE_V4'],   # postures.ARM_DRIVE
            output='both',
            parameters=[{'use_sim_time': True}],
        )
        carry_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_spawners[-1],
                on_exit=[auto_carry]
            )
        )
        extra_actions.append(carry_handler)

    # 7a. RViz. sim.launch.py had no view of its own - nav2.launch.py owned the
    # only one - so a manipulation run had nowhere to see the poses it computes.
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'cube.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': True}],
        output='both',
    )
    # The rgbd sensor stamps the optical frame but ships body-convention data,
    # so RViz draws the raw cloud ~90 deg off to the side. Relay a corrected
    # copy for display; the control path keeps reading the raw topic.
    node_cloud_restamp = Node(
        package='pas_dual_arm_scripts',
        executable='cloud_restamp',
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': True}],
        output='both',
    )

    # 7b. Cube-table staging: raise the carriages to tabletop height and hold
    # ARM_HOME. Same mechanism as the carry posture above - a plain trajectory
    # on the position interface, no regulator anywhere.
    if table_arms == 'true':
        table_ready = Node(
            package='pas_dual_arm_scripts',
            executable='table_ready',
            output='both',
            parameters=[{'use_sim_time': True}],
        )
        extra_actions.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_spawners[-1],
                on_exit=[table_ready]
            )
        ))

    return LaunchDescription([
        headless_arg,
        carry_arms_arg,
        table_arms_arg,
        DeclareLaunchArgument('force_grasp', default_value='false'),
        rviz_arg,
        debug_truth_arg,
        *spawn_args,
        rmw_env,
        zenoh_env,
        ign_resource_env,
        gz_sim,
        node_robot_state_publisher,
        node_spawn_entity,
        node_ros_gz_bridge,
        node_ros_gz_bridge_debug,
        node_loc_error,
        node_rviz,
        node_cloud_restamp,
        detach_box_on_spawn,
        delayed_detach,
        cmd_vel_relay,
        scan_filter,
        footprint_publisher,
        *controller_spawners,
        *extra_actions,
    ])
