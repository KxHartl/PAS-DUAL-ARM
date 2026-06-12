import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
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
    
    world_file = os.path.join(pkg_bringup, 'worlds', 'seminar_world.sdf')
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'robot.urdf.xacro')
    
    # 1. Gazebo Ignition Server
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )
    
    # 2. Robot State Publisher
    robot_description = {'robot_description': ParameterValue(
        Command(['xacro ', urdf_file, ' sim_ignition:=true']), value_type=str)}
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
        'left_arm_controller',
        'right_arm_controller',
        'torso_controller',
        'pan_tilt_controller',
        'left_gripper_controller',
        'right_gripper_controller',
    ]
    controller_spawners = [spawner(n) for n in controller_names]

    return LaunchDescription([
        rmw_env,
        zenoh_env,
        ign_resource_env,
        gz_sim,
        node_robot_state_publisher,
        node_spawn_entity,
        node_ros_gz_bridge,
        *controller_spawners,
    ])
