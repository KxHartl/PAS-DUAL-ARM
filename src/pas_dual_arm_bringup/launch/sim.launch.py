import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_bringup = get_package_share_directory('pas_dual_arm_bringup')
    
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
    robot_description = {'robot_description': Command(['xacro ', urdf_file, ' sim_ignition:=true'])}
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
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        node_robot_state_publisher,
        node_spawn_entity,
        node_ros_gz_bridge,
        joint_state_broadcaster_spawner
    ])
