import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_bringup = get_package_share_directory('pas_dual_arm_bringup')
    params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    # Force Fast DDS locally (see sim.launch.py note on rmw_zenoh override).
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    zenoh_env = SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', '')

    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map')
    # Mapping has its own launch (mapping.launch.py, which deliberately leaves
    # Nav2 out - P-32), so reaching for this one means navigating on the saved
    # map. The old 'mapping' default started a second slam_toolbox against a
    # map file that may not exist.
    mode_arg = DeclareLaunchArgument('mode', default_value='localization',
                                     description='localization (saved map) or mapping (live SLAM)')
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_bringup, 'maps', 'seminar_map.yaml'),
        description='Saved occupancy map for AMCL localization')
    mapping = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='both',
        parameters=[os.path.join(pkg_bringup, 'config', 'slam_params.yaml'),
                    {'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'map': map_file,
                          'params_file': params_file}.items(),
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
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

    # Nav2 publishes /cmd_vel and the base controller listens on its own topic
    # inside the gz controller_manager, so a relay has to bridge the two - but
    # sim.launch.py already starts it. Starting a second one here put two nodes
    # of the same name on the same topic, so this launch relies on the one from
    # sim.launch.py.

    # The keepout mask is generated from the doorways and tables detected in the
    # map, rather than loaded from a hand-drawn PGM, so it follows the world.
    # nav_zones is a plain node, so it is not in the lifecycle manager below.
    zones = LaunchConfiguration('zones')
    zones_arg = DeclareLaunchArgument(
        'zones', default_value='true',
        description='Publish doorway/table keepout zones derived from the map. '
                    'Set false to compare against plain Nav2.')
    nav_zones = Node(
        package='pas_dual_arm_scripts', executable='nav_zones',
        name='nav_zones', output='both',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(zones),
    )
    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )
    # A second one for the global costmap, serving the grown mask. The planner
    # reasons about a point and the controller about a footprint, so they are
    # shown the same zones at two sizes (nav_zones.Zones.mask).
    costmap_filter_info_server_planner = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server_planner',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info_planner',
            'mask_topic': '/keepout_filter_mask_planner',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )

    lifecycle_manager_costmap_filters = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['costmap_filter_info_server',
                           'costmap_filter_info_server_planner'],
        }],
    )

    # Filters Nav2's velocity against the raw scan and the robot's *current*
    # footprint, independently of the costmap. The costmap cannot help with what
    # the laser never saw (a table top at 0.75 m) and the controller can only be
    # as right as its model of the robot; this is the layer that does not depend
    # on either being correct. cmd_vel_relay prefers its output, so it cannot be
    # bypassed while it runs.
    safety = LaunchConfiguration('safety')
    safety_arg = DeclareLaunchArgument(
        'safety', default_value='true',
        description='Run the collision monitor between Nav2 and the base.')
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='both',
        parameters=[os.path.join(pkg_bringup, 'config', 'collision_monitor.yaml')],
        condition=IfCondition(safety),
    )
    lifecycle_manager_safety = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
        condition=IfCondition(safety),
    )

    # Delay the Nav2 stack so the localizer has time to publish map->odom first.
    # Otherwise local_costmap activates while the TF tree is still split
    # (odom and base_link in unconnected trees) and logs a startup-race error.
    # Remap goal_pose so bt_navigator's raw listener does not bypass room_navigator's
    # perpendicular doorway routing. RViz '2D Goal Pose' goes to room_navigator.
    nav2_group = GroupAction([
        SetRemap(src='/goal_pose', dst='/bt_goal_pose'),
        SetRemap(src='goal_pose', dst='/bt_goal_pose'),
        nav2_launch,
    ])
    delayed_nav2 = TimerAction(period=7.0, actions=[nav2_group])
    features = Node(
        package='pas_dual_arm_scripts', executable='feature_registry',
        output='both', parameters=[{'use_sim_time': True}],
    )
    # Drives room-to-room legs off the zone graph. Started with Nav2 so the
    # GUI has something to talk to; it does nothing until asked.
    room_navigator = Node(
        package='pas_dual_arm_scripts', executable='room_navigator',
        name='room_navigator', output='both',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(zones),
    )

    # The button panel comes up with the stack so a navigation run needs two
    # terminals, not one per node: this launch and sim.launch.py.
    gui = LaunchConfiguration('gui')
    gui_arg = DeclareLaunchArgument('gui', default_value='true',
                                    description='Open the room navigation button panel')
    nav_gui = Node(
        package='pas_dual_arm_scripts', executable='nav_gui',
        name='nav_gui', output='both',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(gui),
    )

    rviz = LaunchConfiguration('rviz')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='Open RViz with Nav2 custom view')
    custom_rviz = os.path.join(pkg_bringup, 'rviz', 'nav2.rviz')
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz_config': custom_rviz,
        }.items(),
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        rmw_env,
        zenoh_env,
        mode_arg,
        map_arg,
        rviz_arg,
        zones_arg,
        gui_arg,
        safety_arg,
        mapping,
        localization,
        nav_zones,
        costmap_filter_info_server,
        costmap_filter_info_server_planner,
        lifecycle_manager_costmap_filters,
        collision_monitor,
        lifecycle_manager_safety,
        features,
        room_navigator,
        nav_gui,
        delayed_nav2,
        rviz_cmd,
    ])

