# ROS2 Workspace Scaffold

Ovaj direktorij sadrži ROS2 pakete za simulaciju i upravljanje dual-arm omni robotom:

- `pas_dual_arm_description` - URDF/Xacro i robot description launch
- `pas_dual_arm_gazebo` - Gazebo world i spawn launch
- `pas_dual_arm_control` - `ros2_control` konfiguracija
- `pas_dual_arm_moveit_config` - MoveIt launch i konfiguracija
- `pas_dual_arm_localization` - SLAM Toolbox launch i parametri
- `pas_dual_arm_nav2` - Nav2 launch i parametri
- `pas_dual_arm_perception` - ArUco detekcija i camera/perception launch
- `pas_dual_arm_bringup` - glavni launch koji povezuje stack

Svi paketi su tek početni kostur i dalje se proširuju.
