# ROS2 Workspace (`src/`)

> ⚠ Ranija verzija ovog README-a navodila je nepostojeće pakete (početni kostur). Stvarni sadržaj
> je niže. Arhitektura i status podsustava: `notes/02_rjesenja/` (ulaz `notes/00_MAPA.md`).

## Vlastiti paketi
- `pas_dual_arm_bringup` — centralni URDF (`urdf/robot.urdf.xacro`), svijet (`worlds/seminar_world.sdf`),
  launch (`sim`, `task`, `aruco`, `nav2`, `display`), config (`controllers`, `bridge`, `nav2_params`)
- `pas_dual_arm_scripts` — `main_task` (orkestracija misije), `aruco_detector`, `cmd_vel_relay`
- `pas_dual_arm_moveit_config` — MoveIt2 konfiguracija cijelog robota
- `dual_arm_torso` — vodilice/klizači (STL iz priloga asistenta + xacro)

## Vanjski paketi (pinani u `ros2.repos`, zakrpe u `patches/`)
`omni_base_simulation`, `ros2_kortex`, `pan_tilt_ros`, `realsense-ros`, `aruco_ros`
