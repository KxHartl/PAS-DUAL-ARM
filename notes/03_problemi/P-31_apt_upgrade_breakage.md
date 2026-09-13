---
id: P-31
type: problem
status: rijeseno
requirements: ["[[R-07_ros2_humble_fortress_control]]"]
solutions: ["[[S-10_build_run_environment]]", "[[S-01_robot_description]]"]
decisions: ["[[D-11_project_scoped_ros_env]]"]
updated: 2026-09-13
---
# P-31: `apt upgrade` je slomio okoliš

## Simptom
1. xacro greška `name 'gazebo_version' is not defined`.
2. MoveIt pada: source-build `~/ws_moveit2` linkan na `libgeometric_shapes.so.2.3.2`, a apt je
   donio 2.3.4.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15. 7. `c504720` | definirati `gazebo_version=gazebo` u `robot.urdf.xacro:19` | xacro radi | trajno |
| 2 | 15. 7. | kompatibilnosni symlink za `geometric_shapes` | radi | privremeni hack |
| 3 | 10. 9. `31df80e` `eeefb28` | čisti rebuild **samo** na `/opt/ros/humble`, bez `~/ws_moveit2`, a symlink workaround uklonjen | 25/25 paketa | **rješenje** ([[D-11_project_scoped_ros_env]]) |

## Ne ponavljati
- Kompat-symlinkove za ABI; učitavanje `~/ws_moveit2` u ovaj projekt.
