---
id: P-03
type: problem
status: zaobideno
requirements: ["[[R-01_omni_base]]", "[[R-08_omni_controller]]"]
solutions: ["[[S-01_robot_description]]", "[[S-04_base_drive]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]"]
updated: 2026-09-13
---
# P-03: PAL omni_base ros2_control cilja Gazebo Classic

## Simptom
Uključivanje `omni_base_description/robots/omni_base.urdf.xacro` vuče `gazebo_ros2_control/GazeboSystem`
(Classic), koji `ign_ros2_control` ne može učitati. PAL-ova simulacija bazu vozi kinematski
(`planar_move`, Classic plugin), a kotači su dekorativni (mu1 = mu2 = 0).

## Uzrok
**Potvrđeno:** `omni_base_simulation` (humble-devel) je napravljen za Gazebo Classic.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. | preskočiti PAL ros2_control, bazu voziti Ignition `MecanumDrive` system pluginom preko `/cmd_vel` | kontroleri ruku aktivni; gibanje baze nije provjereno | kasnije utvrđeno da se plugin ne instancira ([[P-09_omni_drive_on_fortress]]) |
| 2 | 23. 6. | uključiti **samo** `base/base_sensors.urdf.xacro` + vlastiti `base_wheels_system` (4 velocity interfacea) + `diff_drive_controller` | baza vozi, odometrija + TF | radi, ali nije omni ([[D-03_diff_drive_base_temporary]]) |

## Trenutno rješenje
`robot.urdf.xacro` l. 9–22 (samo strukturni dio baze) + l. 257–290 (ros2_control kotača).
Svojstvo `gazebo_version` je definirano jer PAL makroi granaju na njemu ([[P-31_apt_upgrade_breakage]]).

## Ne ponavljati
- Uključivati `robots/omni_base.urdf.xacro` (vuče Classic).
