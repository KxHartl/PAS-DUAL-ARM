---
id: R-02
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-01_robot_description]]", "[[S-03_ros2_control_setup]]"]
problems: ["[[P-05_negative_mesh_scale_dart]]"]
decisions: []
updated: 2026-09-13
---
# R-02: Dvije Kinova ruke (ros2_kortex)

## Izvor (doslovno)
> „ruke robota: https://github.com/Kinovarobotics/ros2_kortex“ [MAIL]

## Tehnički znači
Dvije Kinova Gen3 7-DOF ruke s hvataljkama (Robotiq 2F-85 iz kortex paketa), montirane na klizače
vodilica, s realnim kinematikom, masama i limitima iz proizvođačevog opisa.

**Kriterij prihvaćanja:**
- [x] `left_` i `right_` Gen3 (7 zglobova) + `robotiq_2f_85` u jednom URDF-u
- [x] obje ruke se renderiraju i gibaju u Gazebu (JTC kontroleri aktivni)

## Trenutno stanje
✅ Od 12. 6. (`7aaab94`, `b7cfcd2`). Obje ruke dosežu Home preko MoveIt-a, a hvataljke se
zatvaraju. Jedina lokalna zakrpa upstreama je uklanjanje Isaac-Sim xacro argumenata
(`patches/ros2_kortex-robotiq_2f_85-drop-isaac-args.patch`).

## Kako se rješava
- [[S-01_robot_description]]: montaža na klizače (`LINKS.md`: `xyz=0.060 0.0735 0.112`, `rpy=-π/2 0 0`)
- [[S-03_ros2_control_setup]]: `left/right_arm_controller`, `left/right_gripper_controller`

## Problemi
- [[P-05_negative_mesh_scale_dart]]: DART abort na negativnim skalama meshova (riješeno)
