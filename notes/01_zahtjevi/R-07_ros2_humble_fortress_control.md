---
id: R-07
type: zahtjev
status: ispunjeno
source: "[ZAD]"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]", "[[S-03_ros2_control_setup]]", "[[S-10_build_run_environment]]"]
problems: ["[[P-01_shell_zenoh_contamination]]", "[[P-02_robot_description_yaml_parse]]", "[[P-31_apt_upgrade_breakage]]", "[[P-32_gui_starves_controllers]]", "[[P-34_source_provenance]]"]
decisions: ["[[D-10_headless_vs_gui]]", "[[D-11_project_scoped_ros_env]]"]
updated: 2026-09-13
---
# R-07: ROS2 Humble + GZ Fortress + ros2_control

## Izvor (doslovno)
> „Potrebni alati: 1. ROS2 Humble 2. Gazebo simulator: GZ Fortress (LTS) 3. ros2_control“ [ZAD]
> „Sve mora biti napravljeno s ros2_control framework-om“ [MAIL]

## Tehnički znači
Cijeli stack radi na Ubuntu 22.04 / Humble. Simulacija je Ignition Gazebo Fortress, a **svi
aktuatori** (baza, torzo, ruke, hvataljke, pan-tilt) idu kroz `ign_ros2_control` i
controller_manager.

**Kriterij prihvaćanja:**
- [x] `sim.launch.py` digne Fortress + robota + svih 8 kontrolera
- [x] nijedan aktuator nije pomican izvan ros2_control (baza kroz `base_controller`)
- [x] reproducibilan build iz čistog checkouta (`ros2.repos` + `apply_patches.sh`)

## Trenutno stanje
✅ Od 12. 6. (M0 + M1, `7aaab94`). Aktivni kontroleri: `joint_state_broadcaster`,
`left/right_arm_controller`, `torso_controller`, `pan_tilt_controller`,
`left/right_gripper_controller`, `base_controller`. Okoliš je izoliran 10. 9.
(`scripts/run_native.sh`), provenance pinan (`f046e98`).

## Problemi
- [[P-01_shell_zenoh_contamination]], [[P-02_robot_description_yaml_parse]],
  [[P-31_apt_upgrade_breakage]], [[P-32_gui_starves_controllers]], [[P-34_source_provenance]]
