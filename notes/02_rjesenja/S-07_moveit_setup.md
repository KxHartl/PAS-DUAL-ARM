---
id: S-07
type: rjesenje
status: ispunjeno
requirements: ["[[R-09_moveit_arm_control]]", "[[R-17_dual_arm_lift]]"]
problems: ["[[P-23_moveit_blind_to_world]]", "[[P-24_press_path_chain]]", "[[P-30_stale_collision_object]]", "[[P-31_apt_upgrade_breakage]]"]
decisions: []
files: ["src/pas_dual_arm_moveit_config/config/pas_dual_arm.srdf", "src/pas_dual_arm_moveit_config/launch/move_group.launch.py", "src/pas_dual_arm_scripts/pas_dual_arm_scripts/main_task.py"]
updated: 2026-09-13
---
# S-07: MoveIt2 (konfiguracija + način korištenja)

## Konfiguracija (`pas_dual_arm_moveit_config`, M3, `b7cfcd2`)
- SRDF grupe: `left_arm`, `right_arm`, `both_arms`, torzo, pan-tilt, obje hvataljke; stanja
  Home/Retract/Open/Close; 235 `disable_collisions` parova.
- KDL kinematika po ruci, OMPL (RRTConnect default), limiti za svih 20 zglobova.
- `moveit_controllers.yaml`: 4× FollowJointTrajectory + 2× GripperCommand.
- `move_group.launch.py` gradi `robot_description` identično kao `sim.launch.py`.

## Korištenje u `main_task.py`
| Funkcija | MoveIt sučelje | Svrha |
|---|---|---|
| `plan_arm`, `plan_both_arms`, `move_arms_joint` | `/move_action` | pose/joint ciljevi, skaliranje 0.2/0.2, 4 retryja |
| `_ik` | `/compute_ik` | IK press poze, pa pre-poze **seedane** tim rješenjem (ista grana) |
| `_linear_traj`, `move_linear`, `retreat_linear` | `/compute_cartesian_path` + `/execute_trajectory` | ravne linije (`max_step` 0.01), vremenska parametrizacija ≤ ~0.4 rad/s, 2π unwrap |
| `press_both_linear` | cartesian path → **izravno** `left/right_arm_controller` | simultani press obje ruke |
| `publish_collision_scene` | `/planning_scene` diff | pod, stol, kocka kao prepreke |

Lanac popravaka kartezijskog pressa: [[P-24_press_path_chain]]. Planning scena:
[[P-23_moveit_blind_to_world]], [[P-30_stale_collision_object]].

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 12. 6. | headless | obje ruke Home, hvataljka 0.600 | `b7cfcd2` |
| 16. 7. | GUI | squeeze + lift, bez „čudnih rotacija“ | `c504720` |
