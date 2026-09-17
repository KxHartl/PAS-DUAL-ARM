---
id: S-03
type: rjesenje
status: djelomicno
requirements: ["[[R-02_kinova_arms]]", "[[R-03_linear_rails_torso]]", "[[R-07_ros2_humble_fortress_control]]", "[[R-08_omni_controller]]", "[[R-09_moveit_arm_control]]"]
problems: ["[[P-03_pal_base_classic_control]]", "[[P-09_omni_drive_on_fortress]]", "[[P-13_torso_prismatic_no_lift]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]"]
files: ["src/pas_dual_arm_bringup/config/controllers.yaml"]
updated: 2026-09-13
---
# S-03: ros2_control (kontroleri)

## Što radi
Svi aktuatori idu kroz `ign_ros2_control/IgnitionSystem` i controller_manager (100 Hz, sim time).

## Kontroleri (`controllers.yaml`)
| Kontroler | Tip | Zglobovi | Status |
|---|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | svi | ✅ |
| `left_arm_controller`, `right_arm_controller` | JointTrajectoryController | `*_joint_1..7` | ✅ |
| `torso_controller` | JTC (position) | `torso_left/right_carriage_joint` | ⚠ ne diže pod teretom ([[P-13_torso_prismatic_no_lift]]) |
| `pan_tilt_controller` | JTC (position) | `pan_tilt_yaw/pitch_joint` | ✅ |
| `left/right_gripper_controller` | GripperActionController, `allow_stalling` | `*_robotiq_85_left_knuckle_joint` | ✅ |
| `base_controller` | **DiffDriveController** | 4 kotača (skid-steer) | 🔁 treba omni ([[R-08_omni_controller]]) |

`base_controller`: `wheel_separation` 0.44715, `wheel_radius` 0.0762, v ≤ 0.6 m/s, ω ≤ 1.0 rad/s,
`enable_odom_tf: true`, `cmd_vel_timeout` 0.5, `use_stamped_vel: false`. Topic
`/base_controller/cmd_vel_unstamped`.

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 12. 6. | headless | 7 kontrolera aktivno; `left_joint_2` → 0.8 postigao 0.799 | |
| 23. 6. | headless | `base_controller` vozi, odom x 0 → 0.24 | |

## Otvoreno
- Zamjena `base_controller` omni/mecanum kontrolerom, vidi [[S-04_base_drive]].
