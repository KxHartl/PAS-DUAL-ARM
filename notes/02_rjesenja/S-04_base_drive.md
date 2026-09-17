---
id: S-04
type: rjesenje
status: odstupanje
requirements: ["[[R-01_omni_base]]", "[[R-08_omni_controller]]"]
problems: ["[[P-03_pal_base_classic_control]]", "[[P-09_omni_drive_on_fortress]]", "[[P-10_skid_steer_cannot_turn]]", "[[P-11_nav2_slam_drift]]", "[[P-21_spin_once_not_pacing_rtf]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]"]
files: ["src/pas_dual_arm_bringup/urdf/robot.urdf.xacro", "src/pas_dual_arm_bringup/config/controllers.yaml", "src/pas_dual_arm_scripts/pas_dual_arm_scripts/main_task.py"]
updated: 2026-09-13
---
# S-04: Pogon baze

## Što radi (trenutno)
Četiri kotača PAL baze pogonjena su kao **skid-steer** kroz `diff_drive_controller`:
- komanda: `/base_controller/cmd_vel_unstamped` (Twist: `linear.x`, `angular.z`);
- odometrija: `/base_controller/odom` + TF `odom → base_footprint`;
- trenje kotača: mu1 = 0.4 (vuča), mu2 = 0.0 (bočno klizanje, da se može okretati u mjestu).

`main_task.drive(lin, ang, secs)` šalje trapezni profil (0.8 s rampa) tempiran **sim vremenom**
([[P-21_spin_once_not_pacing_rtf]]). Stvarno prevaljeno čita se iz odometrije, a okret i vožnja se
nikad ne rade istovremeno: s mu2 = 0 baza tada „krabira“.

## Povijest pogona
1. PAL ros2_control (Gazebo Classic) → ne učita se na Fortressu ([[P-03_pal_base_classic_control]]).
2. Ignition `MecanumDrive` sistem plugin (, 12. 6.) → prema kasnijem nalazu **nikad se
   ne instancira** na ovoj instalaciji ().
3. `diff_drive_controller` preko ros2_control (, 23. 6.) → radi, ali samo x + yaw.
4. mu2 1.5 → 0 da se može okretati (, 30. 6.), mu1 → 0.4 ().

Detalji pokušaja: [[P-09_omni_drive_on_fortress]].

## Ciljno rješenje (za [[R-08_omni_controller]])
`mecanum_drive_controller` (ros2_controllers 2.53.3, instaliran) na ista 4 kotača. Otvorena
pitanja su u [[P-09_omni_drive_on_fortress]] → „Sljedeći korak“.

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 23. 6. | headless | Nav2 goal → baza se pomakne | |
| 30. 6. | GUI | okret u mjestu, drift ~4 mm | |
| 16. 7. | GUI | odometrijski korigiran dovoz i centrirajući okret | |
