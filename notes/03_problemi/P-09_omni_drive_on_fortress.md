---
id: P-09
type: problem
status: riješeno
requirements: ["[[R-08_omni_controller]]", "[[R-01_omni_base]]"]
solutions: ["[[S-04_base_drive]]", "[[S-03_ros2_control_setup]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]"]
updated: 2026-09-13
---
# P-09: Omnidirekcijski pogon baze na Fortressu (obavezni omni_controller)

## Simptom
Riješeno 13. 9. Baza ima puni omnidirekcijski pogon (vx, vy, wz) preko `mecanum_drive_controller` uz `mu2 = 0.20`.

## Uzrok
**Potvrđeno:**
- PAL rješenje je Classic-only ([[P-03_pal_base_classic_control]]).
- Fortress sistem pluginovi `MecanumDrive` i `VelocityControl` se na ovoj instalaciji ne
  instanciraju (`controllers.yaml` komentar).
- PAL `omni_drive_controller/OmniDriveController` nije instaliran i nije u apt-u (13. 9.).

**Činjenica (13. 9.):** `ros-humble-mecanum-drive-controller` **2.53.3 je instaliran**
(`/opt/ros/humble/share/mecanum_drive_controller`, plugin registriran). To je omni (mecanum)
ros2_control kontroler.
**Hipoteza:** PAL kotači su u Gazebu obični cilindri bez valjčića. Mecanum kinematika će zato
ispravno zapovijedati brzine kotača, ali bočno gibanje fizički neće nastati bez anizotropnog trenja
(`fdir1` pod 45°, mu2 = 0 po smjeru valjčića) ili drugog trika.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. | PAL ros2_control (Classic) | ne učita se | Classic-only |
| 2 | 12. 6. | Ignition `MecanumDrive` system plugin preko `/cmd_vel` | nikad se ne instancira | slijepa ulica na ovoj instalaciji |
| 3 | 23. 6. | Ignition `VelocityControl` / PAL `planar_move` | ne instancira se / Classic-only | slijepa ulica |
| 4 | 23. 6. | `diff_drive_controller` na 4 kotača (skid-steer) | radi: x + yaw, odom, TF | privremeno ([[D-03_diff_drive_base_temporary]]) |
| 5 | 13. 9. (provjera) | traženje omni kontrolera: `apt-cache`, ament index | PAL omni ❌, **`mecanum_drive_controller` ✅** | novi kandidat |
| 6 | 13. 9. | `mecanum_drive_controller` + URDF base prilagodba (effort 100 Nm, damping/friction 0, `fdir1` u `base_footprint`, mu2 0.20) + `cmd_vel_relay` | **PUNI USPJEH** ✅: vx, vy (0.33 m), wz (okret u mjestu), dijagonala rade | Omnidirekcijski pogon potpuno operativan |

## Konačno rješenje
- Kontroler: `mecanum_drive_controller/MecanumDriveController` u `src/pas_dual_arm_bringup/config/controllers.yaml`.
- Poveznica: `cmd_vel_relay.py` preusmjerava `/cmd_vel` i `/base_controller/cmd_vel_unstamped` na `/base_controller/reference_unstamped`, te `/base_controller/odometry` na `/base_controller/odom` i `~/tf_odometry` na `/tf`.
- URDF: `src/pas_dual_arm_bringup/urdf/base/` definira `wheel.urdf.xacro` s effort limitom 100 Nm (DART SERVO motor constraint), bez pasivnog prigušenja i s anizotropnim trenjem `mu=0.80`, `mu2=0.20`, `fdir1 ignition:expressed_in="base_footprint"`.
- Nav2: ažurirani `nav2_params.yaml` (DWB `max_vel_y: 0.2`, `acc_lim_y: 1.0`, AMCL `OmniMotionModel`).

## Ne ponavljati
- Ignition `MecanumDrive` / `VelocityControl` sistem pluginovi (ne instanciraju se).
- PAL `planar_move` / Classic ros2_control.
