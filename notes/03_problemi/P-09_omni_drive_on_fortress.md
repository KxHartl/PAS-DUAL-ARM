---
id: P-09
type: problem
status: otvoreno
requirements: ["[[R-08_omni_controller]]", "[[R-01_omni_base]]"]
solutions: ["[[S-04_base_drive]]", "[[S-03_ros2_control_setup]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]"]
updated: 2026-09-13
---
# P-09: Omnidirekcijski pogon baze na Fortressu (obavezni omni_controller)

## Simptom
Nijedan omni pogon nije proradio. Baza trenutno vozi samo x + yaw kao skid-steer.

## Uzrok
**Potvrđeno:**
- PAL rješenje je Classic-only ([[P-03_pal_base_classic_control]]).
- Fortress sistem pluginovi `MecanumDrive` i `VelocityControl` se na ovoj instalaciji ne
  instanciraju (`controllers.yaml` komentar, `6eb7487`).
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
| 1 | 12. 6. `7aaab94` | PAL ros2_control (Classic) | ne učita se | Classic-only |
| 2 | 12. 6. `7aaab94` | Ignition `MecanumDrive` system plugin preko `/cmd_vel` | nikad se ne instancira | slijepa ulica na ovoj instalaciji |
| 3 | 23. 6. `6eb7487` | Ignition `VelocityControl` / PAL `planar_move` | ne instancira se / Classic-only | slijepa ulica |
| 4 | 23. 6. `6eb7487` | `diff_drive_controller` na 4 kotača (skid-steer) | radi: x + yaw, odom, TF | privremeno ([[D-03_diff_drive_base_temporary]]) |
| 5 | 13. 9. (provjera) | traženje omni kontrolera: `apt-cache`, ament index | PAL omni ❌, **`mecanum_drive_controller` ✅** | novi kandidat |

## Trenutno rješenje
`diff_drive_controller` ([[S-04_base_drive]]).

## Sljedeći korak (time-box ~60 min, vidi [[danas]])
1. `ros2 param describe` / izvor ros2_controllers 2.53.3: potrebni parametri `mecanum_drive_controller`
   (imena kotača, `kinematics.wheels_radius`, `kinematics.sum_of_robot_center_projection_on_X_Y_axis`,
   frameovi) i format reference (TwistStamped/unstamped, topic `~/reference`).
2. U `controllers.yaml` zamijeniti `base_controller` (r = 0.0762, razmak kotača 0.44715, osni
   razmak 0.488 iz PAL `mobile_base_controller.yaml`). Prilagoditi `cmd_vel` izdavače
   (`main_task._send_vel`, `cmd_vel_relay`).
3. Test u GUI-ju: `linear.y = 0.1` → gibanje bočno? Ako **ne** (cilindri), probati anizotropno
   trenje po kotaču (`fdir1`, mu2 0). Ako i to ne uspije, koristiti mecanum kontroler samo za
   x + yaw (formalno ispunjava „omni_controller“, fizički ograničeno), što ide u [[odstupanja]].
4. Regresija: okret u mjestu, dovoz, centriranje. Parametar `mu1/mu2` mijenjati samo uz
   [[06_parametri]].

**Kriterij uspjeha:** `ros2 control list_controllers` pokazuje mecanum kontroler aktivan, baza
radi okret i vožnju naprijed kao prije, a (idealno) i bočni pomak.

## Ne ponavljati
- Ignition `MecanumDrive` / `VelocityControl` sistem pluginovi (ne instanciraju se).
- PAL `planar_move` / Classic ros2_control.
