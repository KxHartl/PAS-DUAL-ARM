---
id: R-08
type: zahtjev
status: otvoreno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-04_base_drive]]"]
problems: ["[[P-09_omni_drive_on_fortress]]", "[[P-10_skid_steer_cannot_turn]]", "[[P-11_nav2_slam_drift]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]"]
updated: 2026-09-13
---
# R-08: Baza OBAVEZNO preko omni_controllera

## Izvor (doslovno)
> „Sve mora biti napravljeno s ros2_control framework-om, **obavezno koristiti omni_controller**“ [MAIL]

## Tehnički znači
Baza se upravlja ros2_control kontrolerom za **omnidirekcijski** pogon: prima `cmd_vel` (vx, vy,
ωz), daje odometriju i `odom → base_footprint`. Kandidati na ovom sustavu:
- PAL `omni_drive_controller/OmniDriveController` (to koristi
  `omni_base_controller_configuration/config/mobile_base_controller.yaml`), **nije instaliran i nije
  u apt-u** (provjereno 13. 9.);
- `mecanum_drive_controller` **2.53.3 JEST instaliran** u `/opt/ros/humble` (plugin
  registriran), provjereno 13. 9.

**Kriterij prihvaćanja:**
- [ ] `ros2 control list_controllers` pokazuje omni/mecanum kontroler aktivan za 4 kotača
- [ ] `cmd_vel` s `linear.y ≠ 0` pomiče bazu bočno u Gazebu (GUI potvrda)
- [ ] odometrija + `odom → base_footprint` TF objavljeni

## Trenutno stanje
❌ Baza je `diff_drive_controller/DiffDriveController` (4-kotačni skid-steer, samo x + yaw), vidi
[[D-03_diff_drive_base_temporary]]. **Ovo je najjasnije odstupanje od obaveznog zahtjeva.**

## Kako se rješava
- [[S-04_base_drive]]: trenutni pogon i plan prelaska

## Problemi
- [[P-09_omni_drive_on_fortress]]: što je sve probano i zašto nije radilo
- [[P-10_skid_steer_cannot_turn]]: nuspojava zamjenskog rješenja
- [[P-11_nav2_slam_drift]]: klizanje skid-steera razbija odometriju. Pravi omni pogon možda
  uklanja i taj uzrok.
