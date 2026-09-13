---
id: R-01
type: zahtjev
status: djelomicno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-01_robot_description]]", "[[S-04_base_drive]]"]
problems: ["[[P-03_pal_base_classic_control]]", "[[P-09_omni_drive_on_fortress]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]"]
updated: 2026-09-13
---
# R-01: Mobilna baza PAL omni_base

## Izvor (doslovno)
> „mobilna baza: https://github.com/pal-robotics/omni_base_simulation“ [MAIL]

## Tehnički znači
Robot stoji na PAL omni_base modelu (geometrija, kotači, lidar) iz `omni_base_simulation`, a baza
se u Gazebu stvarno giba kao **omnidirekcijska** baza (x, y, yaw) preko ros2_control.

**Kriterij prihvaćanja:**
- [x] omni_base URDF (tijelo, 4 kotača, laser, IMU) je korijen robota
- [ ] baza se giba bočno (y) i rotira preko omni kontrolera, vidi [[R-08_omni_controller]]

## Trenutno stanje
⚠ **Geometrija da, omni gibanje ne.** Uključen je samo strukturni dio
(`omni_base_description/urdf/base/base_sensors.urdf.xacro`). PAL-ov `omni_base.urdf.xacro` se
namjerno preskače jer vuče Gazebo Classic ros2_control ([[P-03_pal_base_classic_control]]).
Kotači su pogonjeni kao 4-kotačni skid-steer preko `diff_drive_controller`
([[D-03_diff_drive_base_temporary]]), dakle samo x + yaw.

## Kako se rješava
- [[S-01_robot_description]]: uključivanje base xacroa, trenje kotača
- [[S-04_base_drive]]: pogon baze (sada diff_drive, cilj omni)

## Problemi
- [[P-03_pal_base_classic_control]]: PAL ros2_control cilja Gazebo Classic
- [[P-09_omni_drive_on_fortress]]: niti jedan omni pogon se nije instancirao na Fortressu

## Odluke / odstupanja
- [[D-03_diff_drive_base_temporary]] 🔁
