# Zadatci i Napredak (Dual Arm Torso)

> ⚠ **ZASTARJELO (od 13. 9. 2026.)**: checkboxovi ispod ne odražavaju stvarno stanje (npr. MoveIt,
> ros2_control i Aruco su gotovi, a vrata su trenutno 2.0 m). Aktualno stablo zahtjeva i status:
> `notes/00_MAPA.md`.

Ovdje pratimo zadatke vezane za simulaciju robota s dvije ruke i pokretnom bazom.

## 1. Pokretanje i Instalacija (Setup)
- [x] Inicijalizacija radnog prostora
- [x] Kloniranje vanjskih repozitorija na humble grane
- [x] Kreiranje paketa `dual_arm_torso`, `pas_dual_arm_bringup`, `pas_dual_arm_scripts`
- [x] Instalacija sistemskih ovisnosti (`apt install`, `rosdep install`)
- [x] Kompilacija radnog prostora (`colcon build`)
- [ ] Početni Git commit

## 2. Modeliranje Robota (URDF)
- [x] URDF za torzo i vodilice (s aproksimiranim masama: 12kg vodilice, 2kg klizači)
- [x] Integracija `omni_base` kao korijena
- [x] Integracija dviju `Kinova Gen3` ruku na klizače
- [x] Integracija `pan_tilt` mehanizma i `realsense2` kamere
- [x] Zamjena `gazebo_ros2_control` sa `ign_ros2_control/IgnitionSystem` za Gazebo Fortress

## 3. Gazebo Okruženje i Meta
- [x] Izrada `seminar_world.sdf`
- [x] Kreiranje prolaza/vrata širine točno 0.8m
- [x] Kreiranje stola na fiksnoj lokaciji
- [x] Kutija 30x30x30 cm (1 kg) s Aruco markerom (DICT_4X4_50, ID 0)

## 4. Upravljanje i Planiranje (MoveIt & ros2_control)
- [ ] Postavljanje `omni_controller` za bazu
- [ ] Konfiguracija `joint_trajectory_controller` za ruke, torzo i pan-tilt
- [ ] Generiranje `MoveIt2` konfiguracije za cijelog robota

## 5. Navigacija i Detekcija
- [ ] Podešavanje `aruco_ros` za detekciju
- [ ] Pokretanje `nav2_stack` i `slam_toolbox`

## 6. Glavna Skripta Zadataka
- [ ] Odlazak u zadanu regiju
- [ ] Detekcija markera i izračun IK za hvatanje s obje ruke
- [ ] Sklapanje hvataljki i prolazak kroz vrata
- [ ] Odlaganje kutije na stol

## 7. Dokumentacija i Resursi
- [x] Ažuriranje `README.md` uputa
- [x] Korištenje `HUMAN.md` za zatraživanje vizualne provjere korisnika
