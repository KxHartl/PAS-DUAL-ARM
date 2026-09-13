---
id: P-06
type: problem
status: rijeseno
requirements: ["[[R-04_pan_tilt_camera]]", "[[R-10_mappable_world]]"]
solutions: ["[[S-01_robot_description]]"]
decisions: []
updated: 2026-09-13
---
# P-06: Senzori postoje samo za Gazebo Classic

## Simptom
Pod Fortressom ništa ne objavljuje `/scan` ni sliku.

## Uzrok
**Potvrđeno:** PAL baza ima Classic `gpu_ray` lasere, a RealSense makro nema Gazebo senzor.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. `ae60d9b` | native Ignition `gpu_lidar` na `virtual_base_laser_link` + RGB `camera` na `camera_link` + bridge | `/scan` 360 zraka, slika ~12 Hz | rješenje |
| 2 | 30. 6. `e31f2fa` | kamera → `rgbd_camera` (RGB + dubina + oblak) | 640×480 oblak | potrebno za mjerenje kutije |
