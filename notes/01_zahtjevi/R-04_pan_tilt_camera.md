---
id: R-04
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-01_robot_description]]", "[[S-05_perception]]"]
problems: ["[[P-06_classic_only_sensors]]"]
decisions: []
updated: 2026-09-13
---
# R-04: Pan-tilt na vrhu + RealSense kamera

## Izvor (doslovno)
> „pan-tilt kamera na vrhu robota: https://github.com/I-Quotient-Robotics/pan_tilt_ros“
> „kamera: https://github.com/realsenseai/realsense-ros“ [MAIL]

## Tehnički znači
Na vrhu torza je pan-tilt mehanizam s dva upravljiva zgloba, a na njemu RealSense D435 koji u
Gazebu stvarno objavljuje sliku, dubinu i oblak točaka.

**Kriterij prihvaćanja:**
- [x] `pan_tilt_yaw_joint`, `pan_tilt_pitch_joint` upravljani (`pan_tilt_controller`)
- [x] D435 URDF (`realsense2_description`) na `pan_tilt_surface`
- [x] topici `/camera/image`, `/camera/depth_image`, `/camera/points`, `/camera/camera_info`

## Trenutno stanje
✅ RGB od 12. 6. (), RGBD od 30. 6. (). Senzor je native Ignition
`rgbd_camera` (640×480, 15 Hz, HFOV 1.211), jer originalni opis nema Fortress senzor
([[P-06_classic_only_sensors]]). `realsense-ros` je u `src/` (v4.57.6) i namjerno nadjačava apt
`realsense2_description` (RUNNING.md).

## Kako se rješava
- [[S-01_robot_description]]: montaža, senzor u `robot.urdf.xacro`
- [[S-05_perception]]: korištenje slike i oblaka
