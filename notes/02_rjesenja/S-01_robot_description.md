---
id: S-01
type: rjesenje
status: djelomicno
requirements: ["[[R-01_omni_base]]", "[[R-02_kinova_arms]]", "[[R-03_linear_rails_torso]]", "[[R-04_pan_tilt_camera]]", "[[R-05_visual_match]]", "[[R-06_realistic_parameters]]"]
problems: ["[[P-03_pal_base_classic_control]]", "[[P-04_mesh_uri_not_found]]", "[[P-05_negative_mesh_scale_dart]]", "[[P-06_classic_only_sensors]]", "[[P-13_torso_prismatic_no_lift]]", "[[P-27_contact_sensor_topic_ignored]]", "[[P-31_apt_upgrade_breakage]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]", "[[D-05_contact_verified_attach]]"]
files: ["src/pas_dual_arm_bringup/urdf/robot.urdf.xacro", "src/dual_arm_torso/urdf/dual_arm_torso.urdf.xacro", "LINKS.md"]
updated: 2026-09-13
---
# S-01: Opis robota (URDF/xacro)

## Što radi
Jedan centralni xacro spaja sve komponente iz [MAIL]: omni_base, torzo, 2× Kinova Gen3 + 2F-85,
pan-tilt i D435. Uz to dodaje Fortress-specifične dijelove: ros2_control blokove, senzore,
kontaktne senzore i DetachableJoint.

## Arhitektura (redoslijed u `robot.urdf.xacro`)
1. `gazebo_version` property (l. 19). Nužan jer PAL `pal_urdf_utils` makroi granaju na njemu
   ([[P-31_apt_upgrade_breakage]]).
2. omni_base: **samo** `base/base_sensors.urdf.xacro` (tijelo, kotači, laser, IMU). Namjerno bez
   `robots/omni_base.urdf.xacro` ([[P-03_pal_base_classic_control]]).
3. Torzo (`dual_arm_torso.urdf.xacro`): vodilice 12 kg, klizači 2 kg, `torso_left/right_carriage_joint`
   prismatic **0.05–0.65 m** (hod stvarne vodilice, 13. 9.), efort 1000 N, 0.5 m/s.
4. Ruke: `kortex_robot.xacro` ×2 s prefiksima `left_`/`right_`. Montaža na klizač
   `xyz=0.060 0.0735 0.112`, `rpy=-π/2 0 0`; desni klizač je rotiran 180° oko Z (`LINKS.md`).
5. Pan-tilt (`pan_tilt_description`) + D435 (`realsense2_description`, `sensor_d435` na
   `pan_tilt_surface`).
6. Trenje kotača (l. 94–113): mu1 0.4, mu2 0.0 ([[P-10_skid_steer_cannot_turn]]). Trenje
   jastučića prstiju mu 5.0.
7. `tip_contact` makro (l. 145–164): Contact senzor na 4 vrha prstiju
   ([[P-27_contact_sensor_topic_ignored]]).
8. DetachableJoint (l. 166–181): `left_bracelet_link` ↔ `aruco_box`, `/aruco_box/attach|detach`
   ([[D-05_contact_verified_attach]]).
9. Senzori: `gpu_lidar` 360 zraka / 10 Hz; `rgbd_camera` 640×480 / 15 Hz, HFOV 1.211,
   frame `camera_color_optical_frame` ([[P-06_classic_only_sensors]]).
10. ros2_control: `base_wheels_system` (4 kotača, velocity), torzo (position,
    `position_proportional_gain` 20), pan-tilt, ruke preko kortex makroa, plugin
    `ign_ros2_control-system`.

**Pri učitavanju** (`sim.launch.py`, `move_group.launch.py`) xacro se proširi u Pythonu, a
negativne skale meshova se uklone ([[P-05_negative_mesh_scale_dart]]).

## Parametri
[[06_parametri]]: sekcije „Robot“ i „Kontakt“.

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 12. 6. | RViz + korisnik | geometrija po slici i CAD mjerama | `8045b62`…`d1af724` |
| 12. 6. | Gazebo | 0 urdf2sdf grešaka, 0 DART asserta, 0 mesh grešaka | `7aaab94`, `d29498a`, `f79e393` |
| 30. 6. | Gazebo | RGBD + oblak | `e31f2fa` |
| 16. 7. | Gazebo GUI | kontaktni senzori objavljuju, imena kolizija u poruci | `c504720` |

## Otvoreno
- Masa torza je procjena, a omni pogon nije postignut ([[R-06_realistic_parameters]], [[R-08_omni_controller]]).
