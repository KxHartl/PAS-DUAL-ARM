---
id: PARAMETRI
type: registar
updated: 2026-09-13
---
# Registar parametara (jedini izvor istine za podesive vrijednosti)

> [!warning] Pravilo
> Svaka promjena vrijednosti se upisuje **ovdje** (nova vrijednost + datum) **i** kao novi red u
> tablici pokušaja pripadne P-kartice. Vrijednosti su pročitane iz koda 13. 9. 2026. (`datoteka:linija`).
> Kod: `MT` = `src/pas_dual_arm_scripts/pas_dual_arm_scripts/main_task.py`,
> `URDF` = `src/pas_dual_arm_bringup/urdf/robot.urdf.xacro`,
> `CTRL` = `src/pas_dual_arm_bringup/config/controllers.yaml`,
> `WORLD` = `src/pas_dual_arm_bringup/worlds/seminar_world.sdf`,
> `NAV` = `src/pas_dual_arm_bringup/config/nav2_params.yaml`.

## Svijet
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| raspored | tri sobe u L: HOME x,y ∈ [-2, 2]; PLAVA y ∈ [-6, -2]; CRVENA x ∈ [2, 6] | `WORLD` model `rooms` | [[D-13_three_room_world]] (13. 9.) |
| širina vrata | **0.9 m** (zid y = -2: x ∈ ±0.45; zid x = 2: y ∈ ±0.45) | `WORLD` linkovi `door_*` | [[D-13_three_room_world]]; prije 2.0 m ([[D-08_door_widened]]) |
| zidovi | 0.1 m debljine × 1.2 m visine | `WORLD` model `rooms` | kamera ne vidi kutiju preko zida |
| kutija poza | (0, -4.28, 0.25), yaw -π/2 (marker gleda +y, prema vratima) | `WORLD` model `aruco_box` | [[D-13_three_room_world]] |
| kutija masa / veličina | **0.3 kg** / 0.30 m (I = 0.0045) | `WORLD` model `aruco_box` | [[D-14_light_box_free_size]] (prije 1.0 kg) |
| kutija μ | 5.0 | `WORLD` model `aruco_box` | [[P-15_dart_friction_no_hold]] |
| ploča markera | 0.22 m (-X ploha, x = -0.1505) | `WORLD` model `aruco_box` | [[P-08_marker_not_detected_texture]] |
| `pick_table` | (0, -4.4), yaw -π/2, ploča 0.4 × 0.5 na z = 0.10 | `WORLD` | plava soba |
| `place_table` | (4.3, 0), ploča 0.6 × 0.6 na z = 0.10 | `WORLD` | crvena soba, odredište ([[R-13_destination_place]]) |

## Robot
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| trenje kotača mu1 / mu2 | 0.4 / 0.0 (kp 1e6) | `URDF` l. 103–105 | [[P-10_skid_steer_cannot_turn]] |
| trenje jastučića prstiju | mu1 = mu2 = 5.0 | `URDF` l. 122–123 | squeeze |
| masa vodilice / klizača | 12 kg / 2 kg | `dual_arm_torso.urdf.xacro` l. 24, 55, 92 | procjena ([[R-06_realistic_parameters]]) |
| klizač limit | 0.05–0.8 m, 1000 N, 0.5 m/s | `dual_arm_torso.urdf.xacro` l. 71, 104 | [[P-13_torso_prismatic_no_lift]] |
| torzo `position_proportional_gain` | 20.0 | `URDF` l. 301, 312 | [[P-13_torso_prismatic_no_lift]] |
| lidar | 360 zraka, 10 Hz | `URDF` l. 192–201 | [[P-06_classic_only_sensors]] |
| RGBD kamera | 640×480, 15 Hz, HFOV 1.211 | `URDF` l. 226–236 | [[S-05_perception]] |
| contact senzori | 50 Hz | `URDF` l. 149 | [[P-27_contact_sensor_topic_ignored]] |
| DetachableJoint | `left_bracelet_link` ↔ `aruco_box` | `URDF` l. 175–181 | [[D-05_contact_verified_attach]] |

## ros2_control
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| `update_rate` | 100 Hz | `CTRL` l. 3 | — |
| `base_controller` tip | DiffDriveController | `CTRL` l. 28 | 🔁 [[D-03_diff_drive_base_temporary]] |
| kotač r / razmak | 0.0762 / 0.44715 m | `CTRL` l. 39–40 | PAL vrijednosti (osni razmak 0.488 u PAL yamlu) |
| limiti baze | v ±0.6 m/s, ω ±1.0 rad/s | `CTRL` l. 47–52 | — |
| `cmd_vel_timeout` | 0.5 s | `CTRL` l. 46 | — |
| hvataljke | `allow_stalling: true` | `CTRL` l. 111, 116 | — |
| CM timeout spawnera | 120 s | `sim.launch.py` l. 107 | [[P-32_gui_starves_controllers]] |

## Navigacija (Nav2; trenutno se ne koristi)
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| footprint | ±0.45 × ±0.30 m | `NAV` l. 197, 236 | — |
| `inflation_radius` | **0.40** (13. 9.; bilo 0.35 → 0.15 → 0.05), `cost_scaling_factor` 5.0 | `NAV` `local_costmap`, `global_costmap` | [[P-12_door_too_narrow]]: mora biti ≥ upisanog radijusa 0.31 |
| `xy_goal_tolerance` | 0.10 / 0.25 | `NAV` l. 140, 169 | [[P-33_nav2_undershoot_base_shift]] |
| DWB `max_vel_x` / `max_vel_theta` | **0.3 / 0.4** (bilo 0.5 / 1.0, 13. 9.) | `NAV` `controller_server.FollowPath` | [[P-11_nav2_slam_drift]]: spori okreti |
| DWB `acc_lim_x` / `acc_lim_theta` | **1.0 / 1.0** (bilo 2.5 / 3.2, 13. 9.) | `NAV` `controller_server.FollowPath` | [[P-11_nav2_slam_drift]] |
| velocity smoother max v / ω / akc. | **0.3 / 0.4 / (0.5, 1.0)** (bilo 0.5 / 1.0 / (2.5, 3.2), 13. 9.) | `NAV` `velocity_smoother` | [[P-11_nav2_slam_drift]] |
| slam_toolbox | default `mapper_params_online_sync` (pomak 0.5 m / 0.5 rad, `base_footprint`, `/scan`) | `nav2_params.yaml` nema sekciju | [[R-14_slam_mapping]] |

## Percepcija
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| ArUco | DICT_4X4_50, ID 0, `marker_size` 0.165 m | `aruco.launch.py` l. 19–20 | [[D-01_aruco_dict_4x4_50]] |
| scan pan / pitch | [0, -0.5, -1.0, 0.5, 1.0] / 0.7 | `MT` l. 1247–1248 | [[S-06_navigation]] |
| servo okret: prag / ω max | 0.08 rad / 0.4 | `MT` l. 1297–1300 | — |
| servo vožnja: stop / histereza / v | target + 0.06 m, 0.18 rad / okret > 0.22 / [0.06, 0.15] | `MT` l. 1323–1334 | — |
| prilaz do | 0.90 m | `MT` l. 1437 | [[P-19_aruco_foreshortening_close]] |
| pitch za potvrdu | 0.65 | `MT` l. 1439 | — |
| marker → centar | +0.15 m | `MT` l. 1031 | pola kocke |
| maska oblaka | z ∈ (0.12, min(0.5, seed + 0.2)), x ∈ (0.35, 1.0), \|Δx\| < 0.25, \|Δy\| < 0.30 | `MT` l. 1085–1088 | [[P-22_depth_self_view_clusters]] |
| sanity dimenzija | duljina (0.15, 0.45), visina (0.06, 0.32) | `MT` l. 1128 | — |
| cross-check dubina vs marker | ≤ 0.15 m | `MT` l. 1463, 1471 | [[D-12_honesty_abort_over_fake]] |

## Gibanje baze u misiji
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| rampa `drive()` | 0.8 s | `MT` l. 374 | [[P-21_spin_once_not_pacing_rtf]] |
| dovoz: kocka na | 0.50 m, v = 0.12, +0.8 s | `MT` l. 1485–1488 | — |
| centriranje kocke | y ≈ **-0.075**, ω = 0.3, prag 0.05 rad | `MT` l. 1525–1529 | [[P-25_asymmetric_arm_reach]] |
| transport-proba | 0.10 m/s × 4 s; ω 0.3 × 3.5 s | `MT` l. 1771, 1777 | [[P-18_transport_drops_box]] |

## Hvat
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| `ARM_HOME` | {0, 0.26, 3.14, -2.27, 0, 0.96, 1.57} | `MT` l. 126 | ready poza |
| `ARM_CARRY` | {0, 0.7, 3.14, -2.5, 0, 1.2, 1.57} | `MT` l. 131 | ⚠ izmjereno 13. 9.: laktovi na y = ±0.58, pa je robot ~1.2 m širok i **ne prolazi vrata od 0.9 m** ([[P-12_door_too_narrow]]) |
| `tip_standoff` | 0.145 m (mjeri se iz TF-a) | `MT` l. 238 | zapešće → vrh |
| hvataljka kao jastučić | 0.7 (zatvoreno) | `MT` l. 1561–1562 | [[D-06_cube_squeeze_grasp]] |
| pre-squeeze / press | +0.10 m / **-0.030 m** (3 cm u kocki) | `MT` l. 1569–1570 | [[P-24_press_path_chain]] |
| skaliranje brzine/akceleracije | 0.2 / 0.2 | `MT` l. 573, 856, 879, 909 | — |
| cartesian `max_step` / min frac | 0.01 / 0.7 (retreat 0.5) | `MT` l. 449, 462, 792 | — |
| re-roll prag | fraction ≥ 0.9, do 4× | `MT` l. 652 | [[P-24_press_path_chain]] |
| parametrizacija | ≤ ~0.4 rad/s po zglobu | `MT` l. 512 | [[P-24_press_path_chain]] |
| guard prve točke | 0.15 rad | `MT` l. 601 | [[P-24_press_path_chain]] |
| settle | < 4 mm / 0.4 s, timeout 8 s | `MT` l. 745–760 | [[P-24_press_path_chain]] |
| reach tol | 0.05 m | `MT` l. 1619, 1639 | [[P-28_gate_too_strict]] |
| geometrija vrhova | `half_thick` 0.15, `half_h` 0.16 (+0.04) | `MT` l. 1660, 1369 | [[P-28_gate_too_strict]] |
| kontakt `max_age` / prozor | 0.4 s (pad 1.0 s) / 3.0 s | `MT` l. 327, 1605, 1681 | [[P-27_contact_sensor_topic_ignored]] |
| `ee_near` | 0.12 m | `MT` l. 1700 | [[P-28_gate_too_strict]] |
| desna povlačenje | 0.10 m po −v | `MT` l. 1748 | [[D-07_carry_on_left_wrist]] |
| lift | +0.15 m | `MT` l. 1754 | — |
| place | pick z −0.02 m, tol 0.04, 3 pokušaja | `MT` l. 1788–1795 | [[P-29_place_drop_tips_cube]] |
| odmak nakon placea | 0.10 m | `MT` l. 1800 | — |

## Ostaci M6 (deklarirani, a trenutni `run()` ih ne koristi)
`pregrasp_xy` [0.35, 0], `preplace_xy` [3.0, 0], `door_xy` [1.5, 0], `box_grasp_z` 0.25,
`table_place_z` 0.85, `grasp_half_width` 0.13, `pick_only` True (`MT` l. 152–173).
**`probe_transport`** (False) je jedini aktivno čitan param (l. 1767).
