---
id: PARAMETRI
type: registar
updated: 2026-09-14
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
| raspored | tri sobe u L, svaka **6 × 6 m**: HOME x,y ∈ [-3, 3]; PLAVA y ∈ [-9, -3]; CRVENA x ∈ [3, 9] | `WORLD` model `rooms` | [[D-13_three_room_world]] (13. 9.; prije 4 × 4) |
| širina vrata | **1.0 m** (zid y = -3: x ∈ ±0.5; zid x = 3: y ∈ ±0.5) | `WORLD` linkovi `door_*` | robot je 0.85 m → 7.3 cm po strani; prije 0.9 m pa 2.0 m ([[P-35_arm_span_too_wide_for_door]]) |
| zidovi | 0.1 m debljine × **3.0 m visine** (13. 9.; bilo 1.2 m) | `WORLD` model `rooms` | kamera je na 1.39 m i gledala je preko zidova → [[P-36_walls_lower_than_camera]] |
| visina robota | **1.45 m** (kamera 1.39 m + rub) | izmjereno `scripts/measure_robot.py` | [[08_poze]] |
| kutija poza | (0, **-6.35**, 0.90), yaw -π/2 (marker gleda +y, prema vratima) | `WORLD` model `aruco_box` | dno na stolu z=0.75, težište z=0.90 |
| kutija masa / veličina | **0.3 kg** / 0.30 m (I = 0.0045) | `WORLD` model `aruco_box` | [[D-14_light_box_free_size]] (prije 1.0 kg) |
| kutija μ | 5.0 | `WORLD` model `aruco_box` | [[P-15_dart_friction_no_hold]] |
| ploča markera | 0.22 m (-X ploha, x = -0.1505) | `WORLD` model `aruco_box` | [[P-08_marker_not_detected_texture]] |
| `pick_table` | (0, **-6.5**), 4 noge, ploča 0.8 × 0.8 na **z = 0.75 m** | `WORLD` | plava soba; 4 noge na z=0..0.71 |
| `place_table` | (**6.5**, 0), 4 noge, ploča 0.8 × 0.8 na **z = 0.75 m** | `WORLD` | crvena soba, odredište ([[R-13_destination_place]]) |

## Robot
| trenje kotača mu1 / mu2 | **0.80 / 0.20** (anizotropno, fdir1 ±45° u `base_footprint`) | `src/pas_dual_arm_bringup/urdf/base/wheel.urdf.xacro` | [[P-09_omni_drive_on_fortress]], [[R-08_omni_controller]] |
| effort limit kotača / prigušenje | **100.0 Nm** / damping 0.0, friction 0.0 | `src/pas_dual_arm_bringup/urdf/base/wheel.urdf.xacro` | DART SERVO constraint za omni pogon ([[P-09_omni_drive_on_fortress]]) |
| base controller | `mecanum_drive_controller/MecanumDriveController` | `CTRL` l. 28, 41–64 | [[R-08_omni_controller]], zamijenio diff_drive_controller |
| trenje jastučića prstiju | mu1 = mu2 = 5.0 | `URDF` l. 122–123 | squeeze |
| masa vodilice / klizača | 12 kg / 2 kg | `dual_arm_torso.urdf.xacro` l. 24, 55, 92 | procjena ([[R-06_realistic_parameters]]) |
| klizač limit | **0.05–0.65 m** (13. 9.; bilo 0.05–0.8), 1000 N, 0.5 m/s | `dual_arm_torso.urdf.xacro` l. 71, 105 | hod stvarne vodilice (odluka korisnika); [[P-13_torso_prismatic_no_lift]] |
| torzo `position_proportional_gain` | 20.0 | `URDF` l. 301, 312 | [[P-13_torso_prismatic_no_lift]] |
| lidar | 360 zraka, 10 Hz | `URDF` l. 192–201 | [[P-06_classic_only_sensors]] |
| RGBD kamera | 640×480, 15 Hz, HFOV 1.211 | `URDF` l. 226–236 | [[S-05_perception]] |
| pan-tilt početni pitch | **0.45 rad** (~25.8° dolje prema stolu) | `URDF` l. 355 | usmjerenje prema stolu 75 cm |
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

## Navigacija (Nav2 + zone iz detektiranih značajki)
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| footprint | **±0.52 × ±0.427 m** (1.04 × 0.854 m; bilo ±0.45) | `NAV` local/global costmap | izmjerena širina u `ARM_CARRY_V2` ([[P-35_arm_span_too_wide_for_door]]); 0.90 m je bila procjena zbog koje je `ObstacleFootprint` odbacivao sve trajektorije ([[P-39_nav2_enters_doorway_at_an_angle]]) |
| `inflation_radius` | **0.45**, `cost_scaling_factor` 5.0 | `NAV` `global_costmap` | lokalni costmap nema inflaciju da ne zatvori uski prolaz |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | **0.10 m / 0.05 rad** (bilo 0.20 / 0.25) | `NAV` `general_goal_checker` | 0.25 rad traži 1.085 m otvora, a ima ga 0.95 m ([[P-39_nav2_enters_doorway_at_an_angle]]) |
| DWB upravljač | `dwb_core::DWBLocalPlanner` bez RotationShimController | `NAV` `FollowPath` | **Nav2 vozi svaku dionicu**, i prilaznu i onu kroz vrata; kod vrata je samo preduvjet (`aligned_with`) i keepout traka |
| `ObstacleFootprint.scale` / `PathDist.scale` / `GoalDist.scale` | **2 / 64 / 24** | `NAV` `FollowPath` | `PathAlign.scale` 48, `forward_point_distance` 0.10 m; DWB za prilazne i sobne dionice |
| `local_costmap.filters` | **`["keepout_filter"]`** (prije ga nije bilo) | `NAV` `local_costmap` | bez toga DWB ne vidi zone i siječe kut ([[P-39_nav2_enters_doorway_at_an_angle]]) |
| DWB `max_vel_x` / `max_vel_y` / `max_vel_theta` | **0.30 / 0.30 / 0.60** | `NAV` `FollowPath` | omni raspon s 15 uzoraka po osi; **puni bočni raspon** — tako je vožen provjereni run (sloj koji ga je spustio na 0.20 je povučen) |
| DWB `acc_lim_x` / `acc_lim_y` / `acc_lim_theta` | **1.5 / 1.5 / 3.0** | `NAV` `FollowPath` | usklađeno s postojećim `velocity_smoother` |
| velocity smoother max v / vy / ω | **0.30 / 0.30 / 0.60** | `NAV` `velocity_smoother` | usklađeno s DWB |
| zone: `chute_length` / `chute_thickness` | **0.50 / 0.65 m** | `nav_zones.py` `default_params` | kraći lijevak ne blokira okret na portalu |
| zone: `lane_margin` | **−0.20 m** (negativno = šire) | `nav_zones.py` | vodi globalnu putanju bez sužavanja stvarnog otvora |
| zone: `portal_standoff` | **0.95 m** (portal je 1.45 m od vrata) | `nav_zones.py` | `check_zones.py` prolazi svih osam provjera prostora za okret |
| zone: `table_clearance` / `table_standoff` / `table_gate_margin` | 0.55 / **0.80** / **0.55 m** (gate bio 0.20) | `nav_zones.py` | prolaz kroz halo ide **punom širinom** halo-a: na 0.20 m robot je mogao doći pred stol, ali se nije mogao okrenuti natrag ([[P-39_nav2_enters_doorway_at_an_angle]], pokušaj 7) |
| navigator: gate pred vratima | `centreline_tolerance` **0.08 m**, `heading_tolerance` **0.087 rad (5°)** | `room_navigator.py` `aligned_with` | **vraćeno 14. 9.** na stanje provjerenog runa: prolaz koji je stvarno uspio bio je na **4.2°**, pa bi ga commitanih 0.07 rad (4.0°) odbilo ([[P-39_nav2_enters_doorway_at_an_angle]], pokušaj 10) |
| navigator: `square_corners` | **False** | `room_navigator.py` | L-detour kroz sredinu sobe isključen; provjereni run je praznu sobu prešao izravnom dijagonalom i poravnao se na sljedećem portalu |
| navigator: `min_side_clearance` / `arm_tolerance` | **0.03 m** / 0.15 rad | `room_navigator.py` | vrijednosti provjerenog runa; `/joint_states` se prati tijekom vožnje ([[P-37_arm_position_gain_sag]]) |
| **`footprint_publisher`** (novo 14. 9.) | 5 Hz, okvir `base_link`, `collision` geometrija, `max_vertices` 24, `change_tolerance` 0.01 m, `max_reach` 1.50 m | `footprint_publisher.py`, diže ga `sim.launch.py` | objavljuje **stvarni** obris robota na `footprint` oba costmapa; statični `footprint` u `NAV` ostaje rezervna vrijednost ([[D-19_dynamic_footprint]]) |
| **`collision_monitor`** (novo 14. 9.) | `FootprintApproach` (`time_before_collision` 1.5 s, korak 0.1 s, `max_points` 5) + `BaseStop` 0.80 × 0.56 m; izvor `/scan_filtered`, `source_timeout` 0.5 s | `config/collision_monitor.yaml`, diže ga `nav2.launch.py` (`safety:=false` isključuje) | usporava po **izmjerenom** footprintu prije dodira; jedini sloj koji ne ovisi o tome je li costmap vidio prepreku |
| relay: prednost sigurnosnog topica | `/cmd_vel_safe`, `safe_command_timeout` 1.0 s | `cmd_vel_relay.py` | dok monitor objavljuje, sirovi `/cmd_vel` se ignorira; bez monitora ništa se ne mijenja (teleop radi) |
| `scan_filter` | `half_length` 0.75, `half_width` **0.47** | `scan_filter.py` | **ne dizati na 0.52**: dovratnici stoje na ±0.475 m, pa ih 0.52 briše iz `/scan_filtered`, dakle i iz lokalnog costmapa i iz AMCL-a |
| provjera zona: `TURN_MARGIN` | 0.15 m povrh opisanog radijusa, uz pomak od 0.10 m | `scripts/check_zones.py` | DWB pri „okretu u mjestu" i translatira; golo nepreklapanje nije kriterij |
| **izmjereno** (detekcija iz `maps/seminar_map`) | vrata (0.00, −3.00) i (3.00, −0.01), širina **0.95 m** (stvarna 1.0 m); stolovi (−0.04, −6.53) i (6.54, −0.01) | `scripts/check_doors.py`, `scripts/check_zones.py` | SLAM zadeblja zid ~2.5 cm po strani; zone se grade na izmjerenom, ne na nazivnom |
| slam_toolbox | async, pomak 0.2 m / 0.2 rad, `base_footprint`, `/scan_filtered`, rezolucija 0.05 m | `config/slam_params.yaml`, `mapping.launch.py` | [[R-14_slam_mapping]]; karta prihvaćena u runu 44 |

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

## Poze ruku
Izmjerene dimenzije svake poze su u [[08_poze]] (snima ih `scripts/capture_posture.py` iz RViz-a).
**Stvarna** širina se mjeri s `scripts/fit_test.py` (hodnik s prorezom u MoveIt sceni) ili
`scripts/mesh_extent.py` (vrhovi meshova): `ARM_CARRY_V2` je **85.4 cm** širok, najširi je
`spherical_wrist_2` ([[P-35_arm_span_too_wide_for_door]]).

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
