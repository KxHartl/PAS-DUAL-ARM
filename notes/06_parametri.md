---
id: PARAMETRI
type: registar
updated: 2026-09-17
---
# Registar parametara (jedini izvor istine za podesive vrijednosti)

> [!warning] Pravilo
> Svaka promjena vrijednosti se upisuje **ovdje** (nova vrijednost + datum) **i** kao novi red u
> tablici pokušaja pripadne P-kartice. Vrijednosti su pročitane iz koda; zadnja provjera 17. 9. 2026. (`datoteka:linija`).
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
| točka odlaganja / **ArUco marker** | ploča **0.40 × 0.40** na (**6.32**, 0, **0.7515**), marker **id 3**, **0.36 m** (quiet zone **5 %**, ne 12.5 % kao na kutiji) | `WORLD` model `place_marker`, tekstura `aruco_marker_3.png` (`scripts/make_marker.py`) | 16. 9.: zamijenio crveni X, kojeg robot nije mogao vidjeti. Marker je **veći od kocke** (0.36 > 0.30) namjerno: kocka od 0.30 m prekrila bi marker od 0.30 m u cijelosti, a ovako oko nje ostaje **3 cm** markera sa svake strane — i za oko i za provjeru. 0.22 m od bližeg ruba, kocki ostaje 0.07 m rezerve |

## Robot
| trenje kotača mu1 / mu2 | **0.80 / 0.20** (anizotropno, fdir1 ±45° u `base_footprint`) | `src/pas_dual_arm_bringup/urdf/base/wheel.urdf.xacro` | [[P-09_omni_drive_on_fortress]], [[R-08_omni_controller]] |
| effort limit kotača / prigušenje | **100.0 Nm** / damping 0.0, friction 0.0 | `src/pas_dual_arm_bringup/urdf/base/wheel.urdf.xacro` | DART SERVO constraint za omni pogon ([[P-09_omni_drive_on_fortress]]) |
| base controller | `mecanum_drive_controller/MecanumDriveController` | `CTRL` l. 28, 41–64 | [[R-08_omni_controller]], zamijenio diff_drive_controller |
| trenje jastučića prstiju | mu1 = mu2 = 5.0 | `URDF` l. 122–123 | squeeze |
| masa vodilice / klizača | 12 kg / 2 kg | `dual_arm_torso.urdf.xacro` l. 24, 55, 92 | procjena ([[R-06_realistic_parameters]]) |
| klizač limit | **0.05–0.65 m** (13. 9.; bilo 0.05–0.8), 1000 N, 0.5 m/s | `dual_arm_torso.urdf.xacro` l. 71, 105 | hod stvarne vodilice (odluka korisnika); [[P-13_torso_prismatic_no_lift]] |
| torzo command interface / PID | `effort`; p=1500, i=500, d=100, i_clamp=300; goal tolerancija 0.01 m, goal_time 3 s | `URDF` + `CTRL` `torso_controller` | izolirani headless pokus C2: 0.05→0.20→0.05 m, stvarna greška nakon 10 s <1 mm ([[P-13_torso_prismatic_no_lift]]) |
| vodilice `initial_value` | 0.05 → **0.06 m** (15. 9.) | `URDF` `torso_system` | 0.05 je bio jednak donjem graničniku, pa je robot startao zaglavljen: Ignition poziciju izvodi kao brzinu zgloba, a na graničniku je ponišava **u oba smjera**. Jedina izmjena koja je riješila [[P-13_torso_prismatic_no_lift]] |
| vodilice command interface | **`position`** (zadani profil), bez regulatora | `URDF` + `CTRL` | `effort` + PID više **nije potreban**: 0.06 → 0.20 → 0.35 m pod teretom ruku, greška 0.0000 mm |
| torzo trajna greška | **0.0000 mm** obje vodilice, razmak lijevo-desno 0.0000 mm (15. 9.) | mjereno na `/joint_states` | pojačanja iznad nisu mijenjana; onih „3 mm“ iz `table_ready` bilo je mjereno 3 s nakon putanje, dok vodilice još pužu ([[P-13_torso_prismatic_no_lift]]) |
| pojačanja ruku, `force_grasp` profil | iz **izmjerene** efektivne inercije: ω=150 rad/s, ζ=1, i=p/2, i_clamp=granica momenta (39/9 Nm). p = 387 / 10181 / 266 / 4525 / 178 / 398 / 34.9; d = 5.16 / 135.8 / 3.54 / 60.3 / 2.37 / 5.31 / 0.465 (15. 9.) | `scripts/apply_measured_arm_gains.py` → `force_controllers.yaml` | KDL matrica mase precjenjuje inerciju zakretnih zglobova ~30× i vodi u bang-bang titranje ([[P-41_effort_pid_arm_actuator_profile]]) |
| `force_vision_age` (svježina TF-a markera) | 0.4 → **0.8 s** (15. 9.) | `force_grasp.py` `defaults` | Izmjereno tijekom stiska, 400 očitanja: marker TF kako ga **čvor čita** je median **0.32 s**, p95 0.388, max 0.407 — stara granica je bila **ispod tipične starosti**. Senzor nije kriv: detektor objavljuje svakih 0.066 s bez ijedne rupe, kašnjenje do pretplatnika 22–43 ms ([[P-19_aruco_foreshortening_close]]) |
| `force_status_age` / `force_future` (svježina procjene sile) | 0.15 s / **0.05 s** | `force_grasp.py`, `wrench_estimator.py` | Izmjereno: sila je median 0.008 s, p95 0.023 s. `force_future` pokriva oznake **u budućnosti** — 3.6 % poruka do 19 ms, jer `/clock` i podatkovne teme nemaju zajamčen redoslijed |
| prigušenje diferencijalnog koraka stiska | **0.005** (15. 9.) | `force_model.resolved_rate` | Preko 200 slučajnih Jacobiana: 0.002 gubi 1.0 % traženog pomaka, **0.005 → 5.6 %**, 0.01 → 16.7 %, 0.05 → 54 %. Zamjenjuje MoveIt IK u koraku stiska ([[P-25_asymmetric_arm_reach]]) |
| lidar | 360 zraka, 10 Hz | `URDF` l. 192–201 | [[P-06_classic_only_sensors]] |
| RGBD kamera | 640×480, 15 Hz, HFOV 1.211 | `URDF` l. 226–236 | [[S-05_perception]] |
| pan-tilt početni pitch | **0.45 rad** (~25.8° dolje prema stolu) | `URDF` l. 355 | usmjerenje prema stolu 75 cm |
| contact senzori | 50 Hz | `URDF` l. 149 | [[P-27_contact_sensor_topic_ignored]] |
| DetachableJoint | `left_bracelet_link` ↔ `aruco_box` | `URDF` l. 175–181 | [[D-05_contact_verified_attach]] |

## ros2_control
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| `update_rate` | 100 Hz | `CTRL` l. 3 | — |
| `base_controller` tip | **MecanumDriveController** (od 13. 9.) | `CTRL` l. 28 | [[R-08_omni_controller]] riješen; [[D-03_diff_drive_base_temporary]] više ne vrijedi |
| kotač r / projekcija osi | 0.0762 m / `sum_of_robot_center_projection_on_X_Y_axis` **0.467575** | `CTRL` `base_controller.kinematics` | PAL vrijednosti |
| `reference_timeout` | 0.5 s | `CTRL` `base_controller` | mecanum blok nema `cmd_vel_timeout` ni v/ω limite — oni su na Nav2 strani (`velocity_smoother`, DWB) |
| odom kovarijanca | `pose`/`twist` dijagonala 0.001 (yaw 0.01) | `CTRL` `base_controller` | ulazi u AMCL-ov `OmniMotionModel` preko `alpha1..5`, ne izravno |
| hvataljke | `allow_stalling: true` | `CTRL` l. 111, 116 | — |
| CM timeout spawnera | 120 s | `sim.launch.py` l. 107 | [[P-32_gui_starves_controllers]] |

## Navigacija (Nav2 + zone iz detektiranih značajki)
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| `footprint_padding` | **0.0** (bilo 0.01 po defaultu) | `NAV` oba costmapa | footprint više nije procjena nego **izmjerena** ljuska stvarne geometrije ([[D-19_dynamic_footprint]]); padding je dvostruko brojanje i uzimao je 2 cm od ~8 cm budžeta |
| `local_costmap.resolution` | **0.025 m** (bilo 0.05), frekvencija **15.0 Hz update / 5.0 Hz pub** (bilo 5/2) | `NAV` `local_costmap` | rezerva u vratima je ~4 cm po strani; kašnjenje prepreka smanjeno s 200 ms na 66 ms |
| footprint | **±0.52 × ±0.427 m** (1.04 × 0.854 m; bilo ±0.45) | `NAV` local/global costmap | izmjerena širina u `ARM_CARRY_V2` ([[P-35_arm_span_too_wide_for_door]]); 0.90 m je bila procjena zbog koje je `ObstacleFootprint` odbacivao sve trajektorije ([[P-39_nav2_enters_doorway_at_an_angle]]) |
| `inflation_radius` / `cost_scaling_factor` | **0.45 m / 2.0**, samo globalni costmap | `NAV` `global_costmap.inflation_layer` | **Gornja granica je NavFn-ova, ne izbor.** Centar robota u otvoru od 0.98 m smije biti 0.427–0.49 m od dovratnika; polje koje doseže dalje od 0.49 m taj koridor podigne na vrh zasićene cijene (`50 + 0.8 × v`, strop 253) i NavFn ga ne uspije proći gradijentnim spustom — run 65, `Failed to create a plan from potential`. Na 0.45 m polje je široko svega 2.3 cm, pa putanje opet liježu uz prepreke na 0.451 m, ispod praga 0.52 m ([[P-39_nav2_enters_doorway_at_an_angle]] #24–30) |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | **0.05 m / 0.025 rad (1.4°)** (bilo 0.10 / 0.05, prije toga 0.20 / 0.25) — **postoji na DVA mjesta i moraju biti jednaka**: `general_goal_checker` i `FollowPath` | `NAV` | **mora biti uže od gatea** (`centreline_tolerance` 0.08, `heading_tolerance` 0.087): pritegnuto na 1.4° kako robot ne bi ulazio s kutem koji izbočuje kutove u dovratnik |
| `progress_checker.required_movement_radius` | **0.05 m** / 30 s (bilo 0.10) | `NAV` `controller_server` | mora biti ispod tolerancije cilja, inače završni prilaz ne može zadovoljiti uvjet i Nav2 zdravog robota proglasi zaglavljenim |
| Upravljač baze | **`DWBLocalPlanner`** (direktno), **20 Hz** (bilo 10 Hz) | `NAV` `FollowPath` | Uklonjen `RotationShimController`; `Twirling.scale: 20.0` (bilo 8.0) strogo drži pravac bez yaw rotacije; `debug_trajectory_details: False` |
| DWB uzorkovanje i granularnost | **$25 \times 25 \times 25$ uzoraka**, `linear_granularity` **0.01 m**, `angular_granularity` **0.01 rad** | `NAV` `FollowPath` | Finoća brzine 0.024 m/s (bilo 0.043 m/s), rollout provjera kolizije svakih 1 cm (bilo 5 cm) |
| `ObstacleFootprint.scale` / `PathDist.scale` / `GoalDist.scale` / `Twirling.scale` | **2 / 96 / 24 / 20** | `NAV` `FollowPath` | `PathDist.scale: 96.0` (bilo 64.0) pojačava privlačenje na središnju liniju prolaza vrata; `Twirling.scale: 20.0` |
| `local_costmap.filters` | **`["keepout_filter"]`** (prije ga nije bilo) | `NAV` `local_costmap` | bez toga DWB ne vidi zone i siječe kut ([[P-39_nav2_enters_doorway_at_an_angle]]) |
| DWB `max_vel_x` / `max_vel_y` / `max_vel_theta` | **0.30 / 0.15 / 0.30** | `NAV` `FollowPath` | lateralna brzina i kutna brzina pritegnute za stabilno pravocrtno kretanje kroz prolaz |
| DWB `acc_lim_x` / `acc_lim_y` / `acc_lim_theta` | **1.5 / 1.5 / 3.0** | `NAV` `FollowPath` | usklađeno s `velocity_smoother` |
| velocity smoother frekvencija | **40.0 Hz**, `odom_duration` 0.05 s | `NAV` `velocity_smoother` | dvostruko brže izglađivanje naredbi bazi |
| **LIDAR (`base_lidar`)** (nadograđeno 14. 9.) | **SICK TiM571 standard**: 1080 zraka ($0.33^\circ$), točnost **0.001 m (1 mm)**, min domet **0.05 m**, **25 Hz** (bilo 360 zraka, 10 mm, 0.12 m, 10 Hz) | `robot.urdf.xacro` | Usklađeno sa službenim PAL Tiago Omni Base lidarom (`omni_base_description`). 3x gušći oblak točaka, 10x točnije mjerenje udaljenosti |
| **AMCL lokalizacija** (nadograđeno 14. 9., dvaput) | *(prvo)* `max_beams` 60 → 180, `update_min_d` 0.25 → **0.03 m**, `update_min_a` 11.5° → **0.03 rad (1.7°)**, `pf_err` **0.02**. *(zatim, nakon runa 61)* **mjerni model:** `sigma_hit` 0.2 → **0.05**, `z_hit`/`z_rand` 0.5/0.5 → **0.9/0.1**, `laser_likelihood_max_dist` 2.0 → **0.5**, `laser_max_range` 100 → **12.0**, `laser_min_range` −1 → **0.05**, `max_beams` 180 → **360**; **skup čestica:** `resample_interval` 1 → **2**, min/max 500/2000 → **1000/3000**; `transform_tolerance` 1.0 → **0.3**. `alpha1..5` ostaju **0.2** | `config/nav2_params.yaml` | Prvo je uklonjena mrtva zona ažuriranja. Run 61 je zatim izmjerio **stalan pomak** AMCL-a od 4–6 cm (isti u oba smjera kroz ista vrata) — polje vjerojatnosti sa `sigma_hit` 0.2 i `z_rand` 0.5 je preravno da bi povuklo procjenu, a resampling svaka 3 cm uz 500 čestica je osiromašio skup ([[P-40_amcl_pose_disagrees_with_lidar]]) |
| zone: `chute_length` / `chute_thickness` | **0.50 / 0.65 m** | `nav_zones.py` `default_params` | kraći lijevak uz sam zid ne blokira prilaz vratima |
| zone: `lane_margin` | **−0.15 m** (negativno = šire) | `nav_zones.py` | širina trake 1.25 m (vrata 0.95 m), guide walls odmaknuti 15 cm od dovratnika kako umjetni keepout ne bi gušio DWB |
| zone: `portal_standoff` | **0.95 m** → portal je **1.45 m** od vrata (`chute_length` 0.50 + `portal_standoff` 0.95) | `nav_zones.py` l. 239 | `check_zones.py` prolazi sve provjere prostora za okret. *(Raniji zapis „1.80 m" računao je s `chute_length` 0.85, koji je 14. 9. spušten na 0.50.)* |
| zone: `table_overhang` / `table_standoff` | **0.05 / 1.15 m** (`table_clearance` i `table_gate_margin` **uklonjeni** — oko stola više **nema** keepout zone) | `nav_zones.py` `default_params` | Korisnik, run 63: robot mora doći do stola da uzme i ostavi kutiju, pa se stol ne ograđuje; ostaje prepreka u static layeru (noge) i globalnom `inflation_layer`-u. `table_standoff` se sad mjeri **od ruba stola do centra robota** → prilazna poza 1.55 m od centra stola (radnih 1.50 m iz runova 46–58). `table_overhang` = ploča 0.80 − detektiranih 0.70, po strani |
| **`planner_inflation`** (novo 14. 9.) | **0.427 m** = upisani radijus otiska 1.04 × 0.854 | `nav_zones.py`, `/keepout_filter_mask_planner` | NavFn planira **točku**, DWB provjerava **otisak**, a nav2 filtere obrađuje nakon plugina pa `inflation_layer` nikad ne napuhne keepout → putanja je legalno išla uz sam rub zone, a izvesti je nemoguće. Zone se zato objavljuju u **dvije veličine**: sirova za lokalni costmap, napuhana za globalni ([[P-39_nav2_enters_doorway_at_an_angle]] #21–23) |
| navigator: gate pred vratima | `centreline_tolerance` **0.08 m**, `heading_tolerance` **0.087 rad (5°)** | `room_navigator.py` `aligned_with` | prolaz koji je stvarno uspio bio je na **4.2°** ([[P-39_nav2_enters_doorway_at_an_angle]], pokušaj 10) |
| navigator: `square_corners` | **False** | `room_navigator.py` | L-detour kroz sredinu sobe isključen; provjereni run je praznu sobu prešao izravnom dijagonalom i poravnao se na sljedećem portalu |
| navigator: `min_side_clearance` / `arm_tolerance` | **0.005 m (5 mm)** / 0.15 rad | `room_navigator.py` | smanjeno s 0.03 m na 0.005 m; provjerava se samo za `leg.transit`; 3 cm je prekinulo prolaz u runu 58 pri 0.7 cm čistog razmaka bez kolizije |
| **`footprint_publisher`** | 5 Hz, okvir `base_link`, `collision` geometrija, **`max_vertices` 8**, `change_tolerance` 0.01 m, `resend_period` 2.0 s, `max_reach` 1.50 m | `footprint_publisher.py` | objavljuje **stvarni** obris robota na `footprint` oba costmapa |
| **`collision_monitor`** | `FootprintApproach` (`time_before_collision` 1.5 s, korak **0.05 s**, `max_points` **10**) + `BaseStop` 0.80 × 0.56 m; izvor `/scan_filtered` | `config/collision_monitor.yaml` | usporava po **izmjerenom** footprintu prije dodira; dvostruko češća provjera |
| relay: prednost sigurnosnog topica | `/cmd_vel_safe`, `safe_command_timeout` 1.0 s | `cmd_vel_relay.py` | dok monitor objavljuje, sirovi `/cmd_vel` se ignorira |
| `scan_filter` | `half_length` **0.45**, `half_width` **0.30** (bilo 0.75 / 0.47) | `scan_filter.py` | Dimenzionirano prema onome **što je na visini lasera** (0.2086 m) |
| provjera zona: `TURN_MARGIN` | 0.15 m povrh opisanog radijusa, uz pomak od 0.10 m | `scripts/check_zones.py` | DWB pri „okretu u mjestu" i translatira; golo nepreklapanje nije kriterij |
| **izmjereno** (detekcija iz `maps/seminar_map`, **karta runa 60**) | vrata (+0.000, −2.990) i (+3.010, +0.000), širina **0.980 m** (stvarna 1.0 m), os prolaza **0.0 cm**; zid 0.10 m nacrtan **0.120 m**; lice zida nagib ≤ 0.02°, RMS ≤ 2.3 mm; mjerilo karte **0.0 mm** na 4.243 m; stolovi (+6.509, +0.008) i (+0.011, −6.501) | `scripts/check_doors.py`, **`scripts/check_map_geometry.py`**, `scripts/check_zones.py` | Zone se grade na izmjerenom, ne na nazivnom. *(Karta runa 44 na rešetki 0.05 m davala je 0.950 m, os +1.5 cm i zid 0.150–0.178 m — kvantizacija, ne drift; [[P-40_amcl_pose_disagrees_with_lidar]])* |
| **slam_toolbox** (nadograđeno 14. 9.) | async, `base_footprint`, `/scan_filtered`; **rezolucija 0.02 m** (bilo 0.05), graf svakih **0.1 m / 0.1 rad** (bilo 0.2), `minimum_time_interval` **0.2 s** (bilo 0.5), `scan_buffer_size` **20** (bilo 10), `correlation_search_space_smear_deviation` **0.03** (bilo 0.1), `min_laser_range` **0.05** (bilo 0.12) | `config/slam_params.yaml`, `mapping.launch.py` | [[R-14_slam_mapping]]. Karta iz runa 44 je na 0.05 m crtala zid od 0.10 m kao 0.15 m i otvor od 1.00 m kao 0.950 m, a snimljena je **prije** lidara od 1080 zraka ([[P-40_amcl_pose_disagrees_with_lidar]]) |
| **geometrijski gate karte** (novo 14. 9.) | širina vrata ≥ **0.97 m**, os prolaza ≤ **1 cm**, debljina zida ≤ **0.14 m**, lice zida RMS ≤ **10 mm** / nagib ≤ **1.0°**, stepenica preko otvora ≤ **20 mm**, mjerilo ≤ **20 mm** | `scripts/check_map_geometry.py`, zove ga `scripts/save_map.sh` | Pokrivenost (`check_map.py`) ne kaže ništa o tome je li prolaz još 1.0 m širok; s 7.3 cm rezerve odlučuje geometrija |
| **`loc_error`** (debug, novo 14. 9.) | `sim.launch.py debug_truth:=**false**` po defaultu; ground truth na `/debug/gz_dynamic_pose`, ispis na `/debug/loc_error`, izvještaj 2 Hz | `loc_error.py`, `config/bridge_debug.yaml` | Mjeri \|TF `map→base_footprint` − Gazebo poza\|. **Isključivo dijagnostika** — nijedan upravljački čvor to ne smije čitati jer stack mora raditi i na fizičkom robotu |
| **telemetrija lidar-vs-poza** (novo 14. 9.) | prag `DOORWAY_SPAN` **1.5 m** (ispod toga su bočni povrati dovratnici, ne zidovi sobe) | `room_navigator.py` `_report_disagreement` | Ispisuje razliku lidarske i lokalizirane osi u svakom prolazu, **bez** ground trutha. Samo ispis; ništa se ne gate-a ([[D-18_verified_baseline_first]]) |
| **predviđanje putanje offline** (novo 14. 9.) | prag: putanja ne smije prići stvarnoj prepreci bliže od **0.52 m** (polovica duljine robota) | `scripts/check_costmap_path.py` | Imitira globalni planer iz spremljene karte (static + `inflation_layer`, pa **tek onda** keepout — redoslijedom kojim ih nav2 slaže) i Dijkstrom po NavFn-ovoj cijeni. Daje razmak putanje od prepreka **prije vožnje**; prag je iz `FootprintApproach`, čija projekcija pri `v → 0` i dalje seže pola duljine robota |
| **`field_width` / `field_peak` / `field_core`** (novo 14. 9., [[D-20_single_potential_field_costmap]]) | **0.30 m / 35 / 0.427 m** (`furrow_margin` **0.10 m**, `table_dock_safety` **0.10 m**; `global_costmap.inflation_layer enabled: false`) | `nav_zones.py` `Zones.field`, `/keepout_filter_mask` i `..._planner` | **Jedinstveno potencijalno polje za cijelu navigaciju (D-20).** Zamijenilo stari `table_field` i ugasilo globalni `inflation_layer` koji je stvarao dvostruko brojanje i "djetelinu" (run 68). Rampa sa širinom 0.30 m i vrhom 35 (cijena 89 u Costmap2D) drži putanju na razmaku 0.715 m od stvarnih prepreka (optimalno unutar 0.60–0.90 m). Kroz vrata i prilaz stolu izrezane su brazde nulte cijene (`_furrows`) kako bi NavFn gradijentni spust ostao nezasićen. |
| **polje (`field_width` / `field_peak` / `furrow_margin`)** (novo 14. 9.) | **0.40 m / 50 / 0.10 m**; jezgra = `planner_inflation` 0.427 m | `nav_zones.py` `Zones.field`, u obje maske | **Jedno potencijalno polje umjesto tri izvora odbijanja** ([[D-20_single_potential_field_costmap]]). Udaljenost se mjeri od **oboda ploče** (0.80 m), ne od nogu koje lidar vidi. Vrijednosti odabrane mjerenjem: razmak putanje određuje širina rampe (putanja sjedne tik izvan nje), pa 0.25/0.30/0.35/0.40 daje 0.659/0.715/0.756/0.800 m, a konačnih 0.40/50 daje **0.835 m** — iznad praga 0.52 m (deadlock `collision_monitor`-a) i ispod 0.90 m (ljubljenje suprotnog zida) |
| **brazde (`_furrows`)** (novo 14. 9.) | vrata: polušírina 0.640 m, doseg ±1.45 m; stol: polušírina 0.527 m niz prilaznu os. U brazdi rampa = **0**, jezgra ostaje | `nav_zones.py` `Zones._furrows` | Tako se na costmapu bez negativnih brojeva izvodi **privlačenje**: prolaz se ne privlači, nego se sve pokraj njega odbija. Nulta cijena u brazdi je **uvjet**, ne ugađanje — NavFn potencijal vadi gradijentnim spustom i zasićena brazda ga ruši (run 65) |
| **dock poza (`table_dock_safety`)** (16. 9.: 0.10 → 0.15) | **0.15 m**; dock = ploča 0.401 + pola duljine 0.52 + 0.15 = **1.071 m** od centra stola | `nav_zones.py` `_build_poses`, `room_navigator._undock_leg` | Najbliža poza koju Nav2 smije držati: čelo **15.0 cm** od ploče. **Na njoj se ne smije okretati** (opisani radijus 0.673 m bi zakačio stol), pa se s nje izlazi isključivo unatrag po istoj osi do `approach` poze (1.35 m). Zahtjev `/room_navigator/goto "<soba>:dock"`, izlaz `"undock"` |
| **`inflation_layer` (globalni)** | **`enabled: false`** (blok i vrijednosti ostaju) | `NAV` `global_costmap.inflation_layer` | Izotropan je, a robot 1.04 × 0.854 m nije: čeono je puštao centar na 0.803 m gdje je fizički minimum 0.921 m (12 cm **u** ploču), bočno prestrog. Uz to je neizbježan u vratima. Ostavljen ugašen da se usporedba može vratiti jednom riječi; `/global_costmap/costmap` objavljuje neovisno o tome, pa RViz prikazi rade |

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
| pre-squeeze / press | +0.10 m / **−`squeeze_interference`** | `MT` `squeeze_poses` | [[P-24_press_path_chain]] |
| `squeeze_interference` | **0.010 m** (bilo 0.030) | `MT` parametar | 30 mm je gurao kocku po stolu, a komentar je tvrdio 5 mm; run F9 |
| `press_tilt` | **0.873 rad (50°)** prema dolje | `MT` parametar | vodoravno = 1.003 m širine, 50° = **0.823 m**, uže od `ARM_CARRY_V2`; mjeri `scripts/grasp_width.py`, [[P-43_grasp_pose_wider_than_door]] |
| ready poza prije pred-hvata | **uklonjena** (bila `ARM_HOME`) | `MT` STEP5 | `ARM_HOME` je 1.409 m širok, ruke su se vidno raširile; iz uske spawn poze plan do nje pada (run F10) |
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
| desna nakon attacha | **`release_backoff` 0.005 m** (bilo povlačenje 0.10 m) | `MT` parametar | korisnik 15. 9.: obje ruke moraju ostati na kocki |
| lift | +0.15 m **na vodilicama** (bilo rukom) | `MT` STEP6 | [[P-13_torso_prismatic_no_lift]] riješen |
| `carriage_reference_height` / `_cube_z` | 0.20 m / 0.82 m | `MT` parametar | visina hvata 1:1 iz izmjerene visine kocke |
| `door_width` | 1.0 m | `MT` parametar | usporedba u `measure_width()` |
| place | pick z −0.02 m, tol 0.04, 3 pokušaja | `MT` l. 1788–1795 | [[P-29_place_drop_tips_cube]] |
| odmak nakon placea | 0.10 m | `MT` l. 1800 | — |

## Hvat preko V4 poza (`main_task`, [[P-44_grasp_from_reference_pose]])
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| poza vožnje `ARM_DRIVE` | **`DRIVE_V4`** (bila `ARM_CARRY_V2`), 0.821 m | `postures.py`, xacro spawn, `sim.launch` | korisnik 16. 9. |
| `DRIVE_CARRIAGE` / `DETECTION_CARRIAGE` / `GRASP_CARRIAGE` | 0.20 / 0.40 / 0.40 m | `postures.py` | spremljeno s V4 pozama |
| `GRIPPER_CLOSED` | 0.791 | `postures.py` | hvataljke u V4 pozama |
| `{side}_tool_tip` | 0.127 m duž osi alata od `end_effector_link` | `robot.urdf.xacro` | središte zatvorenih jastučića (prednji rub 0.163 m) |
| `detect_stand_range` | 0.87 m (dock) | `main_task` | lociranje kocke; DRIVE_V4 36 cm od stola |
| `detection_carriage_height` | 0.40 m | `main_task` | DRIVE_V4 na 400 mm: 46.6 cm od stola |
| `detection_cube_range` | 0.63 m | `main_task` | gdje V4 poze očekuju kocku |
| `touch_depth` | **0.002 m** | `main_task` | samo dodir za senzore; stisak izbacuje kocku (korisnik) |
| `grasp_approach_speed` | 0.01 m/s | `main_task` | zadnjih ~2 cm |
| `back_off_after_lift` | **0.50 m** (bilo 0.40) | `main_task` | nakon dizanja unatrag; na 0.40 m prednja ploha kocke je točno na rubu stola i spuštanje ga struže (0.0 cm), na 0.50 m 10 cm |
| `final_carriage_height` | **0.10 m** (bilo 0.20) | `main_task` STEP7 | korisnik: na kraju vodilice na 100 mm; MoveIt: stanje bez samosudara |
| `pull_cube_in` | **0.15 m** | `main_task` STEP6b | korisnik: privući kocku robotu nakon dizanja; širina ostaje 0.822 m. Stražnja ploha kocke 4.7 cm ispred torza (20 cm bi presjekao torzo) |
| `pull_elbow_out_deg` | **20°** (šake stoje) | `main_task` STEP6b | bez zakreta: 6 cm je granica, od 7 cm laktovi diraju stup torza kad se vodilice spuste; 15/20/25° sve valjano u MoveIt-u na 0.55/0.20/0.10 |
| `pull_torso_clearance` / `pull_speed` | 0.03 m / 0.02 m/s | `main_task` STEP6b | povlačenje se skraćuje ako bi kocka prišla torzu bliže od 3 cm; prije pokreta MoveIt provjera konačnog stanja, inače se preskače |
| `detection_widen` | 0.05 m (šake van po y) | `main_task` | korisnik: malo šire; kamere 0.35 m od markera, 20 cm od kocke pri primicanju |
| vodilice + ruke istodobno | ruke kreću na **30 %** hoda vodilica (`arm_start_delay`), kroz točku **12 cm** iznad DETECTION (`detection_via_rise`), pa ravno dolje | `main_task._torso_and_arms_together` | korisnik; offline ≥ 10.3 cm od stola (izravna interpolacija bez međutočke: 1.8 cm) |
| `together_min_range` | 0.85 m | `main_task` | put provjeren s kockom na docku; bliže → vodilice pa MoveIt |

## Misija — konačni sim (`main_task`, 16. 9. 2026.)
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| `mission` | **false** (uključuje ga `mission.launch.py`) | `main_task` | misijski način: ruke u `DRIVE_V4` → čekanje korisnika → vožnja kroz `room_navigator` → hvat. Isključen, ponašanje je točno kao u runovima V2–V5 (spawn na docku) |
| `pick_room` / `place_room` | **`blue`** / **`red`** | `main_task` | imena soba dolaze iz `nav_zones` `room_labels` (`home:0,0`, `blue:0,-6`, `red:6,0`); stol u sobi daje `approach` i `dock` poze |
| `goto_timeout` | **600 s** | `main_task._goto` | gornja granica čekanja na navigator; `failed`/`aborted`/`cancelled` prekidaju odmah, bez čekanja isteka |
| naredba korisnika | tema **`/mission/start`** (`std_msgs/String`, ime sobe; prazno = `pick_room`) | `nav_gui` gumb „MISIJA: po kutiju", ili `ros2 topic pub` | ništa se ne vozi prije te poruke (zahtjev korisnika, 16. 9.) |
| spawn za misiju | `carry_arms:=false` → `ARM_ZERO` (raširene ruke), poza (0, 0, 0°) | `mission.launch.py` | korisnik želi vidjeti kako se ruke slože u `DRIVE_V4`; (0,0,0) je i AMCL `initial_pose` u `NAV`, pa se poza ne postavlja ručno |
| referentna poza ruku za gate | tema **`/room_navigator/arm_posture`** (latched): `DRIVE_V4` prazan, **`CARRY_V4`** s kutijom | `main_task` → `room_navigator.arms_ok` | 16. 9.: misija je pala na vratima jer je gate znao samo `DRIVE_V4`. Usporedba sada omotava kut (2π), inače kontinuirani zglobovi lažno ispadaju daleko ([[P-45_mission_integration]]) |

## Odlaganje na marker (`main_task._place_on_marker`, 16. 9. 2026.)
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| `place_clearance` | **0.20 m** (bilo 0.05) | `main_task` | korisnikovo pravilo (16. 9.): **visina ruku = visina markera + 20 cm + pola kocke**, tj. donja ploha kocke lebdi 20 cm iznad markera dok se baza primiče |
| `place_max_advance` | **0.24 m** | `main_task` | primicanje stolu ide po tome **koliko robot može**, ne koliko marker traži. Izračunato iz runa 16. 9.: marker 0.915 m ispred, rub ploče 0.22 m bliže = 0.695 m, a baza završava 0.41 m ispred `base_link` → **0.285 m** zazora; ostaje 4.5 cm rezerve |
| ostatak do markera | preuzimaju **ruke** (`_push_cube_out`), jer baza dalje ne smije | `main_task` P4 | s dockom i hvatom kakvi jesu kocka je ~28 cm iza markera: baza pokrije 24 cm, ruke ostatak. Ako ni ruke ne mogu, log **kaže koliko kratko** kocka sjeda umjesto da se pretvara da je centrirana ([[D-12_honesty_abort_over_fake]]) |
| `place_touch` | **0.002 m** | `main_task` | kocka se **spušta**, ne pritišće; isti red veličine kao `touch_depth` pri hvatu |
| `place_lower_speed` | **0.01 m/s** | `main_task.move_torso_guarded` | zadnjih 5 cm; guard prekida ako kocka napusti jastučiće |
| izvor pozicije kocke pri odlaganju | **kamere na zapešću** (`_cube_in_hands`), ne mrtvi račun od attacha | `main_task` | dok je kocka u rukama, njezini bočni markeri se gibaju s rukama — isto očitanje koje je vodilo hvat (STEP5a) |
| dokaz | `PLACE VERIFIED: … mm od centra markera` (glavna kamera + dubina, usporedba u `odom`) | `main_task` | bez tog ispisa run nije uspjeh ([[D-12_honesty_abort_over_fake]]) |

## Pripremne poze i finoća gibanja (16. 9. 2026., korisnikov run M2/M3)
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| `portal_standoff` | **0.65 m** (bilo 0.95) | `nav_zones` | korisnik: pripremne točke pred vratima su predaleko. Prilaz je sada **1.15 m** od vrata (bio 1.45) |
| `table_standoff` | **0.95 m** (bilo 1.15) | `nav_zones` | prilaz stolu **1.35 m** od centra (bio 1.55). **0.90 m ne prolazi** `check_zones`: na toj pozi opisani krug od 0.673 m dodiruje keepout, pa se robot ne bi mogao okrenuti |
| `table_dock_safety` | **0.15 m** (bilo 0.10) | `nav_zones` | korisnik: pred stolom u crvenoj sobi stao je premalo blizu. Čelo je sada **15 cm** od ploče |
| finoća vožnje | zadnji centimetri: brzina `min(zadana, max(0.02, ostatak/1.5))`, rampa najviše ⅓ pokreta, tolerancija **1 cm** (vožnja) / **0.8 cm** (strafe) | `base_drive.drive/drive_distance/strafe_distance` | korisnik: „do finoće kontrole gibanja" — kod korekcije od 2 cm najkraći nalet pri punoj brzini prijeđe 4–5 cm, pa je robot **pretjerao** pri centriranju |
| kutija u obrisu robota | tema **`/mission/carried_points`** (latched, kutovi kutije u `base_link`) | `main_task` → `footprint_publisher` | korisnik: dok nosi kutiju, u zonu robota mora ulaziti i kutija, ne samo ruke. URDF za kutiju ne zna, pa je obris inače završavao na šakama |
| kutija u RViz-u | tema `/mission/cube_marker` (`visualization_msgs/Marker`, `base_link`), prikaz „Kutija (izmjerena)" u `nav2.rviz` | `main_task._show_cube` | crta se **izmjerena** kutija (kamere), ne poza iz simulatora — inače provjera ne vrijedi ništa ([[P-40_amcl_pose_disagrees_with_lidar]]) |

## Glatko gibanje (16. 9. 2026., korisnik: „ne sviđa mi se cjepkano gibanje")
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| ruta se vozi u **skupinama** | uzastopne dionice bez gatea = **jedan** cilj (`navigate_through_poses`); **prolaz i dock uvijek sami** | `room_navigator.run` / `_drive_through` | Izmjereno: `blue:dock → red:dock` 7 dionica → **6 ciljeva (1 kontinuiran)**, `home → blue:dock` 4 → **4 (nijedan kontinuiran)**. Dobitak je dakle **malen**: preostala stajanja su upravo ona nosiva — poravnanje pred oboja vrata ([[P-39_nav2_enters_doorway_at_an_angle]]) i pred stolom prije docka ([[D-20_single_potential_field_costmap]]). Glatkoću *unutar* dionice daje `Twirling.scale` |
| `Twirling.scale` | **2.0** (bilo 20.0) | `NAV` `FollowPath` | kritika kažnjava okretanje u vožnji; na diff-driveu to je smisleno, na mecanumu je upravo ono što korisnik traži (translacija i rotacija istovremeno). Ostaje iznad nule da se robot ne vrti bez razloga |

## Brzine (16. 9. 2026., korisnik: „kada je sigurno robot se treba brzo gibati")
| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| Nav2 `max_vel_x` / `max_speed_xy` / `max_vel_theta` | **0.45** / **0.50** / **0.40** (bilo 0.30 / 0.35 / 0.30) | `NAV` `FollowPath` | vožnja po otvorenoj sobi; `velocity_smoother.max_velocity` podignut na **[0.50, 0.30, 0.60]** jer bi inače rezao |
| brzina po dionici | `fast_vel_x` **0.45** / `slow_vel_x` **0.18** (+ `fast/slow_speed_xy` 0.50 / 0.22) | `room_navigator._set_leg_speed` | **sporo kroz vrata i na dock**, brzo drugdje; postavlja se na `controller_server` prije svake dionice. `collision_monitor` i dalje dodatno usporava uz prepreku |
| `vx_samples` | **15** (bilo 25) | `NAV` `FollowPath` | run M2 je imao **624 ×** `Control loop missed its desired rate of 20 Hz`; manje uzoraka = upravljač stiže u takt |
| skaliranje ruku (MoveIt) | **0.7** brzina i ubrzanje (bilo 0.2) | `postures.move_to_posture`, `main_task` planovi | samo **slobodni** pokreti; prilaz kutiji (`grasp_approach_speed` 0.01 m/s), privlačenje (`pull_speed` 0.02 m/s) i spuštanje ostaju spori |
| vodilice | `move_torso` 2 s, `_torso_to` 3 s (bilo 4 i 8), smirivanje **1.2 s** (bilo 3.0) | `main_task` | isto: slobodni hod brzo, vođeno spuštanje na stol i dalje polako |
| kašnjenja pri pokretanju | nav2 **8 s**, RViz 10 s, task 14 s, `main_task` +5 s (bilo 12 / 22 / +12) | `mission.launch.py`, `task.launch.py` | mrtvo vrijeme prije nego se išta dogodi |
| RViz „Oblak - glava" | `Color Transformer: RGB8` (bio FlatColor) | `rviz/cube.rviz` | korisnik |
| dizanje | +0.15 m (0.40 → 0.55), najviše 0.64 m | `main_task` STEP6 | 0.65 m je graničnik (P-13) |

## Postav za ručno definiranje poza (`grasp_stage`)
Upute: `notes/00_run/00_testing/definiranje_poza_hvata.md`. Runovi S1–S4, G1.

| Parametar | Vrijednost | Gdje | Zašto / veza |
|---|---|---|---|
| spawn | dock `(0, −5.479, −90°)`, kocka 0.87 m ispred | `grasp_stage.launch.py` `DOCK_Y` | ista poza kao misija i mapiranje (korisnik, 15. 9.); do 0.62 m se primiče `grasp_stage` |
| profil ruku na spawnu | `ARM_CARRY_V2` (poza vožnje), `table_arms` false | env `PAS_SIM_CARRY_ARMS` / `PAS_SIM_TABLE_ARMS` | na docku 10.5 cm od stola; na 0.62 m bi bila u ploči (0.0 cm) |
| redoslijed | ruke u skeniranje **na docku**, pa primicanje | `grasp_stage` | skenirajuća poza u vožnji: ≥ 6.0 cm od stola, ≥ 20.5 cm od kocke (offline) |
| MoveIt stol | ploča **0.80 × 0.80 × 0.04 m** + **četiri noge** 0.05 m na ±0.35 m, rub 0.25 m ispred kocke (bila ploča 0.55 × 0.65 × 0.10 m bez nogu) | `main_task.publish_collision_scene`, `TABLE_*` | 10 cm debljine je doticalo `ARM_CARRY_V2` ispod stola (F12); bez nogu je planer vodio šaku kroz nogu (S5) |
| `carriage_height` | 0.20 m | `grasp_stage` | ista kao `table_ready` |
| `target_range` | 0.62 m | `grasp_stage` | primiče bazu samo ako je kocka dalje |
| `pre_standoff` | 0.20 m (vrh prsta → ploha) | `grasp_stage` | run 72: obje kamere na zapešću vide marker |
| `scan_tilt` | 0.0 (vodoravno) | `grasp_stage` | skeniranje je potvrđeno vodoravno; hvat ide pod `press_tilt` |
| desna ruka | zrcalo lijeve + **180° oko osi prilaza** (j7 + π) | `kinematics.mirror_right` | čisto zrcalo stavlja desnu kameru ispod osi (run S3) |
| upravljanje šakom | korak 5 mm / 2°, Jacobian (resolved rate), bez globalnog IK-a | `kinematics.ArmJog`, `joint_gui` | globalni IK zna vratiti drugu granu za pomak od milimetra (P-25) |
| odmak od graničnika | **4°** za zglobove 2, 4, 6, u nul-prostoru | `ArmJog.solve` | poza iz S4 imala j6 na −119.7° od ±120°; 20° je svaki korak od 5 mm pretvarao u ~30° pomaka zglobova |

## Ostaci M6 (deklarirani, a trenutni `run()` ih ne koristi)
`pregrasp_xy` [0.35, 0], `preplace_xy` [3.0, 0], `door_xy` [1.5, 0], `box_grasp_z` 0.25,
`table_place_z` 0.85, `grasp_half_width` 0.13, `pick_only` True (`MT` l. 152–173).
**`probe_transport`** (False) je jedini aktivno čitan param (l. 1767).
