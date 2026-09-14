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
| točka odlaganja / vizualni X | središte kocke (**6.32**, 0, 0.90), X na z=0.751 | `WORLD` model `place_target_x` | 0.22 m od bližeg ruba; kocka ima 0.07 m rezerve do ruba |

## Robot
| trenje kotača mu1 / mu2 | **0.80 / 0.20** (anizotropno, fdir1 ±45° u `base_footprint`) | `src/pas_dual_arm_bringup/urdf/base/wheel.urdf.xacro` | [[P-09_omni_drive_on_fortress]], [[R-08_omni_controller]] |
| effort limit kotača / prigušenje | **100.0 Nm** / damping 0.0, friction 0.0 | `src/pas_dual_arm_bringup/urdf/base/wheel.urdf.xacro` | DART SERVO constraint za omni pogon ([[P-09_omni_drive_on_fortress]]) |
| base controller | `mecanum_drive_controller/MecanumDriveController` | `CTRL` l. 28, 41–64 | [[R-08_omni_controller]], zamijenio diff_drive_controller |
| trenje jastučića prstiju | mu1 = mu2 = 5.0 | `URDF` l. 122–123 | squeeze |
| masa vodilice / klizača | 12 kg / 2 kg | `dual_arm_torso.urdf.xacro` l. 24, 55, 92 | procjena ([[R-06_realistic_parameters]]) |
| klizač limit | **0.05–0.65 m** (13. 9.; bilo 0.05–0.8), 1000 N, 0.5 m/s | `dual_arm_torso.urdf.xacro` l. 71, 105 | hod stvarne vodilice (odluka korisnika); [[P-13_torso_prismatic_no_lift]] |
| torzo command interface / PID | `effort`; p=1500, i=500, d=100, i_clamp=300; goal tolerancija 0.01 m, goal_time 3 s | `URDF` + `CTRL` `torso_controller` | izolirani headless pokus C2: 0.05→0.20→0.05 m, stvarna greška nakon 10 s <1 mm ([[P-13_torso_prismatic_no_lift]]) |
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
| **dock poza (`table_dock_safety`)** (novo 14. 9.) | **0.10 m**; dock = ploča 0.401 + pola duljine 0.52 + 0.10 = **1.021 m** od centra stola | `nav_zones.py` `_build_poses`, `room_navigator._undock_leg` | Najbliža poza koju Nav2 smije držati: čelo **10.0 cm** od ploče. **Na njoj se ne smije okretati** (opisani radijus 0.673 m bi zakačio stol), pa se s nje izlazi isključivo unatrag po istoj osi do `approach` poze (1.55 m). Zahtjev `/room_navigator/goto "<soba>:dock"`, izlaz `"undock"` |
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
