---
id: AGENT_GUIDE
type: pravila
updated: 2026-09-14
---
# Vodič za agente (i ljude koji preuzimaju posao)

## 1. Redoslijed čitanja (obavezno prije izmjena)
1. [[00_MAPA]]: što je cilj i gdje smo.
2. [[danas]]: što je trenutno prioritet (dogovoreno s korisnikom).
3. R-kartica zahtjeva na kojem radiš → njezine P-kartice (**cijela tablica pokušaja**) → D-kartice.
4. [[06_parametri]] za vrijednosti koje diraš.
5. Tek onda kod.

## 2. Hijerarhija istine
`[ZAD]`/`[MAIL]` ([[izvori]]) > D-kartice (važeće odluke) > P-kartice > S-kartice > komentari u
kodu. Kad nađeš kontradikciju, **ne biraj tiho**: zapiši je u karticu i pitaj korisnika.

> **16. 9. 2026. (priprema za predaju):** `TASK.md`, `HUMAN.md`, `src/README.md`, `LINKS.md` i
> `STATE.md` više nisu u repozitoriju — bili su zastarjeli i nisu bili izvor istine. Ostali su
> lokalno na disku i u git povijesti. Geometrija montaže iz `LINKS.md` prenesena je u
> [[S-01_robot_description]].

## 3. Invarijante (nikad ne kršiti)
1. **Nikad lažni uspjeh:** attach samo uz box-only kontakt na oba jastučića; nema teleporta, nema
   tihih fallbackova na izmišljene poze; provjere su nezavisne od naredbe ([[D-12_honesty_abort_over_fake]]).
2. **Nikad dvije krute veze na kutiji** (spoj + stisak druge ruke) ([[D-07_carry_on_left_wrist]]).
3. **Okret i vožnja baze nikad istovremeno** dok je skid-steer ([[P-10_skid_steer_cannot_turn]]).
4. Gibanje se tempira **sim vremenom**, a stvarni pomak se čita iz odometrije. `spin_once` nije pauza
   ([[P-21_spin_once_not_pacing_rtf]]).
5. Kratki kontaktni pokreti idu **kartezijskom linijom** s vremenskom parametrizacijom, 2π unwrapom,
   guardom prve točke i settle-waitom ([[P-24_press_path_chain]]).
6. Svaki terminal: `./scripts/run_native.sh`. Ne učitavati tuđe workspaceove ([[D-11_project_scoped_ros_env]]).
7. Svako odstupanje od [ZAD]/[MAIL] → D-kartica s `deviation: true` + red u [[odstupanja]].

## 4. Kako bilježiti rad
- **Svaki pokušaj** (i neuspjeli) = novi red u tablici „Pokušaji“ pripadne P-kartice: datum/commit,
  što, rezultat, zaključak. Ako problem ne postoji, napravi novu P-karticu iz `_templates/P_problem.md`
  i poveži je s R i S.
- **Svaki run** = red u [[runovi]].
- **Promjena parametra** = ažuriraj [[06_parametri]] + red u P-tablici.
- **Nova odluka** koja mijenja staru: nova D-kartica, a staroj postaviš `status: zamijenjena` i
  `superseded_by`. Staru se ne briše.
- Status R-kartice mijenjaj samo uz dokaz (datum + GUI/headless + commit). Ažuriraj i red u [[00_MAPA]].
- Na kraju sesije: red u [[runovi]] (što je napravljeno, gdje je stalo) + commit.

## 5. Slijepe ulice (ne ponavljati bez NOVOG dokaza)
| Slijepa ulica | Zašto | Kartica |
|---|---|---|
| Ignition `MecanumDrive` / `VelocityControl` sistem pluginovi | ne instanciraju se na ovoj instalaciji | [[P-09_omni_drive_on_fortress]] |
| PAL `omni_base.urdf.xacro` / `planar_move` | Gazebo Classic | [[P-03_pal_base_classic_control]] |
| ugađanje trenja/mase/kp da DART drži kutiju | iscrpljeno 30. 6. | [[P-15_dart_friction_no_hold]] |
| dizanje efort/gain klizača torza | probano do 1000 N | [[P-13_torso_prismatic_no_lift]] |
| brzi 360° spin skid-steera uz SLAM | drift ~30 m | [[P-11_nav2_slam_drift]] |
| mjerenje kutije iz blizine | vidi vlastite ruke | [[P-22_depth_self_view_clusters]] |
| RRT za press; korekcija zrcaljenjem greške | obilasci; tuneliranje kroz kocku | [[P-24_press_path_chain]] |
| jednostrani press preko `move_group` | gura kocku | [[P-26_one_sided_press_bulldozes]] |
| `cv_bridge` u Python čvorovima | segfault s numpy 2 | [[P-07_aruco_dict_and_cv_bridge]] |
| kompat-symlink za ABI, `~/ws_moveit2` | slomilo okoliš | [[P-31_apt_upgrade_breakage]] |
| širenje vrata radi Nav2 | krši zahtjev od 80 cm | [[P-12_door_too_narrow]] |
| popuštanje `yaw_goal_tolerance` da cilj „prođe" | 0.25 rad traži 1.085 m otvora | [[P-39_nav2_enters_doorway_at_an_angle]] |
| spuštanje `ObstacleFootprint.scale` da DWB nađe put | skriva da je footprint kriv | [[P-39_nav2_enters_doorway_at_an_angle]] |
| keepout zone samo na globalnom costmapu | upravljač ih ne vidi | [[P-39_nav2_enters_doorway_at_an_angle]] |
| **graditi novi sloj na stanju koje nije odvoženo** | dva sloja bez ijednog runa učinila su sustav gorim nego prije; 14. 9. | [[D-18_verified_baseline_first]] |
| **zamijeniti gate koji je propustio stvarni prolaz strožim izračunom** | `doorway_margin` odbija prolaz od 4.2° koji je dokazano uspio | [[P-39_nav2_enters_doorway_at_an_angle]] |
| popuštanje `min_side_clearance` da prolaz „prođe" | već je na 5 mm, a stvarna greška poze je ~7 cm; prag skriva pomaknutu pozu umjesto da je popravi | [[P-40_amcl_pose_disagrees_with_lidar]] |
| ground truth iz Gazeba bilo gdje u upravljačkom lancu | stack mora raditi i na fizičkom robotu, koji tu pozu nema; `debug_truth` je zato po defaultu isključen | [[P-40_amcl_pose_disagrees_with_lidar]] |
| zaključivanje o točnosti lokalizacije bez mjerenja | do 14. 9. se o 7 cm nagađalo; sad to mjere `loc_error` i `_report_disagreement` | [[P-40_amcl_pose_disagrees_with_lidar]] |
| `scan_filter` `half_width` ≥ 0.35 | briše dovratnike (±0.50) i noge stolova; bez povrata voxel sloj ih ne može ni označiti ni očistiti, pa stare oznake zamrznu prolaz | [[P-39_nav2_enters_doorway_at_an_angle]] |
| **vlastiti upravljač koji vozi po gradijentu polja umjesto Nav2 upravljača** | u vratima je slobodni koridor za **središte** robota samo **12 cm** (otvor 0.98 m − 2 × 0.427 m jezgre); svaka ruka centralne razlike šira od toga uzorkuje u jezgru i smjer skače. Izmjereno 16. 9.: 36–42 zaokreta > 30° po ruti, najgori 180°, uz zaglađivanje polja nepromijenjeno. Nav2 isti prolaz vozi 5/5 jer ga rješava **provjerom otiska**, ne gradijentom | [[P-45_mission_integration]] |

## 6. Rad s korisnikom
- Komunikacija na **hrvatskom**, a kod i commitovi na engleskom (Conventional Commits).
- **GUI checkpoint protokol:** prije demo runa najavi što će se vidjeti u Gazebu, a poslije pitaj
  što je korisnik vidio. Njegova vizualna opažanja su prvorazredni dijagnostički podaci.
- Odluke koje mijenjaju zahtjev ili rezultat (npr. manja kutija, re-parent spoja na torzo) donosi
  **korisnik**.

## 7. Pokretanje (kratko)
```bash
./scripts/run_native.sh colcon build --symlink-install
./scripts/run_native.sh   # T1: ros2 launch pas_dual_arm_bringup sim.launch.py
./scripts/run_native.sh   # T2: ros2 launch pas_dual_arm_bringup task.launch.py
```
Detalji: `RUNNING.md`, [[S-10_build_run_environment]].

## 8. Popis odluka
[[D-01_aruco_dict_4x4_50]] · [[D-02_own_aruco_detector]] · [[D-03_diff_drive_base_temporary]] ·
[[D-04_visual_servo_instead_nav2]] · [[D-05_contact_verified_attach]] · [[D-06_cube_squeeze_grasp]] ·
[[D-07_carry_on_left_wrist]] · [[D-08_door_widened]] · [[D-09_lift_with_arms_not_torso]] ·
[[D-10_headless_vs_gui]] · [[D-11_project_scoped_ros_env]] · [[D-12_honesty_abort_over_fake]] ·
[[D-13_three_room_world]] · [[D-14_light_box_free_size]] ·
[[D-15_door_transit_behaviour]] (zamijenjena) · [[D-16_zones_from_detected_features]] ·
[[D-17_closed_loop_door_transit]] (povučena) · [[D-18_verified_baseline_first]] ·
[[D-19_dynamic_footprint]] · [[D-20_single_potential_field_costmap]]
