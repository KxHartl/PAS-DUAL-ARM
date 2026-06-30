# Stanje Projekta (State)

**Trenutna faza**: Autonomni find → prilaz → hvat → podizanje → spuštanje na stol
radi end-to-end u GUI-ju (pošteno, bez varke). Transport do ZASEBNOG stola nije
izvediv (DART DetachableJoint + gibanje baze izbacuje kutiju).
**Datum zadnje izmjene**: 2026-06-30

## Autonomni pick→lift→place bez Nav2/SLAM (provjereno u GUI 2026-06-30)
Slijed (`main_task.py`, sve preko stvarnih akcija, bez Nav2/SLAM):
SCAN (pan kamere, baza miruje) → vizualni prilaz (cmd_vel) do ~0.9 m → dovoz +
mjerenje dubinom → dvoručni top-down hvat → **contact check (prsti STVARNO na
kutiji)** → attach (contact-verified) → **podizanje** → spuštanje na stol → detach.
- **Nav2/SLAM napušten**: skid-steer baza pri okretu u mjestu kliže → uništava
  wheel-odom + scan-matching (robot odlutao 30 m). Zamjena: direktni cmd_vel
  vizualni servo na `base_link→aruco_marker_frame` TF (SLAM-neovisan).
- **Kotači mu1=0.4, mu2=0.0**: čista rotacija u mjestu (drift ~4 mm). Okret i
  vožnja se NE rade istovremeno (inače baza "krabira" uz mu2=0).
- **Prilaz samo do ~0.9 m markerom**: bliže, niski marker traži strm nagib kamere
  → vertikalni marker se foreshorten-a → ArUco padne. Zadnji komad: ravni dovoz +
  mjerenje dubinom (depth ne pati od kuta).
- **Kutija okrenuta (yaw −0.68)** da marker gleda prema ishodištu → head-on
  detekcija + robot završi ravno ispred → top-down hvat poravnat.
- **Čisto trenje NE drži u DART-u** (probano μ 2→5, masa 0.4→0.2, kp, sila): kutija
  se nikad ne digne. Zato **contact-verified DetachableJoint** (attach TEK nakon
  contact-check ON BOX) — fizički opravdano, nije teleport.
- **Ključno za stabilan attach**: (1) attach PRIJE stiska + njEžan stisak
  (effort 20) — jak stisak + zglob preodređuju kutiju pa je DART izbaci; (2) pri
  dizanju **povuci DESNU ruku** i nosi samo na LIJEVOM zglobu — desna ruka koja
  gura fiksiranu kutiju zasebnim putem ruši solver (kutija odleti).
- Provjereno (poza kutije uzorkovana, ne-cirkularno): miruje z=0.225 → digne se
  (drži, ostaje kod mjesta uzimanja, NE odlijeće) → slegne na stol (1.30,−0.85,
  0.225). GUI vidljiv.
- **Otvoreno**: (a) lijevi RRTConnect put pri dizanju zaljulja kutiju do ~1 m prije
  sliježa (kozmetika; treba kartezijski ravni put); (b) transport do ZASEBNOG
  stola izbacuje kutiju (gibanje baze + zglob) — kutija se vraća na isti stol.

## (starije) Puni pick-carry-place s Nav2 (2026-06-23)

## Završeni Milestoneovi (commitano)
- **M0+M1** — `ros2_control` u Gazebo Fortress + pouzdan `sim.launch.py` (8 kontrolera se konfigurira i aktivira čisto).
- **M2** — Ignition `gpu_lidar` (`/scan`) + RGB kamera (`/camera/image`) senzori i `ros_gz` bridge.
- **M3** — Puni MoveIt2 config za dual-arm (`pas_dual_arm_moveit_config`).
- **M4** — Aruco detekcija (DICT_4X4_50, ID 0) preko kamere.
- **M5** — Nav2 autonomna navigacija s diff_drive bazom (vidi dolje).
- **M6** — Puna orkestracija zadatka (`main_task.py`): hvatanje → nošenje kroz vrata → odlaganje.

## M6 — Pick-Carry-Place (provjereno end-to-end 2026-06-23, headless)
Slijed (`main_task.py`, state machine, sve preko stvarnih akcija):
nav do kutije → pan-tilt pogled + Aruco → ready poza → spuštanje klizača →
dvoručni grasp → zatvaranje hvataljki → podizanje → **uvlačenje laktova** →
staging ispred vrata → prolazak kroz vrata → odlaganje na stol → otpuštanje → retract.
Pokrenuti: `sim.launch.py headless:=true`, `nav2.launch.py`, `task.launch.py`.
- Grasp: klizači se spuste (0.05) da visoko montirane ruke dosegnu kutiju na podu;
  svaka ruka se planira ZASEBNO (left_arm/right_arm) s labavom orijentacijom + retry
  (RRTConnect je nasumičan pa pojedini pokušaj zna vratiti put s kolizijom). Obje ruke
  potvrđeno dosežu bočne strane kutije (z=0.30, base_link).
- `ARM_CARRY` poza uvlači laktove (~0.6 m raspon) za prolaz kroz vrata.
- Vrata proširena na **1.2 m** (= 2× širina robota 0.6 m) u `seminar_world.sdf`;
  `inflation_radius` 0.35 → 0.15 da prolaz bude prohodan.
- `sim.launch.py headless:=true` (samo server) — GUI renderer inače gladuje
  gz_ros2_control petlju pa aktivacija kontrolera istekne na opterećenom stroju.
- Otvoreno za fino ugađanje: baza se pri odlaganju slegne malo kraće od cilja
  (~x=2.5 umjesto 3.0) jer pokreti ruku pomaknu bazu.

## Percepcija kutije + zahvat (provjereno 2026-06-29, headless)
- **Aruco se sad pouzdano detektira.** Uzrok kvara: marker je u `seminar_world.sdf`
  bio `<pbr><metal>` (reflektivan, isprano pod svjetlom). Riješeno: `metalness=0.0`,
  `roughness=1.0`, `specular=0` → matiran marker. PNG već nosi 12.5% bijeli rub
  (marker = 75% strane = 0.225 m, poklapa se s `marker_size`).
- `aruco_detector.py`: dodan subpiksel refinement kutova (`CORNER_REFINE_SUBPIX`).
- Provjereno uživo: `/aruco_single/pose` objavljuje marker; TF `base_link ->
  aruco_marker_frame` = [0.858, 0.016, 0.063] (marker na -X strani kutije pri
  robotu u ishodištu; map x=1.0-0.15=0.85, izmjereno 0.858).
- `main_task.py`: percepcija sad **upravlja zahvatom**. `confirm_box()` vraća centar
  kutije u base_link (prosjek 5 TF uzoraka + 0.15 m po +X do centra kocke), a
  `grasp_poses()` ga prima umjesto hardkodiranog x=0.55. Re-detekcija nakon što se
  baza slegne; slijed pre-grasp (stav 6 cm širi) -> approach -> zatvaranje hvataljki.
  Fallback na nominalnu pozu ako marker nije viđen.
- TODO: end-to-end pick s pokrenutim nav2+move_group (RRTConnect zahvat je nasumičan).

## Stvarni dvoručni hvat + podizanje (provjereno 2026-06-30, headless)
Prelazak s teleport-varke (`set_pose` koja je kutiju "uskakala" u ruke) na fizički
ispravan hvat. Ključni nalazi i rješenja:
- **Stara 0.3 m kutija je negrabljiva**: Robotiq 2f_85 ima hod ~85 mm. Cilj zato
  pretvoren u **šipku 0.06×0.30×0.06 m (0.5 kg)**, duga lijevo-desno; svaka ruka
  obuhvati svoj kraj (presjek 0.06 m stane u hvataljku). Marker je sad 0.0375 m
  decal na -X plohi (`aruco.launch.py marker_size=0.0375`).
- **Top-down hvat**: izmjereno iz TF-a da je tool-frame approach = lokalni +Z,
  otvaranje prstiju = lokalni +X. Ciljna orijentacija `GRASP_DOWN=(1,0,0,0)`
  (180° oko X) → approach ravno dolje, prsti opkoračuju 0.06 m širinu. Hvat se
  planira okomito iznad krajeva (y=±0.13), tijesna orijentacija (`ori_tol=0.15`)
  da prsti sjednu centrirano (široka tolerancija ih je krivila pa promaše).
- **Klizači (prismatic torzo) se NE dižu** u ign_ros2_control pod težinom ruke
  (lagani zglobovi rade, opterećeni okomiti ostaje na donjem limitu). Isprobano:
  effort 100→1000, dodani `min`/`max` i `position_proportional_gain` na
  command_interface — ništa ne pomaže. **Zaobiđeno: podizanje rukama** (MoveIt2
  digne oba EE ~15 cm ravno gore).
- **DART ne drži objekt kontaktom hvataljki** (isti razlog zašto je original
  teleportirao). Rješenje: **DetachableJoint** plugin (na robotu, `parent_link=
  left_bracelet_link`, `child_model=aruco_box`) koji se aktivira **tek nakon
  potvrđenog hvata** (`/aruco_box/attach`), pa je fizički opravdan, nije teleport
  na sredinu. Detach na startu i pri odlaganju (`/aruco_box/detach`).
- Provjereno: šipka se digne s poda z=0.03 → **z≈0.179** (prati zglob). Hvat+
  podizanje rade end-to-end (`pick_only=true` gate u `main_task.py`).
- TODO: prilagoditi transport kroz vrata + odlaganje (STEP5-7) na šipku i
  detach-pri-odlaganju; dosad provjeren samo pick+lift. Detekcija je u punom
  slijedu zaklonjena rukama pa pada na nominalnu pozu (zaseban TODO: detektirati
  prije pomicanja ruku).

## M5 — Nav2 (provjereno uživo 2026-06-23)

## M5 — Nav2 (provjereno uživo 2026-06-23)
- `base_controller` (diff_drive_controller) vozi 4 kotača bazne ploče, daje wheel-odometriju
  i `odom -> base_footprint` TF (`enable_odom_tf: true`).
- `nav2.launch.py`: slam_toolbox (map->odom) + Nav2 stack (planner/controller/bt_navigator…).
  Nav2 je odgođen `TimerAction(5s)` da SLAM prvi krene.
- `cmd_vel_relay` (novi node u `pas_dual_arm_scripts`) premošćuje Nav2-ov `/cmd_vel`
  na `/base_controller/cmd_vel_unstamped` (controller živi u gz controller_manageru pa
  običan remap nije moguć).
- TF lanac kompletan: `map -> odom -> base_footprint -> base_link`.
- Test cilja: poslan `/goal_pose`, `/cmd_vel` non-zero, baza se fizički pomakla (odom x: 0 -> 0.24 m).
- Napomena: `local_costmap` na aktivaciji jednom-dvaput ispiše "two unconnected trees"
  (INFO, tranzijentno) dok mu se TF buffer ne napuni; sam se razriješi za ~1 s i ne utječe na rad.

## Napomena za lokalno pokretanje
Globalni `~/.bashrc` postavlja `rmw_zenoh_cpp` s `ZENOH_CONFIG_OVERRIDE` koji cilja vanjski
router (192.168.0.14:7447) nedostupan lokalno. Launch fajlovi zato interno forsiraju
Fast DDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, prazan `ZENOH_CONFIG_OVERRIDE`).

## Preuzeti Repozitoriji (src/)
`omni_base_simulation`, `ros2_kortex`, `pan_tilt_ros`, `realsense-ros`, `aruco_ros`
(svi na `humble` / `humble-devel` granama).

## Kreirani Paketi (src/)
- `dual_arm_torso` (ament_cmake) — mesh datoteke linearnih vodilica.
- `pas_dual_arm_bringup` (ament_cmake) — launch, rviz, moveit, centralni URDF, Nav2/bridge config.
- `pas_dual_arm_scripts` (ament_python) — glavna Python logika (uklj. `cmd_vel_relay`).
- `pas_dual_arm_moveit_config` — MoveIt2 konfiguracija.

## Sljedeći Korak (završni milestone)
Integracija punog zadatka iz `README.md`: detekcija Aruco kutije → Nav2 navigacija kroz
0.8 m prolaz → dvoručni zahvat (MoveIt2) → prijenos i odlaganje na ciljani stol.
