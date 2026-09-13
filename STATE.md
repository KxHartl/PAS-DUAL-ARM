# Stanje Projekta (State)

> **13. 9. 2026.:** zahtjevi, problemi (s tablicama pokušaja), odluke i parametri sada žive u
> Obsidian bilješkama → **`notes/00_MAPA.md`** (ulaz) i `notes/AGENT_GUIDE.md` (pravila). Plan
> zadnjeg dana: `notes/07_predaja/danas.md`. Sadržaj ispod je povijesni zapis sesija; kod
> kontradikcije vrijede bilješke.

**Trenutna faza**: KOCKA PO ZADATKU (0.3×0.3×0.3 m, 1 kg) — autonomni
find → prilaz → centriranje → DVORUČNI SQUEEZE hvat (kontaktom verificiran)
→ podizanje radi (3 uspješna end-to-end ciklusa u GUI-ju). Spuštanje još
zna ispustiti kocku par cm previsoko (prevrne se) — vidi TODO. Transport do
ZASEBNOG stola i dalje otvoren (Faza 1/3 plana).
**Datum zadnje izmjene**: 2026-07-16

## Dvoručni SQUEEZE hvat kocke po zadatku (2026-07-15/16, GUI provjereno)
Zadatak traži kocku 0.3 m / 1 kg; hvataljke 2f_85 (85 mm) je ne mogu obuhvatiti,
pa OBJE ruke ZATVORENIM hvataljkama (vrhovi prstiju = jastučići s gz Contact
senzorima) istovremeno pritisnu suprotne bočne strane; attach (DetachableJoint,
lijevi zglob) TEK nakon dokazanog obostranog kontakta s kutijom. Ključni dizajn:
- **Slijed**: scan → vizualni prilaz (0.95 m) → mjerenje dubinom S TE UDALJENOSTI
  (bliže, kamera vidi vlastito tijelo u prozoru → junk klasteri) → cross-check
  vs marker → dovoz (odometrijski korigiran; `drive()` tempira SIM vremenom,
  RTF<1 inače prepolovi put) → **centrirajući okret** na y≈-0.075 (asimetrično:
  sweet-spot zone lijeve i desne ruke se NE poklapaju — desna savršena s kockom
  na y=-0.13, lijeva na y=-0.02; sredina služi objema) → planning scena (pod,
  stol, kocka — MoveIt inače zamahuje rukom KROZ stol, reakcija odgurne bazu!)
  → IK/pre-squeeze → SIMULTANI press (direktno na oba JTC kontrolera; jednostrani
  press buldožira kocku 0.3 m po stolu) → gate → attach → desna popusti prva
  (nikad dva kruta držanja) → lijeva digne 15 cm → spusti → detach → odmakni.
- **Press = ravna kartezijska linija** (`compute_cartesian_path` +
  `execute_trajectory`): RRTConnect za 10 cm zna vratiti metarske obilaske koji
  ruše ruku kroz kocku. Nužno: (1) VREMENSKA parametrizacija (servis vraća
  neparametriranu putanju → kontroler inače "skoči" bilo kamo); (2) **2π unwrap**
  kontinuiranih zglobova na granu najbližu stvarnom stanju (IK vraća [-π,π],
  zglob akumulira okretaje → inače puni krug "odmotavanja" usred hvata — TO su
  bile "čudne rotacije"); (3) guard prve točke (putanja iz zastarjelog stanja →
  JTC lansira ruku metar dalje); (4) settle-wait prije računanja (JTC javi
  "gotovo" na ISTEK VREMENA putanje, a sim ruka dopuzava još sekundama).
- **Fallback lanac pritiska**: čista linija (fraction≥0.9) → re-roll pre-squeeze
  konfiguracije (лutrija IK grana; do 4×, uz 180° roll varijantu) → RRT na pozu
  ~2 cm ispred lica (uvijek plannabilna) → per-arm linearni RE-PRESS koji
  konvergira pouzdano (mjereno Δ=0.001–0.005 m).
- **Gate (pošten)**: placement (obje ruke REACHED ili pad-kontakt s kutijom +
  `fingertips_on_box` s dimenzijama kocke, nezavisno od naredbi) **I** fizički
  kontakt na OBJE strane, pri čemu se broji SAMO kontakt s `aruco_box`
  (imena kolizija iz poruke; dodir stola/sebe NIJE dokaz). Stall izbačen
  (pritisak na ravnu plohu ga nema). Press meta 3 cm U kocki — kocka (ne
  poza) zaustavlja jastučić pa je kontakt garantiran za manjak ≤3.5 cm.
- **Contact senzori**: Fortress IGNORIRA `<topic>` tag — objavljuju na
  `/world/<w>/model/<m>/link/<l>/sensor/<s>/contact`; bridge mapira te duge
  putanje na kratke (`bridge.yaml`). Poruka nosi imena kolizija (za box-only).
- Provjereno: 3 puna ciklusa hvat+lift (kocka visi na lijevom zglobu, ne
  odlijeće); SVIH ~15 neuspješnih runova završilo POŠTENIM abortom (nikad
  fake attach) — gate-ovi rade po dizajnu.

### Popravci infrastrukture (2026-07-15/16)
- **apt upgrade slomio okoliš**: (1) `pal_urdf_utils` sad traži `gazebo_version`
  xacro svojstvo → definirano u `robot.urdf.xacro`; (2) source-build MoveIt
  (`~/ws_moveit2`) linkan na `libgeometric_shapes.so.2.3.2`, apt donio 2.3.4 →
  privremeno je korišten kompatibilnosni symlink. **Razriješeno 10. 9. 2026.** Projekt je čisto
  rebuildan samo na `/opt/ros/humble`, a zastarjeli `LD_LIBRARY_PATH` workaround uklonjen je iz
  radnih uputa.
- **spin_once NIJE pacing**: vraća se čim obradi BILO KOJI callback (/clock je
  ~1 kHz) — sve petlje "čekaj N s" pretvorene u monotone/sim-time deadline
  (mjerenje oblaka, `drive()`); posljedica starog buga: "vožnje" od 4 s su se
  izvršavale u milisekundama (seed x=0.55 fantomi iz starih runova).
- **Oblak kamere u tjelesnoj konvenciji**: gz rgbd stampa optički frame, podaci
  su x-naprijed → `measure_box()` transformira preko `camera_link`.
- **Odometrija**: `/base_controller/odom` za stvarno prevaljeni dovoz i stvarni
  kut centrirajućeg okreta (percept se rotira/translatira za IZMJERENO).

## Sesija 2026-07-16b: place-fix + transport proba IMPLEMENTIRANI, NEPROVJERENI
Kod je napisan, build čist, ali NIJEDAN run s ovim izmjenama još nije prošao
hvat (runovi 29-30 pali na PREstrogoj geometriji prije nego su nove stvari
došle na red; run 31 prekinut krajem sesije). Izmjene u `main_task.py`:
1. **Spuštanje s kontaktom** (STEP8): meta z = visina uzimanja − 2 cm — STOL
   (ne poza) zaustavlja kocku pa je pad pri detachu ~0 (isti interferencijski
   trik kao press); do 3 pokušaja spuštanja s provjerom (tol 4 cm).
2. **Kocka SE MIČE iz planning scene odmah nakon attacha** (CollisionObject
   REMOVE) — postala je dio "ruke"; njen zamrznuti kolizijski objekt je
   blokirao SVE kasnije RRT fallbackove (release/lower/retreat frac 0.0x).
3. **`probe_transport` ROS param** (Faza 1 eksperiment B): nakon attach+lift
   preskoči place, OTVORI oba jastučića (ukloni kontakt prsti↔kutija koji se
   tuče s krutim zglobom = glavna hipoteza eksplozije), vozi 0.4 m ravno +
   okret ~60° u mjestu; kocka visi samo na zglobu. Pokretanje:
   `ros2 launch ... task.launch.py` uz param `probe_transport:=true`
   (proslijediti kroz launch ili `ros2 param set` prije runa — provjeriti!
   task.launch.py trenutno NE prosljeđuje parametre main_tasku).
4. **Gate pošteno popušten**: primarni dokaz = SVJEŽ box-only kontakt na OBA
   jastučića (imena kolizija iz poruke — stari fake-hvat to ne može
   proizvesti); placement = stroga geometrija ILI oba EE unutar 12 cm od
   press ciljeva (sanity protiv okrznuća iz daljine).

### Naučeno u sesiji 2026-07-16b
- Runovi 29/30: OBA imala stvaran obostrani kontakt s kutijom, a pali su na
  fingertips-geometriji (ruke 6-13 cm od idealne poze zbog IK/tracking
  lutrije) → zato popuštanje gate-a (točka 4). Očekivani skok uspješnosti
  s ~35% (3/9 runova s kockom) na blizu 100% — ZA POTVRDITI sutra.
- Promašeni press zna ZBACITI kocku sa stola pri retreatu (run 29: kocka
  završila na podu metar dalje) → nakon takvog aborta treba restart sima.
- 5c nudge: izvrši liniju (frac 0.86) ali re-check trči prerano/prestrogo —
  ako gate-popuštanje ne riješi, dodati settle+recheck petlju u 5c.

### TODO (sutra)
1. **Run 31+**: potvrditi hvat s popuštenim gateom + RAVNO odlaganje
   (kocka na stolu, ne prevrnuta) — checkpoint s korisnikom.
2. **Transport proba** (`probe_transport:=true`): prvo provjeriti kako
   proslijediti param kroz task.launch.py (možda treba dodati u launch);
   pratiti pozu kocke izvana (`ign topic -e -t .../dynamic_pose/info`).
   Ishod A (preživi) → integrirati vožnju do target_table (Faza 3).
   Ishod B (odleti) → matrica C-E iz plana (masa/fizika/re-parent torzo).
3. **Negativni test**: kocka izvan dohvata → formalno potvrditi abort.
4. **Faza 3**: Nav2 retest ili waypoint vožnja; odlaganje na `target_table`
   (ploha 0.775 m — provjeriti doseg!).
5. **Faza 4**: vrata 2.0 m → 0.8 m po zadatku.

## (starije) Stanje prije kocke — pločica 0.06×0.30×0.25
**Napomena**: svijet sad ima KOCKU po zadatku; sekcije ispod opisuju stariju
pločicu i vrijede povijesno.

## Pošten, kontaktom-verificiran hvat — redizajn (implementirano 2026-06-30, JOŠ NEPROVJERENO u sim)
Razlog: hvataljke su dolazile ~50 cm ISPRED kutije, zatvarale se u prazno i kutija
se "fake" zavarivala iz daljine. Plan: `~/.claude/plans/ne-radi-hvatanje-kutije-bright-boot.md`.

### NAPRAVLJENO (kod izmijenjen, build/flake8/xacro čisti)
Datoteke: `pas_dual_arm_scripts/.../main_task.py`, `pas_dual_arm_bringup/urdf/robot.urdf.xacro`,
`.../worlds/seminar_world.sdf`, `.../config/bridge.yaml`, `pas_dual_arm_scripts/package.xml`.
- **Cirkularna provjera uklonjena** — stara `verify_contact(grip,half)` mjerila je
  vrhove protiv ISTOG naređenog centra. Zamijenjeno neovisnim signalima.
- **Tihi fallback na x=0.55 uklonjen** — ako `measure_box()` padne → `_fail`.
- **`verify_reached()`** — čita STVARNU EE pozu iz TF-a vs naredbena (tol 5 cm);
  ruka nije stigla → abort. Direktno hvata "50 cm ispred".
- **Yaw-svjesno**: `measure_box()` PCA → duga os `u`; `grasp_poses(center,u,…)`
  poravna poze + orijentaciju prstiju s pravom osi/debljinom.
- **`measure_tip_standoff()`** — wrist→vrh iz TF-a (~0.145 m); vrhovi (ne zglob)
  slijeću na šipku (zamijenjen magični `z_offset=0.12`).
- **Fuzija osjeta kontakta**: (1) gz **Contact senzori** na 4 vrha (`tip_contact`
  makro + `Contact` plugin u svijetu + bridge `ros_gz_interfaces/Contacts`);
  (2) **stall** hvataljke (knuckle position); (3) wrist **effort** — samo LOGIRAN.
  Gejt za attach: **placement (reached + `fingertips_on_box` na PRED-grasp
  percepciji, otporno na okluziju) I fizički signal (senzor ILI stall)** — inače
  otpusti + abort. Ako contact senzor šuti → graceful fallback na depth+stall.

### ZA TESTIRATI (sljedeća sesija — sim još NIJE pokrenut)
1. `ros2 launch pas_dual_arm_bringup sim.launch.py` + `task.launch.py`.
2. **Contact topici objavljuju?** `ros2 topic echo /contact/left_left_tip --once`.
   Ako šute → krivo ime kolizije (`<link>_collision`); naći pravo ime u spawnanom
   SDF-u i ispraviti `tip_contact` makro (hvat dotad radi preko depth+stall).
3. **Pozitivni**: log mora pokazati izmjereni centar+yaw (ne abort), `EE … REACHED`,
   `Grasp evidence: placement=True physical=…`, pa `Box ATTACHED`; kutija se digne.
4. **Negativni**: makni kutiju izvan dohvata → mora ABORTIRATI ("did not reach" /
   "no real contact"), NE fake-attach.

### ZA NAPRAVITI / OTVORENO
- Fino podesiti pragove: `verify_reached` tol (5 cm), stall prag (`target-0.06`),
  `fingertips_on_box` tolerancije, contact `max_age`.
- Ako contact senzor radi: razmotriti zatvaranje hvataljke DO kontakta (ranije
  zaustavljanje) umjesto fiksne mete 0.7.
- Provjeriti ne ruši li attach-poslije-stiska (gentle effort 20) DART solver pri
  dizanju (prije je bio attach-prije-stiska); ako izbaci kutiju → vratiti redoslijed.
- `_log_grasp_geometry` još koristi hardkodirani 0.15/Y (samo telemetrija, ne gejt).
- Transport do ZASEBNOG stola i dalje otvoren (DART + gibanje baze izbacuje kutiju).

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
Globalni shell je neutralan. Za build i pokretanje koristi se `scripts/run_native.sh`, koji učitava
samo `/opt/ros/humble` i PAS-DUAL-ARM lokalni overlay te eksplicitno postavlja Fast DDS, ROS domenu
5 i lokalno otkrivanje čvorova. Mrežni ili hardverski profil mora biti zaseban i eksplicitno odabran.

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
