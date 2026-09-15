# Stanje Projekta (State)

> **Operativne upute, 13. 9. 2026.:** `notes/00_run/README.md` vodi na
> aktualno pokretanje, testiranje i izmjene. Omnidirekcijski pogon (`mecanum_drive_controller`)
> i SLAM mapiranje triju soba su **uspješno riješeni**. Validna karta je spremljena
> u `maps/seminar_map.*` (102.3 m² slobodnog prostora, raspon 11.9 × 11.8 m, vrata 1.0 m čista).
> BATRACS-ov obrazac projektnog okoliša prisutan je kroz `scripts/run_native.sh`;
> `scripts/verify_environment.sh` prolazi 17/17.

> **Status SLAM mapiranja (13. 9. 2026.):** Uspješno završeno ručno teleop mapiranje (run 43)
> uz `ARM_CARRY_V2`, omnidirekcijski pogon (mecanum, `mu1=0.80`, `mu2=0.20`, 100 Nm) i
> `scan_filter`. Karta je provjerena pomoću `scripts/check_map.py` i vizualnim pregledom.
> Vrata i noge stolova su jasno razlučeni. Sljedeći korak: testiranje Nav2 lokalizacije (AMCL)
> i navigacije do regije u plavoj sobi prema planu `notes/07_predaja/danas.md`.

> **13. 9. 2026.:** zahtjevi, problemi (s tablicama pokušaja), odluke i parametri sada žive u
> Obsidian bilješkama → **`notes/00_MAPA.md`** (ulaz) i `notes/AGENT_GUIDE.md` (pravila). Plan
> zadnjeg dana: `notes/07_predaja/danas.md`. Sadržaj ispod je povijesni zapis sesija; kod
> kontradikcije vrijede bilješke.

**Trenutna faza**: HVAT PREKO V4 POZA — headless prolazi do kraja (run V2: kocka podignuta 15 cm i odnesena 0.40 m, provjereno u Gazebu); čeka korisnikovu GUI potvrdu. Navigacija s `DRIVE_V4` nije ponovno provjerena.
**Datum zadnje izmjene**: 2026-09-15

## Dodatak 2026-09-16: hvat preko korisnikovih V4 poza

Korisnik je u `pose_studio` (jedna naredba: Gazebo, RViz, prozor zglobova, prozor spremljenih
poza; ništa se ne miče samo) snimio `DRIVE_V4`, `DETECTION_V4`, `GRASP_V4` (vodilice 200/400/400 mm,
hvataljke 0.791). Slijed u `main_task`: DRIVE_V4 → lociranje na docku → vodilice 400 → DETECTION_V4 →
primicanje na 0.63 m → kamere na zapešću → ciljevi = središta markera → GRASP_V4 → vrhovi u ciljeve
(2 mm dodira, bez stiska — stisak izbacuje kocku) → attach → +0.15 m → unatrag 0.40 m i stop.
Run V2 (headless): `TASK COMPLETE`, `tool_tip` 1.1/2.6 mm od cilja, Gazebo: kocka +15.2 cm.

- `ARM_DRIVE = 'DRIVE_V4'` (spawn, `sim.launch`, navigacija). `GRASP_V3` napušten (samosudar u MoveIt-u).
- URDF `{side}_tool_tip`: 0.127 m od `end_effector_link` (koji je u bazi hvataljke) = središte
  zatvorenih jastučića.
- `capture_posture` sprema i vodilice i hvataljke; `joint_gui` ih učitava.
- Poboljšanja nakon korisnikovog GUI runa (U2, „mislim da je dobro"): vodilice i ruke se dižu **istodobno**
  (ruke kroz točku 12 cm iznad DETECTION, offline ≥ 10 cm od stola), DETECTION 5 cm šira, odmak 0.50 m pa
  vodilice na 200 mm s kockom u rukama (run V3 headless: kocka u Gazebu na z 0.707), RViz oblak glave RGB8.
- Nakon dizanja kocka se **privuče 15 cm** prema robotu (obje šake zajedno, laktovi 20° od torza,
  MoveIt provjera prije pokreta), odmak 0.50 m, vodilice na **100 mm** (run V5: Gazebo z 0.605, širina 0.821 m).
- [[P-44_grasp_from_reference_pose]]. Ništa nije commitano.

## Dodatak 2026-09-15 (večer): širina hvata i postav za ručne poze

**Pet GUI prigovora korisnika** (spawn raširenih ruku, okretanje robota, nesimetrične ruke, desna
ruka se odmakne nakon hvata, hvat širi od vrata): spawn u `ARM_CARRY_V2`, centriranje **strafeom**,
pred-hvat obje ruke zajedno, desna nakon attacha popušta 5 mm. Širina: [[P-43_grasp_pose_wider_than_door]].

**Vodoravni hvat ne može kroz vrata, nikako.** Offline preko cijelog nul-prostora
(`scripts/grasp_width.py`): sferno zapešće leži na osi prilaza, pa je svako od 1620 IK rješenja
**1.003 m** široko. Nagib prilaza 50° prema dolje → **0.823 m** (uže od `ARM_CARRY_V2`, 0.840 m).
`press_tilt` u `main_task`. Misija s tim je stala prije pokreta (F12): MoveIt vidi startnu
`ARM_CARRY_V2` u sudaru sa stolom (model stola 10 cm, stvarni 4 cm). Nijedna visina vodilica ne
drži `ARM_CARRY_V2` iznad stola **i** uz kocku; korisnik je odabrao pred-pozu prije primicanja,
a zatim preusmjerio na **ručno definiranje poza**. Ta izmjena misije **nije napravljena**.

**Postav za ručne poze** — `notes/00_run/00_testing/definiranje_poza_hvata.md`:
- `grasp_stage.launch.py`: robot 0.62 m od kocke, `ARM_HOME` na spawnu, `table_ready`, MoveIt + ArUco,
  pa `grasp_stage` — kocka iz glavne kamere, otvorene šake 0.20 m od ploha, **ruke simetrične**
  (desna = zrcalo lijeve + 180° oko osi prilaza), **obje kamere na zapešću vide marker** (S4).
- `scripts/joint_gui.py --sim`: šalje vodilice → hvataljke → ruke, MoveIt ili direktno; „Provjeri"
  (jastučići, širina, šake); „Zrcali" točno (0.000 mm). Logika isprobana bez prozora (G1);
  **sam Tk prozor nije isproban**.
- Popravljeno usput: `table_arms:=true` je bacao `ValueError` otkad je `carry_arms` zadano `true`;
  `joint_gui` je kontinuirane zglobove rezao na ±180° (294° bi okrenuo ruku za 114°); zrcaljenje je
  kopiralo iste kutove; `measure_width()` je tiho preskakao linkove bez TF-a.
- Moja greška, nađena i ispravljena: pri uvođenju nagiba okrenuta je os x alata (šaka 180°), pa je
  kamera na zapešću gledala pokraj markera (S1). F12 je imao isto.

**Nastavak iste večeri (korisnik: upravljanje šakom po osima, zamrznuta šaka, start u poziciji vožnje):**
- `grasp_stage.launch.py` sada kreće s **dock poze u `ARM_CARRY_V2`**, ruke idu u skeniranje na docku,
  pa primicanje 0.25 m — S6 prolazi, centar kocke iz kamera na zapešću 1–3 mm od odometrije.
- MoveIt scena ima **stvarni stol** (ploča 0.80 × 0.80 × 0.04 m + noge). Stari model bez nogu pustio je
  planer da šaku vodi kroz nogu, a kontroler je javio uspjeh (S5). Utječe i na misiju (`main_task`).
- `joint_gui` panel „Šaka po osima": ±X/±Y/±Z, zakretanja, lijeva/desna/obje simetrično, osi baze ili
  alata, i **lakat uz zamrznutu šaku** (jedini slobodni pomak 7-DOF ruke). Koraci po Jacobianu
  (`kinematics.ArmJog`), zglobovi 2/4/6 4° od graničnika kroz nul-prostor. Isprobano bez prozora (G2).
- `pas_dual_arm_scripts/kinematics.py`: FK cijelog robota, zrcaljenje ruku, upravljanje šakom.

**Otvoreno:** korisnik definira poze u GUI-ju (sam Tk prozor još nije isproban); nakon toga misija
(pred-poza prije primicanja, [[P-43_grasp_pose_wider_than_door]]) i negativan test. Ništa nije commitano.

## Dodatak 2026-09-15 (kasno): povratak na jednostavan put

**Odluka korisnika:** MoveIt i pozicijske naredbe **posvuda**, nijedan regulator, hvat po
[[D-05_contact_verified_attach]]. Dizanje trenjem **nije zahtjev zadatka** — `R-17` doslovno traži
„obostrani kontakt dokazan senzorima **prije attacha**". Rad na sili odložen, ostaje u repozitoriju.

**`P-13` riješen, otvoren od 30. 6.** Vodilice se nisu dizale jer im je `initial_value` bio **0.05**,
jednak donjem graničniku, a Ignition poziciju izvodi kao brzinu zgloba i na graničniku je poništava
**u oba smjera**. Robot je startao zaglavljen. Jedina izmjena: `initial_value` → **0.06**. Devet
ranijih pokušaja tražilo je krivca u gravitaciji, efortu i gainu.

**Zbog toga otpada i `effort` profil** za misiju: vodilice drže 0.2000 m i dižu 0.20 → 0.35 → 0.20 m
s greškom 0.0000 mm na običnom `position` sučelju. [[D-09_lift_with_arms_not_torso]] zamijenjena,
[[R-03_linear_rails_torso]] ispunjen.

**Tri izmjene tražene 13. 9., sve u `main_task`:** vodilice postavljaju visinu hvata iz izmjerene
kocke (1:1, jer je vodilica vertikalna); širina hvata iz **izmjerene** duljine umjesto pretpostavljenih
0.15 m; pritisak 30 → 10 mm, i to kao ROS parametar `squeeze_interference`. Dizanje i spuštanje
prebačeni s ruke na vodilice.

**Stanje:** run F9 prošao **headless** do `TASK COMPLETE`. **Nije GUI i nije dokaz.** Prije nego se
`R-17` uopće dira treba korisnikova GUI potvrda **i** negativan test (kocka izvan dohvata → pošten
abort). Upute: `notes/00_run/00_testing/hvat_kocke.md`, odjeljak „Test dizanja kutije".

## Dodatak 2026-09-15: pokus hvata silom (Codexov plan, nastavak)

Radi se u **izoliranoj domeni 76** (`scripts/run_force_isolated.sh`), zasebno od GUI pokusa u domeni 75.
Upute za pokretanje: `notes/00_run/00_testing/hvat_kocke.md`, odjeljak „Pokus hvata silom“.
Ništa nije commitano.

**Što sada radi (izmjereno, headless):**
- Aktuatorski profil `force_grasp:=true`: nema titranja ni na jednom od 14 zglobova, svi zglobovi točno na naredbi, vodilice **0.2000 / 0.2000 m (greška 0.0000 mm)** u četiri uzastopna diza.
- Puni lanac hvata prolazi do kraja: `characterization_complete`. Kocka izmjerena 0.300 m, pred-hvat pogođen u 8 mm, standoff 20.3–20.9 mm, stisak 21.4 s, procjena **lijevo 3.58 N / desno 3.85 N**.
- Procjena sile u slobodnom gibanju 0.02–0.26 N (cilj 5 N), uvjetovanost poze 7.12, rezidual ~0.001 Nm.

**Ključni nalazi (detalji u karticama):**
- [[P-41]] Pojačanja se **ne smiju** računati iz KDL matrice mase: za zakretne zglobove DART se ponaša kao da je ruka ~30× lakša. Inercija je izmjerena preko granice prigušenja (`d < 2I/dt`).
- [[P-42]] Fortress kontaktni senzor daje **samo točke dodira, bez sila** (49 035 zapisa, 0 sa silom). Zato su dodani FT senzori na zapešća — isključivo kao dokaz za ocjenu.
- `/clock` i podatkovne teme nemaju zajamčen redoslijed: 3.6 % poruka stigne s oznakom do 19 ms „u budućnosti“. Pragovi svježine sada to podnose na obje strane.
- `grasp_cube.py` se **mora** pokretati s `--ros-args -p use_sim_time:=true`.
- Tvrdnja „samo 2 od 4 jastučića dodiruju“ bila je pogrešna: sva četiri su na plohi unutar 0.3 mm.

**Gdje je stalo:** FT senzori rade i snimaju se, kvalifikacija je prepisana na dvije reference
(FT kao apsolutna, slaganje ruku kao dosljednost) i ima 12 testova, **ali kvalifikacijski zapis
još nije dobiven** — tri pokusa s FT-om pala su prije stiska.

**Otvoreno, i to je sada glavna prepreka: lijeva ruka.** Izmjereno preko svih runova —
pred-hvat median 15.0 mm naspram 10.0 mm desne, **4 od 17** preko praga 20 mm naspram **0 od 13**;
standoff 20.0–56.9 mm naspram 20.2–**20.9** mm. Desna drži standoff u rasponu od 0.7 mm kroz devet
runova, dakle cjevovod može biti tijesan. Lijeva povremeno grubo promaši, titra 53.7 N naspram
21.8 N, preskakala je IK granu i nije bila smirena pri nuliranju. Hipoteza: `left_joint_3` i
`left_joint_5` stoje oko ±3.14 rad, tj. na točki 2π omota. Vidi [[P-25_asymmetric_arm_reach]].
**Ne popuštati prag pred-hvata** dok se ovo ne razjasni — sakrilo bi kvar.

**Riješeno 15. 9. poslijepodne: vid više ne ide kroz `/tf`.** `robot_state_publisher` republicira
`/tf` na 1000 Hz (takt kontrolera), a Python konzument to ne stigne isprazniti dok petlja drži GIL,
pa je očitanje markera zaostajalo do koje god granice se postavi (median 0.32 s uz prag 0.4;
0.73 s uz prag 0.8), dok je u mirnom čvoru isti TF star 0.060 s. `spin_thread=True` nije pomogao.
`force_grasp` sada čita `/wrist_*/marker_pose` izravno, a kamera→alat je fiksna pa se čita jednom.
Smanjenje takta kontrolera je odbačeno: srušilo bi izmjerena pojačanja (`d < 2I/dt`).

**Riješeno 15. 9. poslijepodne: nuliranje čeka da se ruke smire.** Tražilo se prerano — lijeva je
imala rasap momenta 0.069–0.258 Nm uz prag 0.05, a smiri se na 0.004 Nm tek ~8 s nakon standoffa.
Sada ponavlja pokušaj dok procjenitelj sam ne potvrdi mir.

**Riješeno 15. 9. poslijepodne: korak stiska više ne zove IK.** Za pomak od 0.5 mm koristi se
diferencijalni korak preko Jacobiana (`force_model.resolved_rate`), koji je lokalan po
konstrukciji pa promjena grane nije moguća. Prigušenje 0.005 odabrano mjerenjem (gubi 5.6 %
traženog pomaka u najgorem slučaju, naspram 54 % pri 0.05). Provjera sudara ostaje.

**Bilo je (povijesno): izbor IK grane na lijevoj ruci.** Izmjereno 15. 9.: MoveIt-ov IK je
točan (pogreška poze **0.00 mm**, obje ruke), ali za pomak od **0.5 mm** lijeva ruka vrati rješenje
**3.04 rad** od trenutne konfiguracije, dok desna vrati 0.0036 rad. Zbog toga stisak pukne na
`left IK branch/tracking jump` — zaštita ispravno odbija izvesti zamah od 3 rad za pola milimetra.
Sjeme iz trenutnog stanja to ne spriječi. Vidi [[P-25_asymmetric_arm_reach]].

Drugi, manji: `verify_reached` za pred-hvat pao je s Δ=0.021 m uz prag 0.020 — promašaj za 1 mm,
dok zglobovi prate na < 1 mrad. Uzrok nije IK (točan je); treba provjeriti pomiče li se `base_link`
između planiranja i dolaska.

Dizanje ostaje zaključano dok kvalifikacija ne prođe i dok ne postoji nezavisan dokaz trenja.


## Dodatak 2026-09-14 (noć): Run 70 — korisnik potvrdio u GUI-ju

**„super radi“** — potencijalno polje, prolazi kroz oboja vrata u oba smjera, vožnja oko stolova
i dock/undock potvrđeni uživo. Time su zatvorena tri dijela koja su cijeli dan bila otvorena:

| | stanje |
|---|---|
| Omnidirekcijski pogon (`mecanum_drive_controller`) | ✅ potvrđen |
| SLAM + lokalizacija | ✅ karta run 60 (0.02 m, novi lidar), vršna greška AMCL-a **3.0 / 2.9 cm, 0.3°** izmjerena ground truthom |
| Gibanje / navigacija | ✅ jedno potencijalno polje, 5/5 prolaza, obilazak stolova, dock 10 cm od ploče |

## Dodatak 2026-09-14 (noć): Jedinstveno potencijalno polje, brazde i dock/undock geometrija (D-20, Run 69)

- **Jedinstvena ploha cijene i gašenje konfliktnih slojeva**:
  - Globalni `inflation_layer` ugašen (`enabled: false`) u `nav2_params.yaml` — uklonjena "djetelina" oko nogu stola i dvostruko brojanje prepreka.
  - `Zones.field(grow=...)` u `nav_zones.py` generira jedinstvenu plohu cijene iz karte (`distance_transform_edt`): lethal jezgra do upisanog radijusa (0.427 m), glatka rampa širine **0.40 m** s vrhom **`field_peak = 50`** (cost 127 u Costmap2D, NavFn 152 naspram 50 za slobodan pod), koja drži putanju na **0.835 m** od stvarnih prepreka. Udaljenost se mjeri od **oboda ploče stola (0.80 m)**, ne od nogu koje lidar vidi.
  - Objavljuju se dvije maske: `/keepout_filter_mask_planner` (napuhana za planera) i `/keepout_filter_mask` (za lokalni DWB).
- **Brazde nulte cijene (`_furrows`)**:
  - Kroz prolaze vrata i duž prilaza stolu izrezane su trake nulte cijene koje sprječavaju zasićenje NavFn potencijala na 253 (problem runa 65) i osiguravaju čist gradijentni spust.
- **Dock / undock geometrija i navigator**:
  - Uvedena `dock` poza (`half + HALF_LENGTH + 0.10 m = 1.021 m` od centra stola, 10 cm od prednjeg brida ploče).
  - Na `dock` pozi zabranjena rotacija u mjestu (radijus 0.673 m zakačio bi stol).
  - `room_navigator.py` dovršen: implementirani `_docked_table()`, `_docked_room()`, `_undock_leg()`, `_table_by_pose()` i podrška za `:dock` / `undock` ciljeve. Robot pri odlasku sa stola obavezno vozi unatrag do `approach` poze (1.55 m) prije skretanja.
- **Verifikacija offline gate-ova (sve zeleno)**:
  - `scripts/check_map_geometry.py` → **PASS** (vrata 0.980 m, os 0.0 cm, stepenica ≤0.9 mm).
  - `scripts/check_zones.py` → **PASS** (brazde nulte cijene, prohodnost soba, 11 poza).
  - `scripts/check_costmap_path.py` → **PASS** (putanja drži razmak 0.715 m, točno unutar ciljanog raspona 0.60–0.90 m).
  - `python3 -m py_compile` i `colcon build` prolaze čisto bez grešaka.

## Dodatak 2026-09-14 (večer): Nova karta na 0.02 m i AMCL mjerni model (Runovi 60–62)

- **Nova SLAM karta na punoj rezoluciji 0.02 m**:
  - Snimljena novim lidarom (1080 zraka, 1 mm, 25 Hz) uz `slam_params.yaml` s rezolucijom 0.02 m i gušćim grafom.
  - Vrata očitana kao 0.980 m (umjesto 0.950 m), a os prolaza na 0.0 cm (umjesto +1.5 cm).
- **AMCL kalibracija za milimetarski lidar**:
  - `sigma_hit: 0.05` (bilo 0.20), `z_hit: 0.9`, `z_rand: 0.1`, `laser_likelihood_max_dist: 0.5`, `max_beams: 360`, `resample_interval: 2`, `min/max_particles: 1000/3000`.
  - Uklonjen stalan pomak lokalizacije od ~6 cm. Razlika lidar-vs-AMCL smanjena na šum (1.8–3.5 cm).


- **Podignute rezolucije i senzori na PAL Tiago standard**:
  - Virtual base lidar nadograđen na službeni PAL SICK TiM571 standard (`1080` zraka, `1 mm` dometna točnost, `25 Hz`, min domet `0.05 m`).
  - AMCL podešen na `180` zraka, pragovi ažuriranja `0.03 m` i `0.03 rad` (1.7°).
  - DWB lokalni planer na `20 Hz`, $25 \times 25 \times 25 = 15.625$ trajektorija po ciklusu, linearna i angularna granularnost `0.01` (1 cm / 0.01 rad).
  - Uklonjen `use_final_approach_orientation: true` i `RotationShimController` (uzrok bočnog zakretanja od 132° pri prilazu vratima).
  - `Twirling.scale: 20.0`, `yaw_goal_tolerance: 0.025 rad` (1.4°), `max_vel_theta: 0.30 rad/s`, `max_vel_y: 0.15 m/s`.
  - Zone: `lane_margin: -0.15` (otvor 1.25 m), `chute_length: 0.50 m`.
- **Rezultati verifikacije uživo (Run 58)**:
  1. Home -> Crvena soba (stol): **PROŠAO Vrata 1**, najmanji razmak 4.7 cm, stigao na cilj.
  2. Manevar u Crvenoj sobi (okret udesno): **USPJEŠNO**.
  3. Crvena -> Srednja (Home) soba: **PROŠAO Vrata 1**, najmanji razmak 3.0 cm, stigao na cilj.
  4. Srednja -> Plava soba (pred stol): **PROŠAO Vrata 0**, najmanji razmak 4.5 cm, stigao na cilj.
  5. Plava -> Srednja (automatski): **PROŠAO Vrata 0**, najmanji razmak 6.1 cm, stigao na cilj.
  - Na povratku u Crvenu sobu (dionica 4/5) navigator je zaustavio kretanje zbog `only 0.7 cm beside the robot, limit 3 cm` (asimetričan ulaz od 6 cm ulijevo). DWB i robot nisu udarili dovratnik.
- **Sljedeći korak**: prilagodba `min_side_clearance` u navigatoru i centriranja prilaza.


## Sesija 2026-09-14 (Omnidirekcijski DWB & Prolaz kroz vrata & Prijelaz soba)
**Riješeno i verificirano (End-to-End):**
- **Omnidirekcijski lokalni planer (DWB za mecanum bazu)**:
  - Uklonjen diferencijalni pure-pursuit efekt koji je sprječavao bočno gibanje i izazivao trzanje/rotaciju pri malim pomacima.
  - Aktivirani omni rasponi: `min_vel_x: -0.30`, `max_vel_x: 0.30`, `min_vel_y: -0.30`, `max_vel_y: 0.30`, `max_vel_theta: 0.60`.
  - Povećana rezolucija uzorkovanja brzina na `vx_samples: 15`, `vy_samples: 15`, `vtheta_samples: 15` (finoća 0.043 m/s), čime je uklonjen overshoot/undershoot na granici tolerancije cilja.
  - Usklađen `velocity_smoother` s punom podrškom za reverzno i lateralno gibanje (`min_velocity: [-0.30, -0.30, -0.60]`, `acc_lim: 1.5 m/s²`, `acc_lim_theta: 3.0 rad/s²`).
  - U `local_costmap` uklonjen nepotrebni `inflation_layer` koji je u vratima širine 1.0 m umjetno zatvarao prolaz (inscribed radius 0.427 m generirao je cost 253 po cijeloj širini). `ObstacleFootprintCritic` direktno rasterizira stvarni pravokutni otisak robota (1.04 × 0.854 m) nad laserskim točkama bez lažnih kolizija.
  - Verificiran čisti holonomski bočni pomak ($v_y = 0.17\text{ m/s}$, $\omega_z = 0.000\text{ rad/s}$) i vožnja unatrag bez rotacije.
- **Geometrija kretanja između soba (uklonjen Manhattan L-detour)**:
  - Isključen `square_corners` (postavljen na `False`) u `room_navigator.py`.
  - Robot pri prelasku iz Plave u Crvenu sobu prolazi Vrata 0 okomito u Home sobu, preseca Home sobu izravnom dijagonalom prema prilazu Vratima 1, poravnava se okomito na normalu Vrata 1 ($\le 5.0^\circ$), prolazi ravno kroz Vrata 1 i parkira se ispred stola u Crvenoj sobi.
- **Grafički prikaz za lokalizaciju**:
  - Konfiguracija vraćena točno na stanje prije posljednje dvije izmjene kada je vožnja radila pouzdano.
  - U stock Nav2 RViz dodan **isključivo prikaz elipse sigurnosti lokalizacije**: `AMCL Pose (Kovarijanca)` (`rviz_default_plugins/PoseWithCovariance` na `/amcl_pose` s prikazom elipse kovarijance). Svi ostali elementi RViz-a i GUI-ja su u svom originalnom obliku.
- **Dvoetapna End-to-End verifikacija**:
  1. **Ručni RViz 2D Goal Pose** (`/goal_pose` -> `/bt_goal_pose` via `room_navigator`):
     - Polazak iz Home `(0.0, 0.0, 0.0°)`, zadani cilj u Plavoj sobi `(0.0, -4.5, -90.0°)`.
     - Vrata 0 prijeđena s odstupanjem centra od samo 1.8 cm i greškom kuta 4.2° ($\le 5.0^\circ$).
     - Uspješan dolazak u Plavu sobu za **33.1 s** (`x = -0.018 m, y = -4.361 m, yaw = -85.8°`).
  2. **Autonomna tranzicija (`/room_navigator/goto "red"`)**:
     - 5 dionica (Blue -> Vrata 0 -> dijagonala preko Home -> Vrata 1 -> Crvena soba -> stol).
     - Uspješan dolazak pred crveni stol za **60.6 s**!
     - Konačna poza pred stolom: `x = 4.693 m, y = -0.016 m, yaw = 3.5°`.


## Sesija 2026-09-13 (primopredaja)
**Napravljeno:**
- `notes/` — Obsidian baza: 21 zahtjev, 10 podsustava, 36 problema s tablicama pokušaja,
  15 odluka, registar parametara i poza, plan predaje. Ulaz `notes/00_MAPA.md`.
- **Svijet prepravljen** (`5f6d016`…`e99316e`): tri sobe u L, svaka 6×6 m, zidovi 3 m,
  prolazi 1.0 m. HOME (siva, robot) → PLAVA (kutija, stol na (0,−6.5)) i CRVENA
  (odredište, stol na (6.5,0)). Kutija 0.30 m, **0.3 kg**.
- **Izmjerene stvarne dimenzije** (prije se samo procjenjivalo): robot je u korisnikovoj
  pozi `ARM_CARRY_V2` **85.4 cm** širok, 1.04 m dug, 1.45 m visok. Potvrđeno dvjema
  neovisnim metodama (`scripts/fit_test.py`, `scripts/mesh_extent.py`).
- **Novi alati**: `scripts/joint_gui.py` (zglobovi u stupnjevima, upis vrijednosti),
  `capture_posture.py`, `measure_robot.py`, `fit_test.py`, `mesh_extent.py`,
  `door_gauge.py`. Popravljen `display.launch.py` (bio neupotrebljiv od 12. 6.).
- Klizači torza: gornji limit 0.8 → **0.65 m**.
- **Omnidirekcijski pogon baze (R-08, P-09) RIJEŠEN**: `mecanum_drive_controller` uspješno
  integriran u `ros2_control` i DART fiziku. Prilagođen URDF baze (`wheel.urdf.xacro`, `base.urdf.xacro`)
  s effort limitom 100 Nm, 0 prigušenjem/trenjem i anizotropnim trenjem (`mu1=0.80`, `mu2=0.20`,
  `fdir1 ignition:expressed_in="base_footprint"`). Uživo potvrđeno gibanje: vx, vy (dy = 0.33 m),
  wz i dijagonale; odometrija i TF stabilni. Odstupanja #1 i #7 precrtana.

**Otvoreno (redom po prioritetu):** vidi `notes/07_predaja/danas.md`.
1. Nav2 lokalizacija (AMCL s novom kartom `seminar_map`) i vožnja do regije u plavoj sobi (`R-15`, `P-11`).
2. Ponoviti dvoručni hvat na 0.75 m stolu u novom svijetu (`P-28`, `R-17`).
3. Poza za prolaz kroz vrata i transport kocke (`P-18`, `P-35`, `D-15`).
4. Odlaganje kocke na odredišni stol u crvenoj sobi (`R-20`).
5. Seminar, video, prezentacija (`R-21`).

**Slijepe ulice iz ove sesije:** prilagodba Gazebo `<gui>` sekcije (FOV, ViewAngle plugin)
— pogoršala je stvari, vraćeno na zadano; vidi `notes/02_rjesenja/S-02_world_and_sim_launch.md`.

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
