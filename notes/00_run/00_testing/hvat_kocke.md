---
id: RUN_TEST_GRASP
type: upute
updated: 2026-09-15
---
# Testiranje: dolazak hvataljkama u točke hvata i dizanje

Slijed testa, kako ga je korisnik zadao:

> 1. robot dođe na poziciju pred stol
> 2. digne ruke vodilicama na visinu stola
> 3. ako treba, približi se još malo stolu
> 4. svaku ruku vodimo u točku koju treba
> 5. kada ruke dođu u točke, podižemo vodilice za 10 cm

Bez pritiska, bez attacha, bez gate-ova. To je **prvi test** — gleda se u GUI-ju što se dogodi,
pa se tek onda odlučuje dalje. Vidi [[R-17_dual_arm_lift]].

> [!warning] Pokus sa silom nije više u repozitoriju (16. 9. 2026.)
> Čvorovi `force_grasp`, `wrench_estimator`, `grasp_force_diagnostics`, `force_estimation.launch.py`
> te skripte `grasp_cube.py`, `run_force_isolated.sh`, `stop_force_isolated.sh`, `new_force_run.sh`,
> `qualify_force_log.py`, `apply_measured_arm_gains.py` i `run_cube_isolated.sh` ostali su
> **lokalno na disku**, ali su izašli iz repoa: pokus je napušten ([[D-21_effort_pid_actuator_profile]])
> i nije dio gotove misije, koja radi na običnom `position` sučelju.
> Biblioteka `force_model.py` je zadržana jer je koristi `scripts/grasp_width.py`.
> Izmjerene brojke iz pokusa ostaju u [[P-41]], [[P-42]] i [[06_parametri]].

> [!important] Zadani profil i dalje ide na `position` sučelje
> Bez zastavice `force_grasp:=true` ni ruke ni vodilice nemaju regulator. Ako nešto ne stigne na
> cilj, to se **izmjeri i zapiše**, ne zaobiđe se PID-om.

> [!warning] Korak 2 ne prolazi na zadanom profilu
> Izmjereno 15. 9. 2026.: vodilice pod težinom ruku ostaju na 0.0500 m iako je naređeno 0.200 m,
> a akcija javi `SUCCEEDED`. Vidi [[P-13_torso_prismatic_no_lift]]. U `scripts/joint_gui.py`
> (RViz, bez fizike) se pomiču jer ondje naredba izravno postaje stanje.
> Na profilu `force_grasp:=true` iste vodilice pogađaju **0.2000 / 0.2000 m, greška 0.0000 mm**
> ([[D-21_effort_pid_actuator_profile]]).

---

## Korak 1 — robot na poziciju pred stol

Dock poza iz `nav_zones.py`: `dock = table_half (0.40) + HALF_LENGTH (0.52) +
table_dock_safety (0.10) = 1.021 m` od centra stola. Plavi stol je u `(0, −6.5)`, prilazi se
s `+Y` strane.

| | vrijednost |
|---|---|
| dock poza | `x = 0.0`, `y = −5.479`, `yaw = −90° (−1.5708 rad)` |
| centar kocke odatle | ~0.87 m ispred `base_link` |
| ploha stola | `z = 0.75 m`, centar kocke `z = 0.90 m` |

Za sam test robot se spawna na dock pozi — ne troši se vrijeme na punu vožnju.

> [!note] Svi terminali prvo
> ```bash
> cd /home/khartl/FSB/PAS-DUAL-ARM
> ```

> [!note] Terminal 1 — simulacija + RViz (spawn na dock pozi, zapešća iznad plohe)
> ```bash
> bash scripts/run_cube_isolated.sh ros2 launch pas_dual_arm_bringup sim.launch.py headless:=false rviz:=true table_arms:=true robot_spawn_x:=0.0 robot_spawn_y:=-5.479 robot_spawn_yaw:=-1.5708
> ```
> `rviz:=true` otvara prikaz `rviz/cube.rviz` — robot, TF markera i obiju hvataljki, oblak
> dubinske kamere i **poze hvata** (`/cube_grasp/poses`, narančaste osi) čim ih skripta iz koraka 4
> izračuna. Sve je uključeno odmah; ništa se ne dodaje rukom.
>
> `table_arms:=true` spawna zapešća **iznad** plohe. Carry poza ih drži na 0.49 m, a ploha je na
> 0.75 m — zato su ruke dotad zapinjale za rub stola.

> [!note] Terminal 2 — MoveIt i ArUco (obavezno `auto_start:=false`)
> ```bash
> bash scripts/run_cube_isolated.sh ros2 launch pas_dual_arm_bringup task.launch.py auto_start:=false
> ```

Kad je test gotov, puna vožnja do te poze ide preko `room_navigator` (`blue:dock` na
`/room_navigator/goto`), ne izravno na `/navigate_to_pose`.

> [!danger] Nikad dvije simulacije odjednom
> Oba Gazeba objavljuju `/clock` na istoj domeni, pa vrijeme skače naprijed-natrag. RViz javlja
> `Detected jump back in time` stotinama puta u sekundi, resetira se na svaki, i na kraju pukne s
> `Cannot create GL vertex buffer`. Iz simptoma se uzrok ne vidi. Prije pokretanja:
> ```bash
> bash scripts/clean_ros.sh
> ```
> `run_cube_isolated.sh` to od 15. 9. i sam odbija — javi `GRESKA: simulacija vec radi`.

> [!important] `grasp_cube.py` je jedini vlasnik gibanja
> `task.launch.py` sada ima `auto_start:=false` kao zadanu vrijednost. Stari `main_task`
> orkestrator pokreće se samo eksplicitnim `auto_start:=true`. Pričekati da T1 ispiše završetak
> `table_ready` čvora prije pokretanja T3.

**Terminal 3** ostaje slobodan za korake 2–4. Do tada se robot ne smije micati sam od sebe —
jedino što se pomakne bez tvoje naredbe su vodilice i ruke u `ARM_HOME`, iz čvora `table_ready`.

## Korak 2 — vodilice dižu ruke na visinu stola

Čvor `table_ready` krene sam čim se kontroleri aktiviraju: zada visinu vodilica kao običnu
trajektoriju na `torso_controller` (isto kao ruke, `position`, bez regulatora), pa **očita
`/joint_states`** i javi stvarnu visinu. Status akcije se ne uzima kao dokaz.

U logu terminala 1 tražiti redak:

```
CARRIAGE MEASURED left=… right=… m (commanded 0.200, error …, action reported …)
```

> [!note] Terminal 3 — ponoviti mjerenje zasebno, s drugom visinom
> ```bash
> bash scripts/run_cube_isolated.sh python3 scripts/probe_torso.py --height 0.20
> ```

Taj broj ide u tablicu pokušaja [[P-13_torso_prismatic_no_lift]], kakav god bio.

## Korak 3 — lociranje kocke i (ako treba) primicanje

Prvo mjerenja, robot se **ne miče**.

> [!note] Terminal 3 — centar kocke iz markera + dubine, obje točke hvata, IK
> ```bash
> bash scripts/run_cube_isolated.sh python3 scripts/probe_cube_perception.py --ros-args -p use_sim_time:=true
> ```

> [!note] Terminal 3 — treba li se uopće primicati: IK na stvarnoj dock udaljenosti
> ```bash
> bash scripts/run_cube_isolated.sh python3 scripts/probe_cube_ik_variants.py --ros-args -p use_sim_time:=true
> ```

Prihvatljivo: `EXTENT` duga os **0.300 ± 0.01 m**, `AGREEMENT` `xy ≤ 0.03` i `z ≤ 0.05`.
Z se uzima **iz markera** (on je na sredini plohe); dubina vidi samo gornji dio kocke pa joj je
Z sredina ~2.7 cm previsoka.

Ako IK prolazi na ~0.87 m → baza se ne miče. Ako pada, primiče se odometrijom (`drive_distance`),
izvan Nav2. Granica je noga stola na `y = −6.125`. Nakon pomaka marker zna izaći iz vidnog polja
→ naciljati kameru i **ponovno izmjeriti** prije pomicanja ruku.

## Koraci 3–5 — jedna naredba

Sve od lociranja do dizanja je u jednoj skripti, u **jednom terminalu**.

> [!note] Terminal 3 — puni hvat
> ```bash
> bash scripts/run_cube_isolated.sh python3 scripts/grasp_cube.py --ros-args -p use_sim_time:=true
> ```
> Zaustavljanje ranije: `--pregrasp-only` (ništa ne dodiruje) ili `--no-lift` (stane na kontaktu).

Slijed:

| #   | radnja                                                                     |
| --- | -------------------------------------------------------------------------- |
| 1   | glavna kamera izmjeri kocku (marker + dubina)                              |
| 2   | baza se primakne na doseg ruku, uz granicu stola                           |
| 3   | hvataljke se otvore, obje ruke **paralelno** na pred-poze (20 cm od ploha) |
| 4   | svaka ruka očita **svoj** marker na plohi koju će pritisnuti               |
| 5   | obje ruke brzinom najviše 25 mm/s na 2 cm od ploha i na **istu visinu**   |
| 6   | obje ruke prilaze 2 mm/s; svaka staje na svom prvom kontaktu              |
| 7   | obje hvataljke se zatvore; nepotpuna ruka radi korake 2 mm do 4/4         |
| 8   | nakon stabilne 1 s vodilice dignu 20 cm brzinom 10 mm/s                   |

> [!important] Visina se poravnava prije dodira, i poslije se ne dira
> Korak 5 obje ruke dovede na **istu visinu** dok još ništa ne dodiruju. U koraku 6 svaka ruka
> zadrži visinu na kojoj **stvarno jest** i miče se čisto vodoravno.
>
> Ispravljanje visine **dok se pritišće** je ono što je 15. 9. nakosilo kocku: zapešće se pomakne
> po z uz plohu koju već dodiruje, kocka se nagne, desna popusti, lijeva učini isto, i obje je
> onda drže ukoso.

> [!important] Svaka ruka staje zasebno
> Od standoffa od 2 cm obje ruke krenu zajedno, ali cilj pojedine ruke otkazuje se na njezinu prvom
> kontaktu. Zatim se samo ruka koja još nema 2/2 kontakta primiče u koracima od 2 mm;
> dovršena ruka miruje. Z ostaje trenutačni, a orijentacija se vraća na onu izvedenu iz
> markera, najviše 10 mm preko izmjerene plohe. Ako sva četiri kontakta nisu stabilna 1 s,
> dizanje ne počinje.

Ako naredba od 2 mm tri puta zaredom daje manje od 0,2 mm stvarnog pomaka,
nepotpuna ruka se odmakne 3 mm radi rasterećenja i poravnanja prije nastavka.

Tijekom friction-only dizanja nema `DetachableJoint` attacha. Gubitak kontakta zaustavlja
vodilice, vraća postupak na ograničenu finu korekciju i zatim nastavlja prema istom cilju
`početna visina + 20 cm`. Iscrpljen hod ili nemoguć povrat kontakta prekidaju run.

---

## Što zabilježiti

| # | podatak | kartica |
|---|---|---|
| 1 | naređena vs. stvarna visina vodilica | [[P-13_torso_prismatic_no_lift]] |
| 2 | centar kocke i obje točke hvata u `base_link` | [[R-17_dual_arm_lift]] |
| 3 | je li IK prošao na dock udaljenosti, ili koliko se trebalo primaknuti | [[P-25_asymmetric_arm_reach]] |
| 4 | odstupanje stvarne poze hvataljki od zadanih | [[P-37_arm_position_gain_sag]] |
| 5 | što se dogodilo pri +10 cm na vodilicama | [[R-17_dual_arm_lift]] |

Svaki run → novi redak u [[runovi]].

---

## Test dizanja kutije (misijski put, D-05)

> [!warning] Spawn 1.05 m od kocke, ne na dock pozi (od 15. 9.)
> Robot sada vozi u `GRASP_V3`. Na dock pozi (`y −5.479`, kocka 0.87 m) su mu šake ispod ploče
> stola i vodilice se **ne mogu dići** (0.0 cm). Sve naredbe ispod koriste `robot_spawn_y:=-5.479`
> (kocka 1.05 m). Slijed hvata: [[P-44_grasp_from_reference_pose]].

Ovo je **misijski** test: MoveIt i pozicijske naredbe posvuda, **nijedan regulator**, hvat po
[[D-05_contact_verified_attach]] — kruti spoj se uključi tek nakon dokazanog obostranog kontakta.
Dizanje rade **vodilice**, ne ruke ([[R-03_linear_rails_torso]], [[P-13_torso_prismatic_no_lift]]).

> [!important] Zašto ovaj put, a ne dizanje trenjem
> Zadatak ne traži dizanje trenjem. `R-17` doslovno traži „obostrani kontakt s `aruco_box` dokazan
> senzorima **prije attacha**", a to je `D-05`, odlučen 30. 6. uz korisnikovo odobrenje. Zabranjen
> je teleport i zavarivanje iz daljine ([[P-16_fake_teleport_grasp]]), ne attach sam po sebi.
> Pokus s procjenom sile ostaje u repozitoriju, ali se u misiji **ne pokreće**.

> [!warning] Prvo provjeri da nema zaostalih čvorova
> ```bash
> bash scripts/stop_force_isolated.sh --check
> ```
> Mora javiti `Partition pas_dual_arm_force_76 is clear.` Ako radiš u domeni 75, isto vrijedi za
> `bash scripts/clean_ros.sh`.

> [!note] 1. Build
> ```bash
> ./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
> ```

> [!note] 2. Terminal 1 — simulacija, vidljivi Gazebo + RViz
> **Bez** `force_grasp` i **bez** `table_arms` — misija sama diže vodilice i postavlja ruke.
> ```bash
> bash scripts/run_force_isolated.sh ros2 launch pas_dual_arm_bringup sim.launch.py rviz:=true robot_spawn_y:=-5.479 robot_spawn_yaw:=-1.5708 |& tee log/run-t1-sim.log
> ```
> Čekaj da se aktivira svih osam kontrolera. Vodilice startaju na **0.06 m**, milimetrima iznad
> graničnika — to je popravak iz [[P-13_torso_prismatic_no_lift]] i ne smije se vratiti na 0.05.

> [!note] 3. Terminal 2 — misija
> `auto_start:=true` pokreće `main_task`, koji vodi cijeli slijed.
> ```bash
> bash scripts/run_force_isolated.sh ros2 launch pas_dual_arm_bringup task.launch.py auto_start:=true |& tee log/run-t2-mission.log
> ```

Slijed koji misija vodi sama:

| korak | radnja |
|---|---|
| 0 | vodilice na visinu vožnje (zapešća iznad plohe stola) |
| 1–2 | traženje markera i vizualni prilaz na ~0.9 m |
| 3–4 | mjerenje kutije markerom i dubinskom kamerom |
| 4d | **vodilice na visinu hvata**, izvedenu iz izmjerene visine kutije |
| 5 | obje ruke pritisnu suprotne plohe zatvorenim jastučićima |
| 5d | **gate**: obostrani kontakt s `aruco_box` + geometrijska provjera |
| 6 | attach, desna ruka se povuče, **vodilice dignu 15 cm** |
| 8 | **vodilice spuste**, detach, odmak |

> [!note] 4. Ugađanje pritiska bez rebuilda
> Ako gate padne na kontaktu, pritisak je premali; ako kocka otpluta, prevelik.
> ```bash
> bash scripts/run_force_isolated.sh ros2 launch pas_dual_arm_bringup task.launch.py auto_start:=true -p squeeze_interference:=0.015 |& tee log/run-t2-mission.log
> ```

Uredan tijek u logu, po redu:

```
CARRIAGE TRAVEL MEASURED left=0.2000 right=0.2000 m (commanded 0.2000)
Box measured from depth (...): length=0.300 height=0.252 ...
Grasp height: cube centre 0.821 m -> carriages 0.201 m
CARRIAGE GRASP MEASURED left=0.2008 right=0.2008 m (commanded 0.2008)
Squeeze: half-depth 0.150 (measured) m, interference 10 mm
Squeeze evidence: placement(...)=True physical(contact L=True, R=True)=True.
CARRIAGE LIFT MEASURED left=0.3508 right=0.3508 m (commanded 0.3508)
PICK+LIFT done (cube held by verified attach).
TASK COMPLETE: cube placed on the table.
```

> [!warning] Što gledati u GUI-ju, ne u logu
> Log može reći `TASK COMPLETE`, a kocka biti nakrivljena ili gurnuta. Gledaj: diže li se kocka
> **zajedno s vodilicama**, ostaje li vodoravna, i pomakne li se po stolu tijekom pritiska.
> Status akcije nikad nije dokaz ([[D-12_honesty_abort_over_fake]]).

> [!warning] Negativan test — obavezan
> Makni kutiju izvan dohvata u `seminar_world.sdf` i ponovi. Misija **mora** pošteno abortirati
> (`squeeze not confirmed`), nikad attachati. Bez tog testa prolaz ne vrijedi.

> [!note] 5. Gašenje
> ```bash
> bash scripts/stop_force_isolated.sh
> ```

---

## Pokus hvata silom (`force_grasp` profil)

Zaseban pokus, u **svojoj** ROS domeni (76) i Gazebo particiji, da ne dira postojeći pokus u
domeni 75. Upravljanje koristi samo signale koje stvarni robot ima: momente zglobova Gen3,
kamere i položaje. Gazebo kontakti i FT senzori idu **isključivo** u zapis za naknadnu ocjenu.
Pozadina: [[D-21_effort_pid_actuator_profile]], [[P-41_effort_pid_arm_actuator_profile]],
[[P-42_contact_sensor_has_no_forces]].

Sve naredbe se izvršavaju iz korijena repozitorija. Svaki terminal ide kroz
`scripts/run_force_isolated.sh` — on postavlja domenu 76, Gazebo particiju i `ROS_HOME`, i
odbija podizanje drugog simulatora u istoj particiji.

> [!warning] Prvo provjeri da nema zaostalih čvorova iz prethodnog ciklusa
> Najbrže: `bash scripts/stop_force_isolated.sh`. Samo provjera, bez gašenja:
> ```bash
> bash scripts/stop_force_isolated.sh --check
> ```
> Mora javiti `Partition pas_dual_arm_force_76 is clear.`
>
> Gašenje `ros2 launch`-a **nije dovoljno**: njegovi čvorovi prežive kao siročad i dalje pričaju
> na istoj particiji. Nakon četiri ciklusa ostalo je četiri para `wrench_estimator` /
> `grasp_force_diagnostics`, `ros2 node list` je pokazivao svaki čvor **četiri puta**, a u logu se
> to javilo kao `Ignoring unexpected result response. There may be more than one action server`.
> **Takav run je smeće** i treba ga ponoviti.
>
> Skripta bira procese po **particiji** s kojom su pokrenuti (`IGN_PARTITION`), a ne po imenu
> naredbe, pa ne može dirati tvoju sesiju na domeni 75. Ručno, ako te zanima i što je izvan
> particije:
> ```bash
> ps -eo pid,comm,etimes | grep -E 'ign|ruby|wrench_estimato|grasp_force_dia|move_group|aruco_detector|parameter_bridg|robot_state_pub|table_ready|rviz2'
> ```
> Ako išta izlista, ugasi po PID-u. (`comm` je skraćen na 15 znakova, pa `wrench_estimator`
> izgleda kao `wrench_estimato`, a `grasp_force_diagnostics` kao `grasp_force_dia` — zato je u
> obrascu baš tako.)

> [!important] Svaka naredba ispisuje i u terminal i u log
> Naredbe završavaju s `|& tee log/run-…`, pa ispis vidiš uživo **i** ostane zapisan. Imena
> logova su uvijek ista, pa se naredbe kopiraju bez ijedne izmjene — korak 1 prethodni run
> odloži u `log/archive/`. Tako netko drugi (ili ja u idućoj sesiji) može pročitati točno ono
> što si vidio na ekranu.

> [!note] 1. Pripremi run — gasi staro, arhivira prethodne logove
> ```bash
> bash scripts/new_force_run.sh
> ```
> Mora završiti s `Spremno.` i popisom pet datoteka. Ako javi da particija nije čista, riješi to
> prije nego kreneš dalje.

> [!note] 2. Build (samo ako si mijenjao kod, launch, xacro ili config)
> ```bash
> ./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup |& tee log/run-build.log
> ```

> [!note] 3. Terminal 1 — simulacija, vidljivi Gazebo + RViz
> ```bash
> bash scripts/run_force_isolated.sh ros2 launch pas_dual_arm_bringup sim.launch.py rviz:=true force_grasp:=true table_arms:=true robot_spawn_y:=-5.479 robot_spawn_yaw:=-1.5708 |& tee log/run-t1-sim.log
> ```
> `headless` je po zadanom `false`, pa se Gazebo prozor otvara sam. `rviz:=true` podiže RViz s
> `rviz/cube.rviz`: poze hvata (`/cube_grasp/poses`), markeri (`/cube_grasp/markers`), oblak
> točaka glavne kamere i slike obiju kamera na zapešću.
>
> **Čekaj ovaj redak prije nego otvoriš idući terminal:**
> ```
> table-ready staging done (carriages=ok, arms=ok)
> ```
> Prije njega mora proći i mjerenje vodilica; ovako izgleda kad je dobro:
> ```
> CARRIAGE MEASURED left=0.2000 right=0.2000 m (commanded 0.200, error +0.0000 / +0.0000 m, action reported SUCCEEDED)
> ```

> [!warning] Na opterećenom stroju GUI zna izgladniti kontrolere ([[P-32_gui_starves_controllers]])
> Ako spawneri kontrolera isteknu ili staging ne prođe, ponovi run headless (korak 3b). Profil
> `force_grasp` vrti `controller_manager` na **1000 Hz**, pa je osjetljiviji od zadanog profila.
> Provjereno 15. 9.: u GUI-ju je staging prošao uredno, vodilice 0.2000 / 0.2000 m.

> [!note] 3b. Terminal 1 — headless varijanta, bez prozora
> ```bash
> bash scripts/run_force_isolated.sh ros2 launch pas_dual_arm_bringup sim.launch.py headless:=true rviz:=false force_grasp:=true table_arms:=true robot_spawn_y:=-5.479 robot_spawn_yaw:=-1.5708 |& tee log/run-t1-sim.log
> ```

> [!note] 4. Terminal 2 — procjenitelj sile i zapis mjerenja
> ```bash
> bash scripts/run_force_isolated.sh ros2 launch pas_dual_arm_bringup force_estimation.launch.py record:=true output:=log/run-force.jsonl |& tee log/run-t2-estimator.log
> ```
> Provjeri da zapis **stvarno raste** — bez toga run ne mjeri ništa:
> ```bash
> sleep 5; ls -l log/run-force.jsonl
> ```

> [!danger] Ako `log/run-force.jsonl` ostane prazan ili ga nema
> `grasp_force_diagnostics` otvara zapis s `x` (isključivo stvaranje) da nikad ne prepiše tuđe
> mjerenje. Ako datoteka postoji, čvor pukne na `FileExistsError`, **`ros2 launch` ostane živ** i
> run izgleda uredno — a ne snima se ništa. Upravo se to dogodilo 15. 9. Rješenje je korak 1,
> koji prethodni zapis odloži u `log/archive/`.

> [!note] 5. Terminal 3 — MoveIt i ArUco
> `auto_start:=false` da stari `main_task` ne preuzme gibanje — `grasp_cube.py` mora biti jedini
> vlasnik gibanja.
> ```bash
> bash scripts/run_force_isolated.sh ros2 launch pas_dual_arm_bringup task.launch.py auto_start:=false |& tee log/run-t3-task.log
> ```
> Čekaj da se `move_group` i tri `aruco_detector` čvora pojave.

> [!note] 6. Terminal 4 — stanje ROS grafa prije pokusa
> Svaki čvor mora biti **točno jedan**; ako se koji pojavi dvaput, prekini i vrati se na korak 1.
> ```bash
> bash scripts/run_force_isolated.sh ros2 node list |& tee log/run-t4-nodes.log
> ```
> Vidi li procjenitelj obje ruke (dok hvataljke još nisu otvorene, uredan odgovor je
> `"reason": "gripper not confirmed open"`):
> ```bash
> bash scripts/run_force_isolated.sh ros2 topic echo /grasp/estimator_status --once |& tee log/run-t4-estimator-status.log
> ```

> [!note] 7. Terminal 4 — pokus
> `--characterize` je ograničen pokus **bez dizanja**, samo da se provjeri mjerenje sile:
> ```bash
> bash scripts/run_force_isolated.sh python3 scripts/grasp_cube.py --characterize --ros-args -p use_sim_time:=true |& tee log/run-t4-grasp.log
> ```
> Samo do pred-hvata, ništa se ne dodiruje:
> ```bash
> bash scripts/run_force_isolated.sh python3 scripts/grasp_cube.py --pregrasp-only --ros-args -p use_sim_time:=true |& tee log/run-t4-grasp.log
> ```
> S dizanjem — traži **prošlu** kvalifikaciju iz koraka 8:
> ```bash
> bash scripts/run_force_isolated.sh python3 scripts/grasp_cube.py --qualification log/run-qualification.json --ros-args -p use_sim_time:=true |& tee log/run-t4-grasp.log
> ```

> [!warning] `--ros-args -p use_sim_time:=true` nije neobavezan
> `grasp_cube.py` se pokreće kao obična skripta, pa bez toga njegov sat ostaje zidni. Svaka
> provjera svježine tada uspoređuje zidno vrijeme sa sim oznakom i **svaki** marker ispadne
> zastario: run pukne na `No fresh marker` iako kamera uredno vidi marker.

> [!important] Pokus pokreni **samo jednom** po simulaciji
> Ako prvi pokušaj pukne, ruke ostanu tamo gdje su stale. Drugi pokušaj tada MoveIt odbije s
> `trajectory start differs from the actual state (left_joint_5: -3.14 vs 3.14)` — kontinuirani
> zglob stoji na −3.14, a plan kreće od +3.14, isti kut a 2π razlike, pa se ruke **uopće ne
> pomaknu**. Za novi pokušaj idi na korak 1 i digni simulaciju iznova.

Uredan tijek u logu terminala 4, po redu:

```
Aruco confirmed (5 samples); box centre base_link (...)
cube width between the faces: 0.300 m
left pre-grasp: EE at (...) vs goal Δ=0.008 m -> REACHED.
left measured standoff 20.5 mm to its contact pose
Force grasp: zeroing
Force grasp: squeeze
Force grasp: characterization_complete
```

> [!note] 8. Ocjena, offline
> ```bash
> ./scripts/run_native.sh python3 scripts/qualify_force_log.py log/run-force.jsonl --output log/run-qualification.json |& tee log/run-qualify.log
> ```
> Prolaz traži, po ruci: ≥50 slobodnih i ≥50 opterećenih uzoraka, najveću lažnu silu u slobodnom
> gibanju < 20 % cilja, p95 pogreške prema **FT senzoru** < 20 % cilja, i p95 razlike između
> lijeve i desne ruke < 20 % cilja. Tek `passed: true` (uz nezavisan dokaz trenja) otključava
> dizanje.

> [!note] 9. Gašenje
> ```bash
> bash scripts/stop_force_isolated.sh
> ```

> [!important] Što ostaviti da se run može pročitati
> Ne brisati ništa iz `log/`. Nakon runa ondje stoji sve što treba za dijagnozu:
>
> | datoteka | što sadrži |
> |---|---|
> | `log/run-t1-sim.log` | Gazebo, kontroleri, staging, mjerenje vodilica |
> | `log/run-t2-estimator.log` | procjenitelj sile i snimač |
> | `log/run-t3-task.log` | MoveIt i ArUco |
> | `log/run-t4-grasp.log` | sam pokus hvata |
> | `log/run-force.jsonl` | mjerenja: procjena sile, FT zapešća, kontakti |
> | `log/force_ros_home/log/` | ROS logovi po čvoru, ako treba dublje |

---

### Zašto baš tako

> [!note] Dvije reference, jer nijedna sama nije dovoljna
> **Apsolutna** je FT senzor na zapešću: kontaktna poruka u Fortressu nosi samo točke dodira, bez
> sile ([[P-42_contact_sensor_has_no_forces]]). FT mjeri i težinu hvataljke, pa se uspoređuje
> **promjena** od osnovice uzete neposredno prije prvog dodira.
> **Dosljednost** je slaganje dviju ruku na istoj kocki — provjerava dvije kinematičke grane jednu
> o drugu, ali ne vidi pogrešku zajedničku objema, pa ne može zamijeniti apsolutnu.
> Kontaktni senzori određuju **kada** je jastučić opterećen, FT senzor **koliko**.

> [!warning] Ni FT ni kontaktni senzori ne smiju ući u upravljanje
> Stvarni Gen3 ima senzore momenta u zglobovima, a ne FT na zapešću. Oba su isključivo dokaz za
> ocjenu i čita ih samo `grasp_force_diagnostics`.

> [!note] Tišina kontaktnog senzora
> Fortress ne objavljuje ništa dok prst ne dodiruje, pa mrtav senzor izgleda isto kao savršeno
> slobodna ruka. `qualify_force_log.py` zato priznaje tišinu kao „nema dodira" **samo** u zapisu u
> kojem se svaki od četiri jastučića barem jednom javio — bilo čime, kutijom, stolom ili drugim
> prstom. Polje `contact_sensors_heard` u izvještaju kaže koji su se javili.
>
> Tražiti da svaki jastučić javi dodir **s kutijom** bilo bi strože nego što pitanje zahtijeva i u
> praksi neispunjivo: sva četiri sjede na plohi unutar 0.3 mm, a tko će od njih prodrijeti dovoljno
> da okine senzor odlučuje desetinka milimetra ([[P-42_contact_sensor_has_no_forces]]).

> [!warning] Poznat kvar: preskok IK grane na lijevoj ruci
> Stisak povremeno pukne na `left IK branch/tracking jump`. Izmjereno 15. 9.: IK je točan
> (pogreška poze **0.00 mm**), ali za pomak od **0.5 mm** lijeva ruka vrati rješenje **3.04 rad**
> od trenutne konfiguracije, dok desna vrati 0.0036 rad. Zaštita ispravno odbija izvesti zamah od
> 3 rad za pola milimetra. Nije riješeno → [[P-25_asymmetric_arm_reach]].
