---
id: P-45
type: problem
status: rijeseno
requirements: ["[[R-15_region_goal_nav2]]", "[[R-17_dual_arm_lift]]", "[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
solutions: ["[[S-06_navigation]]", "[[S-08_grasp_squeeze_attach]]", "[[S-09_task_orchestration]]"]
decisions: ["[[D-12_honesty_abort_over_fake]]", "[[D-18_verified_baseline_first]]", "[[D-19_dynamic_footprint]]", "[[D-20_single_potential_field_costmap]]"]
updated: 2026-09-16
---
# P-45 — Vožnja i hvat nikad nisu radili zajedno (konačni sim)

## Simptom
Oba dijela rade, ali svaki u svom postavu:

- vožnja (runovi 46, 62, 70) vožena je u **`ARM_CARRY_V2`**, s robotom koji ništa ne nosi;
- hvat (runovi V2–V5, GUI U2) radi samo sa **spawnom na dock pozi** (`robot_spawn_y:=-5.479`),
  dakle bez ijedne vožnje prije njega.

Nijedan run nije prošao put *home → plava soba → hvat → crvena soba → odlaganje*.

## Uzrok
**Potvrđeno:** jedina spona u kodu (`main_task._navigate_to_region`, `main_task.py:1987`) šalje
sirov cilj na `/navigate_to_pose` i time zaobilazi portalne poze, gate pred vratima i dock
geometriju — upravo ono čime je [[P-39_nav2_enters_doorway_at_an_angle]] riješen. Zato se u misiji
vozi isključivo preko `room_navigator`-a.

**Potvrđeno:** poza vožnje je od 16. 9. `DRIVE_V4` ([[P-44_grasp_from_reference_pose]]), a sve
provjerene vožnje su bile u `ARM_CARRY_V2`. `DRIVE_V4` je 0.821 m (naspram 0.854 m), ali u
navigaciji **nije provjeren** — zato je M1 prvi inkrement i ide **bez ijedne izmjene koda**
([[D-18_verified_baseline_first]]).

**Hipoteza:** gate ruku u navigatoru (`room_navigator.arms_ok`) pašće čim robot ponese kocku, jer
uspoređuje zglobove s `POSTURES[ARM_DRIVE]`. Rješenje nije popuštanje tolerancije nego referentna
poza koju misija objavi iz **izmjerenih** vrijednosti, uz tvrdi uvjet na izmjerenu širinu
(pravilo korisnika: širina + 10 cm ≤ vrata; nošenje je 0.821 → 0.921 < 0.980).

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 16. 9. / radno stablo | Offline provjera preduvjeta: `check_doors.py`, `check_zones.py` | ✅ `PASS`; oba vrata 0.980 m; sobe `home`/`blue`/`red`; stolovi u `blue` i `red` s `approach` (1.55 m) i dock pozama; svih 11 stop poza međusobno dohvatljivo | `red:dock` postoji → odlaganje ima kamo doći |
| 2 | 16. 9. / radno stablo | Misijski način u `main_task` (`mission`, `/mission/start`, `_goto` preko `room_navigator`-a), `mission.launch.py`, gumb u `nav_gui` | build prolazi, `--show-args` i import čisti; **nije voženo** | čeka M1 i M2 ([[misija]]) |
| 3 | 16. 9. / radno stablo | **M1 u korisnikovom GUI-ju**: prva vožnja u `DRIVE_V4`, prazan robot (RViz cilj, `goto red`, `goto blue`) | ✅ 9 ciljeva, **3 prolaza kroz vrata bez aborta**; `arms: DRIVE_V4 held, worst joint 0.000 rad`; poravnanje traži 0.854–0.879 m; najtješnji razmak 4.4–4.9 cm; dolasci 2.8–5.0 cm i ≤ 1.4°; lidar vs AMCL 2.1–3.5 cm | ⚠ **dock dionica nije vožena** — traženi su `red`/`blue` (approach), ne `blue:dock`; undock također ne | `DRIVE_V4` kao poza vožnje je **potvrđen**, na razini runa 62. Dock/undock ostaju nepoznanica koju M2 vozi kao prvu radnju |
| 4 | 16. 9. / radno stablo | **M2 u korisnikovom GUI-ju**: `mission.launch.py`, vožnja + hvat, pa korisnik poslao robota s kutijom u drugu sobu | ⚠ hvat prošao i kutija je bila u rukama, ali **abort na vratima**: `ABORT before leg 3/6 - blue -> home: left_joint_5 is 2.547 rad off DRIVE_V4 (limit 0.15)`. Uz to: **RViz se nije pokrenuo**, izmjena zadataka presporo, marker odlaganja ne postoji (još X) | prolaz kroz vrata s kutijom | Korisnikova dijagnoza točna: „očekujem da smo u drive pozi, a nismo, u carry smo". Izmjereno s robota: `left_joint_5` = −1.958 vs `DRIVE_V4` 0.589 = **točno 2.547 rad** |
| 5 | 16. 9. / radno stablo | Popravci: izmjerena poza **`CARRY_V4`** ([[08_poze]]), gate gleda **referentnu** pozu s teme `/room_navigator/arm_posture` i **omotava kut (2π)**; brzine (ruke 0.2 → 0.7, vodilice, Nav2 0.30 → 0.45 uz sporo kroz vrata/dock, `vx_samples` 25 → 15, kraća kašnjenja); RViz pokreće `mission.launch.py` sam; ArUco **marker id 3** (ploča 0.40 m) umjesto crvenog X-a | build i `py_compile` prolaze; **nije voženo** | čeka M2 ponovno |
| 6 | 16. 9. / radno stablo | **M2+M3 u korisnikovom GUI-ju** s popravljenim gateom | ✅ **kutija je prošla kroz vrata u crvenu sobu** — `CARRY_V4` i referentni gate rade. ❌ RViz se **opet** nije pokrenuo; start i dalje spor; nakon `DETECTION_V4` robot **strafeao u stranu s već centrirane kocke**, pa su ruke radile nejednake korake | — | Centriranje je išlo po **ustajaloj** `center.y` iz dubinske kamere (izmjerena prije primicanja). Sada: svježe očitanje markera, mrtva zona 2 cm, i ispis reziduala nakon strafea |
| 7 | 16. 9. / radno stablo | Faza **odlaganja** (`_place_on_marker`): marker s docka → vodilice na visinu iznad ploče → kocka izgurana natrag 15 cm → primicanje po svježim očitanjima → vođeno spuštanje 1 cm/s → detach → jastučići s plohe → odmak → `PLACE VERIFIED`. Uz to marker povećan na **0.36 m** (quiet zone 5 %), jer bi ga kocka od 0.30 m inače potpuno prekrila | build i `py_compile` prolaze; **nije voženo** | čeka M4 |
| 8 | 16. 9. / radno stablo | **M2+M3+M4 u korisnikovom GUI-ju** | ✅ robot je **sam** odvezao kutiju u crvenu sobu (`arms: CARRY_V4 held` na oba prolaza) i stao pred stol. ❌ dalje se **ne zna što se dogodilo**: izlaz `main_task`-a nije nigdje spremljen (mapa runa ima samo `launch.log`), pa nema odgovora ni na pitanje je li marker viđen | nepoznato, pred stolom | `task.launch.py` sada pokreće `main_task` s `output='both'`. **Bez zapisa nema dijagnoze** — propust postava, ne runa |
| 9 | 16. 9. / radno stablo | Korisnikove primjedbe: pripremne poze predaleko, dock preblizu, centriranje pretjeruje, kutija nije ni u obrisu ni u RViz-u | `portal_standoff` 0.95 → **0.65**, `table_standoff` 1.15 → **0.95** (0.90 pada `check_zones`), `table_dock_safety` 0.10 → **0.15**; fina vožnja na zadnjim centimetrima; `/mission/carried_points` u obris; `/mission/cube_marker` u RViz | build prolazi; **nije voženo** | čeka run |
| 10 | 16. 9. / radno stablo, `log/run-mission.log` | Run sa zapisom: kutija nije u RViz-u, nije u obrisu, odlaganja nema | ❌ sve tri iz **istog uzroka**: `wrist cameras cannot see the cube they are holding: ['left','right']` → `cannot measure the held cube` (obris) i `Task aborted during: cannot measure the cube in the hands` (odlaganje). **Marker na stolu JE viđen**: `PLACE marker at base_link (0.915, -0.020, 0.672)` | P2 odlaganja | Kad jastučići stisnu kutiju, kamera na zapešću je **~2 cm** od markera i ne dekodira ga. Vezati pozu držane kutije na te kamere bila je moja greška pri izvedbi — plan je tražio da se pri attachu zapamti transformacija kutija↔alat |
| 11 | 16. 9. / radno stablo | Poza držane kutije iz **transformacije zapamćene pri attachu** (`_hold`, u okviru `left_tool_tip`), kamere se koriste samo **prije** hvata; marker kutije se crta 4×/s uz `frame_locked`; obris se osvježi i nakon izguravanja kutije | build i `py_compile` prolaze; **nije voženo** | čeka run |
| 12 | 16. 9. / radno stablo | Korisnikov redoslijed pred stolom: stop → dizanje na **visinu markera + 20 cm + pola kocke** → primicanje stolu **koliko robot može** → spuštanje. `place_clearance` 0.05 → **0.20**, novi `place_max_advance` **0.24** (iz izmjerenih 0.285 m zazora), ostatak do markera preuzimaju ruke | build i `py_compile` prolaze; offline geometrija: baza 24 cm + ruke ~4.5 cm, zazor do ruba ploče **4.5 cm**; **nije voženo** | — | Prva verzija s granicom 0.20 m ostavljala bi kocku 8.5 cm prekratko uz jednako toliko neiskorištenog zazora |
| 13 | 16. 9. / radno stablo | Korisnik: **prvo baza do kraja pa ruke** (ruke su u zraku, ne smetaju), i **prvo spustiti pa odspojiti**. Rano izguravanje od 15 cm maknuto; ruke sada pokrivaju cijeli ostatak u jednom potezu nakon vožnje. Dodan korak **P5b**: mjeri se donja ploha kutije i, ako je > 2 cm iznad ploče, kutija se ne pušta | build i `py_compile` prolaze; offline: baza 24 cm + ruke 19.5 cm, doseg 0.675 m (u hvatu 0.63 m); **nije voženo** | — | Stara verzija s granicom 0.20 m ostavljala bi kocku **8.5 cm** prekratko, a baza bi imala jednako toliko neiskorištenog zazora — granica je bila prestroga |
| 14 | 16. 9. / radno stablo | Korisnik zadao **cijeli slijed odlaganja u 11 koraka**. Prepisano po tome: korak 5 poravnava **robota** s markerom (strafe), korak 6 **ruke** dovode kutiju na centar **po x i y** (`_move_cube_by`, prije se moglo samo naprijed), koraci 9–11 su `DETECTION_V4` → odmak → `DRIVE_V4` | build i `py_compile` prolaze; **nije voženo** | — | Prije je korak 6 bio samo pomak naprijed, pa bočna greška hvata nije imala tko ispraviti |
| 15 | 16. 9. / radno stablo | **Napušteno:** vlastiti gradijentni vozač po potencijalnom polju umjesto Nav2 upravljača (Dijkstra navigacijska funkcija, nos prati gradijent, translacija + rotacija zajedno) | Offline je ruta radila (sve tri misijske rute stižu, razmak od zida 48–50 cm), ali **smjer titra**: 36–42 zaokreta > 30° po ruti, najgori 180°. Uzrok izmjeren: u vratima je slobodni koridor za **središte** robota samo **12 cm**, pa svaka ruka centralne razlike šira od toga uzorkuje u napuhanu jezgru. Zaglađivanje polja nije pomoglo | prekinuo korisnik | Korisnik: „previše se komplicira… vrati što je radilo". Vraćeno na Nav2; datoteke `field_driver.py` i `check_field_route.py` uklonjene, nalaz upisan u [[AGENT_GUIDE]] §5 |
| 16 | 16. 9. / radno stablo | Umjesto novog upravljača: **glađe gibanje na postojećem Nav2** — uzastopne dionice bez gatea idu kao **jedan** cilj (`navigate_through_poses`), prolaz i dock ostaju zasebni; `Twirling.scale` 20 → 2 da baza smije rotirati u vožnji | ruta `blue:dock → red:dock`: 7 dionica → **6 ciljeva**, od toga 1 kontinuiran; build i `py_compile` prolaze; **nije voženo** | — | Dock se namjerno **ne** spaja: poza kroz koju se samo prolazi ne slegne kut, a ondje je stol 15 cm od nosa ([[D-20_single_potential_field_costmap]]) |
| 17 | 16. 9. / radno stablo, `log/run-mission.log` | Run: odlaganje opet ne prolazi | ❌ **dva kvara, oba moja**. (1) Korak 4 javlja „kutija je 43.2 cm iza markera", baza prijeđe 24 cm, a korak 6 **opet** javlja 43.2 cm → marker izbliza **nije ponovno viđen** (plocha je pod nosom robota), pa je ostalo očitanje s docka; ruke dobiju zahtjev od 43 cm i IK pošteno odbije (`nema konvergencije`). (2) Spuštanje prekinuto **u istoj milisekundi**: `contact guard lost` — čuvar gleda senzore u jastučićima, a kutiju drži kruti spoj uz dodir od 2 mm, pa kontakta nema | korak 6, pa korak 7 | Sigurnosna provjera je **ispravno** odbila pustiti kutiju 20 cm iznad stola (`the cube is still 20.0 cm above the table`) |
| 18 | 16. 9. / radno stablo | Marker se čita **jednom, s docka**, pamti u `odom` i dalje računa iz odometrije (`_marker_in_base`); čuvar dodira pri spuštanju uklonjen, odluku donosi izmjerena visina nakon spuštanja | offline s brojkama iz loga: nakon 24 cm baze ostaje **19.3 cm naprijed, 0.0 cm bočno** (ruke su 19.5 cm već offline svladale); build i `py_compile` prolaze; **nije voženo** | — | Ustajalo očitanje izgleda isto kao svježe — zato se sada uopće ne pokušava ponovno čitati izbliza |
| 19 | 16. 9. / **run M4, GUI korisnika** | Cijela misija iz jedne naredbe | ✅ **`MISSION COMPLETE`**: baza 24.0 od 43.3 cm, ruke +19.6 / +1.3 cm → kutija +0.2 / +0.0 cm od centra, donja ploha **+4 mm** od ploče, **`PLACE VERIFIED: 5 mm`** (dx −4, dy +3 mm) | — | **Riješeno.** Korisnik: „super. radi." |

## Trenutno rješenje
- `main_task.py`: parametri `mission` / `pick_room` / `place_room` / `goto_timeout`; `_wait_for_start`
  (čeka korisnika) i `_goto` (vozi preko `room_navigator`-a). `_goto` prihvaća `arrived` **tek nakon**
  što je za isti cilj vidio `driving` — status je latched, pa bi stara poruka inače prošla kao dolazak.
- `launch/mission.launch.py`: `sim` (`carry_arms:=false`, spawn (0,0,0) = AMCL `initial_pose`) +
  `nav2` (lokalizacija, RViz, panel) + `task` (`auto_start:=true`, `mission:=true`).
- `nav_gui.py`: gumb **„MISIJA: po kutiju"** objavljuje na `/mission/start`.
- Upute i kriteriji: [[misija]].

## Sljedeći korak
1. **M1** — vožnja u `DRIVE_V4`, prazan robot, bez izmjena koda: `blue:dock` → `red` → `home`.
   Kriterij: bez aborta na oba prolaza, `arms: DRIVE_V4 held`, bez dodira stola na docku.
2. **M2** — `mission.launch.py` do `TASK COMPLETE` (hvat), uz korisnikovu GUI potvrdu.
3. Tek onda M3 (nošenje, gate ruku s referencom) i M4 (odlaganje na marker).

## Ne ponavljati
- Sirov cilj na `/navigate_to_pose` za misijske dionice — nema portala ni docka ([[P-39_nav2_enters_doorway_at_an_angle]]).
- Popuštanje `arm_tolerance`/`min_side_clearance` da bi nošenje „prošlo" gate — prag bi sakrio
  stvarnu širinu; mjeri se obris ([[D-19_dynamic_footprint]]).
- Slaganje M2–M4 jedan na drugi bez runa između ([[D-18_verified_baseline_first]]).
