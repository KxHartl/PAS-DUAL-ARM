---
id: TEST_NAVIGACIJA
type: upute
updated: 2026-09-14
---
# Navigacija — pokretanje i testiranje

> Svaki test ima isti oblik: **prvo naredbe za copy-paste, po terminalima**, pa ispod
> „Što gledati" i „Što treba znati". Ne treba skakati po dokumentu — sve za jedan test je
> na jednom mjestu.
>
> Povezano: [[01_pokretanje]] (opće pokretanje) · [[02_testiranje]] (kriteriji prihvata) ·
> [[D-18_verified_baseline_first]] (jedan inkrement = jedan run).

---

## T0 — Start (uvijek ovim redom)

**Terminal 1**
```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
./scripts/run_native.sh python3 scripts/check_doors.py
./scripts/run_native.sh python3 scripts/check_zones.py
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
```

**Terminal 2** (čekaj da T1 javi `Configured and activated` za sve kontrolere)
```bash
mkdir -p /tmp/pas
cd /home/khartl/FSB/PAS-DUAL-ARM
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup nav2.launch.py 2>&1 | tee /tmp/pas/t2.log
```

**Terminal 3** — ostaje slobodan za naredbe iz testova ispod.

### Što gledati
Oba `check_*` moraju završiti s **`PASS`**. Ako ne prođu, **ne pokretati simulaciju** — zone
su krive i vožnja nema smisla.

### Što treba znati
- `clean_ros.sh` **nije opcionalan**: živ čvor iz prethodne sesije ostane `active`, novi
  `lifecycle_manager` ga ne može konfigurirati i **prekine cijeli bringup** — karta i costmap
  se onda ne pojave, a izgleda kao da ništa ne radi ([[P-39_nav2_enters_doorway_at_an_angle]] #8).
- `tee` na T2 treba jer `output='both'` hvata samo **naše** čvorove (`footprint_publisher`,
  `cmd_vel_relay`, `room_navigator`, `nav_zones`, `collision_monitor`) u `~/.ros/log/`.
  Nav2-ovi (`controller_server`, `planner_server`, `bt_navigator`) dolaze iz
  `navigation_launch.py` sa svojim `output='screen'` i u `launch.log` ostave samo
  `process started`.
- Argumenti za T2: `gui:=false`, `rviz:=false`, `zones:=false` (A/B bez zona),
  `safety:=false` (bez `collision_monitor`).

---

## T1 — Prikaz u RViz-u

Nema naredbi. Pogledaj popis prikaza čim se RViz otvori.

### Što gledati
Sve mora biti vidljivo **odmah, bez reseta i bez ručnog dodavanja**:

- **robot** (`RobotModel`) — ne samo TF osi;
- **zelena traka** kroz oba prolaza, 0.50 m sa svake strane;
- **crveni lijevci** lijevo i desno od trake;
- **narančasti halo** oko oba stola, otvoren prema vratima;
- **plave strelice** = portalne poze, 1.45 m ispred i iza svakih vrata, okomite na zid;
- **linija po rubu robota** = footprint;
- **žuti klin ispred robota** = `collision_monitor`, footprint projiciran unaprijed.

### Što treba znati
- Ako prikaz nedostaje, to je **greška u konfiguraciji**, ne nešto što treba dodati rukom.
  Najčešći uzrok: prikaz je na `Volatile`, a naši izdavači objave **jednom**, latched — pa
  `Volatile` pretplatnik ne dobije ništa. Treba `Durability Policy: Transient Local`.
- **Zeleni footprint nećeš vidjeti zasebno.** Isti poligon ide na oba costmapa, pa
  `Footprint (lokalni)` (zelen) i `Footprint (globalni)` (narančast) leže jedan na drugome i
  vidi se samo onaj koji se crta zadnji. Narančasta linija po rubu robota **jest** footprint.

---

## T2 — Dinamični footprint

**Terminal 3**
```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_HOME
./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2
```

Provjera brojke iz drugog kuta:
```bash
./scripts/run_native.sh ros2 topic echo /local_costmap/published_footprint --once
```

### Što gledati
Poligon oko robota se **raširi** pa **stisne**. U T1 istovremeno ide redak
`footprint now X m long, Y m wide, N vertices`.

Izmjereno (run 48, [[runovi]]) — s ovime uspoređuješ:

| Poza | Širina | Duljina |
|---|---|---|
| `ARM_CARRY_V2` | **0.861 m** | 1.040 m |
| tijekom zamaha (max) | **1.428 m** | 1.349 m |

### Što treba znati
- Brojke uključuju `footprint_padding` 0.01 (+0.02 po osi). Bez njega je `ARM_HOME` 1.408 m,
  što se poklapa s [[P-35_arm_span_too_wide_for_door]] (1.41 m, druga metoda).
- **Prije vožnje vrati ruke u `ARM_CARRY_V2`.** Sa širokim rukama `room_navigator` odbija
  prolaz kroz vrata — to je ispravno, robot je tada 1.41 m širok, a vrata su 0.95 m.
- Ako `echo` vrati **točno 4 točke na `±0.53, ±0.437`**, to je statični YAML poligon + padding,
  dakle `footprint_publisher` ne radi:
  ```bash
  ps -eo pid,cmd | grep footprint_publisher | grep -v grep
  ```
- Costmap **pamti zadnji primljeni footprint**. Ako čvor umre, poligon ostane zamrznut na
  zadnjoj vrijednosti i **ne** vraća se na YAML — mrtav čvor izgleda kao živ.

---

## T3 — Collision monitor

Nema naredbi. U RViz-u zadaj cilj **„2D Goal Pose"** prema zidu ili stolu.

### Što gledati
**Usporavanje prije zaustavljanja**, i **nijedan dodir**. Zatim ponovi referentnu rutu (T4)
da se potvrdi da monitor ne koči normalnu vožnju.

### Što treba znati
- Za usporedbu bez monitora: `safety:=false` na T2.
- Kad monitor preuzme, relay javi `collision monitor is live; raw /cmd_vel is now ignored`.
  Dok je živ, sirovi `/cmd_vel` se ignorira i ne može ga se zaobići.

---

## T4 — Referentna ruta (uvijek ista, da su brojevi usporedivi)

**U RViz-u**, alat **„2D Goal Pose"**, iz Home `(0, 0, 0°)` na **`(0.0, −4.5, −90°)`**.

**Terminal 3** (ili tipka **CRVENA soba** na panelu)
```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: red}"
```

Ostale tipke: **PLAVA soba**, **HOME**, **STOP**.

### Što gledati
| Korak | Očekivano |
|---|---|
| ručni cilj → plava soba | **~33 s** |
| `goto red` | **5 dionica**, pred crvenim stolom za **~60 s** |
| oboje | bez dodira zida, bez aborta |

To je run 46 iz [[runovi]] — stanje koje je dokazano radilo.

### Što treba znati

> [!warning] U RViz-u postoje **dva** alata za cilj i ne rade isto
> - **„2D Goal Pose"** (`rviz_default_plugins/SetGoal`) objavi pozu na `/goal_pose`, odakle je
>   preuzme `room_navigator` i razloži na portalne poze — **ovo je normalan način**.
> - **„Nav2 Goal"** (`nav2_rviz_plugins/GoalTool`) šalje **izravno akciju** `navigate_to_pose`
>   i **zaobilazi `room_navigator`**: nema portalnih poza, planer vuče dijagonalu i robot uđe
>   u vrata ukoso.
>
> Provjera je li cilj otišao kamo treba:
> ```bash
> grep "room_navigator\]" /tmp/pas/t2.log | tail -5
> ```
> Ako ondje nema ničega osim `zone graph:`, cilj je otišao mimo navigatora.

---

## T5 — Što zabilježiti nakon svakog runa

**Terminal 3**
```bash
grep -hE "footprint now|alignment:|arms:|tightest|aborted|ABORT|Recover" /tmp/pas/t2.log ~/.ros/log/*/launch.log
```

Upisati u [[runovi]] i u pripadnu P-karticu — **i kad ne uspije**.

| Mjerilo | Gdje se vidi |
|---|---|
| vrijeme po dionici i ukupno | log `room_navigator`-a |
| dolazna poza | `arrived N cm and ±N deg from the goal` |
| kut i otklon na pragu vrata | redak `alignment:` prije prolaza |
| najmanji bočni razmak | log `room_navigator`-a |
| najgora izmjerena širina | redak `footprint now ...` |
| ima li dodira sa zidom/stolom | vizualno u Gazebu |

### Što treba znati
- `NavigateToPose` koji javi uspjeh **nije dokaz** — mjeri se izmjerena poza.
- `~/.ros/log` raste (već ~120 MB). Povremeno:
  ```bash
  find ~/.ros/log -maxdepth 1 -type d -mtime +7 -exec rm -rf {} +
  ```

---

## Kad odbije voziti

`room_navigator` prije prolaza provjerava ruke i poravnatost i **pošteno stane**
([[D-12_honesty_abort_over_fake]]):

| Poruka | Značenje | Što učiniti |
|---|---|---|
| `arms: … off ARM_CARRY_V2` | ruke su se raširile ([[P-37_arm_position_gain_sag]]) | ponovi `set_posture ARM_CARRY_V2` |
| `alignment: … off the lane centreline` | AMCL netočan ili je prethodna dionica podbacila | pošalji robota malo unazad i ponovi |
| `No valid trajectories out of N!` (DWB) | u costmapu **nema** prolaza dovoljno širokog za footprint — nije greška upravljača | izmjeri stvarnu širinu koju costmap vidi (dolje) |

Kad DWB javi da nema valjanih trajektorija, izmjeri koliko je slobodno **u costmapu**, ne u svijetu:

```bash
./scripts/run_native.sh ros2 topic echo /local_costmap/published_footprint --once   # sirina robota
./scripts/run_native.sh ros2 topic echo /local_costmap/costmap --once > /tmp/pas/cm.txt  # pa izmjeri prolaz
```
14. 9. je tako nađeno: otvor 100 cm, costmap pokazuje 80 cm, robot 86.2 cm → prolaz ne postoji.
Uzrok su bili `footprint_padding` i rezolucija od 5 cm, oboje popravljeno.
| `only X cm beside the robot` | prekid usred prolaza, razmak ispod praga | provjeri footprint i pozu ruku |
