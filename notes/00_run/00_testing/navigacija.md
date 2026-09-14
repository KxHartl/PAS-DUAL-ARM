---
id: TEST_NAVIGACIJA
type: upute
updated: 2026-09-14
---
# Navigacija — pokretanje i testiranje

> Puni postupak za testiranje navigacije: što pokrenuti, u kojem terminalu, što gledati i
> što zabilježiti. Ovo je jedino mjesto gdje ti koraci žive — ne traži ih po chatu.
> Povezano: [[01_pokretanje]] (opće pokretanje), [[02_testiranje]] (kriteriji prihvata),
> [[D-18_verified_baseline_first]] (pravilo: jedan inkrement = jedan run).

## 0. Prije svakog starta

```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh
```

**Ovo nije opcionalno.** Živ čvor istog imena iz prethodne sesije ostane u stanju `active`,
novi `lifecycle_manager` ga ne može konfigurirati (`No transition matching 1 found`) i
**prekine cijeli bringup** — karta i costmap se onda ne pojave, a izgleda kao da ništa ne
radi ([[P-39_nav2_enters_doorway_at_an_angle]], pokušaj 8).

Nakon izmjena koda ili konfiguracije:

```bash
./scripts/run_native.sh colcon build --symlink-install \
    --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
```

## 1. Provjere bez simulatora

Brze su i hvataju geometrijske greške prije nego se išta pokrene.

```bash
./scripts/run_native.sh python3 scripts/check_doors.py     # oboja vrata, sirina 0.85-1.05 m
./scripts/run_native.sh python3 scripts/check_zones.py     # 3 sobe, 2 vrata, 2 stola, okret na svakoj pozi
./scripts/run_native.sh python3 -m pytest -q src/pas_dual_arm_scripts/test/
```

Oba `check_*` moraju završiti s `PASS`. Ako ne prođu, **ne pokretati simulaciju** — zone su
krive i vožnja nema smisla.

## 2. Terminali

| # | Naredba | Što diže |
|---|---|---|
| **T1** | `PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py` | Gazebo, kontroleri, `cmd_vel_relay`, `scan_filter`, **`footprint_publisher`**; ruke se spawnaju u `ARM_CARRY_V2` |
| **T2** | `./scripts/run_native.sh ros2 launch pas_dual_arm_bringup nav2.launch.py` | AMCL + karta, `nav_zones`, `feature_registry`, `room_navigator`, **`collision_monitor`**, Nav2, RViz, panel s tipkama |
| **T3** | po potrebi, za poze ruku (v. dolje) | — |

Pričekaj u T1 da svi kontroleri jave `Configured and activated` prije nego pokreneš T2.

**Argumenti za T2:** `gui:=false` (bez panela), `rviz:=false`, `zones:=false` (A/B bez zona),
`safety:=false` (bez `collision_monitor`; tada sirovi `/cmd_vel` opet izravno vozi bazu).

## 3. Što se mora vidjeti u RViz-u odmah, bez reseta

Sve je unaprijed upisano u `rviz/nav2.rviz`. **Ništa se ne dodaje ručno.** Ako nešto
nedostaje, to je greška u konfiguraciji, ne nešto što treba dodati rukom.

- **robot** (`RobotModel`) — ne samo TF osi;
- **zelena traka** kroz oba prolaza, 0.50 m sa svake strane;
- **crveni lijevci** lijevo i desno od trake;
- **narančasti halo** oko oba stola, otvoren prema vratima;
- **plave strelice** = portalne poze, 1.45 m ispred i iza svakih vrata, okomite na zid;
- **zeleni poligon** oko robota = `Footprint (lokalni)`;
- **žuta/narančasta** = zone `collision_monitor`-a.

> Ako se prikaz ne pojavi, provjeri da mu je `Durability Policy: Transient Local` — naši
> izdavači objave **jednom**, latched, pa `Volatile` pretplatnik ne dobije ništa.

## 4. Test: dinamični footprint (Korak 1)

Ovo je cijeli test u jednoj provjeri i **ne treba vožnju**.

U T3:
```bash
./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_HOME       # siroka poza, 1.41 m
./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2   # natrag u usku, 0.854 m
```
MoveIt nije potreban — `set_posture` pada na izravno slanje JTC kontrolerima.

**Prihvat:** zeleni poligon u RViz-u se **raširi** pa **stisne**; u T1 se pojavi redak
`footprint now X m long, Y m wide, N vertices`. U `ARM_CARRY_V2` očekuj ≈ **1.02 × 0.84 m**.

Provjera s naredbenog retka:
```bash
./scripts/run_native.sh ros2 topic echo /local_costmap/published_footprint --once
```
Ako dobiješ točno 4 točke na `±0.53, ±0.437`, to je **statični YAML poligon + padding** —
znači `footprint_publisher` ne radi. Provjeri je li živ:
```bash
ps -eo pid,cmd | grep -E "footprint_publisher|collision_monitor" | grep -v grep
```

> Costmap **pamti zadnji primljeni footprint**. Ako čvor umre, poligon ostane zamrznut na
> zadnjoj vrijednosti i ne vraća se na YAML — nemoj to čitati kao da radi.

**Prije vožnje vrati ruke u `ARM_CARRY_V2`.** Sa širokim rukama `room_navigator` će odbiti
prolaz kroz vrata — to je ispravno ponašanje, ne kvar.

## 5. Test: collision monitor (Korak 2)

Vozi robota prema prepreci (RViz „2D Goal Pose" na zid ili stol).

**Prihvat:** vidi se **usporavanje prije** zaustavljanja, i **nijedan dodir**. Zatim ponovi
referentnu rutu (dolje) da se potvrdi da monitor ne koči normalnu vožnju.

`safety:=false` u T2 isključuje monitor, za usporedbu.

## 6. Referentna ruta (uvijek ista, da su brojevi usporedivi)

Ovo je run 46 iz [[runovi]] — stanje koje je dokazano radilo.

1. **Ručni cilj:** RViz „2D Goal Pose" iz Home `(0, 0, 0°)` na **`(0.0, −4.5, −90°)`**.
   Očekivano: plava soba za **~33 s**.
2. **Autonomno:** tipka **CRVENA soba** u panelu, ili
   ```bash
   ./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: red}"
   ```
   Očekivano: **5 dionica**, pred crvenim stolom za **~60 s**.

Ostale tipke: **PLAVA soba**, **HOME**, **STOP**.

## 7. Što zabilježiti nakon svakog runa

Upisati u [[runovi]] i u pripadnu P-karticu — **i kad ne uspije**.

| Mjerilo | Gdje se vidi |
|---|---|
| vrijeme po dionici i ukupno | log `room_navigator`-a |
| dolazna poza | `arrived N cm and ±N deg from the goal` |
| kut i otklon na pragu vrata | redak `alignment:` prije prolaza |
| najmanji bočni razmak | log `room_navigator`-a |
| najgora izmjerena širina | redak `footprint now ...` u T1 |
| ima li dodira sa zidom/stolom | vizualno u Gazebu |

`NavigateToPose` koji javi uspjeh **nije dokaz** — mjeri se izmjerena poza.

## 8. Kad odbije voziti

`room_navigator` prije prolaza provjerava ruke i poravnatost i **pošteno stane**
([[D-12_honesty_abort_over_fake]]):

- `arms: left_joint_6 is … off ARM_CARRY_V2` → ruke su se raširile
  ([[P-37_arm_position_gain_sag]]); ponovi `set_posture ARM_CARRY_V2`.
- `alignment: … off the lane centreline` → AMCL nije dovoljno točan ili je prethodna dionica
  podbacila; pošalji robota malo unazad i ponovi.
- `only X cm beside the robot` → prekid usred prolaza, razmak pao ispod praga.
