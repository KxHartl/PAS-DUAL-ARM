---
id: RUN_POKRETANJE
type: upute
updated: 2026-09-13
---
# Pokretanje

Sve naredbe izvršavaj iz korijena repozitorija. **Svaki terminal** treba projektni
ROS okoliš; najjednostavnije je ispred svake naredbe staviti `./scripts/run_native.sh`.
Ne učitavati drugi ROS workspace u isti shell.

## 1. Build

```bash
cd /home/khartl/FSB/PAS-DUAL-ARM
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup pas_dual_arm_moveit_config
```

Za puni build iz čistog stanja: `./scripts/run_native.sh colcon build --symlink-install`.
Nakon izmjene launch/config/xacro datoteka napravi rebuild pogođenog paketa i
ponovno pokreni simulaciju. `--symlink-install` ne znači da se sve instalirane
konfiguracije automatski osvježe.

## 2. Simulacija — terminal 1

Prije pokretanja možeš provjeriti projektni okoliš bez pokretanja robota:

```bash
bash scripts/verify_environment.sh
```

Skripta sama ulazi kroz `run_native.sh` i zato ne nasljeđuje BATRACS ni drugi
ROS overlay. Nakon podizanja simulacije, `bash scripts/verify_environment.sh --live`
provjerava dolaze li `/clock`, `/joint_states`, `/scan_filtered` i odometrija.

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
```

Za test bez GUI-ja dodaj `headless:=true`. Pričekaj aktivaciju kontrolera.
Robot se pojavi sa širokim početnim položajem ruku; **ne šalji ga kroz vrata**.

## 3. Mapiranje — terminal 2

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mapping.launch.py
```

Ovo podiže `slam_toolbox`, `feature_registry` i RViz. Za headless test dodaj
`rviz:=false`. Simulacija već podiže `scan_filter` i relay za `/cmd_vel`.
Provjeri `/scan_filtered`, `/map` i TF `map → odom → base_link` prije vožnje.

Automatska tura traži i MoveIt (terminal 3):

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_moveit_config move_group.launch.py
```

### Opcija A: Automatska tura (terminal 4)

```bash
./scripts/run_native.sh ros2 run pas_dual_arm_scripts mapping_tour
```

Tura prvo pokušava postaviti `ARM_CARRY_V2`, zadržati ruke i provjeriti stvarne
zglobove. **Očekuje se sigurnosni prekid** ako odstupanje premaši 0.10 rad.
Zbog male tolerancije prolaza (vrata 1.0 m, robot 85.4 cm), otvorena petlja
odometrije može zapeti na štok vrata.

### Opcija B: Ručno mapiranje (preporučeno za preciznost)

1. U **terminalu 4** postavi ruke u usku pozu `ARM_CARRY_V2`:
   ```bash
   ./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2
   ```
2. U **terminalu 5** pokreni teleoperaciju baze:
   ```bash
   ./scripts/run_native.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. **Pravila vožnje (invarijanta trenja `mu1=0.4, mu2=0.2`)**:
   - Uvijek **zaustaviti robot prije skretanja** (rotirati na mjestu, pa voziti ravno; ne voziti u luku).
   - Smanjiti brzine tipkama `x` (linearna na ~0.2 m/s) i `c` (kutna na ~0.3 rad/s).
   - Voziti: HOME → PLAVA soba (krug) → povratak u HOME (loop closure) → CRVENA soba (krug) → povratak u HOME.


## 4. Spremanje karte — tek nakon uspješne, provjerene ture

Spremanje se uvijek vrši u projektni direktorij `src/pas_dual_arm_bringup/maps/` neovisno o tome iz koje mape pokreneš skriptu.

```bash
# Primjer s vlastitom oznakom testa (npr. run43 ili teleop_test):
./scripts/save_map.sh run43
```

Skripta automatski:
1. Sprema trajnu arhivsku kopiju s datumom i vremenom: `maps/map_YYYYMMDD_HHMMSS_<tag>.{yaml,pgm,posegraph,data}`
2. Ažurira aktivnu kartu `maps/seminar_map.{yaml,pgm,posegraph,data}` kako bi Nav2 odmah radio sa zadanom kartom.
3. Pokreće `check_map.py` koji provjerava pokrivenost (>9×9 m raspon, >55 m² slobodnog prostora za sve tri sobe).

Nakon spremanja napravi rebuild paketa kako bi `install/` vidio novu kartu:

```bash
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_bringup
```

## 5. Lokalizacija i Nav2 navigacija

Nakon što je `seminar_map.yaml` spremljena i instalirana, navigacija se pokreće u čistoj sesiji
(**bez** `mapping.launch.py`). Za vožnju praznog robota trebaju **dva terminala**:

> [!warning] Prvo provjeri da nema zaostalih čvorova iz prethodne sesije
> Živ čvor istog imena iz starog runa ostane u stanju `active`; novi `lifecycle_manager` ga
> pokuša konfigurirati, dobije `No transition matching 1 found for current state active` i
> **prekine cijeli bringup** (karta i costmap se onda ne pojave). Provjera i čišćenje:
> ```bash
> ps -eo pid,comm,etimes | grep -E 'velocity_smoo|smoother_ser|controller_se|planner_serv|bt_navigat|behavior_ser|waypoint_fol|lifecycle_ma|map_server|amcl|nav_zones|room_navigat|ign'
> ```
> Ako išta izlista, ugasi po PID-u. (`comm` je skraćen na 15 znakova, pa `velocity_smoother`
> izgleda kao `velocity_smooth` — zato je u obrascu baš tako.)

1. **Terminal 1** (simulacija; ruke se spawnaju odmah u `ARM_CARRY_V2`):
   ```bash
   PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
   ```
2. **Terminal 2** (Nav2 + zone + RViz + tipke; `mode:=localization` je zadano):
   ```bash
   ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup nav2.launch.py
   ```

Argumenti: `gui:=false` (bez tipaka), `rviz:=false`, `zones:=false` (A/B bez zona).

MoveIt i `set_posture` trebaju samo ako ruke treba **prepozirati** tijekom rada; za vožnju
praznog robota ne trebaju, jer ih JTC drži u pozi u kojoj su spawnane (tako je radio i run 44):
```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_moveit_config move_group.launch.py
./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2
```

AMCL automatski inicijalizira početnu pozu na `(0, 0, yaw=0)` (gdje se robot spawna u HOME sobi).

### Što provjeriti u RViz-u prije vožnje
Dodaj `MarkerArray` na **`/nav_zones_markers`**. Mora se vidjeti:
- **zelena traka** točno kroz oba prolaza, duga 1.2 m sa svake strane;
- **crveni lijevci** lijevo i desno od svake trake;
- **narančasti halo** oko oba stola, **otvoren prema vratima sobe** (U-oblik);
- **plave strelice** = portalne poze (1.95 m ispred i iza svakih vrata), sve okomite na zid;
- **žuta strelica** = prilazna poza ispred stola.

Ako se to ne vidi, `nav_zones` nije našao vrata — ne voziti. Provjeri offline:
```bash
./scripts/run_native.sh python3 scripts/check_zones.py
```

### Dva načina slanja robota
- **Ručno (Nav2 izravno)**: alat **„2D Goal Pose"** u RViz-u. Zone vrijede i tu — putanja
  ulazi u prolaz okomito čak i kad je cilj u drugoj sobi.
- **Tipkom (panel `nav_gui`, diže ga `nav2.launch.py`)**: **PLAVA soba** / **CRVENA soba** vode robota pred stol te sobe,
  **HOME** u sredinu polazne sobe, **STOP** prekida. Isto bez GUI-ja:
  ```bash
  ./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: blue}"
  ```
  (isto vrijedi bez tipaka, ako si pokrenuo `nav2.launch.py gui:=false`.)
  Ruta je uvijek: portal ispred vrata → **ravno kroz vrata** → (kut, ako treba) → pred stol.

### Kad odbije voziti
`room_navigator` prije svakog prolaza provjerava ruke i poravnatost i **pošteno stane**
([[D-12_honesty_abort_over_fake]]). Poruke koje se mogu vidjeti:
- `arms: left_joint_6 is … off ARM_CARRY_V2` → ruke su se raširile ([[P-37_arm_position_gain_sag]]);
  ponoviti `set_posture ARM_CARRY_V2`. Robot širi od ~0.92 m ne stane kroz 0.95 m otvora.
- `alignment: … off the lane centreline` → AMCL nije dovoljno točan ili je prethodna dionica
  podbacila; poslati robota malo unazad i ponoviti.
- `only X cm beside the robot` → prekid usred prolaza, razmak pao ispod 3 cm.


## 6. Zadatak — eksperimentalno

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup task.launch.py auto_start:=false
```

To podiže MoveIt i ArUco bez automatskog `main_task`. Tek nakon valjane karte i
lokalizacije može se probati `auto_start:=true navigate_region:=true` uz
`region_x`, `region_y`, `region_yaw`. Zadani cilj je plava soba, ali dolazak,
hvat i transport u novom svijetu nisu potvrđeni; ne predstavljati to kao demo.

Detaljniji tehnički plan: `docs/MAPPING_LOCALIZATION.md`. Stariji `RUNNING.md`
opisuje pretežno prethodni zadatak i nije izvor za aktualni SLAM tijek.
