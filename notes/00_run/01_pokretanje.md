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

Nakon što je `seminar_map.yaml` spremljena i instalirana, navigacija se pokreće u čistoj sesiji (**bez** `mapping.launch.py`):

1. **Terminal 1** (Simulacija):
   ```bash
   ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
   ```
2. **Terminal 2** (MoveIt):
   ```bash
   ./scripts/run_native.sh ros2 launch pas_dual_arm_moveit_config move_group.launch.py
   ```
3. **Terminal 3** (Sklapanje ruku u `ARM_CARRY_V2` prije vožnje):
   ```bash
   ./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2
   ```
4. **Terminal 4** (Nav2 stog u načinu lokalizacije + RViz):
   ```bash
   ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup nav2.launch.py mode:=localization
   ```

AMCL automatski inicijalizira početnu pozu na `(0, 0, yaw=0)` (gdje se robot spawna u HOME sobi).
U RViz-u se prikazuje karta, costmap i čestice lokalizacije. Za slanje navigacijskog cilja:
- **Preko RViz-a**: Klikni alat **"2D Goal Pose"** i postavi cilj u plavoj sobi ispred stola (npr. oko `x=0.0, y=-5.0`, usmjeren prema `-Y`).
- **Ili preko CLI-ja**:
  ```bash
  ./scripts/run_native.sh ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -5.0, z: 0.0}, orientation: {z: -0.7071, w: 0.7071}}}}"
  ```


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
