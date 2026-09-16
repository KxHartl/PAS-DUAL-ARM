# Kako pokrenuti simulaciju i misiju

Instalacija i dohvat izvora su u [`README.md`](README.md). Ovdje su samo naredbe za rad.
Detaljni postupci po podsustavu (što gledati, što zabilježiti) su u `notes/00_run/00_testing/`:
[`misija.md`](notes/00_run/00_testing/misija.md), [`navigacija.md`](notes/00_run/00_testing/navigacija.md),
[`hvat_kocke.md`](notes/00_run/00_testing/hvat_kocke.md).

## 0. Projektno okruženje (jednom po terminalu)

```bash
cd ~/FSB/PAS-DUAL-ARM
./scripts/run_native.sh
```

Otvara izolirani PAS-DUAL-ARM shell (Fast DDS, ROS domena 5, lokalno otkrivanje). Svaki terminal
koji sudjeluje u istom ROS grafu otvori na isti način; prije prelaska u drugi ROS projekt izađi s
`exit` i ne učitavaj njegov `setup.bash` u isti shell.

Nakon izmjene koda, xacro/URDF-a, svijeta ili configa:

```bash
./scripts/run_native.sh colcon build --symlink-install
# brže, samo ono što si dirao:
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
```

> **Nikad dvije simulacije odjednom.** Oba Gazeba objavljuju `/clock`, vrijeme skače naprijed-natrag
> i RViz pukne s `Cannot create GL vertex buffer`, a iz simptoma se uzrok ne vidi. Prije pokretanja:
> `bash scripts/clean_ros.sh`.

## 1. Cijela misija iz jedne naredbe

```bash
bash scripts/clean_ros.sh
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py |& tee log/run-mission.log
```

Kad u logu piše `WAITING for the user`, pritisni **„MISIJA: po kutiju"** na panelu (ili
`ros2 topic pub --once /mission/start std_msgs/String "{data: blue}"`). Dalje ide samo:
plava soba → hvat → kroz vrata → crvena soba → odlaganje na marker → `MISSION COMPLETE`.

Što gledati u Gazebu i koji se redci očekuju u logu: [`misija.md`](notes/00_run/00_testing/misija.md).

## 2. Po dijelovima (kad se nešto razvija)

**Terminal 1 — simulacija**
```bash
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
```
Pričekaj da se aktivira svih osam kontrolera. Bez GUI-ja: `headless:=true`.

**Terminal 2 — navigacija** (karta + AMCL + Nav2 + zone + panel)
```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup nav2.launch.py 2>&1 | tee /tmp/pas/t2.log
```

**Terminal 3 — vožnja po sobama**
```bash
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: blue:dock}"
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: red}"
```

**Samo hvat** (robot se stvori na dock pozi, bez vožnje): postupak u
[`hvat_kocke.md`](notes/00_run/00_testing/hvat_kocke.md).

**Mapiranje** (zaseban korak; misija vozi po spremljenoj karti iz repoa). Cijeli postupak —
pravila vožnje, ruta, gate-ovi — je u [`MAPPING.md`](MAPPING.md); ukratko:
```bash
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mapping.launch.py
./scripts/run_native.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
./scripts/save_map.sh moja_tura
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_bringup
```

## 3. Provjere bez simulatora

```bash
./scripts/run_native.sh bash scripts/verify_environment.sh   # 17/17
./scripts/run_native.sh python3 scripts/check_doors.py       # vrata iz karte
./scripts/run_native.sh python3 scripts/check_zones.py       # zone, portali, dock poze
./scripts/run_native.sh python3 scripts/check_map.py \
    src/pas_dual_arm_bringup/maps/seminar_map.yaml           # kvaliteta karte (traži putanju)
```
Ako `check_*` ne prođu, **ne pokreći simulaciju** — zone su krive i vožnja nema smisla.

## 4. Gdje su logovi

`ros2 launch` piše svakom čvoru zaseban log u `~/.ros/log/<čvor>_<pid>_*.log`, a `tee` iz naredbi
gore daje jednu datoteku sa svime isprepletenim. Nakon runa najkorisnije:

```bash
grep -hE "PLACE|NAV:|MISSION|Task aborted|ABORT" log/run-mission.log | tail -30
ls -t ~/.ros/log/controller_server_*.log | head -1 | xargs grep -cE "WARN|ERROR"
```

## 5. Poznati problemi

- **`colcon` javlja override `realsense2_description`** — namjerno: cijeli `realsense-ros` dolazi iz
  izvora (v4.57.6) radi usklađenosti s driverom. Upozorenje je benigno.
- **`gazebo_version is not defined`** — ispravljeno u `robot.urdf.xacro`; namjerno se preskače
  `omni_base.urdf.xacro` jer vuče Gazebo Classic ros2_control. Ako se vrati nakon `apt upgrade`
  paketa `ros-humble-pal-urdf-utils`, provjeri je li svojstvo definirano prije include-a.
- **Middleware i domena** su postavljeni isključivo u `scripts/run_native.sh`; ne postavljaj ROS
  varijable, router ni adresu robota globalno u `~/.bashrc`.
- **Kamere na zapešćima ne vide markere dok jastučići drže kutiju** (2 cm od markera). Poza držane
  kutije zato dolazi iz transformacije zapamćene pri attachu — vidi `notes/03_problemi/P-45…`.

Status svih zahtjeva: [`notes/00_MAPA.md`](notes/00_MAPA.md). Svjesna odstupanja:
[`notes/07_predaja/odstupanja.md`](notes/07_predaja/odstupanja.md).
