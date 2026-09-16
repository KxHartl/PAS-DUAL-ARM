# PAS-DUAL-ARM — simulacijski model dual-arm robota (ROS 2 Humble + Gazebo Fortress)

Simulacijski model mobilnog robota s **dvije Kinova Gen3 ruke** na **vertikalnim linearnim
vodilicama**, **omnidirekcijskom bazom** i **pan-tilt kamerom**, te cijela misija u jednoj naredbi:

> robot mapira prostor → korisnik ga pošalje u sobu s kutijom → nađe kutiju → **podigne je objema
> rukama** → pronese je kroz vrata → **odloži je na označeno mjesto** u drugoj sobi.

Zadnji potvrđeni run (GUI, 16. 9. 2026.): kutija spuštena **4 mm** iznad ploče i sjela **5 mm** od
centra markera — `PLACE VERIFIED`, `MISSION COMPLETE`.

---

## 1. Preduvjeti

| | |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Simulator | Gazebo **Fortress** (LTS) + `ros-humble-ros-gz` |
| Ostalo | `python3-vcstool`, `python3-rosdep`, `colcon` |

```bash
sudo apt update
sudo apt install ros-humble-desktop ignition-fortress ros-humble-ros-gz \
                 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-moveit \
                 ros-humble-ros2-control ros-humble-ros2-controllers \
                 python3-vcstool python3-rosdep python3-colcon-common-extensions
```

## 2. Dohvat

Pet paketa u `src/` su vanjski repozitoriji i **ne dolaze** s `git clone`; njihovi točni commitovi
pinani su u `ros2.repos`, pa je radni prostor reproducibilan.

```bash
git clone git@github.com:KxHartl/PAS-DUAL-ARM.git
cd PAS-DUAL-ARM

vcs import src < ros2.repos     # aruco_ros, omni_base_simulation, pan_tilt_ros,
                                # realsense-ros, ros2_kortex — na pinane commitove
./scripts/apply_patches.sh      # lokalne zakrpe iz patches/ (idempotentno)

rosdep install --from-paths src --ignore-src -y -r
pip install -r requirements.txt
```

## 3. Build

Sve ide kroz **projektno okruženje** (`scripts/run_native.sh`): učitava samo `/opt/ros/humble` i
ovaj overlay, Fast DDS, ROS domenu 5 i lokalno otkrivanje čvorova. Globalni `~/.bashrc` namjerno
ne postavlja ROS varijable.

```bash
./scripts/run_native.sh colcon build --symlink-install
./scripts/run_native.sh bash scripts/verify_environment.sh     # 17/17 provjera
```

> `colcon` će javiti da `realsense2_description` nadjačava apt verziju — **namjerno je**, cijeli
> `realsense-ros` dolazi iz izvora radi usklađenosti s driverom.

## 4. Pokretanje misije — jedna naredba

```bash
bash scripts/clean_ros.sh        # nikad dvije simulacije odjednom
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py |& tee log/run-mission.log
```

Robot se stvori u srednjoj (home) sobi **s raširenim rukama**, sam ih složi u pozu vožnje i
**čeka**. Kad u logu piše `WAITING for the user`, pritisni zeleni gumb **„MISIJA: po kutiju"** u
navigacijskom panelu, ili iz drugog terminala:

```bash
./scripts/run_native.sh ros2 topic pub --once /mission/start std_msgs/String "{data: blue}"
```

Dalje ide samo: plava soba → hvat → kroz vrata → crvena soba → odlaganje.

**Argumenti** (`mission.launch.py`):

| argument | zadano | značenje |
|---|---|---|
| `headless` | `false` | Gazebo bez GUI-ja (kad GUI izgladnjuje upravljačku petlju) |
| `open_rviz` | `true` | RViz; pogled biraš s `rviz_config:=…/cube.rviz` |
| `gui` | `true` | panel s gumbima (`nav_gui`) |
| `map` | `maps/seminar_map.yaml` | karta za AMCL |
| `pick_room` / `place_room` | `blue` / `red` | odakle se uzima i kamo se odlaže |

Mapiranje je zaseban korak (`mapping.launch.py`); misija vozi po **spremljenoj** karti.

## 5. Što se očekuje u logu

```
mission: drive posture ... verified
WAITING for the user: press "MISIJA: po kutiju" ...
NAV: arrived at "blue:dock"
STEP5d left tool tip 1.1 mm from its target
CARRIAGE LIFT MEASURED left=0.5500 right=0.5500 m
NAV: arrived at "red:dock"
PLACE step 6: the arms carry the cube +19.6 cm forward and +1.3 cm across
PLACE the cube bottom is +4 mm from the table top
PLACE VERIFIED: the centre of the cube is 5 mm from the marker centre
MISSION COMPLETE: the cube is on the marker.
```

Bez retka `PLACE VERIFIED` run **nije** uspjeh, ma što ostalo pisalo — sve provjere su neovisne o
naredbi, a neuspjeh se prijavljuje i prekida
([odluka D-12](notes/04_odluke/D-12_honesty_abort_over_fake.md)).

## 6. Provjere bez simulatora

```bash
./scripts/run_native.sh python3 scripts/check_doors.py    # vrata iz karte
./scripts/run_native.sh python3 scripts/check_zones.py    # zone, portali, dock poze
./scripts/run_native.sh python3 scripts/check_map.py      # kvaliteta karte
```

## 7. Arhitektura

| Dio | Izvor | Upravljanje |
|---|---|---|
| Mobilna baza | PAL `omni_base_simulation` (geometrija, kotači, lidar) | `mecanum_drive_controller` (omnidirekcijski, x/y/yaw) |
| Ruke (2 × Kinova Gen3, 7-DOF) | `ros2_kortex` | `joint_trajectory_controller` + **MoveIt 2** |
| Hvataljke Robotiq 2F-85 | `ros2_kortex` | `GripperActionController` |
| Torzo: 2 vertikalne vodilice | STL iz priloga zadatka | `joint_trajectory_controller` (prismatic, 0.05–0.65 m) |
| Pan-tilt + RealSense D435 | `pan_tilt_ros`, `realsense-ros` | `joint_trajectory_controller` |
| Senzori | lidar (1080 zraka), RGBD na glavi, **2 × RGBD na zapešćima**, kontaktni senzori na jastučićima, FT na zapešćima | — |
| Sučelje prema Gazebu | `ign_ros2_control/IgnitionSystem` | — |
| Mapiranje / navigacija | `slam_toolbox` + `nav2` (AMCL, NavFn, DWB, collision monitor) | — |
| Percepcija kutije | ArUco `DICT_4X4_50` (vlastiti detektor, `cv2.aruco`) | — |

Kutija: **0.30 m, 0.3 kg**, ArUco marker na prednjoj plohi i po jedan na bočnima (za kamere na
zapešćima). Svijet: tri sobe u obliku slova L, dva otvora od **0.98 m**.

## 8. Poznata ograničenja (iskreno)

| Što | Zašto |
|---|---|
| Kutija se drži **krutim spojem** (`DetachableJoint`), uključenim tek nakon dokazanog obostranog kontakta | DART je ne drži trenjem — iscrpno probano (`notes/03_problemi/P-15…`) |
| Mase torza su **procjena** (12 kg vodilica, 2 kg klizač) | nema podataka proizvođača vodilica |
| Nošenje visi o spoju na **lijevom** zapešću, iako obje ruke drže kutiju | dvije krute veze ruše solver (`P-17`) |
| Pogon je `mecanum_drive_controller`, ne PAL-ov `omni_drive_controller` | PAL-ov nije dostupan za Humble (`P-09`) |

Puni popis s obrazloženjima: `notes/07_predaja/odstupanja.md`.

## 9. Dokumentacija

| Gdje | Što |
|---|---|
| `notes/00_MAPA.md` | stablo zahtjeva i status svakog (ulazna točka) |
| `notes/00_run/00_testing/misija.md` | postupak pokretanja i što gledati, korak po korak |
| `notes/03_problemi/` | svaki problem s **tablicom svih pokušaja** i izmjerenim ishodima |
| `notes/04_odluke/` | odluke (ADR) |
| `notes/06_parametri.md` | svaka podesiva vrijednost i zašto je takva |
| `RUNNING.md` | kraće upute za rad po terminalima |
