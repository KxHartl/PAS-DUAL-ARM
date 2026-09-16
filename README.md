# PAS-DUAL-ARM — simulacijski model dual-arm robota (ROS 2 Humble + Gazebo Fortress)

Mobilni robot s **dvije Kinova Gen3 ruke** na **vertikalnim linearnim vodilicama**,
**omnidirekcijskom bazom** i **pan-tilt kamerom**. Cijela misija ide iz **jedne naredbe**:

> robot čeka naredbu → odveze se u sobu s kutijom → nađe je → **podigne objema rukama** →
> pronese kroz vrata → **odloži na označeno mjesto** u drugoj sobi.

Zadnji potvrđeni run (GUI, 16. 9. 2026.): kutija spuštena **4 mm** iznad ploče i sjela **5 mm**
od centra markera — `PLACE VERIFIED`, `MISSION COMPLETE`.

> **Karta je već u repou.** `src/pas_dual_arm_bringup/maps/seminar_map.yaml` je zadana karta i
> misija se vozi po njoj — **ne moraš mapirati da bi pokrenuo demo**. Ako želiš snimiti vlastitu
> kartu od nule, cijeli je postupak u [`MAPPING.md`](MAPPING.md).

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
                 ros-humble-teleop-twist-keyboard \
                 python3-vcstool python3-rosdep python3-colcon-common-extensions
```

## 2. Dohvat i instalacija

Pet paketa u `src/` su **tuđi repozitoriji** i namjerno nisu dio ovog repoa — skidaju se izravno
od autora, na točno pinane commitove iz `ros2.repos` (popis i licence: [§9](#9-vanjski-paketi--nisu-naši)).

```bash
git clone https://github.com/KxHartl/PAS-DUAL-ARM.git
cd PAS-DUAL-ARM

vcs import src < ros2.repos     # aruco_ros, omni_base_simulation, pan_tilt_ros,
                                # realsense-ros, ros2_kortex — s GitHuba autora
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

Dalje ide samo: plava soba → hvat → kroz vrata → crvena soba → odlaganje na marker.

**Argumenti** (`mission.launch.py`):

| argument | zadano | značenje |
|---|---|---|
| `headless` | `false` | Gazebo bez GUI-ja (kad GUI izgladnjuje upravljačku petlju) |
| `open_rviz` | `true` | RViz; pogled biraš s `rviz_config:=…/cube.rviz` |
| `gui` | `true` | panel s gumbima (`nav_gui`) |
| `map` | `src/pas_dual_arm_bringup/maps/seminar_map.yaml` | karta za AMCL (učitava se iz `install/…/share/`) |
| `pick_room` / `place_room` | `blue` / `red` | odakle se uzima i kamo se odlaže |

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

## 6. Mapiranje od nule (nije potrebno za demo)

Misija vozi po spremljenoj karti iz repoa. Ako želiš proći cijeli SLAM sam:

```bash
# Terminal 1 — simulacija, ruke odmah u uskoj pozi ARM_CARRY_V2
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py

# Terminal 2 — slam_toolbox + RViz (bez Nav2, namjerno)
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mapping.launch.py

# Terminal 3 — ručna vožnja kroz sve tri sobe
./scripts/run_native.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4 — spremanje kad je karta potpuna
./scripts/save_map.sh moja_tura
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_bringup
```

`save_map.sh` sam ažurira `seminar_map.*`, pa nakon rebuilda `mission.launch.py` vozi po **tvojoj**
karti. **Pravila vožnje, ruta, geometrijski gate-ovi i što provjeriti prije spremanja:
[`MAPPING.md`](MAPPING.md)** — bez toga karta prođe pokrivenost, a padne na vratima.

> U repou su samo `.yaml` + `.pgm` — to je sve što `map_server` i AMCL trebaju. `.posegraph` i
> `.data` (≈ 44 MB) nisu u gitu jer služe samo za **nastavak** SLAM-a, ne za vožnju.

## 7. Provjere bez simulatora

```bash
./scripts/run_native.sh python3 scripts/check_doors.py    # vrata iz karte
./scripts/run_native.sh python3 scripts/check_zones.py    # zone, portali, dock poze
./scripts/run_native.sh python3 scripts/check_map.py src/pas_dual_arm_bringup/maps/seminar_map.yaml
```

Ako `check_*` ne prođu, **ne pokreći simulaciju** — zone su krive i vožnja nema smisla.

## 8. Arhitektura

| Dio | Izvor | Upravljanje |
|---|---|---|
| Mobilna baza | PAL `omni_base_simulation` (geometrija, kotači, lidar) | `mecanum_drive_controller` (omnidirekcijski, x/y/yaw) |
| Ruke (2 × Kinova Gen3, 7-DOF) | `ros2_kortex` | `joint_trajectory_controller` + **MoveIt 2** |
| Hvataljke Robotiq 2F-85 | `ros2_kortex` | `GripperActionController` |
| Torzo: 2 vertikalne vodilice | STL: **Branimir Ćaran** (prilog zadatka, `dual_arm_torso-main.zip`, 4. 5. 2026.) | `joint_trajectory_controller` (prismatic, 0.05–0.65 m) |
| Pan-tilt + RealSense D435 | `pan_tilt_ros`, `realsense-ros` | `joint_trajectory_controller` |
| Senzori | lidar (1080 zraka), RGBD na glavi, **2 × RGBD na zapešćima**, kontaktni senzori na jastučićima, FT na zapešćima | — |
| Sučelje prema Gazebu | `ign_ros2_control/IgnitionSystem` | — |
| Mapiranje / navigacija | `slam_toolbox` + `nav2` (AMCL, NavFn, DWB, collision monitor) | — |
| Percepcija kutije | ArUco `DICT_4X4_50` (vlastiti detektor, `cv2.aruco`) | — |

Kutija: **0.30 m, 0.3 kg**, ArUco marker na prednjoj plohi i po jedan na bočnima (za kamere na
zapešćima). Svijet: tri sobe u obliku slova L, dva otvora od **0.98 m**.

## 9. Vanjski paketi — nisu naši

Ovih pet paketa **nije** u repozitoriju: `vcs import` ih skida s GitHuba autora, na pinane
commitove. Repo sadrži samo manifest `ros2.repos`.

| Paket | Autor | Licenca | Commit | Čemu služi |
|---|---|---|---|---|
| [`omni_base_simulation`](https://github.com/pal-robotics/omni_base_simulation) | PAL Robotics | Apache-2.0 | `77248ac` | mobilna baza: geometrija, kotači, lidar |
| [`ros2_kortex`](https://github.com/Kinovarobotics/ros2_kortex) | Kinova | BSD | `116d87a` | Kinova Gen3 ruke + Robotiq 2F-85 hvataljke |
| [`pan_tilt_ros`](https://github.com/I-Quotient-Robotics/pan_tilt_ros) | I-Quotient-Robotics | MIT | `b0f6534` | pan-tilt mehanizam na vrhu robota |
| [`realsense-ros`](https://github.com/realsenseai/realsense-ros) | Intel RealSense | Apache-2.0 | `6d87b07` | opis RealSense D435 kamere |
| [`aruco_ros`](https://github.com/pal-robotics/aruco_ros) | PAL Robotics | MIT | `86a0bbb` | ArUco (koristi se vlastiti detektor, [D-02](notes/04_odluke/D-02_own_aruco_detector.md)) |

**Jedina izmjena tuđeg koda** je `patches/ros2_kortex-robotiq_2f_85-drop-isaac-args.patch` — miče
tri Isaac argumenta iz `robotiq_2f_85_macro.xacro` kojih na Humble grani nema. Primjenjuje je
`scripts/apply_patches.sh`.

**STL vodilica i torza** isporučuju se **s ovim repoom** (`src/dual_arm_torso/meshes/`) i autor im
je **Branimir Ćaran** — prilog uz mail od 4. 5. 2026., korišteno uz dopuštenje autora zadatka.
Vidi [`src/dual_arm_torso/README.md`](src/dual_arm_torso/README.md).

Ovaj repo je Apache-2.0 (`LICENSE`); sve gornje licence su s njom kompatibilne.

## 10. Poznata ograničenja (iskreno)

| Što | Zašto |
|---|---|
| Kutija se drži **krutim spojem** (`DetachableJoint`), uključenim tek nakon dokazanog obostranog kontakta | DART je ne drži trenjem — iscrpno probano (`notes/03_problemi/P-15…`) |
| Mase torza su **procjena** (12 kg vodilica, 2 kg klizač) | nema podataka proizvođača vodilica |
| Nošenje visi o spoju na **lijevom** zapešću, iako obje ruke drže kutiju | dvije krute veze ruše solver (`P-17`) |
| Pogon je `mecanum_drive_controller`, ne PAL-ov `omni_drive_controller` | PAL-ov nije dostupan za Humble (`P-09`) |

Puni popis s obrazloženjima: [`notes/07_predaja/odstupanja.md`](notes/07_predaja/odstupanja.md).

## 11. Dokumentacija

| Gdje | Što |
|---|---|
| [`MAPPING.md`](MAPPING.md) | SLAM od nule: vožnja, spremanje karte, gate-ovi |
| [`RUNNING.md`](RUNNING.md) | rad po terminalima, logovi, poznati problemi |
| [`notes/00_MAPA.md`](notes/00_MAPA.md) | stablo zahtjeva i status svakog (ulazna točka) |
| `notes/00_run/00_testing/misija.md` | postupak pokretanja i što gledati, korak po korak |
| `notes/03_problemi/` | svaki problem s **tablicom svih pokušaja** i izmjerenim ishodima |
| `notes/04_odluke/` | odluke (ADR) |
| `notes/06_parametri.md` | svaka podesiva vrijednost i zašto je takva |
