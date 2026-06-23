# Stanje Projekta (State)

**Trenutna faza**: Autonomna navigacija (Nav2) radi end-to-end u simulaciji.
**Datum zadnje izmjene**: 2026-06-23

## Završeni Milestoneovi (commitano)
- **M0+M1** — `ros2_control` u Gazebo Fortress + pouzdan `sim.launch.py` (8 kontrolera se konfigurira i aktivira čisto).
- **M2** — Ignition `gpu_lidar` (`/scan`) + RGB kamera (`/camera/image`) senzori i `ros_gz` bridge.
- **M3** — Puni MoveIt2 config za dual-arm (`pas_dual_arm_moveit_config`).
- **M4** — Aruco detekcija (DICT_4X4_50, ID 0) preko kamere.
- **M5** — Nav2 autonomna navigacija s diff_drive bazom (vidi dolje).

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
Globalni `~/.bashrc` postavlja `rmw_zenoh_cpp` s `ZENOH_CONFIG_OVERRIDE` koji cilja vanjski
router (192.168.0.14:7447) nedostupan lokalno. Launch fajlovi zato interno forsiraju
Fast DDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, prazan `ZENOH_CONFIG_OVERRIDE`).

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
