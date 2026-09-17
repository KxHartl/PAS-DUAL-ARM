# Running the simulation and the mission

Installation and source fetching are covered in [`../README.md`](../README.md). This page lists
only the commands used while working.

## 0. Project-scoped environment (once per terminal)

```bash
cd ~/FSB/PAS-DUAL-ARM
./scripts/run_native.sh
```

This opens an isolated PAS-DUAL-ARM shell (Fast DDS, ROS domain 5, localhost-only discovery).
Open every terminal that takes part in the same ROS graph the same way; before switching to a
different ROS project, leave with `exit` and do not source its `setup.bash` into the same shell.

After changing code, xacro/URDF, the world or a config:

```bash
./scripts/run_native.sh colcon build --symlink-install
# faster, only what you touched:
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_scripts pas_dual_arm_bringup
```

> **Never run two simulations at once.** Both Gazebo instances publish `/clock`, time jumps back
> and forth, and RViz dies with `Cannot create GL vertex buffer` — a symptom that gives no hint of
> the cause. Before starting: `bash scripts/clean_ros.sh`.

## 1. The whole mission from one command

```bash
bash scripts/clean_ros.sh
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py |& tee log/run-mission.log
```

When the log prints `WAITING for the user`, press **"MISIJA: po kutiju"** on the panel (or publish
`ros2 topic pub --once /mission/start std_msgs/String "{data: blue}"`). Everything after that is
autonomous: blue room → grasp → through the doorway → red room → place on the marker →
`MISSION COMPLETE`.

## 2. The two scenarios

Each is one command; they differ only in where the map comes from.

```bash
# 1 - map by driving it yourself, then run the mission on that map
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup scenario_manual_map.launch.py

# 2 - the mission only, on the map in the repository
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup scenario_mission.launch.py
```

In 1, press **MAPIRANJE GOTOVO** in the panel when the map is complete.
The map is saved to `~/.ros/pas_dual_arm/live_map.*`, SLAM is stopped, AMCL comes
up on that map and is seeded with the pose where mapping ended, and the mission
node starts. Nothing is rebuilt in between, because the map is loaded by absolute
path rather than out of `install/`.

If the map cannot be saved, the mission is deliberately **not** started and the
simulation stays up so the map can be looked at.

All three take `world:=<path>` to run a generated layout (see `scripts/gen_world.py`),
and `headless:=true` when the GUI starves the control loop.

## 3. Piece by piece (while developing)

**Terminal 1 — simulation**
```bash
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
```
Wait for all eight controllers to become active. Without the GUI: `headless:=true`.

**Terminal 2 — navigation** (map + AMCL + Nav2 + zones + panel)
```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup nav2.launch.py 2>&1 | tee /tmp/pas/t2.log
```

**Terminal 3 — driving between rooms**
```bash
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: blue:dock}"
./scripts/run_native.sh ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: red}"
```

**Manipulation only**, without driving — the robot spawns at the dock pose and the task node is
started without auto-start, so the grasp can be triggered by hand:
```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py robot_spawn_y:=-5.479
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup task.launch.py auto_start:=false
```

**Mapping** is a separate step; the mission drives on the map stored in the repository. The full
procedure — driving rules, route, acceptance gates — is in [`MAPPING.md`](MAPPING.md). In short:
```bash
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mapping.launch.py
./scripts/run_native.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
./scripts/save_map.sh my_tour
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_bringup
```

## 4. Checks that run without the simulator

```bash
./scripts/run_native.sh bash scripts/verify_environment.sh   # all checks must pass
./scripts/run_native.sh python3 scripts/check_doors.py       # doorways from the map
./scripts/run_native.sh python3 scripts/check_zones.py       # zones, portals, dock poses
./scripts/run_native.sh python3 scripts/check_map.py \
    src/pas_dual_arm_bringup/maps/seminar_map.yaml           # map quality (takes a path)
```
If `check_*` does not pass, **do not start the simulation** — the zones are wrong and driving is
meaningless.

## 5. Where the logs are

`ros2 launch` writes a separate log per node under `~/.ros/log/<node>_<pid>_*.log`, while the `tee`
in the commands above produces one file with everything interleaved. After a run, the most useful
commands are:

```bash
grep -hE "PLACE|NAV:|MISSION|Task aborted|ABORT" log/run-mission.log | tail -30
ls -t ~/.ros/log/controller_server_*.log | head -1 | xargs grep -cE "WARN|ERROR"
```

## 6. Known issues

- **`colcon` reports an override of `realsense2_description`** — intentional: the whole
  `realsense-ros` comes from source (v4.57.6) to stay consistent with the driver. The warning is
  benign.
- **`No rule to make target '.../librealsense2.so.2.58.3'`** (or any other library version) after
  an `apt upgrade`. The system now has a newer version and the CMake cache of that one package
  still points at the file that was removed. Delete that package's build output and rebuild — not
  the whole `build/` directory:
  ```bash
  rm -rf build/realsense2_camera install/realsense2_camera
  ./scripts/run_native.sh colcon build --symlink-install
  ```
  Note that `colcon` also aborts whatever was queued behind the failure, so
  `pas_dual_arm_bringup` and `pas_dual_arm_scripts` will be reported as aborted even though
  nothing is wrong with them. `realsense2_camera` is the RealSense *driver*; this project only
  needs `realsense2_description`, so the failure never affects the simulation itself.
- **`gazebo_version is not defined`** — fixed in `robot.urdf.xacro`; `omni_base.urdf.xacro` is
  skipped deliberately because it pulls in Gazebo Classic ros2_control. If this returns after an
  `apt upgrade` of `ros-humble-pal-urdf-utils`, check that the property is defined before the
  include.
- **Middleware and domain** are set exclusively in `scripts/run_native.sh`; do not set ROS
  variables, a router or a robot address globally in `~/.bashrc`.
- **The wrist cameras cannot see the markers while the pads hold the box** (they sit 2 cm from the
  marker). The pose of the held box therefore comes from the transform recorded at attach time.

The requirement status and the deliberate deviations from the assignment are documented in
`seminar.pdf` (sections 2 and 10).
