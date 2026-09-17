# Building a map from scratch (SLAM)

You do **not** need this for the demo — the map `src/pas_dual_arm_bringup/maps/seminar_map.yaml`
ships with the repository and the mission drives on it ([`../README.md`](../README.md) §4). This
document is for rebuilding the map yourself: after changing the world, or to repeat the whole SLAM
procedure.

Mapping has **its own launch file and deliberately does not start Nav2**: there is nothing to plan
while mapping, and the full navigation stack competes for CPU with the `gz_ros2_control` update
loop, which is what starved the controllers in earlier runs.

---

## 1. Before you start

```bash
cd ~/FSB/PAS-DUAL-ARM
bash scripts/clean_ros.sh        # never two simulations at once
./scripts/run_native.sh colcon build --symlink-install
```

Every terminal goes through `./scripts/run_native.sh`. Do not source another ROS workspace into
the same shell.

## 2. Terminal 1 — simulation

```bash
PAS_SIM_CARRY_ARMS=true ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup sim.launch.py
```

`PAS_SIM_CARRY_ARMS=true` spawns the robot **already in the narrow `ARM_CARRY_V2` posture**
(0.854 m). This matters: in the spread spawn posture the robot is too wide for a 0.98 m opening and
catches on the door frame. Wait for **all eight controllers** to become active.

If the arms drift apart while driving (position-gain sag), fold them back:

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_moveit_config move_group.launch.py   # terminal X
./scripts/run_native.sh ros2 run pas_dual_arm_scripts set_posture ARM_CARRY_V2
```

## 3. Terminal 2 — SLAM

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mapping.launch.py
```

This starts `slam_toolbox` (resolution **0.02 m**), `feature_registry` and RViz with the mapping
view. For headless operation: `rviz:=false`. The simulation already starts `scan_filter` and the
`/cmd_vel` relay.

**Before driving, check:** `/scan_filtered` is arriving, `/map` exists, and the TF chain
`map → odom → base_link` is complete.

## 4. Terminal 3 — driving

```bash
./scripts/run_native.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Driving rules** (they follow from the mecanum wheel friction, `mu1=0.80`, `mu2=0.20`):

- **Stop before turning.** Rotate in place, then drive straight — not along an arc.
- Reduce the speeds: `x` → linear to ≈ 0.2 m/s, `c` → angular to ≈ 0.3 rad/s.
- Go through a doorway **perpendicular** and slowly; the margin is about 7 cm per side.

**Route** (loop closure is what keeps the map straight):

```
HOME → BLUE room (loop around it) → back to HOME → RED room (loop) → back to HOME
```

Watch the walls close up in RViz. If the map "splits" (duplicated walls), abort and drive again,
more slowly — saving a bad map only moves the problem into Nav2.

### Alternative: the scripted tour

```bash
./scripts/run_native.sh ros2 launch pas_dual_arm_moveit_config move_group.launch.py   # terminal 4
./scripts/run_native.sh ros2 run pas_dual_arm_scripts mapping_tour                    # terminal 5
```

The tour checks the arms and **aborts on purpose** if the deviation exceeds 0.10 rad. Because the
doorway margin is so tight, open-loop odometry can still catch on the frame — **manual driving is
more reliable**, and it is how the map in this repository was produced.

## 5. Saving the map

Only after a complete and verified tour:

```bash
./scripts/save_map.sh my_tour
```

The script:

1. saves an archive copy `maps/map_YYYYMMDD_HHMMSS_my_tour.{yaml,pgm,posegraph,data}`;
2. updates the **active** map `maps/seminar_map.{yaml,pgm,posegraph,data}` (the one the mission drives);
3. runs `check_map.py` (coverage) and `check_map_geometry.py` (doorway geometry).

Everything lands in `src/pas_dual_arm_bringup/maps/`, no matter where you run the script from.

> Only `.yaml` and `.pgm` are committed. The `.posegraph` and `.data` files (≈ 44 MB per map) stay
> local — they are only needed to *continue* a SLAM session, not to drive.

## 6. The gate: when a map may be accepted

Coverage tells you that all three rooms are on the map, but **not** whether the doorways on it are
still passable. With about 7 cm of margin per side, geometry is what decides:

| Measure | Passes if |
|---|---|
| width of both doorways | ≥ **0.97 m** (actual 1.00 m) |
| doorway axis (along the wall) | ≤ **1 cm** from the true one |
| wall thickness | ≤ **0.14 m** (actual 0.10 m) |
| wall face: RMS / tilt | ≤ **10 mm** / ≤ **1.0°** |
| step between the faces on either side of an opening | ≤ **20 mm** |
| distance between the two doorways (map scale) | ≤ **20 mm** from the true 4.243 m |

The gate can also be run afterwards, without the simulator:

```bash
./scripts/run_native.sh python3 scripts/check_map_geometry.py
./scripts/run_native.sh python3 scripts/check_map_geometry.py src/pas_dual_arm_bringup/maps/map_2026...yaml
```

**If the gate fails, the map is not accepted — the tour is repeated.** The previous map stays in
the archive. The reference map in this repository: 102.3 m² of free space, extent 11.9 × 11.8 m,
doorways 0.980 m.

## 7. Driving on your own map

```bash
./scripts/run_native.sh colcon build --symlink-install --packages-select pas_dual_arm_bringup
bash scripts/clean_ros.sh
./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py
```

The rebuild is mandatory — Nav2 reads the map from
`install/…/share/pas_dual_arm_bringup/maps/`, not from `src/`.

Before the first drive, run the offline zone checks on the new map:

```bash
./scripts/run_native.sh python3 scripts/check_doors.py
./scripts/run_native.sh python3 scripts/check_zones.py
```

The navigation zones (doorways, tables, dock poses) are derived **from the map**, so a new map
means new zones. In RViz add a `MarkerArray` on `/nav_zones_markers`: a green band through each
opening, an orange halo around the tables, blue arrows for the portal poses. If they are missing,
`nav_zones` did not find the doorways — do not drive.
