# Sensors: what is measured, and what actually uses it

This page exists because the distinction is easy to get wrong. The robot carries more sensors than
the control software reads, and two of them are deliberately excluded from control. Each sensor
below is classified by **role**, not by whether it produces data:

| Role | Meaning |
|---|---|
| **closed loop** | the measurement changes the command that is issued next |
| **gate** | the measurement does not shape a command, but it decides whether the next step is allowed to happen at all; failing it aborts the mission |
| **evaluation** | recorded or logged for offline assessment; no decision depends on it |
| **not bridged** | present in the robot description, but never reaches ROS |

## Summary

| Sensor | Count | Topic | Rate | Role |
|---|---|---|---|---|
| 2D lidar (base) | 1 | `/scan` → `/scan_filtered` | 25 Hz | **closed loop** |
| Wheel odometry (mecanum base) | — | `/base_controller/odom` | — | **closed loop** |
| Head RGB-D (pan-tilt RealSense D435) | 1 | `/camera/image`, `/camera/points` | — | **closed loop** |
| Wrist RGB-D cameras | 2 | `/wrist_{left,right}/image`, `…/points` | 15 Hz | **closed loop** |
| Fingertip contact sensors | 4 | `/contact/{left,right}_{left,right}_tip` | 50 Hz | **gate** |
| Wrist force-torque sensors | 2 | `/ft/{left,right}_wrist` | 200 Hz | **evaluation** |
| Joint torque (`effort` in `/joint_states`) | — | `/joint_states` | — | **evaluation** |
| Base IMU | 1 | — | — | **not bridged** |

---

## Closed loops

**2D lidar.** 1080 rays, 25 Hz, 0.05–25 m, defined in `robot.urdf.xacro`. The raw `/scan` is not
consumed directly: `scan_filter` removes the returns from the robot's own body and republishes
`/scan_filtered`, and *everything* that makes a decision reads that same filtered signal — AMCL,
both costmaps, the DWB controller, the collision monitor and `slam_toolbox`. On top of the Nav2
loop, `room_navigator.side_clearance()` measures the lateral gap directly from the scan while the
robot is in a doorway and cancels the Nav2 goal if it drops below `min_side_clearance`. The same
scan is also compared against the AMCL pose, but that comparison is diagnostic only.

**Wheel odometry.** The mecanum `base_controller` publishes odometry that the low-level motions
close on: `base_drive.drive_distance()`, `strafe_distance()` and `turn_angle()` iterate until the
odometry shows the requested displacement, and abort on yaw drift beyond 0.08 rad or on a lack of
progress. This is the only low-level feedback loop in the mission itself.

**Head RGB-D.** Drives two things: `visual_approach()` servos the base toward the ArUco marker, and
`measure_box()` derives the box centre and dimensions from the point cloud. Those measurements
*become* the grasp targets, so a perception error changes where the arms go.

**Wrist RGB-D cameras.** Each wrist runs its own ArUco detector (marker ids 1 and 2) whose output
becomes the target for that hand's tool tip during the final approach.

## The gate: fingertip contact sensors

Four `contact` sensors, one per fingertip, at 50 Hz. Gazebo Fortress ignores the `<topic>` tag on a
contact sensor, so `bridge.yaml` maps the long Gazebo paths onto short ROS topics
`/contact/<hand>_<finger>_tip`. **The world name is part of those paths** — renaming the world
without updating `bridge.yaml` silently disables the gate.

The callback records two timestamps per fingertip: any contact, and contact **whose other collider
is the box**. Only the second counts as evidence, so a pad brushing the table or the robot itself
does not qualify.

The gate itself is step 5d of the mission, immediately before the box is attached:

> Lifting is allowed only when **both hands report contact with the box** *and* **both tool tips
> are within 10 mm of their targets**, where the targets come from a *fresh* depth and marker
> reading rather than from the pose the arms were commanded to reach. If either condition fails,
> the arms are released back to the pre-grasp posture and the mission aborts with the reason
> recorded.

Contact also terminates the gripper closing motion early, as soon as a pad touches the box.

This is what makes the requirement "lift it with both arms" verifiable: the rigid joint that
carries the box is engaged *after* the evidence, never before it.

## Evaluation only

**Wrist force-torque sensors.** Mounted on joint 7, 200 Hz. The robot description states the reason
in a comment: the real Gen3 has joint torque sensors, not a wrist FT sensor, so **no control code
may read these**. They exist to give an offline assessment an independent reference. Their only
subscriber, `grasp_force_diagnostics`, is not registered as an entry point in `setup.py`, so in
practice it never runs.

**Joint torque.** The `effort` field of `/joint_states` is monitored and logged during the grasp,
but it is explicitly not a hard gate: a top-down straddle grasp loads the wrist mostly with
gravity, so an uncalibrated threshold would false-fire.

### Why force is not used at all

A force-based grasp was implemented and then abandoned. The blocking reason is a property of the
simulator: in Gazebo Fortress the contact message carries contact **points** with empty wrenches —
in one measurement, 49 035 contact records contained **zero** force values. Without force in the
contact message there is no independent reference against which a torque-based force estimate could
be qualified, so the estimate could not be validated. The mission therefore runs entirely on the
`position` interface, and the box is held by a rigid joint rather than by pad friction.

A second reason disappeared later: the force profile was originally needed because the torso rails
would not lift under load, but the real cause turned out to be an initial value sitting exactly on
the lower joint limit. Once corrected, the `position` profile lifts the full load.

## Not bridged

**Base IMU.** The IMU is present in the inherited base description, but its Gazebo plugin is guarded
for Gazebo Classic while this project builds for Fortress, and there is no entry for it in
`bridge.yaml`. It therefore publishes nothing and nothing reads it. It is listed here so that the
sensor inventory of the model is not mistaken for the sensor inventory of the running system.
