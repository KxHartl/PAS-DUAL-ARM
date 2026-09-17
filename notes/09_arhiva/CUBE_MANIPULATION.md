# Cube manipulation workstream

This branch develops perception, pick, carry, and place independently of the
ongoing navigation work on `main`. Run every ROS/Gazebo command through
`bash scripts/run_cube_isolated.sh`: it selects ROS domain 75, Gazebo partition
`pas_dual_arm_cube_75`, and a worktree-local ROS home. Build and launch from
this worktree only; never source the main worktree's `install/` overlay.

## Ground truth and existing behavior

- The current SDF, not older notes, defines a 0.30 m / 0.3 kg cube at
  `(0, -6.35, 0.90)` on a table whose top is `z = 0.75 m`. The destination
  table is centered at `(6.5, 0)` with the same top height.
- `aruco_detector.py` identifies marker 0; `main_task.py` combines the marker
  and depth cloud before approaching. This path has prior GUI evidence, but
  must be repeated against this world's geometry.
- The current pick uses closed grippers as pads, presses both sides, enables
  Gazebo's DetachableJoint, and lifts with the left arm. Project requirement
  R-17 explicitly rejects this as a completed grasp. The current place returns
  the cube to the pickup table, not the destination.
- The arm position controller can report success despite substantial measured
  joint error (P-37); loaded torso carriages currently do not rise (P-13).
  Read back `/joint_states` and actual transforms after every motion.

## Experiments and acceptance gates

1. **Actuation baseline (passed with arm load):** with no cube contact, command
   and measure both carriage heights. The old position interface returned
   `SUCCEEDED` but stayed at 0.050 m for a 0.200 m goal (C1). The new effort
   interface with JTC PID reached 0.1944 m after 2 s, held 0.2000 m after
   10 s, and returned to 0.0500 m on both sides (C2). Cube-contact load remains
   to be tested. Do not repeat the failed effort-limit/gain-only tweaks.
2. **Perception (marker baseline passed):** `sim.launch.py` now accepts
   `robot_spawn_x`, `robot_spawn_y`, and `robot_spawn_yaw` so isolated tests can
   start beside the blue table without exercising navigation. With the base at
   `(0, -5.0, -pi/2)`, marker 0 was detected at about 13 Hz. Its transform in
   `base_link` was stable at `(1.211, 0.016, 0.820)` m; adding the known 0.15 m
   face-to-center offset gives `(1.361, 0.016, 0.820)` m. The world-defined
   center is 1.35 m ahead of this spawn pose, an approximately 1 cm X error.
   A fresh `/camera/points` cloud was also observed. At the closer
   `(0, -5.5, -pi/2)` spawn, the original depth crop found zero box points:
   its `z < 0.50 m` limit came from the old floor-level scene. Anchoring the
   crop to the observed marker gave 37,789 candidate points, a 0.300 m long
   axis and 0.252 m visible height. Marker-derived centre was
   `(0.8535, 0.0157, 0.8211)` m and depth-derived centre was
   `(0.8490, 0.0000, 0.8479)` m in `base_link`: 4.5 mm X, 15.7 mm Y, and
   26.8 mm Z disagreement. The partial visible height and Z offset need
   further validation before using depth height to set a grasp. Recheck after
   base motion and reject stale or disagreeing samples. Gazebo pose is
   diagnostic only, never a control input. Reproduce with
   `bash scripts/run_cube_isolated.sh python3 scripts/probe_cube_perception.py --ros-args -p use_sim_time:=true`
   while the isolated sim and ArUco launches run.
3. **Contact pick:** derive both pad targets from the measured cube pose and
   width. Move to collision-checked pregrasp poses, then to a measured 2 cm
   standoff at no more than 25 mm/s. Approach both faces at 2 mm/s and
   independently stop each arm on
   its first box-only pad contact. Close both grippers together for the fine
   grasp. Keep the marker-derived wrist orientation
   and advance only an incomplete hand in 2 mm corrections, bounded to 10 mm
   past its measured face, until all four pads touch. A hand already at 2/2
   holds position. If a commanded 2 mm move repeatedly yields less than
   0.2 mm of actual wrist travel, back that hand off 3 mm and realign.
   With Gazebo's rigid joint disabled, hold four contacts for 1 s
   and lift 0.20 m at 10 mm/s. Stop, restore the bounded contact and resume if
   any pad loses contact. Repeat before trying transit.
   Current read-only pose probe in the GUI sim (base at `(0,-5.5,-pi/2)`):
   the fused centre is `(0.849,0.000,0.821)` in `base_link`, marker tangent
   nearly `+Y`, and measured wrist-to-tip stand-off 0.115 m. Left/right
   pre-contact wrist positions are `(0.849,+0.365,0.821)` and
   `(0.849,-0.365,0.821)`; contact positions are
   `(0.849,+0.265,0.821)` and `(0.849,-0.265,0.821)` (all metres).
   Both contact and pre-contact IK fail from this observation range. Read-only
   predicted centres at X=0.65 and X=0.55 m pass IK for both arms and both
   poses. The next trial must move the base closer, remeasure the cube, add
   collision objects, and verify executable paths before moving the arms.
4. **Carry:** verify the contact-held cube through a 0.4 m straight move and a
   slow turn; compare its motion with the hands. Check the full robot-plus-box
   swept width before attempting the 1.0 m doors. Stop on loss of contact or
   tracking. A rigid joint is a fallback only after the user's explicit decision.
5. **Place:** use a fixed point on the near side of the red table. The
   cube-center target is `(6.32, 0, 0.90)`, leaving 0.07 m between cube and
   near tabletop edge. A small visual-only X is in the SDF at `(6.32, 0, 0.751)`.
   Solve collision-free base/arm staging before lowering. Set the cube down gently,
   open/retreat both hands, and confirm that it remains upright on the X.

Keep the cube in MoveIt's planning scene throughout: as a world object before
contact, an [attached collision object](https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)
during transport, and a world object at the observed destination after release.
That planning representation does not create a physical Gazebo joint. ROS 2
Control documents [effort-interface PID tracking](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html)
for the carriage experiment.
