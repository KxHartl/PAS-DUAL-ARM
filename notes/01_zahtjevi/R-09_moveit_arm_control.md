---
id: R-09
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-03_ros2_control_setup]]", "[[S-07_moveit_setup]]"]
problems: ["[[P-23_moveit_blind_to_world]]", "[[P-24_press_path_chain]]"]
decisions: []
updated: 2026-09-13
---
# R-09: Ruke na ros2_control, upravljanje MoveIt-om

## Izvor (doslovno)
> „ruke također moraju biti na ros2_control i upravljanje mora biti s MoveIt-om“ [MAIL]

## Tehnički znači
Ruke izvršavaju trajektorije preko `JointTrajectoryController`. Gibanja planira MoveIt2
(`move_group`: IK, OMPL planiranje, kartezijske putanje), s ispravnom SRDF grupom i planning
scenom.

**Kriterij prihvaćanja:**
- [x] `pas_dual_arm_moveit_config` (SRDF: `left_arm`, `right_arm`, `both_arms`, torzo, pan-tilt, hvataljke)
- [x] `move_group` spojen na sve kontrolere, planira i izvršava
- [x] hvat koristi MoveIt (IK `/compute_ik`, `/compute_cartesian_path`, `/move_action`, `/execute_trajectory`)

## Trenutno stanje
✅ Od 12. 6. (M3). Napomena za seminar: **simultani press** obje ruke šalje putanje
koje je izračunao MoveIt (`compute_cartesian_path`) **izravno na oba JTC-a**, jer `move_group`
izvršava jednu trajektoriju odjednom ([[P-26_one_sided_press_bulldozes]]). Planiranje je i dalje
MoveIt-ovo.

## Problemi
- [[P-23_moveit_blind_to_world]]: bez planning scene ruka ide kroz stol
- [[P-24_press_path_chain]]: RRT obilasci, parametrizacija, 2π unwrap
