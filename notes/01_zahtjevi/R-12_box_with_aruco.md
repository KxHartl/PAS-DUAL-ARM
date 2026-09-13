---
id: R-12
type: zahtjev
status: ispunjeno
source: "[MAIL] + [USM] (dimenzije)"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]", "[[S-05_perception]]"]
problems: ["[[P-08_marker_not_detected_texture]]", "[[P-14_gripper_too_small_for_cube]]"]
decisions: ["[[D-01_aruco_dict_4x4_50]]", "[[D-06_cube_squeeze_grasp]]"]
updated: 2026-09-13
---
# R-12: Kutija s ArUco markerom

## Izvor (doslovno)
> „Na kutiju slobodno stavite aruco markere (https://github.com/pal-robotics/aruco_ros) kako bi
> ju lakše detektirali.“ [MAIL]
> Dimenzije 0.3 × 0.3 × 0.3 m, 1 kg: **[USM]**, nije u mailu ni u task.pdf-u (vidi [[izvori]]).

## Tehnički znači
Dinamički objekt (kutija) s ArUco markerom koji kamera pouzdano detektira i daje pozu.

**Kriterij prihvaćanja:**
- [x] `aruco_box` u svijetu: kocka 0.30 m, 1.0 kg, μ = 5.0, na `pick_table` (1.28, -1.0), yaw -0.68
- [x] marker DICT_4X4_50 ID 0, matiran, na -X plohi (ploča 0.22 m, marker 0.165 m)
- [x] detekcija daje `/aruco_single/pose` + TF `aruco_marker_frame`

## Trenutno stanje
✅ Kocka po [USM] vraćena 15./16. 7. (`c504720`). Prije toga se koristila šipka/ploča koju
hvataljka može obuhvatiti ([[P-14_gripper_too_small_for_cube]]).
⚠ **Za potvrdu:** budući da 0.3 m / 1 kg nije obvezujuće, manja „hvatljiva“ kutija bila bi
legitimna. Ostajemo pri kocki, vidi [[D-06_cube_squeeze_grasp]].

## Kako se rješava
- [[S-02_world_and_sim_launch]]: model kutije
- [[S-05_perception]]: detekcija ([[D-01_aruco_dict_4x4_50]], [[D-02_own_aruco_detector]])
