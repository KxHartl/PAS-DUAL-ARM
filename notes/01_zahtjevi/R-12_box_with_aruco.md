---
id: R-12
type: zahtjev
status: ispunjeno
source: "[MAIL]; dimenzije i masa slobodne (korisnik 13. 9.)"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]", "[[S-05_perception]]"]
problems: ["[[P-08_marker_not_detected_texture]]", "[[P-14_gripper_too_small_for_cube]]"]
decisions: ["[[D-01_aruco_dict_4x4_50]]", "[[D-06_cube_squeeze_grasp]]", "[[D-14_light_box_free_size]]"]
updated: 2026-09-13
---
# R-12: Kutija s ArUco markerom

## Izvor (doslovno)
> „Na kutiju slobodno stavite aruco markere (https://github.com/pal-robotics/aruco_ros) kako bi
> ju lakše detektirali.“ [MAIL]
> Dimenzije i masa **nisu nigdje zadane**. Korisnik 13. 9.: proizvoljne, lagana („plastika“),
> odabrati tako da je kutiju što lakše naći i podignuti.

## Tehnički znači
Dinamički objekt (kutija) s ArUco markerom koji kamera pouzdano detektira i daje pozu. Veličina je
odabrana prema mogućnostima hvata.

**Kriterij prihvaćanja:**
- [x] `aruco_box`: kocka 0.30 m, **0.3 kg**, μ 5.0, na `pick_table` u plavoj sobi (0, -4.28), yaw -π/2
- [x] marker DICT_4X4_50 ID 0, matiran, na plohi prema vratima plave sobe (ploča 0.22 m, marker 0.165 m)
- [x] detekcija daje `/aruco_single/pose` + TF `aruco_marker_frame`

## Trenutno stanje
✅ Od 13. 9. masa je 0.3 kg, a veličina 0.30 m je zadržana ([[D-14_light_box_free_size]]), jer je
dvoručni squeeze ugođen i provjeren za nju ([[D-06_cube_squeeze_grasp]]). Prije 15. 7. se koristila
šipka/ploča koju hvataljka može obuhvatiti ([[P-14_gripper_too_small_for_cube]]).

## Kako se rješava
- [[S-02_world_and_sim_launch]]: model kutije
- [[S-05_perception]]: detekcija ([[D-01_aruco_dict_4x4_50]], [[D-02_own_aruco_detector]])
