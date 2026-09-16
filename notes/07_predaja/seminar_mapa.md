---
id: SEMINAR_MAPA
type: plan
updated: 2026-09-13
---
# Mapa seminara: koja kartica hrani koje poglavlje

Format: FSB seminar (`latex_format: fsb-seminar`). Predložak i pravila su **lokalno**, u
`.ai/templates/fsb-seminar/` — od 16. 9. izvan repozitorija (vidi [[vanjski_paketi]]).
Izvor teksta su kartice.

| Poglavlje | Sadržaj | Izvori (kartice) | Slike |
|---|---|---|---|
| 1. UVOD | cilj, alati, motivacija | [[izvori]], R0 u [[00_MAPA]] | slika sustava iz maila |
| 2. ZADATAK I ZAHTJEVI | stablo zahtjeva, izvori, kriteriji prihvaćanja | [[00_MAPA]], `R-01`…`R-21` | stablo (mermaid/canvas) |
| 3. MODEL ROBOTA | baza, torzo, ruke, pan-tilt, kamera; mjere; mase | [[S-01_robot_description]], [[R-01_omni_base]]…[[R-06_realistic_parameters]] | Gazebo render, TF stablo, CAD mjere |
| 4. SIMULACIJSKO OKRUŽENJE | svijet, vrata, stolovi, kutija s markerom, bridge | [[S-02_world_and_sim_launch]], [[R-10_mappable_world]]…[[R-13_destination_place]] | tlocrt svijeta |
| 5. UPRAVLJANJE (ros2_control) | kontroleri, pogon baze | [[S-03_ros2_control_setup]], [[S-04_base_drive]] | tablica kontrolera |
| 6. PERCEPCIJA | ArUco (DICT_4X4_50), RGBD mjerenje, cross-check | [[S-05_perception]], [[D-01_aruco_dict_4x4_50]], [[D-02_own_aruco_detector]] | detekcija markera, oblak |
| 7. NAVIGACIJA | SLAM + Nav2 (M5/M6), visual servo | [[S-06_navigation]], [[D-04_visual_servo_instead_nav2]] | karta u RViz-u |
| 8. MANIPULACIJA (MoveIt2) | SRDF, planiranje, kartezijski press | [[S-07_moveit_setup]], [[P-24_press_path_chain]] | MoveIt u RViz-u |
| 9. DVORUČNI HVAT | squeeze, kontaktni gate, attach | [[S-08_grasp_squeeze_attach]], [[D-05_contact_verified_attach]], [[D-06_cube_squeeze_grasp]], [[D-12_honesty_abort_over_fake]] | sekvenca hvata (4–6 slika) |
| 10. ORKESTRACIJA MISIJE | state machine, koraci, gateovi | [[S-09_task_orchestration]] | dijagram slijeda |
| 11. REZULTATI I ISPITIVANJA | runovi, uspješnost, što je provjereno | [[runovi]], [[timeline]], „Provjereno“ tablice u S-karticama | tablica runova |
| 12. PROBLEMI I RJEŠENJA | 6–8 ključnih P-kartica (tablice pokušaja) | [[P-09_omni_drive_on_fortress]], [[P-11_nav2_slam_drift]], [[P-15_dart_friction_no_hold]], [[P-17_detachable_joint_explodes]], [[P-18_transport_drops_box]], [[P-24_press_path_chain]], [[P-16_fake_teleport_grasp]] | — |
| 13. ODSTUPANJA I OGRANIČENJA | iskreno | [[odstupanja]] | — |
| ZAKLJUČAK | što radi, što ne, sljedeći koraci | [[00_MAPA]], [[danas]] | — |
| LITERATURA | upstream repoi (autori, licence, commitovi → [[vanjski_paketi]]), STL vodilica (B. Ćaran), dokumentacija, predavanja | [[izvori]]; predavanja `03 ros2_control`, `05 MoveIt`, `06 Gazebo, SLAM, navigacija` (`~/FSB/projektiranje-autonomnih-sustava/seminar/sources/docs/`) | — |
| PRILOZI | pokretanje, parametri | `RUNNING.md`, [[06_parametri]] | — |
