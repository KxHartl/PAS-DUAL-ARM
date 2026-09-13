---
id: TIMELINE
type: povijest
updated: 2026-09-13
---
# Povijest projekta (timeline)

| Datum | Faza | Što se dogodilo | Commit | Kartice |
|---|---|---|---|---|
| 30. 11. 2025. | zadatak | task.pdf (B. Ćaran) | — | [[izvori]] |
| 30. 4. 2026. | zadatak | upit asistentu za podatke o komponentama | — | — |
| 4. 5. | zadatak | **mail s konkretnim zahtjevima** + prilog (vodilice) + slika | — | [[izvori]] |
| 25.–26. 5. | setup | workspace, 5 upstream repoa, prvi URDF, svijet s 0.8 m vratima, kocka 0.3 m | `5d0eca0`…`90fd347` | [[S-01_robot_description]] |
| 12. 6. | geometrija | kalibracija geometrije uz korisnikove vizualne provjere | `5b96ffc`…`d1af724` | [[R-05_visual_match]] |
| 12. 6. | M0 + M1 | ros2_control u Fortressu, Fast DDS, `ParameterValue` | `7aaab94` | [[P-01_shell_zenoh_contamination]], [[P-02_robot_description_yaml_parse]], [[P-03_pal_base_classic_control]] |
| 12. 6. | M2 | lidar + kamera (native Ignition), mesh putanje, negativne skale | `ae60d9b`, `d29498a`, `f79e393` | [[P-04_mesh_uri_not_found]], [[P-05_negative_mesh_scale_dart]], [[P-06_classic_only_sensors]] |
| 12. 6. | M3 | MoveIt2 config za cijelog robota | `b7cfcd2` | [[S-07_moveit_setup]] |
| 12. 6. | M4 | vlastiti ArUco detektor, novi PNG markera | `45ba359` | [[P-07_aruco_dict_and_cv_bridge]], [[D-02_own_aruco_detector]] |
| 23. 6. | M5 | diff_drive baza, SLAM + Nav2, `cmd_vel_relay` | `6eb7487` | [[D-03_diff_drive_base_temporary]], [[S-06_navigation]] |
| 23. 6. | M6 | puni pick-carry-place (headless), vrata 1.2 m, **hvat = teleport** | `45c32f1` | [[P-12_door_too_narrow]], [[P-16_fake_teleport_grasp]] |
| 29. 6. | percepcija | matiran marker, detekcija upravlja hvatom; vrata → 2.0 m | `e9157e1` | [[P-08_marker_not_detected_texture]], [[D-08_door_widened]] |
| 30. 6. | hvat | stvarni hvat šipke + lift (DetachableJoint), torzo ne diže | `397f080`, `d346d71` | [[P-13_torso_prismatic_no_lift]], [[P-14_gripper_too_small_for_cube]] |
| 30. 6. | autonomija | RGBD, spin → find → approach; mu2 = 0; trenje ne drži | `e31f2fa`, `f62ebbd`, `580948b` | [[P-10_skid_steer_cannot_turn]], [[P-15_dart_friction_no_hold]], [[P-11_nav2_slam_drift]] |
| 30. 6. | autonomija | **Nav2/SLAM napušten**, visual servo | `33bc2ac` | [[D-04_visual_servo_instead_nav2]] |
| 30. 6. | hvat | kontaktom verificiran attach; nosi lijevi zglob | `ec766a2`, `5c9e201`, `90e7638` | [[D-05_contact_verified_attach]], [[D-07_carry_on_left_wrist]], [[P-17_detachable_joint_explodes]] |
| 15.–16. 7. | kocka | **kocka po zadatku + dvoručni squeeze**, 3 GUI ciklusa | `c504720` | [[D-06_cube_squeeze_grasp]], [[P-24_press_path_chain]]…[[P-27_contact_sensor_topic_ignored]] |
| 16. 7. (b) | kocka | kontaktni place, REMOVE kocke, transport-proba, popušten gate (**neprovjereno**) | `73617e8` | [[P-28_gate_too_strict]], [[P-29_place_drop_tips_cube]], [[P-30_stale_collision_object]], [[P-18_transport_drops_box]] |
| 10. 9. | okoliš | projektni ROS okoliš, čisti rebuild, provenance | `31df80e`, `eeefb28`, `ebd392e`, `f046e98` | [[D-11_project_scoped_ros_env]], [[P-31_apt_upgrade_breakage]], [[P-34_source_provenance]] |
| 13. 9. | organizacija | Obsidian bilješke (ovo), gap analiza, plan zadnjeg dana | `5f6d016` | [[00_MAPA]], [[danas]] |
| 13. 9. | svijet | **tri sobe u L** (HOME / PLAVA s kutijom / CRVENA odredište), vrata 0.9 m, kutija 0.3 kg; redoslijed misije iz [MAIL]: SLAM prvo | (sljedeći commit) | [[D-13_three_room_world]], [[D-14_light_box_free_size]] |

## Obrasci iz povijesti (za pouku)
- **Dva puta** smo „zaobišli“ obavezni zahtjev tehničkim prečacem ([[D-03_diff_drive_base_temporary]],
  [[D-04_visual_servo_instead_nav2]]) i nismo ga zapisali kao odstupanje. Bilješke to sada ispravljaju.
- Najviše vremena je otišlo na hvat kocke (lipanj–srpanj), a transport i vrata su ostali za kraj.
- Svaki veliki napredak došao je nakon što je korisnik u GUI-ju **vidio** problem (ruka kroz stol,
  „čudne rotacije“, zavarivanje iz daljine).
