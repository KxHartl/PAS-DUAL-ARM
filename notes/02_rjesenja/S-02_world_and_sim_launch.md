---
id: S-02
type: rjesenje
status: djelomicno
requirements: ["[[R-07_ros2_humble_fortress_control]]", "[[R-10_mappable_world]]", "[[R-11_door_80cm]]", "[[R-12_box_with_aruco]]", "[[R-13_destination_place]]"]
problems: ["[[P-02_robot_description_yaml_parse]]", "[[P-04_mesh_uri_not_found]]", "[[P-08_marker_not_detected_texture]]", "[[P-12_door_too_narrow]]", "[[P-32_gui_starves_controllers]]"]
decisions: ["[[D-08_door_widened]]", "[[D-10_headless_vs_gui]]"]
files: ["src/pas_dual_arm_bringup/worlds/seminar_world.sdf", "src/pas_dual_arm_bringup/launch/sim.launch.py", "src/pas_dual_arm_bringup/config/bridge.yaml"]
updated: 2026-09-13
---
# S-02: Svijet + pokretanje simulacije

## Što radi
`sim.launch.py` digne Ignition Fortress sa `seminar_world.sdf`, spawna robota, pokrene
`ros_gz_bridge` i spawnere za svih 8 kontrolera.

## Svijet (`seminar_world.sdf`)
| Model | Poza | Bitno |
|---|---|---|
| `wall_with_door` | x = 2.0 | zidovi y ∈ [1, 3] i [-3, -1], nadvoj z 2.0–2.5 → **otvor 2.0 m** ([[D-08_door_widened]]) |
| `pick_table` | (1.40, -1.0) | niski stol, ploča 0.4 × 0.5 m na z ≈ 0.10 |
| `aruco_box` | (1.28, -1.0, 0.25), yaw -0.68 | kocka 0.30 m, 1.0 kg, μ 5.0; ploča markera 0.22 m na -X plohi, matirana |
| `place_table` | (1.55, 0.25) | pomoćni stol ispred zida (0.6 × 0.6) |
| `target_table` | (4.0, 0) | **odredište iza vrata**, ploča 1.0 × 1.5, gornja ploha ≈ 0.775 m |

Kutija je zakrenuta (yaw -0.68) da marker gleda prema ishodištu robota (frontalna detekcija).

## Launch
- `IGN_GAZEBO_RESOURCE_PATH` iz svih `AMENT_PREFIX_PATH/share` ([[P-04_mesh_uri_not_found]]).
- `robot_description` omotan u `ParameterValue` ([[P-02_robot_description_yaml_parse]]), a
  negativne skale uklonjene.
- `headless:=true` → samo server (`-s -r`) ([[P-32_gui_starves_controllers]], [[D-10_headless_vs_gui]]).
- Spawneri s `--controller-manager-timeout 120` (Fortressu treba 50–60 s za učitavanje).
- RMW forsiran na Fast DDS ([[P-01_shell_zenoh_contamination]]).

## Bridge (`bridge.yaml`)
`/clock`, `/scan`, `/camera/{image,camera_info,depth_image,points}` i 4 kontaktna topica. Duge
gz putanje kontakata mapiraju se na `/contact/<ruka>_<prst>_tip`
([[P-27_contact_sensor_topic_ignored]]).

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 12. 6. | headless | 7 kontrolera aktivno, senzori | `7aaab94`, `ae60d9b` |
| 16. 7. | GUI | cijela misija do lifta | `c504720` |

## Otvoreno
- Vrata 2.0 → 0.8 m ([[R-11_door_80cm]]).
- Visina `target_table` (0.775) vs doseg ([[R-13_destination_place]]).
