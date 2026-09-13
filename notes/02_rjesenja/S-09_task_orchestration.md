---
id: S-09
type: rjesenje
status: djelomicno
requirements: ["[[R-15_region_goal_nav2]]", "[[R-16_find_box]]", "[[R-17_dual_arm_lift]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
problems: ["[[P-16_fake_teleport_grasp]]", "[[P-18_transport_drops_box]]", "[[P-21_spin_once_not_pacing_rtf]]", "[[P-25_asymmetric_arm_reach]]"]
decisions: ["[[D-04_visual_servo_instead_nav2]]", "[[D-12_honesty_abort_over_fake]]"]
files: ["src/pas_dual_arm_scripts/pas_dual_arm_scripts/main_task.py", "src/pas_dual_arm_bringup/launch/task.launch.py"]
updated: 2026-09-13
---
# S-09: Orkestracija misije (`main_task.py`)

## Pokretanje
`task.launch.py` = `move_group` + `aruco.launch.py` + `main_task` nakon 12 s
(`use_sim_time: True`, `auto_start` arg).
⚠ **`task.launch.py` NE prosljeđuje ostale parametre** (`probe_transport`, `pick_only`…) u
`main_task`. Za transport-probu treba dodati launch arg ili ga postaviti s
`ros2 param set /main_task_node probe_transport true` prije nego što run dođe do koraka 7
([[P-18_transport_drops_box]]).

## Trenutni slijed (`run()`, l. 1422–1801)
| Korak | Što | Funkcija | Gate / abort |
|---|---|---|---|
| 0 | detach na startu | `_attach_box(False)` | — |
| 1 SCAN | pan kamere dok se ne vidi marker | `scan_for_marker` | marker nije nađen |
| 2 APPROACH | visual servo do ~0.90 m | `visual_approach` | servo nije uspio |
| 3 CONFIRM + MEASURE | marker + dubina s ~0.95 m, cross-check | `confirm_box`, `measure_box` | neslaganje 2× → abort |
| 4 CLOSE-IN | ravno do kocke na ~0.50 m, korekcija odometrijom | `drive` | — |
| 4b CENTER | okret da kocka bude na y ≈ -0.075 | `drive`, `_odom_yaw` | ([[P-25_asymmetric_arm_reach]]) |
| 4c–6 | squeeze, gate, attach, lift | [[S-08_grasp_squeeze_attach]] | gate → abort |
| 7 | transport-proba (param) | `drive` | 🧪 |
| 8 | place na **isti** stol | `move_linear`, `_attach_box(False)` | 🧪 |

## Što nedostaje za misiju iz [MAIL]
Između koraka treba umetnuti (vidi [[danas]]):
- **prije 1:** SLAM + Nav2 do regije koju zada korisnik ([[R-15_region_goal_nav2]]);
- **nakon 6:** carry poza → staging ispred vrata → kroz vrata → pred `place_table` (crvena soba)
  ([[R-19_door_pass_with_box]]);
- **8:** odlaganje na `place_table` (crvena soba) umjesto na isti stol ([[R-20_place_at_destination]]);
- **nakon 8:** povratak kroz vrata bez kutije ([[R-18_door_pass_empty]]).

## Ostaci M6
Parametri `pregrasp_xy`, `door_xy` (1.5, 0), `preplace_xy` (3.0, 0), `table_place_z` 0.85,
`box_grasp_z`, `grasp_half_width` i `pick_only` su deklarirani, ali ih trenutni `run()` **ne
koristi**. Docstring na vrhu datoteke (l. 1–20) opisuje stari M6 slijed i **zastario je**.
