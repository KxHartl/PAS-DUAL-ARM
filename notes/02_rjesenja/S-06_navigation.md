---
id: S-06
type: rjesenje
status: odstupanje
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-11_nav2_slam_drift]]", "[[P-12_door_too_narrow]]", "[[P-19_aruco_foreshortening_close]]", "[[P-33_nav2_undershoot_base_shift]]"]
decisions: ["[[D-04_visual_servo_instead_nav2]]", "[[D-08_door_widened]]"]
files: ["src/pas_dual_arm_bringup/launch/nav2.launch.py", "src/pas_dual_arm_bringup/config/nav2_params.yaml", "src/pas_dual_arm_scripts/pas_dual_arm_scripts/cmd_vel_relay.py", "src/pas_dual_arm_scripts/pas_dual_arm_scripts/main_task.py"]
updated: 2026-09-13
---
# S-06: Navigacija (Nav2 + SLAM → trenutno visual servo)

## Dvije implementacije

### A) Nav2 + slam_toolbox (M5/M6, 23. 6.): u repou, trenutno se NE koristi
- `nav2.launch.py`: slam_toolbox (`map → odom`) + Nav2 (NavFn planer, DWB kontroler). Nav2 je
  odgođen 5 s (`TimerAction`) da SLAM krene prvi.
- `cmd_vel_relay`: Nav2 `/cmd_vel` → `/base_controller/cmd_vel_unstamped` (kontroler živi u gz
  controller_manageru, pa običan remap ne radi).
- `nav2_params.yaml`: footprint ±0.45 × ±0.30 m, `inflation_radius` **0.05** (bio je 0.35 → 0.15
  → 0.05), `xy_goal_tolerance` 0.10/0.25, `max_vel_x` 0.5, `max_vel_theta` 1.0.
- M6 slijed: nav do kutije → staging ispred vrata (`door_xy` (1.5, 0)) → kroz vrata → `preplace_xy`
  (3.0, 0). Provjereno headless kroz 1.2 m vrata (x 0.45 → 3.07).

### B) Visual servo bez Nav2/SLAM (od 30. 6.): TRENUTNO
[[D-04_visual_servo_instead_nav2]]:
- `scan_for_marker()`: pan kamere [0, -0.5, -1.0, 0.5, 1.0] uz pitch 0.7, baza miruje.
- `visual_approach()`: `_servo_turn` (okret dok |bearing| < 0.08, ω ≤ 0.4) → `_servo_creep`
  (vožnja do `target_x` + 0.06; okret samo ako |bearing| > 0.22; v ∈ [0.06, 0.15]).
- Nema karte, nema regije, nema vrata.

## Zašto B
Skid-steer pri okretu u mjestu kliže → wheel-odom i scan-matching se razbiju → lokalizacija skoči
~30 m ([[P-11_nav2_slam_drift]]).

## Put natrag na A (za [[R-14_slam_mapping]], [[R-15_region_goal_nav2]])
Hipoteza je da pravi omni/mecanum pogon ([[P-09_omni_drive_on_fortress]]) smanjuje klizanje, a time
i drift. Nezavisno od toga, SLAM i Nav2 mogu voziti **dugačke ravne dionice** (regija → vrata →
odredište), a visual servo ostaje samo za finalni prilaz kutiji. Tako se okreti u mjestu ne rade
dok SLAM gradi kartu. Detalji su u [[P-11_nav2_slam_drift]] → „Sljedeći korak“.

## Stanje 13. 9. (podignuto, ali NIJE provjereno vožnjom)
- `nav2.launch.py` sada sam pokreće i `cmd_vel_relay` (prije se morao pokretati ručno).
- DWB i velocity smoother usporeni radi [[P-11_nav2_slam_drift]] (v 0.3, ω 0.4).
- `inflation_radius` 0.05 → 0.40 ([[P-12_door_too_narrow]]).
- Provjereno samo da se stack digne i da `/map` postoji. **Nijedan Nav2 cilj nije vožen**, jer
  robot u carry pozi ne stane kroz vrata ([[P-35_arm_span_too_wide_for_door]]).

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 23. 6. | headless | SLAM + Nav2 goal → baza vozi | `6eb7487` |
| 23. 6. | headless | prolaz kroz 1.2 m vrata | `45c32f1` |
| 30. 6. | GUI | visual servo do kutije | `33bc2ac` |
