---
id: S-06
type: rjesenje
status: odstupanje
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-11_nav2_slam_drift]]", "[[P-12_door_too_narrow]]", "[[P-19_aruco_foreshortening_close]]", "[[P-33_nav2_undershoot_base_shift]]", "[[P-39_nav2_enters_doorway_at_an_angle]]"]
decisions: ["[[D-04_visual_servo_instead_nav2]]", "[[D-08_door_widened]]", "[[D-16_zones_from_detected_features]]"]
files: ["src/pas_dual_arm_bringup/launch/nav2.launch.py", "src/pas_dual_arm_bringup/config/nav2_params.yaml", "src/pas_dual_arm_scripts/pas_dual_arm_scripts/nav_zones.py", "src/pas_dual_arm_scripts/pas_dual_arm_scripts/room_navigator.py", "src/pas_dual_arm_scripts/pas_dual_arm_scripts/feature_registry.py", "scripts/nav_gui.py", "scripts/check_doors.py", "scripts/check_zones.py"]
updated: 2026-09-14
---
# S-06: Navigacija (Nav2 + zone iz detektiranih značajki)

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


## C) Zone iz detektiranih značajki (od 14. 9.): AKTUALNO
[[D-16_zones_from_detected_features]], razlog [[P-39_nav2_enters_doorway_at_an_angle]].
Nav2 i dalje vozi; promijenjeno je **u čemu smije planirati**.

```
/map ──▶ feature_registry ──▶ vrata (centar, normala, širina), stolovi (klasteri nogu)
                                   │
                              nav_zones
                 ┌─────────────────┼──────────────────┐
       /keepout_filter_mask   /nav_graph      /nav_zones_markers
       (KeepoutFilter na           │               (RViz)
        GLOBAL *i* LOCAL)    room_navigator ──▶ Nav2 NavigateToPose
                                   ▲
                            scripts/nav_gui.py (tipke)
```

**Tri sloja**
1. *Prostor*: lijevci uz svaka vrata ostavljaju traku širine izmjerenog otvora, dugu 1.2 m u
   svaku sobu; halo oko stola otvoren na licu prilaza. Filter je na **oba** costmapa.
2. *Poza*: dva cilja po vratima na osi prolaza, s točnim yawom iz normale vrata; tolerancije
   0.10 m / 0.05 rad, `RotationShimController` 0.06 rad.
3. *Nadzor*: prije prolaza se provjeri `ARM_CARRY_V2` i stvarna poravnatost; tijekom prolaza se
   prati bočni razmak iz `/scan_filtered` i cilj se **otkazuje**, ne struže ([[D-12_honesty_abort_over_fake]]).

**Sobe se ne upisuju** — dobiju se segmentacijom: zatvori prolaze, označi povezani slobodan
prostor, odbaci komponentu koja dodiruje rub karte (to je vanjština zgrade). Iz karte ispada
3 sobe, 2 vrata, 2 stola. Jedino ručno je **ime** sobe (`room_labels`), jer occupancy grid nema boju.

**Pokretanje**
```bash
ros2 launch pas_dual_arm_bringup nav2.launch.py mode:=localization   # mode je sada default
python3 scripts/nav_gui.py                                            # tipke PLAVA / CRVENA / HOME / STOP
ros2 topic pub --once /room_navigator/goto std_msgs/String "{data: blue}"   # isto bez GUI-ja
```
`zones:=false` isključuje zone za A/B usporedbu.

**Offline provjere (bez simulatora)**
```bash
python3 scripts/check_doors.py            # detekcija vrata nad spremljenom kartom
python3 scripts/check_zones.py --png /tmp/zones.png   # zone, graf soba, sve poze
```

## Zašto je B postojao
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
| 14. 9. | offline | detekcija vrata iz `seminar_map`: (0.00, −3.00) i (3.00, −0.01), širina 0.95 m | `497a69a` |
| 14. 9. | offline | zone, graf 3 soba, portali i prilazne poze; sve provjere prolaze | `497a69a` |
| 14. 9. | headless | stack se digne, keepout filter aktivan na **oba** costmapa, 14/14 sondi costmapa | `497a69a` |
| 14. 9. | headless | planirane putanje sijeku prag vrata pod 0.00–1.07° i 2.5 cm od osi (5 slučajeva) | `497a69a` |
| 23. 6. | headless | SLAM + Nav2 goal → baza vozi | `6eb7487` |
| 23. 6. | headless | prolaz kroz 1.2 m vrata | `45c32f1` |
| 30. 6. | GUI | visual servo do kutije | `33bc2ac` |
