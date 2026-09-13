---
id: P-11
type: problem
status: otvoreno
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-08_omni_controller]]"]
solutions: ["[[S-06_navigation]]", "[[S-04_base_drive]]"]
decisions: ["[[D-04_visual_servo_instead_nav2]]"]
updated: 2026-09-13
---
# P-11: Nav2/SLAM lokalizacija odluta (~30 m) pri okretima u mjestu

## Simptom
Tijekom 360° spina za traženje markera lokalizacija je skočila ~30 m i robot je odvozio daleko
(30. 6., `580948b`).

## Uzrok
**Potvrđeno (djelomično):** skid-steer se okreće klizanjem kotača ([[P-10_skid_steer_cannot_turn]]),
pa wheel-odometrija daje krivi yaw. slam_toolbox scan-matching tada ne uspijeva i karta/`map → odom`
skače.
**Hipoteza:** brzi okret (0.4–0.8 rad/s) pogoršava stvar. Sporiji okret + manje oslanjanja na
odometriju mogao bi biti dovoljan.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 23. 6. `6eb7487` `45c32f1` | SLAM + Nav2, pretežno ravne vožnje (M5/M6) | radi, kroz 1.2 m vrata | ravne dionice su OK |
| 2 | 30. 6. `f62ebbd` | Nav2 `/spin` 360° za scan + iterativni prilaz | drift ~30 m | okret u mjestu razbija lokalizaciju |
| 3 | 30. 6. `33bc2ac` | izbaciti Nav2/SLAM: pan kamere uz mirnu bazu + cmd_vel servo | prilaz pouzdan | zaobiđeno ([[D-04_visual_servo_instead_nav2]]) |

## Trenutno rješenje
Bez Nav2/SLAM, što **odstupa od obaveznih** [[R-14_slam_mapping]] i [[R-15_region_goal_nav2]].

## Sljedeći korak (hibrid, time-box ~60 min, vidi [[danas]])
1. SLAM + Nav2 samo za **putovanje**: regija (zadaje korisnik, RViz Goal) → staging ispred vrata →
   kroz vrata → pred `place_table` (crvena soba). Traženje markera ide **pan-tilt kamerom uz mirnu bazu** (već
   radi), a finalni prilaz ostaje visual servo.
2. Okreti spori (ω ≤ 0.3); Nav2 `max_vel_theta` spustiti. U slam_toolboxu provjeriti
   `minimum_travel_heading` / `minimum_travel_distance`.
3. Ako se prijeđe na mecanum ([[P-09_omni_drive_on_fortress]]), ponoviti test: je li drift manji?
4. Test: okret 360° na mjestu uz SLAM → karta konzistentna (RViz), `map → odom` bez skoka > 0.2 m.

**Kriterij uspjeha:** karta obje sobe u RViz-u + jedan Nav2 cilj koji je zadao čovjek, postignut bez
drifta.

## Ne ponavljati
- Brzi 360° spin skid-steera uz aktivan SLAM kao metoda traženja.
