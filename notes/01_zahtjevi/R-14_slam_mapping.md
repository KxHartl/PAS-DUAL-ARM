---
id: R-14
type: zahtjev
status: otvoreno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-06_navigation]]"]
problems: ["[[P-11_nav2_slam_drift]]"]
decisions: ["[[D-04_visual_servo_instead_nav2]]"]
updated: 2026-09-13
---
# R-14: Mapiranje slam_toolboxom

## Izvor (doslovno)
> „okruženje u Gazebo-u koje će robot mapirati … Za mapiranje i navigaciju koristiti nav2_stack
> i slam_toolbox.“ [MAIL]

## Tehnički znači
slam_toolbox iz `/scan` i odometrije gradi kartu (`/map`) i objavljuje `map → odom`. Karta
služi Nav2 za planiranje. Karta se može spremiti (map_saver) i pokazati u RViz-u.

**Kriterij prihvaćanja:**
- [ ] slam_toolbox radi tijekom misije, `/map` pokriva obje sobe
- [ ] karta se ne „raspada“ pri okretima u mjestu (lokalizacija ne skače)
- [ ] snimka karte u RViz-u za seminar

## Trenutno stanje
❌ Radilo je 23. 6. (M5, `6eb7487`: puni TF lanac `map → odom → base_footprint`). Napušteno je
30. 6. (`33bc2ac`): skid-steer pri okretu u mjestu kliže → odometrija i scan-matching se razbiju
→ robot „odluta“ ~30 m ([[P-11_nav2_slam_drift]], [[D-04_visual_servo_instead_nav2]]).
`nav2.launch.py` i `nav2_params.yaml` još postoje u repou.

## Kako se rješava
- [[S-06_navigation]]
