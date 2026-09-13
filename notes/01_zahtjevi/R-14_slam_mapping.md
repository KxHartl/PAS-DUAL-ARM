---
id: R-14
type: zahtjev
status: riješeno
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
- [x] slam_toolbox radi tijekom misije, `/map` pokriva sve tri sobe
- [x] karta se ne „raspada“ pri okretima u mjestu (zahvaljujući popravku mecanum pogona)
- [x] snimka karte u RViz-u spremljena za seminar (`maps/seminar_map.*`)

## Trenutno stanje
✅ Mapiranje triju soba uspješno završeno uz mecanum omni kontroler i `ARM_CARRY_V2` pozu.
Karta pokriva 102.3 m² čistog prostora s rasponom 11.9 × 11.8 m, čisti 1.0 m prolazi i noge stolova.
Spremljeno u `src/pas_dual_arm_bringup/maps/seminar_map.*`.

## Kako se rješava
- [[S-06_navigation]]
