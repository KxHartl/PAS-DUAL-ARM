---
id: P-19
type: problem
status: rijeseno
requirements: ["[[R-16_find_box]]"]
solutions: ["[[S-05_perception]]", "[[S-06_navigation]]"]
decisions: ["[[D-04_visual_servo_instead_nav2]]"]
updated: 2026-09-13
---
# P-19: Marker se gubi kad je robot blizu kutije

## Simptom
U vizualnom prilazu ArUco nestaje ispod ~0.9 m.

## Uzrok
**Potvrđeno:** nisko postavljen vertikalni marker traži strm nagib kamere, pa se marker
foreshorten-a i detektor ga odbaci.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `33bc2ac` | servo samo do ~0.9 m, zatim ravni dovoz + mjerenje dubinom (dubina ne pati od kuta) | pouzdan prilaz | rješenje |
| 2 | 16. 7. `c504720` | mjerenje s 0.95 m **prije** dovoza, dovoz korigiran odometrijom | točan centar nakon dovoza | poboljšanje ([[P-22_depth_self_view_clusters]]) |

## Trenutno rješenje
`visual_approach(target_x=0.90)`, `look_down(0.65)`, `measure_box`, `drive` + odometrija.
