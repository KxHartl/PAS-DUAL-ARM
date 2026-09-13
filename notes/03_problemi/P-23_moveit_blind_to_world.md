---
id: P-23
type: problem
status: rijeseno
requirements: ["[[R-09_moveit_arm_control]]", "[[R-17_dual_arm_lift]]"]
solutions: ["[[S-07_moveit_setup]]"]
decisions: []
updated: 2026-09-13
---
# P-23: MoveIt ne zna za svijet, pa ruka ide kroz stol

## Simptom
RRT je zamahnuo rukom **kroz stol**. Reakcija je odgurnula cijelu bazu metrima dalje (vidio
korisnik u GUI-ju).

## Uzrok
**Potvrđeno:** planning scena je prazna. MoveIt po defaultu ne zna ništa o okolini.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15./16. 7. `c504720` | `publish_collision_scene`: pod, stol (iz visine kocke), kocka kao `CollisionObject` | nema prolaza kroz stol | rješenje |

## Povezano
Nakon attacha kocku treba ukloniti iz scene ([[P-30_stale_collision_object]]).
