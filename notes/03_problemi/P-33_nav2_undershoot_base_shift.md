---
id: P-33
type: problem
status: otvoreno
requirements: ["[[R-15_region_goal_nav2]]", "[[R-20_place_at_destination]]"]
solutions: ["[[S-06_navigation]]"]
decisions: []
updated: 2026-09-13
---
# P-33: Baza stane kraće od Nav2 cilja; pokreti ruku pomiču bazu

## Simptom
U M6 (23. 6.) se baza pri odlaganju slegnula na x ≈ 2.5 umjesto 3.0. Reakcije pokreta ruku
pomiču bazu. Ekstremni slučaj je [[P-23_moveit_blind_to_world]] (ruka kroz stol → baza odgurnuta).

## Uzrok
**Hipoteza:** `xy_goal_tolerance` (0.25) + reakcijske sile ruku na bazu s niskim trenjem kotača
(mu2 = 0).

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 23. 6. `45c32f1` | ništa (zabilježeno kao fino ugađanje) | — | — |

## Sljedeći korak
Ako se Nav2 vrati ([[P-11_nav2_slam_drift]]): nakon Nav2 cilja lokalna korekcija (odometrija ili
servo na `place_table`) prije odlaganja. Ne gibati ruke dok baza vozi.
