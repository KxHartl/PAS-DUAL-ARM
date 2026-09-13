---
id: P-12
type: problem
status: neprovjereno
requirements: ["[[R-11_door_80cm]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
solutions: ["[[S-02_world_and_sim_launch]]", "[[S-06_navigation]]"]
decisions: ["[[D-13_three_room_world]]", "[[D-08_door_widened]]"]
updated: 2026-09-13
---
# P-12: Prolaz kroz uska vrata (0.8/0.9 m) s Nav2

## Simptom
Nav2 nije mogao isplanirati prolaz kroz 0.8 m (lipanj).

## Uzrok
**Potvrđeno:** robot je širok ~0.6 m (footprint ±0.30 m), a uz `inflation_radius` 0.35 costmap
zatvara otvor. Ispružene ruke su šire od baze.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 23. 6. `45c32f1` | vrata 1.2 m + inflation 0.35 → 0.15 + `ARM_CARRY` (laktovi uvučeni, ~0.6 m) + staging ispred vrata | prolaz headless, x 0.45 → 3.07 | radi, ali nije standardna širina |
| 2 | 29. 6. `e9157e1` | vrata 2.0 m (usput, razlog nije zapisan) | — | 🔁 udaljilo od zahtjeva ([[D-08_door_widened]]) |
| 3 | kasnije | `inflation_radius` 0.05 (lokalni i globalni costmap) | nije testirano kroz vrata | kandidat |
| 4 | 13. 9. | novi svijet: dvoja vrata **0.9 m** ([[D-13_three_room_world]]) | 🧪 prolaz nije testiran | — |

## Trenutno rješenje
Vrata 0.9 m u svijetu. Prolaz tek treba provjeriti.

## Sljedeći korak (time-box ~30 min, u sklopu Nav2 koraka iz [[danas]])
1. Ruke u `ARM_CARRY` prije svakog prolaza.
2. Nav2 s footprintom ±0.45 × ±0.30 i inflacijom 0.05: koridor je 0.9 − 2 × 0.35 = 0.2 m, što bi
   planer trebao naći. Ako ne nađe: staging poza 0.8 m ispred vrata, poravnanje, pa ravna vožnja
   kroz otvor (odometrija).
3. GUI provjera: nema kontakta sa zidom (i `/scan` minimum > 0.1 m).

**Kriterij uspjeha:** GUI prolaz HOME → PLAVA i HOME → CRVENA bez kontakta, zatim isto s kutijom
([[R-19_door_pass_with_box]]).

## Ne ponavljati
- Širiti vrata radi Nav2: pravi popravak je costmap/footprint + poravnanje.
