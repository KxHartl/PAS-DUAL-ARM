---
id: P-12
type: problem
status: otvoreno
requirements: ["[[R-11_door_80cm]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
solutions: ["[[S-02_world_and_sim_launch]]", "[[S-06_navigation]]"]
decisions: ["[[D-08_door_widened]]"]
updated: 2026-09-13
---
# P-12: Vrata od 0.8 m su preuska za Nav2 prolaz

## Simptom
Nav2 nije mogao isplanirati prolaz kroz 0.8 m.

## Uzrok
**Potvrđeno:** robot je širok ~0.6 m (footprint ±0.30 m), a uz `inflation_radius` 0.35 costmap
zatvara otvor. Ispružene ruke su šire od baze.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 23. 6. `45c32f1` | vrata 1.2 m + inflation 0.35 → 0.15 + `ARM_CARRY` (laktovi uvučeni, ~0.6 m) + staging ispred vrata | prolaz headless, x 0.45 → 3.07 | radi, ali nije 0.8 m |
| 2 | 29. 6. `e9157e1` | vrata 2.0 m (usput, razlog nije zapisan) | — | 🔁 udaljilo od zahtjeva |
| 3 | kasnije | `inflation_radius` 0.05 (lokalni i globalni costmap) | nije testirano kroz vrata | kandidat za 0.8 m |

## Trenutno rješenje
Vrata 2.0 m ([[D-08_door_widened]]).

## Sljedeći korak (time-box ~30 min)
1. `seminar_world.sdf` `wall_with_door`: unutarnji rubovi na y = ±0.40 (0.8 m). Zidovi moraju i dalje
   dosezati y = ±3.0, a nadvoj treba skratiti na 0.8 m.
2. Prolaz **bez kutije**: ruke u `ARM_CARRY`, staging `door_xy` (1.5, 0) s yaw 0, pa ravna vožnja
   kroz otvor. Nav2 uz inflation 0.05 ili odometrijska vožnja ako costmap blokira.
3. Provjera zazora: baza ±0.30 u 0.8 m → ±0.10 m. Kutija 0.3 m ispred tijela ne povećava širinu
   ako je unutar širine baze.

**Kriterij uspjeha:** GUI prolaz bez kontakta sa zidom, zatim isto s kutijom ([[R-19_door_pass_with_box]]).

## Ne ponavljati
- Širiti vrata radi Nav2: pravi popravak je costmap/footprint + poravnanje.
