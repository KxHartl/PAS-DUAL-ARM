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
| 6 | 13. 9. | nakon spawna su **svi zglobovi ruku na 0** (ispružena Kinova), pa ruke s bočnih klizača strše u stranu (robot > 2 m širok). Obje ruke u `ARM_CARRY` preko MoveIt-a (`both_arms`, pomoćna skripta kao `main_task.move_arms_joint`) | MoveIt OK, ali TF mjerenje: laktovi (`forearm_link`) na **y = ±0.58**, pa je robot širok ~1.2–1.3 m | **`ARM_CARRY` nije uska poza**; bilješka iz lipnja („~0.6 m“) je netočna → treba posebna poza za vrata (|y| < 0.40), tražena preko `/compute_fk` + `/check_state_validity` |
| 5 | 13. 9. | Nav2 pri pokretanju javlja `[ERROR] inflation radius (0.050) is smaller than the computed inscribed radius (0.310)`: NavFn planira za točku i treba inflaciju ≥ upisanog radijusa, inače put ljubi zidove i dovratke → `inflation_radius` 0.05 → **0.40** (oba costmapa) | 🧪 | koridor u vratima 0.9 m: centar je 0.45 m od dovratka > 0.31 (nije smrtonosno) |
| 7 | 13. 9. | Povećano: `inflation_radius: 0.65`, `cost_scaling_factor: 3.0`, `footprint_padding: 0.03` radi zaobilaženja stola. | ❌ NavFn nije mogao isplanirati prolaz kroz vrata (`GridBased: failed to create plan`). Pading 0.03 m je povećao širinu robota na 0.914 m, preostali luft od 8.6 cm je na mreži 0.05 m spojio lethal zone dovrataka i zatvorio vrata. | Vraćeno na `inflation_radius: 0.45`, `cost_scaling_factor: 5.0`, bez paddinga. Za stol treba prepoznati/označiti punu ploču stola jer lidar vidi samo 4 noge dok je ploča iznad snopa. |
| 8 | 13. 9. | Prilagodba po zahtjevu korisnika: `footprint` širine 0.95 m (`y = ±0.475`), ostavlja točno 5 cm lufta u vratima 1.0 m; `inflation_radius: 0.48`, `use_astar: true` u Navfn; stolovi ispunjeni u `seminar_map.pgm`. | 🧪 u testiranju | Širi footprint sprječava rotaciju ruku u vratima; Nav2 planira čistu ravnu liniju kroz sredinu. |

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

## Povezano
Prije ovoga treba riješiti **širinu ruku**: [[P-35_arm_span_too_wide_for_door]] (robot je u carry
pozi ~1.26 m širok, vrata su 0.9 m). Dok to stoji, Nav2 test prolaza nema smisla.

## Ne ponavljati
- Širiti vrata radi Nav2: pravi popravak je costmap/footprint + poravnanje.
