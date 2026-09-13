---
id: P-35
type: problem
status: otvoreno
requirements: ["[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]", "[[R-11_door_80cm]]"]
solutions: ["[[S-06_navigation]]", "[[S-09_task_orchestration]]"]
decisions: ["[[D-13_three_room_world]]"]
updated: 2026-09-13
---
# P-35: Ruke su šire od vrata — `ARM_CARRY` nije uska poza

## Simptom
Robot s rukama u „carry“ pozi je **~1.2 m širok**, a vrata su 0.9 m. Baza sama (0.6 m) prolazi bez
problema, ruke ne.

## Mjerenje (13. 9. 2026., TF `base_link`, nakon MoveIt `both_arms` → `ARM_CARRY`)
| Link | lijevo y | desno y |
|---|---|---|
| `shoulder_link` | +0.26 | -0.26 |
| `half_arm_1_link` | +0.35 | -0.36 |
| `half_arm_2_link` | +0.46 | -0.48 |
| **`forearm_link` (lakat)** | **+0.57** | **-0.59** |
| `bracelet_link` | +0.43 | -0.33 |
| `end_effector_link` | +0.48 | -0.31 |

Raspon ishodišta linkova: **1.16 m**, uz polumjer linkova ~0.05 m ukupno **~1.26 m**.
Za prolaz kroz 0.9 m treba |y| < ~0.40 m po svakoj strani.

## Uzrok
**Potvrđeno:** ruke su montirane na **bočne** klizače (ramena već na y = ±0.26), a `ARM_CARRY`
(`j2 = 0.7`, `j4 = -2.5`, `j6 = 1.2`) diže ramena i savija laktove **prema van**, a ne prema
naprijed. Komentar u kodu i bilješka iz lipnja („laktovi uvučeni, raspon ~0.6 m“,
`main_task.py:128-131`) **nisu točni**: nikad nisu izmjereni, a prolaz je tada bio 1.2 m širok.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 23. 6. `45c32f1` | `ARM_CARRY` + vrata 1.2 m | prolaz headless uspio | prošao zbog **širokih vrata**, ne zbog uske poze |
| 2 | 13. 9. | `ARM_CARRY` preko MoveIt-a (`both_arms`), pa izmjeriti TF linkova | MoveIt OK (`error_code=1`), ali širina **1.26 m** | poza je neupotrebljiva za 0.9 m |

## Sljedeći korak (NIJE rađeno danas, samo identificirano)
1. Naći novu pozu „kroz vrata“: simetrična `both_arms` konfiguracija s |y| < 0.40 za sve linkove,
   provjerena `/compute_fk` (geometrija) + `/check_state_validity` (samokolizija). Pripremljena
   skripta za pretragu: `scratchpad/find_door_posture.py` (nije pokrenuta).
2. Ako takva poza ne postoji zbog bočne montaže ramena, opcije su:
   - spustiti klizače i preklopiti ruke ispred tijela (laktovi naprijed, ne u stranu);
   - proširiti vrata (vraća [[D-08_door_widened]], odstupanje);
   - vrata 0.9 m, ali prolaz samo bazom uz ruke gore (provjeriti visinu nadvoja).
3. Tek nakon toga ima smisla testirati Nav2 prolaz ([[P-12_door_too_narrow]]).

**Kriterij uspjeha:** poza u kojoj su svi linkovi ruku unutar |y| ≤ 0.40 m, bez samokolizije, iz
koje se može doći u press pozu.

## Ne ponavljati
- Vjerovati komentaru „ARM_CARRY ~0.6 m“ bez TF mjerenja.
