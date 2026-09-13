---
id: P-30
type: problem
status: neprovjereno
requirements: ["[[R-20_place_at_destination]]", "[[R-17_dual_arm_lift]]"]
solutions: ["[[S-07_moveit_setup]]", "[[S-08_grasp_squeeze_attach]]"]
decisions: []
updated: 2026-09-13
---
# P-30: Zamrznuti kolizijski objekt kocke blokira planiranje nakon attacha

## Simptom
Nakon attacha svi RRT fallbackovi (release, lower, retreat) padaju s fraction 0.0x.

## Uzrok
**Potvrđeno (analizom):** `target_cube` je ostao u planning sceni na mjestu hvata, a kocka se sad
giba s rukom. Ruka „sudara“ vlastiti teret.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 16. 7. `73617e8` | `CollisionObject.REMOVE` za `target_cube` odmah nakon attacha | 🧪 nije pokrenuto | — |

## Sljedeći korak
Provjeriti u runu 31+. Kasnije razmotriti `AttachedCollisionObject` na `left_bracelet_link`, da
MoveIt planira s kockom u ruci. To je važno za prolaz kroz vrata s kockom.
