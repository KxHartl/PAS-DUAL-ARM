---
id: P-25
type: problem
status: rijeseno
requirements: ["[[R-17_dual_arm_lift]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]", "[[S-09_task_orchestration]]"]
decisions: []
updated: 2026-09-13
---
# P-25: Zone dosega lijeve i desne ruke se ne preklapaju

## Simptom
S kockom na y ≈ -0.13 desna ruka je savršena, a lijeva kronično ne može ravnu liniju (preko
središta torza). S kockom na y ≈ -0.02 je obrnuto.

## Uzrok
**Potvrđeno empirijski:** asimetrična montaža (desni klizač rotiran 180°, klinovi pod 45°) daje
različite radne prostore za press.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15. 7. | kocka gdje je prilaz ostavi (y ≈ -0.13) | lijeva pada | — |
| 2 | 16. 7. | centrirati kocku na y = 0 | desna pada | — |
| 3 | 16. 7. `c504720` | **centrirajući okret na y ≈ -0.075** (sredina), kut iz odometrije | obje ruke izvedive | rješenje |

## Trenutno rješenje
`main_task.py:1525` (`center.y + 0.075`). Vrijednost je u [[06_parametri]].
