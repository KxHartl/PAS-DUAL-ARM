---
id: P-26
type: problem
status: rijeseno
requirements: ["[[R-17_dual_arm_lift]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]", "[[S-07_moveit_setup]]"]
decisions: ["[[D-06_cube_squeeze_grasp]]"]
updated: 2026-09-13
---
# P-26: Jednostrani press gura kocku po stolu

## Simptom
Kad jedna ruka pritisne prva, kocka se odgura ~0.3 m po stolu.

## Uzrok
**Potvrđeno:** `move_group` izvršava **jednu** trajektoriju odjednom, pa ruke pritišću redom, a ne
istovremeno.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15. 7. | press ruku redom kroz `move_group` | kocka odgurnuta | — |
| 2 | 16. 7. `c504720` | obje MoveIt kartezijske putanje poslati **istovremeno** izravno na `left/right_arm_controller` (FollowJointTrajectory) | sile se poništavaju, kocka ostaje | rješenje |

## Trenutno rješenje
`press_both_linear` (`main_task.py:620`).
