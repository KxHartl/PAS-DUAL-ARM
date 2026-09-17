---
id: P-34
type: problem
status: rijeseno
requirements: ["[[R-07_ros2_humble_fortress_control]]", "[[R-21_deliverables]]"]
solutions: ["[[S-10_build_run_environment]]"]
decisions: []
updated: 2026-09-13
---
# P-34: Vanjski paketi nisu rekonstruktibilni iz čistog clonea

## Simptom
Pet paketa u `src/` su gitlinkovi bez `.gitmodules`, pa svježi clone ne može složiti workspace.
Lokalna izmjena `robotiq_2f_85_macro.xacro` nije bila zabilježena.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 10. 9. | `ros2.repos` (vcstool, pin na commit) + `patches/` + `apply_patches.sh` + `.gitattributes` za byte-exact zakrpe | reproducibilno | rješenje |

## Trenutno rješenje
README §2: `vcs import src < ros2.repos && ./scripts/apply_patches.sh`.
