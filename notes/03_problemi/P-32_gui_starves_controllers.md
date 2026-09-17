---
id: P-32
type: problem
status: zaobideno
requirements: ["[[R-07_ros2_humble_fortress_control]]"]
solutions: ["[[S-02_world_and_sim_launch]]"]
decisions: ["[[D-10_headless_vs_gui]]"]
updated: 2026-09-13
---
# P-32: GUI renderer izgladnjuje `gz_ros2_control` (aktivacija kontrolera istekne)

## Simptom
Uz Gazebo GUI na opterećenom stroju spawneri kontrolera isteknu.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. | `--controller-manager-timeout 120` | pomaže (učitavanje traje 50–60 s) | — |
| 2 | 23. 6. | `sim.launch.py headless:=true` (samo server) | pouzdano | za automatske provjere |
| 3 | 30. 6. – 16. 7. | GUI runovi (s timeoutom) | rade | GUI je OK za demo |

## Trenutno rješenje
[[D-10_headless_vs_gui]]. Za video: GUI + pričekati da se svi kontroleri aktiviraju prije
`task.launch.py`.
