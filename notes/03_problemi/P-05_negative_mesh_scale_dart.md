---
id: P-05
type: problem
status: rijeseno
requirements: ["[[R-02_kinova_arms]]", "[[R-07_ros2_humble_fortress_control]]"]
solutions: ["[[S-01_robot_description]]", "[[S-02_world_and_sim_launch]]"]
decisions: []
updated: 2026-09-13
---
# P-05: DART abortira na negativnim skalama meshova

## Simptom
Nakon popravka meshova ([[P-04_mesh_uri_not_found]]) Gazebo server se ruši odmah po učitavanju
modela.

## Uzrok
**Potvrđeno:** PAL kolizijski meshovi baze su zrcaljeni skalom `1 -1 1`, a DART radi
`assert (scale > 0).all()`.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. | pri launchu proširiti xacro u Pythonu i ukloniti `-` iz svih `scale` atributa | 0 DART asserta, 7 kontrolera aktivno | rješenje (vizualno zanemarivo) |

## Trenutno rješenje
Isti postupak u `sim.launch.py` i `move_group.launch.py`.
