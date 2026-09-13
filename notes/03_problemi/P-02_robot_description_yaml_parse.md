---
id: P-02
type: problem
status: rijeseno
requirements: ["[[R-07_ros2_humble_fortress_control]]"]
solutions: ["[[S-02_world_and_sim_launch]]"]
decisions: []
updated: 2026-09-13
---
# P-02: `robot_description` se parsirao kao YAML (simulacija se nikad nije ni pokrenula)

## Simptom
`sim.launch.py` nije stvarno digao robota u Gazebu (prije M0).

## Uzrok
**Potvrđeno:** xacro `Command` supstitucija je predana kao parametar bez tipa, pa ju je launch
parsirao kao YAML.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. `7aaab94` | omotati u `ParameterValue(..., value_type=str)` | robot se učita | rješenje |
| 2 | 12. 6. `f79e393` | xacro proširivati u Pythonu (`subprocess` + `re`) jer `Command` spaja argumente bez razmaka i ne može pipe | radi, uz uklanjanje negativnih skala | trajni oblik ([[P-05_negative_mesh_scale_dart]]) |

## Trenutno rješenje
Python ekspanzija xacroa u `sim.launch.py` i `move_group.launch.py` (isti model u Gazebu i MoveIt-u).
