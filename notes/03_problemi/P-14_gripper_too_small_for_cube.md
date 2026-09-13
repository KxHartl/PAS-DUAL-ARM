---
id: P-14
type: problem
status: rijeseno
requirements: ["[[R-12_box_with_aruco]]", "[[R-17_dual_arm_lift]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-06_cube_squeeze_grasp]]"]
updated: 2026-09-13
---
# P-14: Hvataljka 2F-85 (hod ~85 mm) ne može obuhvatiti kocku od 0.3 m

## Simptom
Klasičan hvat (prsti oko plohe) je geometrijski nemoguć.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 23. 6. `45c32f1` | ruke na bočne plohe kocke na podu, klizači spušteni | dosežu, ali ne drže | M6 „hvat“ je bio teleport ([[P-16_fake_teleport_grasp]]) |
| 2 | 30. 6. `397f080` | šipka 0.06 × 0.30 × 0.06 m, 0.5 kg; svaka ruka obuhvati kraj; top-down (`GRASP_DOWN`) | digne se z 0.03 → 0.179 (s attachom) | hvatljivo, ali nije kutija |
| 3 | 30. 6. `d346d71` | šipka 0.07 × 0.30 × 0.10 na niskom stolu | lift ~15 cm | stol poboljšava IK |
| 4 | 30. 6. `f62ebbd` | uspravna ploča 0.06 × 0.30 × 0.20, čisto trenje | ne digne se | [[P-15_dart_friction_no_hold]] |
| 5 | 15./16. 7. `c504720` | **kocka 0.3 m, zatvorene hvataljke kao jastučići, dvoručni squeeze** | 3 puna ciklusa, ~35 % | **trenutno** ([[D-06_cube_squeeze_grasp]]) |

## Trenutno rješenje
[[S-08_grasp_squeeze_attach]].
