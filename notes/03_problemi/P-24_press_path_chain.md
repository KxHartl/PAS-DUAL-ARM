---
id: P-24
type: problem
status: rijeseno
requirements: ["[[R-17_dual_arm_lift]]", "[[R-09_moveit_arm_control]]"]
solutions: ["[[S-07_moveit_setup]]", "[[S-08_grasp_squeeze_attach]]"]
decisions: []
updated: 2026-09-13
---
# P-24: Press putanja: obilasci, skokovi, „čudne rotacije“

## Simptom (GUI, korisnik)
- RRTConnect za press od 10 cm vraća **metarske obilaske** kroz kocku.
- Kontroler „skoči“ bilo kamo.
- **Pun okret** zgloba usred hvata („čudne rotacije“).
- Ruka lansirana metar dalje.
- Lijeva ruka: fraction linije 0.09–0.30 run za runom.

## Uzrok
**Potvrđeno, više uzroka:**
1. RRT nije ravna linija.
2. `compute_cartesian_path` vraća **neparametriranu** putanju (bez vremena).
3. IK vraća kutove u [-π, π], a kontinuirani zglobovi akumuliraju okretaje → „odmotavanje“.
4. Putanja iz zastarjelog stanja → prva točka daleko.
5. JTC javi „gotovo“ na isteku vremena, a sim ruka dopuzava sekundama.
6. Pose-goal RRT parkira ruku u proizvoljnoj IK grani iz koje linija nije izvediva.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15. 7. | RRT pose goal za press | obilasci | ne za kratke kontaktne pokrete |
| 2 | 15. 7. | `compute_cartesian_path` + `execute_trajectory` | skokovi | treba vremenska parametrizacija |
| 3 | 15. 7. | vlastita parametrizacija ≤ ~0.4 rad/s po zglobu | glatko | ✅ |
| 4 | 16. 7. | **2π unwrap** na granu najbližu stvarnom stanju | nema punih okreta | ✅ |
| 5 | 16. 7. | guard prve točke (`_traj_starts_here`, tol 0.15) | nema lansiranja | ✅ |
| 6 | 16. 7. | `_wait_settle` (< 4 mm / 0.4 s) prije čitanja poze | točniji reach | ✅ |
| 7 | 16. 7. | IK press poze → IK pre-poze **seedan** njome → joint goal | linija izvediva češće | ✅ |
| 8 | 16. 7. | re-roll pre-squeeze do 4× (+180° roll) dok fraction ≥ 0.9 | — | fallback |
| 9 | 16. 7. | RRT na ~2 cm ispred plohe + linearni re-press | Δ 0.001–0.005 m | pouzdan fallback |
| 10 | 16. 7. | re-press sa **zrcaljenjem greške** | overshoot 10 cm u kocku → ruka tunelira | ❌ uklonjeno |

Sve je u `c504720`.

## Trenutno rješenje
`_linear_traj`, `press_both_linear`, `_ik`, `_wait_settle`, `_traj_starts_here` u `main_task.py`.

## Ne ponavljati
- RRT za kratke kontaktne pokrete.
- Kartezijsku putanju bez vremenske parametrizacije i unwrapa.
- Korekciju zrcaljenjem greške.
