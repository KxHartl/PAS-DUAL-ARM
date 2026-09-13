---
id: P-21
type: problem
status: rijeseno
requirements: ["[[R-16_find_box]]", "[[R-17_dual_arm_lift]]"]
solutions: ["[[S-04_base_drive]]", "[[S-09_task_orchestration]]"]
decisions: []
updated: 2026-09-13
---
# P-21: `spin_once` nije tempiranje; RTF < 1 prepolovi vožnje

## Simptom
- „Vožnje od 4 s“ su se izvršile u milisekundama. Posljedica su fantomski seedovi x = 0.55 iz
  starih runova ([[P-16_fake_teleport_grasp]]).
- Uz wall-clock tempiranje, naredba 0.48 m je dala 0.24 m (RTF ~0.5).

## Uzrok
**Potvrđeno:**
1. `rclpy.spin_once` se vraća čim obradi **bilo koji** callback, a `/clock` je ~1 kHz.
2. Wall-time petlja pri RTF < 1 šalje komande kraće u sim vremenu.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 15./16. 7. `c504720` | sve „čekaj N s“ petlje → monotoni ili **sim-time** rok; `drive()` tempiran sim vremenom | vožnje točne | rješenje |
| 2 | 15./16. 7. `c504720` | stvarni pomak iz `/base_controller/odom`, a ne iz naredbe | dovoz/okret korigirani | rješenje |

## Ne ponavljati
- `spin_once` kao pauzu; wall-clock tempiranje gibanja u simu.
