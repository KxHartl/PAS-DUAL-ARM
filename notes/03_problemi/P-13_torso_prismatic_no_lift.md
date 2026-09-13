---
id: P-13
type: problem
status: zaobideno
requirements: ["[[R-03_linear_rails_torso]]", "[[R-06_realistic_parameters]]"]
solutions: ["[[S-01_robot_description]]", "[[S-03_ros2_control_setup]]"]
decisions: ["[[D-09_lift_with_arms_not_torso]]"]
updated: 2026-09-13
---
# P-13: Klizači torza (prismatic) se ne dižu pod težinom ruke

## Simptom
`torso_controller` naredi visinu, a opterećeni vertikalni klizač ostaje na donjem limitu (0.05).
Lagani zglobovi (pan-tilt) rade normalno.

## Uzrok
**Nepotvrđeno. Hipoteze:**
- (a) position interface u `IgnitionSystem` (v = gain · greška) ne svladava gravitaciju ruke
  (~100 N);
- (b) kolizija klizača s meshom vodilice (samokolizija);
- (c) efort limit se ne primjenjuje kako očekujemo.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `397f080` | efort limit 100 → 1000 N | bez promjene | nije samo efort |
| 2 | 30. 6. `397f080` | `min`/`max` na command interfaceu | bez promjene | — |
| 3 | 30. 6. `397f080` | `position_proportional_gain` (sada 20) | bez promjene | — |

## Trenutno rješenje
Dizanje rukama ([[D-09_lift_with_arms_not_torso]]). Meta je na stolu, pa visina dohvata nije
problem.

## Sljedeći korak (niski prioritet, samo ako ostane vremena)
Provjeriti hipotezu (b): u Gazebu isključiti koliziju klizača ili pogledati kontakte. Za (a):
effort interface + PID. Za seminar je dovoljno opisati ograničenje.

## Ne ponavljati
- Dalje dizati efort ili gain (probano do 1000 N).
