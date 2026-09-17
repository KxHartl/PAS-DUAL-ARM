---
id: P-27
type: problem
status: rijeseno
requirements: ["[[R-17_dual_arm_lift]]"]
solutions: ["[[S-01_robot_description]]", "[[S-02_world_and_sim_launch]]", "[[S-08_grasp_squeeze_attach]]"]
decisions: []
updated: 2026-09-13
---
# P-27: Contact senzori ne objavljuju na zadanom topicu

## Simptom
`/contact/left_left_tip` šuti iako su prsti na kutiji.

## Uzrok
**Potvrđeno:** Fortress **ignorira `<topic>`** Contact senzora i objavljuje na
`/world/<w>/model/<m>/link/<l>/sensor/<s>/contact`. Uz to je bila potrebna `Contact` sistem
plugin u svijetu.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. | `tip_contact` makro s `<topic>` + bridge kratkih imena | šuti; fallback depth + stall | krivi topic |
| 2 | 16. 7. | bridge mapira **duge** gz putanje na `/contact/<ruka>_<prst>_tip` (`bridge.yaml:44-69`) | kontakti stižu, s imenima kolizija | rješenje |
| 3 | 16. 7. | brojati samo kontakt čija je druga kolizija `aruco_box` (stol ili sebe ne) | pošten dokaz | rješenje ([[D-12_honesty_abort_over_fake]]) |

## Ne ponavljati
- Oslanjati se na `<topic>` u Fortress senzorima; prvo `ign topic -l`.
