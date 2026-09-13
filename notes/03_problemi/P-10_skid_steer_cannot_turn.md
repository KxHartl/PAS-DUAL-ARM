---
id: P-10
type: problem
status: zaobideno
requirements: ["[[R-08_omni_controller]]", "[[R-06_realistic_parameters]]"]
solutions: ["[[S-04_base_drive]]", "[[S-01_robot_description]]"]
decisions: ["[[D-03_diff_drive_base_temporary]]"]
updated: 2026-09-13
---
# P-10: Skid-steer baza se ne može okretati u mjestu

## Simptom
Baza vozi samo ravno. Svaki okret, uključujući 360° scan, je nemoguć.

## Uzrok
**Potvrđeno:** kruti 4-kotačni skid-steer se okreće bočnim klizanjem kotača. PAL je stavio
mu1 = mu2 = 0 (kinematski Classic), a naš prvi override mu2 = 1.5 je klizanje blokirao.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 23. 6. `6eb7487` | override trenja kotača (visok mu2) | vozi ravno, ne okreće | bočno trenje blokira skid |
| 2 | 30. 6. `f62ebbd` | mu2 = 0 | okreće (yaw 0 → 2.36 uz ω = 0.8) | okret moguć |
| 3 | 30. 6. `33bc2ac` | mu1 = 0.4 | čist okret u mjestu, drift ~4 mm | okret moguć |
| 4 | 13. 9. | mu2 = 0.2 | minimalni bočni otpor umjesto nule | sprječava rubne numeričke greške nule uz zadržan okret |

## Trenutno rješenje
`robot.urdf.xacro:120-132` mu1 0.4, mu2 0.2. **Pravilo:** okret i vožnja nikad istovremeno (inače
baza „krabira“).

## Nuspojave
- Nerealno bočno trenje ([[R-06_realistic_parameters]] → [[odstupanja]]).
- Klizanje pri okretu razbija odometriju ([[P-11_nav2_slam_drift]]).

## Ne ponavljati
- Visok mu2 na skid-steeru.
