---
id: P-29
type: problem
status: neprovjereno
requirements: ["[[R-20_place_at_destination]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: []
updated: 2026-09-13
---
# P-29: Pri odlaganju kocka se ispusti previsoko i prevrne

## Simptom
Detach nekoliko cm iznad stola, a kocka padne i prevrne se (GUI, 16. 7.).

## Uzrok
**Potvrđeno:** spuštanje je ciljalo točno na visinu uzimanja. Tracking greška ostavlja kocku iznad
stola.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 16. 7. `c504720` | spusti na pick visinu, detach | prevrne se | premalo |
| 2 | 16. 7. `73617e8` | cilj **−2 cm ispod** pick visine (stol zaustavi kocku) + do 3 pokušaja uz provjeru (tol 0.04) | 🧪 nije pokrenuto | — |

## Sljedeći korak
Provjeriti u runu 31+. Na `target_table` ([[R-20_place_at_destination]]) isti princip, s visinom
plohe 0.775 m.

**Kriterij uspjeha:** kocka nakon detacha stoji uspravno (nagib < 5°).
