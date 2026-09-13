---
id: P-15
type: problem
status: zaobideno
requirements: ["[[R-17_dual_arm_lift]]", "[[R-06_realistic_parameters]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-05_contact_verified_attach]]"]
updated: 2026-09-13
---
# P-15: DART ne drži slobodan objekt trenjem hvataljke

## Simptom
Prsti su stvarno na kutiji (kontakt potvrđen), stisnu, ruke se dignu, a kutija ostane na stolu.

## Uzrok
**Potvrđeno empirijski:** DART kontaktni model u Fortressu ne održava trenjem držanje objekta
tijekom gibanja (poznato ograničenje; zato je izvorni kod teleportirao).

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `f62ebbd` | μ kutije 2.0, efort stiska 120, bez DetachableJointa | ne digne se | — |
| 2 | 30. 6. `33bc2ac` | μ do 5, masa 0.4 → 0.2 kg, kp | ne digne se | trenje iscrpljeno |
| 3 | 30. 6. `ec766a2` | DetachableJoint tek nakon kontakt-provjere (odobrio korisnik) | digne se, drži | **trenutno** ([[D-05_contact_verified_attach]]) |

## Ne ponavljati
- Daljnje ugađanje μ/mase/kp/efort radi čistog trenja: iscrpljeno 30. 6.
