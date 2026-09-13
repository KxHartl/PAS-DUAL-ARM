---
id: P-17
type: problem
status: rijeseno
requirements: ["[[R-17_dual_arm_lift]]", "[[R-19_door_pass_with_box]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]"]
decisions: ["[[D-07_carry_on_left_wrist]]", "[[D-05_contact_verified_attach]]"]
updated: 2026-09-13
---
# P-17: DetachableJoint „eksplodira“ (kutija odleti metrima daleko)

## Simptom
Nakon attacha kutija nedeterministički odleti preko svijeta (GUI i headless).

## Uzrok
**Potvrđeno:** **preodređen sustav**. Kruti spoj + nešto drugo što na kutiju djeluje zasebnim putem
(jak stisak prstiju ili druga ruka koja se giba svojom putanjom) → DART constraint solver divergira.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `ec766a2` | jak stisak (120) + attach | kutija odleti | stisak se tuče sa spojem |
| 2 | 30. 6. `ec766a2` | **attach PRIJE stiska** + nježan stisak (efort 20) | drži pri dizanju | ✅ za ploču |
| 3 | 30. 6. `5c9e201` | desna ruka ostaje stisnuta i diže se svojim RRT putem | kutija odleti | nikad dvije krute veze |
| 4 | 30. 6. `5c9e201` | **desna se otvori i povuče, nosi samo lijevi zglob** | drži, slegne na stol | ✅ ([[D-07_carry_on_left_wrist]]) |
| 5 | 16. 7. `c504720` | squeeze: attach nakon gatea, desna odmah popušta (linearno −v 0.10) | 3 ciklusa bez odlijetanja | ✅ trenutno |

## Otvoreno
Vožnja baze s kutijom na spoju → [[P-18_transport_drops_box]]. Glavna hipoteza je isti mehanizam:
kontakt lijevih prstiju s kutijom se tuče s krutim spojem.

## Ne ponavljati
- Dvije istodobne krute veze (spoj + stisak druge ruke); jak stisak uz spoj.
