---
id: P-16
type: problem
status: rijeseno
requirements: ["[[R-17_dual_arm_lift]]", "[[R-16_find_box]]"]
solutions: ["[[S-08_grasp_squeeze_attach]]", "[[S-05_perception]]"]
decisions: ["[[D-12_honesty_abort_over_fake]]", "[[D-05_contact_verified_attach]]"]
updated: 2026-09-13
---
# P-16: Lažni hvat (teleport, zavarivanje iz daljine, cirkularne provjere)

## Simptom
- M6 (23. 6.): kutija „uskoči“ u ruke (`set_pose` teleport na sredinu hvataljki).
- Kasnije (30. 6.): hvataljke se zatvore **~50 cm ispred** kutije, a kutija se svejedno „zavari“.

## Uzrok
**Potvrđeno:**
1. teleport u orkestraciji;
2. `verify_contact(grip, half)` je mjerio vrhove prema **istom naređenom** centru (cirkularno);
3. tihi fallback na x = 0.55 kad `measure_box` padne;
4. fantomski seed x = 0.55 iz starih runova ([[P-21_spin_once_not_pacing_rtf]]).

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. `397f080` | teleport → DetachableJoint nakon „zatvorenih hvataljki“ provjerenih TF-om | pošteniji, ali slab dokaz | nedovoljno |
| 2 | 30. 6. (plan `ne-radi-hvatanje-kutije`) | uklonjeni cirkularni check i x = 0.55 fallback; `verify_reached` (TF vs naredba); `fingertips_on_box`; fuzija kontakt senzora + stall | aborti umjesto lažnih hvata | smjer ispravan |
| 3 | 16. 7. `c504720` | dokaz = kontakt s kolizijom **`aruco_box`** (ime iz poruke) na oba jastučića | 3 stvarna uspjeha, ~15 poštenih aborta | **rješenje** |

## Ne ponavljati
- Bilo koji attach/teleport bez nezavisnog fizičkog dokaza ([[D-12_honesty_abort_over_fake]]).
- Provjeru „naredba vs naredba“.
