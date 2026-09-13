---
id: P-22
type: problem
status: rijeseno
requirements: ["[[R-16_find_box]]"]
solutions: ["[[S-05_perception]]"]
decisions: []
updated: 2026-09-13
---
# P-22: Mjerenje dubinom vidi robotovo tijelo („junk“ klasteri)

## Simptom
Izmjerena duljina 0.63 m za šipku od 0.30 m, a hvataljke se zatvore u zraku.

## Uzrok
**Potvrđeno:** nakon dovoza nagnuta kamera vidi robotovu prednju stranu (torzo/baza na x ~0.25–0.35)
i zapešća u prozoru oko seeda.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. | donja granica maske x 0.2 → 0.35 | manje smeća | nedovoljno |
| 2 | 15. 7. | mjerenje **nakon** dovoza, ruke u spawn pozi | junk | ruke u prozoru |
| 3 | 15. 7. | mjerenje nakon dovoza, ruke u `ARM_CARRY` | junk | isto |
| 4 | 16. 7. `c504720` | mjerenje s **~0.95 m prije dovoza** + cross-check s markerom (≤ 0.15 m) | čist klaster | **rješenje** |

## Ne ponavljati
- Mjeriti kutiju iz blizine s rukama ispred kamere.
