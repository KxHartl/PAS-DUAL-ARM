---
id: P-08
type: problem
status: rijeseno
requirements: ["[[R-12_box_with_aruco]]", "[[R-16_find_box]]"]
solutions: ["[[S-02_world_and_sim_launch]]", "[[S-05_perception]]"]
decisions: []
updated: 2026-09-13
---
# P-08: Marker se ne detektira (tekstura)

## Simptom
Kamera vidi kutiju, a detektor nikad ne nađe marker. Hvat pada na hardkodiranu pozu.

## Uzrok
**Potvrđeno (dva uzroka):**
1. Stari PNG nema tihu zonu: crni rub ide do ruba slike pa kontura ne postoji.
2. Marker je bio PBR `<metal>`, pa se pod svjetlima ispere.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. `45ba359` | novi PNG: marker 75 % plohe + bijela margina | detekcija uz pitch 0.72 | uzrok 1 riješen |
| 2 | 29. 6. `e9157e1` | materijal: `metalness` 0, `roughness` 1, `specular` 0 + `CORNER_REFINE_SUBPIX` | pouzdana detekcija, TF 0.858 vs 0.85 | uzrok 2 riješen |

## Trenutno rješenje
Ploča markera 0.22 m na -X plohi kocke, marker 0.165 m (`aruco.launch.py:20`), matiran.

## Ne ponavljati
- Metalni/sjajni materijal na markeru; teksturu bez tihe zone.
