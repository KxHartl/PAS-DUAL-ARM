---
id: P-20
type: problem
status: rijeseno
requirements: ["[[R-16_find_box]]"]
solutions: ["[[S-05_perception]]"]
decisions: []
updated: 2026-09-13
---
# P-20: Oblak točaka gladuje `/clock` callback, a petlje se vješaju

## Simptom
Servo petlje stoje, a sim vrijeme u čvoru ne napreduje.

## Uzrok
**Potvrđeno:** callback za oblak 640×480 je težak. Uz stalnu pretplatu executor ne stigne obraditi
`/clock`.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 30. 6. | pretplata na `/camera/points` samo tijekom `measure_box`; wall-clock timeouti u petljama | petlje rade | rješenje |

## Ne ponavljati
- Trajnu pretplatu na oblak u `main_task`.
