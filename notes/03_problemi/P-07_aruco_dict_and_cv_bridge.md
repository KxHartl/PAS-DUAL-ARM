---
id: P-07
type: problem
status: rijeseno
requirements: ["[[R-12_box_with_aruco]]", "[[R-16_find_box]]"]
solutions: ["[[S-05_perception]]"]
decisions: ["[[D-01_aruco_dict_4x4_50]]", "[[D-02_own_aruco_detector]]"]
updated: 2026-09-13
---
# P-07: `aruco_ros` ne zna DICT_4X4_50; `cv_bridge` segfaulta

## Simptom
Nema detekcije markera DICT_4X4_50. Čvor s `cv_bridge` se ruši.

## Uzrok
**Potvrđeno:**
1. PAL `aruco_ros` biblioteka poznaje samo svoje rječnike (ARUCO_MIP_36h12, ARTAG, AprilTag…).
2. Humble binarni `cv_bridge` segfaulta pod numpy 2.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 12. 6. `45ba359` | vlastiti čvor `aruco_detector` (`cv2.aruco`, `solvePnP` IPPE_SQUARE, slika bez `cv_bridge`) | poza ~1 cm od stvarne | rješenje ([[D-02_own_aruco_detector]]) |

## Ne ponavljati
- Uvoditi `cv_bridge` u Python čvorove (numpy 2).
