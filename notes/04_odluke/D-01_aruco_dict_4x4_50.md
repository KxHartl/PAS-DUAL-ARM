---
id: D-01
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-12_box_with_aruco]]", "[[R-16_find_box]]"]
problems: ["[[P-07_aruco_dict_and_cv_bridge]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-01: ArUco DICT_4X4_50, ID 0

## Kontekst
[MAIL] predlaže ArUco markere na kutiji, a rječnik nije propisan.

## Odluka (26. 5., README)
**DICT_4X4_50, ID 0.** Mali rječnik (50 markera) smanjuje lažne detekcije i ubrzava obradu. 4×4
unutarnja matrica je dovoljno robusna za veliki objekt, a u sceni je samo jedna oznaka.

## Posljedice
PAL `aruco_ros` ne dekodira OpenCV DICT_4X4_50, pa je potreban vlastiti detektor
([[D-02_own_aruco_detector]]).

## Odnos prema zahtjevima
Ispunjava [[R-12_box_with_aruco]]. Obrazloženje ide u seminar.
