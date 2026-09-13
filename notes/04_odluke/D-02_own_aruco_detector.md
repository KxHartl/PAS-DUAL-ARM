---
id: D-02
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-12_box_with_aruco]]", "[[R-16_find_box]]"]
problems: ["[[P-07_aruco_dict_and_cv_bridge]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-02: Vlastiti `cv2.aruco` detektor umjesto `aruco_ros`

## Kontekst
[MAIL] navodi `pal-robotics/aruco_ros` („slobodno stavite“, prijedlog). Njegova biblioteka zna
samo vlastite rječnike (ARUCO_MIP_36h12, ARTAG, AprilTag…), a ne OpenCV DICT_4X4_50. Humble
binarni `cv_bridge` segfaulta pod numpy 2.

## Opcije
1. Promijeniti rječnik na onaj koji `aruco_ros` zna.
2. **Vlastiti čvor s `cv2.aruco`**, s pretvorbom slike bez `cv_bridge`.

## Odluka (12. 6., `45ba359`)
Opcija 2: `pas_dual_arm_scripts/aruco_detector.py`. Sučelje je isto kao kod `aruco_ros`
(`/aruco_single/pose`, TF `aruco_marker_frame`), pa ostatak sustava ne ovisi o izboru.

## Posljedice
`aruco_ros` ostaje u `src/` (pinan u `ros2.repos`), ali se ne pokreće.

## Odnos prema zahtjevima
Ne odstupa: [MAIL] traži „aruco markere“, a paket je samo preporuka. Za seminar: izvor je
naveden, a razlog tehnički.
