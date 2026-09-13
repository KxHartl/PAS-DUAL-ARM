---
id: S-05
type: rjesenje
status: ispunjeno
requirements: ["[[R-04_pan_tilt_camera]]", "[[R-12_box_with_aruco]]", "[[R-16_find_box]]"]
problems: ["[[P-07_aruco_dict_and_cv_bridge]]", "[[P-08_marker_not_detected_texture]]", "[[P-19_aruco_foreshortening_close]]", "[[P-20_pointcloud_starves_clock]]", "[[P-22_depth_self_view_clusters]]"]
decisions: ["[[D-01_aruco_dict_4x4_50]]", "[[D-02_own_aruco_detector]]"]
files: ["src/pas_dual_arm_scripts/pas_dual_arm_scripts/aruco_detector.py", "src/pas_dual_arm_bringup/launch/aruco.launch.py", "src/pas_dual_arm_scripts/pas_dual_arm_scripts/main_task.py"]
updated: 2026-09-13
---
# S-05: Percepcija (ArUco + dubina)

## Što radi
Nalazi kutiju i daje njezin centar, os stiska i visinu u `base_link`, iz dva **nezavisna**
izvora (marker + oblak točaka) koji se međusobno provjeravaju.

## Arhitektura
1. **`aruco_detector`** (vlastiti čvor, [[D-02_own_aruco_detector]]):
   - `cv2.aruco`, DICT_4X4_50, ID 0, `marker_size` 0.165 m;
   - `CORNER_REFINE_SUBPIX`, `solvePnP(IPPE_SQUARE)`;
   - objavljuje `/aruco_single/pose` + TF `camera_color_optical_frame → aruco_marker_frame`;
   - sliku pretvara bez `cv_bridge` ([[P-07_aruco_dict_and_cv_bridge]]).
2. **`main_task.confirm_box()`**: prosjek 5 TF uzoraka markera, centar = ploha + 0.15 m.
3. **`main_task.measure_box(seed)`**:
   - privremena pretplata na `/camera/points` ([[P-20_pointcloud_starves_clock]]);
   - transformacija preko `camera_link` (optički frame → tijelo);
   - maska prozora oko seeda (x ∈ (0.35, 1.0), |x − seed| < 0.25, |y − seed| < 0.30, z iznad stola);
   - PCA → os + dimenzije;
   - sanity: duljina ∈ (0.15, 0.45), visina ∈ (0.06, 0.32).
4. **`marker_tangent()`**: os stiska iz orijentacije markera, jer je PCA degenerirana na
   kvadratnom vrhu kocke.
5. **Cross-check**: centar iz dubine vs markera ≤ 0.15 m. Pri neslaganju slijedi jedno ponovno
   mjerenje, a pri ponovnom neslaganju **abort** ([[D-12_honesty_abort_over_fake]]).

## Ključna pravila (naučeno)
- Mjeri se s ~0.95 m, **prije** dovoza: bliže kamera vidi vlastite ruke u prozoru
  ([[P-22_depth_self_view_clusters]]).
- Marker se prati samo do ~0.9 m: bliže strmi nagib kamere foreshorten-a marker
  ([[P-19_aruco_foreshortening_close]]).

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 12. 6. | headless | poza markera ~1 cm od ground trutha | `45ba359` |
| 29. 6. | headless | pouzdana detekcija nakon matiranja | `e9157e1` |
| 16. 7. | GUI | mjerenje + cross-check u svim runovima s kockom | `c504720` |
