---
id: R-16
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-05_perception]]", "[[S-09_task_orchestration]]"]
problems: ["[[P-07_aruco_dict_and_cv_bridge]]", "[[P-08_marker_not_detected_texture]]", "[[P-19_aruco_foreshortening_close]]", "[[P-20_pointcloud_starves_clock]]", "[[P-22_depth_self_view_clusters]]"]
decisions: ["[[D-01_aruco_dict_4x4_50]]", "[[D-02_own_aruco_detector]]"]
updated: 2026-09-13
---
# R-16: Robot sam pronađe kutiju

## Izvor (doslovno)
> „tamo pronađe kutiju“ [MAIL]

## Tehnički znači
Bez unaprijed poznate poze robot kamerom nađe marker, odredi pozu kutije u `base_link` i izmjeri
je (dubina), dovoljno točno za hvat (reda veličine 1–2 cm).

**Kriterij prihvaćanja:**
- [x] skeniranje pan-tilt kamerom, baza miruje (pan 0, -0.5, -1.0, 0.5, 1.0; pitch 0.7)
- [x] vizualni prilaz do ~0.9 m, pa mjerenje dubinom + cross-check s markerom (≤ 0.15 m)
- [x] greška poze ~1 cm (M4: izmjereno 0.861 vs stvarno ~0.85)

## Trenutno stanje
✅ Radi u svim runovima s kockom (16. 7.). Nijedan hardkodirani fallback (x = 0.55 je uklonjen,
[[P-16_fake_teleport_grasp]]).

## Kako se rješava
- [[S-05_perception]]: detektor, `measure_box()`, `confirm_box()`
- [[S-09_task_orchestration]]: koraci 1–3 (`scan_for_marker`, `visual_approach`, mjerenje)
