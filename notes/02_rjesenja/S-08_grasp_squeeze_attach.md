---
id: S-08
type: rjesenje
status: djelomicno
requirements: ["[[R-17_dual_arm_lift]]", "[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
problems: ["[[P-14_gripper_too_small_for_cube]]", "[[P-15_dart_friction_no_hold]]", "[[P-16_fake_teleport_grasp]]", "[[P-17_detachable_joint_explodes]]", "[[P-18_transport_drops_box]]", "[[P-24_press_path_chain]]", "[[P-25_asymmetric_arm_reach]]", "[[P-26_one_sided_press_bulldozes]]", "[[P-27_contact_sensor_topic_ignored]]", "[[P-28_gate_too_strict]]", "[[P-29_place_drop_tips_cube]]", "[[P-30_stale_collision_object]]"]
decisions: ["[[D-05_contact_verified_attach]]", "[[D-06_cube_squeeze_grasp]]", "[[D-07_carry_on_left_wrist]]", "[[D-12_honesty_abort_over_fake]]"]
files: ["src/pas_dual_arm_scripts/pas_dual_arm_scripts/main_task.py"]
updated: 2026-09-13
---
# S-08: Dvoručni SQUEEZE hvat + kontaktom verificiran attach

## Ideja
Hvataljka od 85 mm ne može obuhvatiti kocku od 0.3 m ([[P-14_gripper_too_small_for_cube]]). Zato
obje ruke **zatvorenim** hvataljkama (vrhovi prstiju kao jastučići s Contact senzorima) istovremeno
pritisnu suprotne bočne plohe. DART ne drži objekt trenjem ([[P-15_dart_friction_no_hold]]), pa
kutiju nakon **dokazanog** obostranog kontakta nosi DetachableJoint na lijevom zglobu
([[D-05_contact_verified_attach]]).

## Slijed (`main_task.run()`, koraci 4c–8)
1. **4c Plan**: `grip` = centar kocke, z = visina markera. `publish_collision_scene` (pod, stol,
   kocka).
2. **5 Ready**: `ARM_HOME` → obje hvataljke 0.7 (zatvoreno = jastučić).
3. **5 Pre-squeeze**: IK press poze (−0.03 m, 3 cm U kocki) → IK pre-poze (+0.10 m) seedan njome →
   joint goal. Ako IK lanac padne, fallback je pose-goal RRT.
4. **5 Press**: `press_both_linear`, ravna linija za obje ruke **simultano** na oba JTC-a
   ([[P-26_one_sided_press_bulldozes]]). Fallback lanac: linija (frac ≥ 0.9) → re-roll
   pre-squeeze (do 4×, uz 180° roll) → RRT na ~2 cm ispred plohe → per-arm linearni re-press
   ([[P-24_press_path_chain]]).
5. **5b Reach**: `_wait_settle` → `verify_reached` (tol 0.05). Ako ruka nije stigla, ali jastučić
   ima box-kontakt, to je OK (press ne može konvergirati u kocku). Inače jedan re-press prema
   **originalnom** cilju.
6. **5c Geometrija**: `fingertips_on_box` (dimenzije kocke) + jedan nudge po ruci.
7. **5d Fizički dokaz**: svjež kontakt s **`aruco_box`** (ime kolizije iz poruke) na **OBA**
   jastučića u prozoru od 3 s.
8. **Gate** ([[P-28_gate_too_strict]]): `placement` (geom ILI oba EE < 0.12 m od cilja) **I**
   `physical` (kontakt L i D). Ako padne: povlačenje ruku + abort ([[D-12_honesty_abort_over_fake]]).
9. **6 Attach**: `/aruco_box/attach`, a kocka se **uklanja** iz planning scene
   ([[P-30_stale_collision_object]]). **Desna** se povlači prva, 0.10 m po −v
   ([[D-07_carry_on_left_wrist]]). **Lijeva** diže 0.15 m linijom.
10. **7 Transport-proba** (`probe_transport`): otvori jastučiće, vozi 0.4 m + okret ~60°.
    🧪 nije pokrenuto ([[P-18_transport_drops_box]]).
11. **8 Place**: lijeva spušta na pick visinu −0.02 m (stol zaustavlja kocku; do 3 pokušaja,
    tol 0.04) → detach → odmak 0.10 m. 🧪 nije provjereno ([[P-29_place_drop_tips_cube]]).

## Parametri
[[06_parametri]]: sekcija „Hvat“.

## Provjereno
| datum | način | što | commit |
|---|---|---|---|
| 30. 6. | GUI | ploča: contact check, attach, lift, place na isti stol | `ec766a2`, `5c9e201` |
| 16. 7. | GUI | kocka: 3 puna ciklusa hvat + lift; ~15 poštenih aborta | `c504720` |
| 16. 7. | — | gate popuštanje, kontaktni place, REMOVE, transport-proba: **samo build** | `73617e8` |

## Otvoreno
- Potvrditi uspješnost s novim gateom (run 31+).
- Kontaktni place: kocka ravno na stolu.
- Transport s kockom ([[R-19_door_pass_with_box]]).
