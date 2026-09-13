---
id: R-17
type: zahtjev
status: ispunjeno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-08_grasp_squeeze_attach]]", "[[S-07_moveit_setup]]"]
problems: ["[[P-14_gripper_too_small_for_cube]]", "[[P-15_dart_friction_no_hold]]", "[[P-16_fake_teleport_grasp]]", "[[P-17_detachable_joint_explodes]]", "[[P-23_moveit_blind_to_world]]", "[[P-24_press_path_chain]]", "[[P-25_asymmetric_arm_reach]]", "[[P-26_one_sided_press_bulldozes]]", "[[P-27_contact_sensor_topic_ignored]]", "[[P-28_gate_too_strict]]"]
decisions: ["[[D-05_contact_verified_attach]]", "[[D-06_cube_squeeze_grasp]]", "[[D-07_carry_on_left_wrist]]", "[[D-12_honesty_abort_over_fake]]"]
updated: 2026-09-13
---
# R-17: Podizanje kutije objema rukama

## Izvor (doslovno)
> „Nakon što ju detektirate morate ju podići s obje ruke.“ [MAIL]

## Tehnički znači
Obje ruke sudjeluju u hvatu. Kutija se odvoji od stola i drži se, a hvat je fizički opravdan:
nema teleporta ni „zavarivanja“ iz daljine.

**Kriterij prihvaćanja:**
- [x] obje ruke istovremeno pritisnu suprotne plohe (squeeze)
- [x] obostrani kontakt s `aruco_box` dokazan senzorima prije attacha
- [x] kutija se digne ~15 cm i ne odleti
- [x] svaki neuspjeh završi poštenim abortom (nikad lažni attach)

## Trenutno stanje
✅ **3 puna ciklusa hvat + podizanje u GUI-ju, 16. 7.** (`c504720`). Svih ~15 neuspjelih runova
završilo je poštenim abortom. Uspješnost s kockom je ~35 % (3/9, [[runovi]]).
🧪 Zadnje izmjene (`73617e8`: popušten gate, uklanjanje kolizijskog objekta) **nisu provjerene**.
Očekivani skok uspješnosti prema ~100 % još treba potvrditi ([[P-28_gate_too_strict]]).

**Iskreno za seminar:** kutiju nakon podizanja nosi `DetachableJoint` na lijevom zglobu
([[D-05_contact_verified_attach]]), a desna ruka se povuče ([[D-07_carry_on_left_wrist]]). Hvat je
dvoručan, a nošenje fizički jednoručno + kruti spoj.

## Kako se rješava
- [[S-08_grasp_squeeze_attach]]: cijeli lanac press → gate → attach → lift
