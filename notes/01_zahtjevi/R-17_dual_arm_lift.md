---
id: R-17
type: zahtjev
status: otvoreno
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
- [ ] obje ruke istovremeno pritisnu suprotne plohe (squeeze)
- [ ] obostrani kontakt s `aruco_box` dokazan senzorima prije attacha
- [ ] kutija se digne ~15 cm i ne odleti
- [ ] svaki neuspjeh završi poštenim abortom (nikad lažni attach)

## Trenutno stanje

> [!warning] Ova kartica je 13. 9. 2026. ispravljena
> Do tada je stajala kao `ispunjeno`, s tvrdnjom „3 puna ciklusa hvat + podizanje u GUI-ju,
> 16. 7.“. **Korisnik je tu ocjenu povukao: hvat nije dobar i nije dobro napravljen.**
> Tvrdnja je zavela i agenta koji je 13. 9. planirao rad, pa je zapisana i u memoriji.

❌ **Hvat se ne smatra riješenim.** Runovi iz srpnja su se dogodili i zapisani su u [[runovi]] i u
P-karticama, ali ono što su proizveli nije upotrebljiv hvat, nego lanac zaobilaženja
(stisak zatvorenim hvataljkama + kruti spoj) koji se ne drži.

**Jedino što je iz tog rada zadržano** je poza `ARM_CARRY_V2`, koju je korisnik ručno namjestio u
RViz-u ([[08_poze]]). Ona valja i od 13. 9. je u kodu (`postures.py`).

### Što konkretno treba (korisnik, 13. 9.)
1. **Visinu hvata trebaju postavljati vodilice** (klizači torza), a ne samo poza ruku
   ([[R-03_linear_rails_torso]], [[P-13_torso_prismatic_no_lift]]).
2. **Širinu hvata** prilagoditi kutiji.
3. **Stiskanje** prilagoditi — sadašnji press je grub.

**Iskreno za seminar:** kutiju nakon podizanja nosi `DetachableJoint` na lijevom zglobu
([[D-05_contact_verified_attach]]), a desna ruka se povuče ([[D-07_carry_on_left_wrist]]). Hvat je
zamišljen dvoručno, a nošenje je fizički jednoručno + kruti spoj.

## Kako se rješava
- [[S-08_grasp_squeeze_attach]]: cijeli lanac press → gate → attach → lift
