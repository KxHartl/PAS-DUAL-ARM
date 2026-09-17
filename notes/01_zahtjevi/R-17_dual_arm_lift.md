---
id: R-17
type: zahtjev
status: ispunjeno
verified: "run M4, 16. 9.: dodir obiju ruku potvrdjen, TASK COMPLETE"
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-08_grasp_squeeze_attach]]", "[[S-07_moveit_setup]]"]
problems: ["[[P-14_gripper_too_small_for_cube]]", "[[P-15_dart_friction_no_hold]]", "[[P-16_fake_teleport_grasp]]", "[[P-17_detachable_joint_explodes]]", "[[P-23_moveit_blind_to_world]]", "[[P-24_press_path_chain]]", "[[P-25_asymmetric_arm_reach]]", "[[P-26_one_sided_press_bulldozes]]", "[[P-27_contact_sensor_topic_ignored]]", "[[P-28_gate_too_strict]]"]
decisions: ["[[D-05_contact_verified_attach]]", "[[D-06_cube_squeeze_grasp]]", "[[D-07_carry_on_left_wrist]]", "[[D-12_honesty_abort_over_fake]]"]
updated: 2026-09-17
---
# R-17: Podizanje kutije objema rukama

## Izvor (doslovno)
> „Nakon što ju detektirate morate ju podići s obje ruke.“ [MAIL]

## Tehnički znači
Obje ruke sudjeluju u hvatu. Kutija se odvoji od stola i drži se, a hvat je fizički opravdan:
nema teleporta ni „zavarivanja“ iz daljine.

**Kriterij prihvaćanja:**
- [x] obje ruke istovremeno prilaze suprotnim plohama (pravocrtni prilaz, `GRASP_V4`)
- [x] obostrani kontakt s `aruco_box` dokazan senzorima **prije** attacha
- [x] kutija se digne ~15 cm i ne odleti
- [x] svaki neuspjeh završi prekidom uz zapis razloga (nikad attach bez potvrde)

## Trenutno stanje

✅ **Ispunjeno — run M4, 16. 9. 2026.** (`TASK COMPLETE` unutar pune misije, korisnikov GUI).

> [!note] Povijest ocjene ove kartice
> Do 13. 9. stajala je kao `ispunjeno` s tvrdnjom „3 puna ciklusa hvat + podizanje u GUI-ju,
> 16. 7.“. **Korisnik je tu ocjenu povukao** — taj hvat je bio lanac zaobilaženja (stisak
> zatvorenim hvataljkama + kruti spoj) koji se nije držao. Kartica je zatim stajala kao
> `otvoreno` do 16. 9., kada je hvat iznova napravljen na ručno snimljenim V4 pozama
> ([[P-44_grasp_from_reference_pose]]). Iz starog rada je zadržana samo poza `ARM_CARRY_V2`.

### Kako je riješeno (sve tri korisnikove primjedbe od 13. 9.)
1. **Visinu hvata postavljaju vodilice**, ne poza ruku — riješeno kad je nađen pravi uzrok
   zašto se nisu dizale: `initial_value = 0.05` točno na donjem graničniku, → 0.06
   ([[P-13_torso_prismatic_no_lift]], run F8).
2. **Širina hvata** prilagođena kutiji: `GRASP_V4` snimljen ručno u `pose_studio`; širina kroz
   cijeli hvat je 0.827 m, dakle prolazi vrata ([[P-43_grasp_pose_wider_than_door]]).
3. **Stiskanje je izbačeno.** Vrhovi ulaze 2 mm u plohu (`touch_depth`) — cilj je **dokazati
   dodir**, ne stvoriti silu trenja; stisak je izbacivao kocku ([[D-06_cube_squeeze_grasp]]).

### Uvjet za dizanje (korak 5d u `main_task`)
Dizanje je dopušteno tek kad su **istovremeno** ispunjena dva neovisna uvjeta:
- **kontakt obiju ruku s kutijom** — broji se samo dodir čiji je drugi sudarač `aruco_box`, pa
  dodir stola ili vlastitog tijela ne prolazi ([[D-05_contact_verified_attach]], [[P-27_contact_sensor_topic_ignored]]);
- **oba vrha alata unutar 10 mm od cilja**, pri čemu cilj dolazi iz **svježeg** očitanja dubine i
  markera, a ne iz poze koja je naredbom zadana.

Ako bilo koji uvjet padne, ruke se rasterete natrag u `GRASP_V4` i misija se prekida uz zapis
razloga ([[D-12_honesty_abort_over_fake]]).

## Odstupanje? Ne — detalj izvedbe
**Dizanje jest dvoručno**: bez potvrđenog dodira **obiju** ruku nema dizanja. Ono što je
jednoručno je **kruta veza za nošenje**: `DetachableJoint` se uspostavlja na jedno zapešće, jer
dvije istovremene krute veze čine kutiju kinematički preodređenom i ruše numeričko rješavanje
fizike ([[P-17_detachable_joint_explodes]], [[D-07_carry_on_left_wrist]]). Obje ruke pritom
**ostaju na kocki** — desna nakon attacha popušta samo 5 mm (`release_backoff`), po korisnikovoj
uputi od 15. 9. da se ruka **ne** povlači.

Zato se R-17 u seminaru vodi kao **ispunjen**, a jednoručna kruta veza se opisuje u poglavlju o
ograničenjima simulatora, ne u tablici odstupanja.

## Kako se rješava
- [[S-08_grasp_squeeze_attach]]: cijeli lanac prilaz → gate → attach → lift
