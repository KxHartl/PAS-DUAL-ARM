---
id: R-06
type: zahtjev
status: djelomicno
source: "[ZAD]"
parent: "[[00_MAPA]]"
solutions: ["[[S-01_robot_description]]", "[[S-03_ros2_control_setup]]"]
problems: ["[[P-13_torso_prismatic_no_lift]]", "[[P-15_dart_friction_no_hold]]", "[[P-10_skid_steer_cannot_turn]]"]
decisions: ["[[D-05_contact_verified_attach]]"]
updated: 2026-09-13
---
# R-06: Realni parametri i realistična interakcija

## Izvor (doslovno)
> „Cilj je omogućiti realističnu simulaciju kretanja i interakcije robota s okolinom. Parametre
> simulacije potrebno je podesiti tako da odgovaraju tehničkim specifikacijama stvarnog robota i
> aktuatora.“ [ZAD]

## Tehnički znači
Mase, inercije, limiti zglobova (pozicija, brzina, sila) i dinamika kontakta približno odgovaraju
stvarnim komponentama, a interakcija s objektom je fizikalna (ne teleport).

**Kriterij prihvaćanja:**
- [x] Kinova + Robotiq: proizvođačev URDF (mase/limiti iz `ros2_kortex`)
- [x] omni_base: PAL model (mase, dimenzije kotača r = 0.0762 m, razmak 0.44715 m)
- [ ] torzo: mase su procjena (12 kg vodilica, 2 kg klizač), efort 1000 N je podignut radi
  simulacije, a ne iz datasheeta
- [ ] kontakt/trenje: hvat stvarno drži objekt trenjem
- [x] nema teleporta: attach tek nakon fizički dokazanog kontakta

## Trenutno stanje
⚠ Djelomično. Tri svjesna kompromisa s obrazloženjem:
1. **Trenje u DART-u ne drži objekt** ([[P-15_dart_friction_no_hold]]), pa se koristi
   kontaktom verificiran DetachableJoint ([[D-05_contact_verified_attach]]).
2. **Trenje kotača** je umjetno postavljeno (mu1 0.4, mu2 0) da skid-steer može rotirati
   ([[P-10_skid_steer_cannot_turn]]).
3. **Torzo** se ne diže pod teretom ([[P-13_torso_prismatic_no_lift]]).

Sve tri stavke idu u [[odstupanja]].

## Kako se rješava
- [[S-01_robot_description]]: mase, trenje, limiti
- [[S-03_ros2_control_setup]]: limiti kontrolera (baza 0.6 m/s, 1.0 rad/s)
- Sve vrijednosti: [[06_parametri]]
