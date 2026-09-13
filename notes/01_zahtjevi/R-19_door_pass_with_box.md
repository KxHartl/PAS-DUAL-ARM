---
id: R-19
type: zahtjev
status: otvoreno
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-06_navigation]]", "[[S-08_grasp_squeeze_attach]]"]
problems: ["[[P-18_transport_drops_box]]", "[[P-12_door_too_narrow]]", "[[P-35_arm_span_too_wide_for_door]]"]
decisions: ["[[D-07_carry_on_left_wrist]]"]
updated: 2026-09-13
---
# R-19: Prolaz kroz vrata S kutijom

## Izvor (doslovno)
> „vrata … kroz koje robot mora proći s i bez kutije.“ [MAIL]

## Tehnički znači
Robot drži kocku (0.3 m) i vozi bazu (ravno + okreti) kroz otvor od 0.9 m. Kocka ostaje u
hvatu, a robot s kockom stane u otvor.

**Kriterij prihvaćanja:**
- [ ] kocka preživi vožnju baze (ravno 0.4 m + okret 60°) bez odlijetanja
- [ ] kocka + ruke stanu u 0.9 m (kocka ispred tijela, unutar širine baze)
- [ ] prolaz iz sobe A u sobu B s kockom

## Trenutno stanje
❌ **Nikad nije uspjelo.** Gibanje baze s kockom na DetachableJointu izbacivalo je kutiju (30. 6.).
Transport-proba je **napisana, a nije pokrenuta** (`probe_transport`, `73617e8`), a
`task.launch.py` taj parametar **ne prosljeđuje** ([[P-18_transport_drops_box]]).

## Kako se rješava
- [[S-08_grasp_squeeze_attach]]: način držanja tijekom vožnje
- [[S-06_navigation]]: vožnja kroz vrata
