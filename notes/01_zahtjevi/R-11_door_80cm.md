---
id: R-11
type: zahtjev
status: djelomicno
source: "[MAIL] + odluka korisnika 13. 9. (0.9 m)"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]", "[[S-06_navigation]]"]
problems: ["[[P-12_door_too_narrow]]", "[[P-35_arm_span_too_wide_for_door]]"]
decisions: ["[[D-13_three_room_world]]", "[[D-08_door_widened]]"]
updated: 2026-09-13
---
# R-11: Prolaz(i) standardne širine: 0.9 m

## Izvor (doslovno)
> „Gazebo okruženje mora imat jedan prolaz, vrata standardne dimenzije recimo 80cm“ [MAIL]
> Korisnik 13. 9.: vrata **90 cm**, tri sobe u L ([[D-13_three_room_world]]).

## Tehnički znači
Sobe su povezane otvorima standardne širine (0.9 m), a do kutije i do odredišta se može doći
samo kroz njih. „Recimo 80cm“ ostavlja slobodu, a 0.9 m su standardna sobna vrata.
Napomena: [MAIL] kaže „jedan prolaz“. Naš svijet ima dva otvora (HOME↔plava, HOME↔crvena), pa robot
s kutijom prolazi **dvoja** vrata. To je stroži test od traženog, a ne odstupanje.

**Kriterij prihvaćanja:**
- [x] otvori 0.9 m u SDF-u (zid y = -2: x ∈ [-0.45, 0.45]; zid x = 2: y ∈ [-0.45, 0.45])
- [x] jedini put do kutije i odredišta vodi kroz otvore (zatvorene sobe)
- [ ] prolaz robota provjeren ([[R-18_door_pass_empty]], [[R-19_door_pass_with_box]])

## Trenutno stanje
⚠ **Vrata od 0.9 m postoje (13. 9.), a prolaz još nije testiran.** Povijest: 0.8 m (26. 5.) →
1.2 m (`45c32f1`, 23. 6.) → 2.0 m (`e9157e1`, 29. 6.) → **0.9 m** (13. 9.). Vidi
[[P-12_door_too_narrow]], [[D-08_door_widened]] (zamijenjena) i [[D-13_three_room_world]].

Baza je široka ~0.6 m (footprint ±0.30 m), pa u 0.9 m ima ±0.15 m zazora. **Ruke su problem:**
ramena su na y = ±0.26, a u `ARM_CARRY` laktovi na ±0.58 (izmjereno 13. 9.). Potrebna je uža poza
za vrata (|y| < 0.40), vidi [[P-12_door_too_narrow]].

## Kako se rješava
- [[S-02_world_and_sim_launch]]: geometrija zidova
- [[S-06_navigation]]: prolazak (Nav2 inflation, poravnanje ispred vrata)
