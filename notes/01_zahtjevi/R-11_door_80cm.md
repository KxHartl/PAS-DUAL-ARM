---
id: R-11
type: zahtjev
status: ispunjeno
verified: "vrata 1.0 m; prolaz 5/5 prazan i s kutijom (runovi 62, 70, M4)"
source: "[MAIL] + odluka korisnika 13. 9. (0.9 m)"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]", "[[S-06_navigation]]"]
problems: ["[[P-12_door_too_narrow]]", "[[P-35_arm_span_too_wide_for_door]]"]
decisions: ["[[D-13_three_room_world]]", "[[D-08_door_widened]]"]
updated: 2026-09-17
---
# R-11: Prolaz(i) standardne širine: 1.0 m

## Izvor (doslovno)
> „Gazebo okruženje mora imat jedan prolaz, vrata standardne dimenzije recimo 80cm“ [MAIL]
> Korisnik 13. 9.: tri sobe u L ([[D-13_three_room_world]]).

## Tehnički znači
Sobe su povezane otvorima standardne širine, a do kutije i do odredišta se može doći samo kroz
njih. „Recimo 80 cm“ ostavlja slobodu. **Konačna širina je 1.0 m**, jer je robot s uvučenim
rukama 0.821 m širok — kroz 0.80 m fizički ne prolazi. Zato je test otežan na drugoj osi:
**dvoja** vrata, u oba smjera, prazan i s kutijom. To je i zapisano kao odstupanje
([[odstupanja]] #3).

[MAIL] kaže „jedan prolaz“. Naš svijet ima dva otvora (HOME↔plava, HOME↔crvena), pa robot s
kutijom prolazi **dvoja** vrata.

**Kriterij prihvaćanja:**
- [x] otvori 1.0 m u SDF-u (zid y = -3: x ∈ [-0.5, 0.5]; zid x = 3: y ∈ [-0.5, 0.5])
- [x] jedini put do kutije i odredišta vodi kroz otvore (zatvorene sobe)
- [x] prolaz robota potvrđen prazan ([[R-18_door_pass_empty]], 5/5, runovi 62 i 70)
- [x] prolaz robota potvrđen s kutijom ([[R-19_door_pass_with_box]], run M4)

## Tri veličine koje se ne smiju miješati
Ovo je izvor zabune u seminaru; razlikovati ih doslovno ovim imenima:

| Veličina | Iznos | Odakle |
|---|---|---|
| **fizička širina otvora** | **1.000 m** | SDF, `seminar_world.sdf` (dva zidna segmenta, središta ±1.75) |
| **širina otvora očitana na karti** | **0.980 m** | SLAM karta runa 60, `check_map_geometry.py` |
| **nominalni geometrijski zazor** | ≈ **8 cm po strani** | (0.980 − 0.821) / 2, robot u `DRIVE_V4` |
| **raspoloživa rezerva uz objavljeni obris** | **7.3 cm** | obris koji se objavlja nije jednak geometrijskoj širini |
| **najmanji izmjereni bočni razmak u vožnji** | **3.0–6.1 cm** | runovi 62 i 70, 5/5 prolaza |

## Trenutno stanje
✅ **Riješeno.** Povijest širine: 0.8 m (26. 5.) → 1.2 m (23. 6.) → 2.0 m (29. 6.) → 0.9 m
(13. 9.) → **1.0 m** (13. 9., [[D-13_three_room_world]]).

Baza je široka ~0.6 m (footprint ±0.30 m). **Ruke su šire:** u `DRIVE_V4` robot je 0.821 m,
u `CARRY_V4` 0.827–0.854 m (potvrđeno dvjema neovisnim metodama,
[[P-35_arm_span_too_wide_for_door]]). Stara `ARM_CARRY` iz koda bila je ~1.2 m i nije prolazila.
Vidi [[08_poze]].

## Kako se rješava
- [[S-02_world_and_sim_launch]]: geometrija zidova
- [[S-06_navigation]]: prolazak (jedinstveno potencijalno polje, brazda nulte cijene kroz otvor,
  portalne poze i poravnanje ispred vrata — [[D-20_single_potential_field_costmap]],
  [[P-39_nav2_enters_doorway_at_an_angle]])
