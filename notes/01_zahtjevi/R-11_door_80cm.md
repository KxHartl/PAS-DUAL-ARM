---
id: R-11
type: zahtjev
status: odstupanje
source: "[MAIL]"
parent: "[[00_MAPA]]"
solutions: ["[[S-02_world_and_sim_launch]]"]
problems: ["[[P-12_door_too_narrow]]"]
decisions: ["[[D-08_door_widened]]"]
updated: 2026-09-13
---
# R-11: Jedan prolaz, vrata ~80 cm

## Izvor (doslovno)
> „Gazebo okruženje mora imat jedan prolaz, vrata standardne dimenzije recimo 80cm“ [MAIL]

## Tehnički znači
U svijetu je točno jedan otvor u zidu, širine ~0.8 m, kroz koji je jedini put do odredišta.
„Recimo“ ostavlja malo slobode, ali red veličine je standardna vrata (0.8–0.9 m).

**Kriterij prihvaćanja:**
- [ ] otvor u `wall_with_door` širok ≈ 0.8 m
- [x] jedini put do `target_table` vodi kroz otvor (zid y ∈ [-3, 3])

## Trenutno stanje
🔁 **Otvor je 2.0 m** (y ∈ [-1, 1], `seminar_world.sdf` model `wall_with_door`). Povijest:
0.8 m (26. 5.) → 1.2 m (`45c32f1`, 23. 6., Nav2 prolaz) → 2.0 m (`e9157e1`, 29. 6.). Vidi
[[P-12_door_too_narrow]] i [[D-08_door_widened]].

Robot je širok ~0.6 m (footprint ±0.30 m), a ispružene ruke su šire. Uz uvučene laktove
(`ARM_CARRY`, raspon ~0.6 m) fizički prolaz kroz 0.8 m je moguć, ali tijesan (±0.1 m zazora).

## Kako se rješava
- [[S-02_world_and_sim_launch]]: geometrija zida
- [[S-06_navigation]]: prolazak kroz vrata (Nav2 inflation, poravnanje ispred vrata)
