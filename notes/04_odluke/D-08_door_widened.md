---
id: D-08
type: odluka
status: zamijenjena
deviation: true
requirements: ["[[R-11_door_80cm]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-12_door_too_narrow]]"]
superseded_by: "[[D-13_three_room_world]]"
updated: 2026-09-13
---
# D-08: Vrata proširena (0.8 → 1.2 → 2.0 m)

> [!note] ZAMIJENJENO 13. 9. 2026.
> Novi svijet s tri sobe ima vrata od **0.9 m** → [[D-13_three_room_world]]. Ova kartica ostaje kao povijest.

## Kontekst
Nav2 nije mogao isplanirati prolaz kroz 0.8 m: robot je širok 0.6 m, a footprint je ±0.30 uz
inflaciju 0.35 m.

## Odluka
- 23. 6. (`45c32f1`): **1.2 m** (= 2× širina robota) + `inflation_radius` 0.35 → 0.15. Prolaz
  provjeren headless.
- 29. 6. (`e9157e1`): **2.0 m** (uzgredna izmjena u commitu percepcije; razlog nije zapisan,
  vjerojatno slobodan prostor za eksperimente s kutijom).

## Posljedice
Zahtjev od 80 cm nije ispunjen. `inflation_radius` je u međuvremenu smanjen na 0.05.

## Odnos prema zahtjevima
🔁 **Odstupa od [[R-11_door_80cm]].** U [[odstupanja]]. Vraćanje na 0.8 m je jeftina izmjena
SDF-a. Test je prolaz s `ARM_CARRY` pozom, vidi [[P-12_door_too_narrow]].
