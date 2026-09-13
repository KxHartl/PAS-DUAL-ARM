---
id: D-13
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-10_mappable_world]]", "[[R-11_door_80cm]]", "[[R-13_destination_place]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
problems: ["[[P-12_door_too_narrow]]", "[[P-35_arm_span_too_wide_for_door]]", "[[P-36_walls_lower_than_camera]]"]
superseded_by: ""
updated: 2026-09-13
---
# D-13: Svijet s tri sobe u obliku slova L, vrata 0.9 m

## Kontekst
[MAIL] traži okruženje koje robot mapira, zadavanje regije, **jedan prolaz ~80 cm** („recimo“) i
zadano odredište. Stari svijet je imao jedan zid s otvorom od 2.0 m ([[D-08_door_widened]]) i stol
iza zida na 0.775 m, do kojeg doseg nije provjeren.

## Odluka (korisnik, 13. 9. 2026.)
Tri prostorije u obliku slova **L**. Robot je na ishodištu i gleda prema +x:

| Soba | Boja | Područje (6 × 6 m) | Sadržaj |
|---|---|---|---|
| **HOME** (srednja, kut L-a) | siva | x ∈ [-3, 3], y ∈ [-3, 3] | početna poza robota, start mapiranja |
| **KUTIJA** (desno, −y) | plava | x ∈ [-3, 3], y ∈ [-9, -3] | `pick_table` (0, -6.5) + `aruco_box` (0, -6.38) |
| **ODREDIŠTE** („gore“ = naprijed, +x) | crvena | x ∈ [3, 9], y ∈ [-3, 3] | `place_table` (6.5, 0) |

- Otvori HOME↔PLAVA (zid y = -3, x ∈ [-0.5, 0.5]) i HOME↔CRVENA (zid x = 3, y ∈ [-0.5, 0.5])
  su široki **1.0 m** (odluka korisnika, 13. 9.). Robot je s uvučenim rukama 85.4 cm širok
  ([[P-35_arm_span_too_wide_for_door]]), pa ostaje **7.3 cm zazora po strani**.
- Sobe su **6 × 6 m** (13. 9., povećane s 4 × 4): više prostora za manevar, dužu vožnju i
  smisleniju kartu.
- Zidovi su visoki **3.0 m** (13. 9., odluka korisnika). Prvotnih 1.2 m nije blokiralo pogled: kamera
  je na 1.39 m, dakle iznad njih → [[P-36_walls_lower_than_camera]].
- **Oba stola su iste niske izvedbe** (ploha z = 0.10). Odlaganje zato spušta kocku na visinu
  uzimanja, a postojeći STEP8 to već radi.
- Marker na kutiji gleda prema vratima plave sobe (+y).
- Tumačenje „gore“ = naprijed (+x) od home poze i „desno“ = −y **korisnik je potvrdio u GUI-ju
  13. 9.** (svih 8 kontrolera aktivno, bez grešaka).

## Posljedice
- Kutija više nije vidljiva s početne poze. Misija mora ići redom iz [MAIL]:
  **mapiranje (SLAM) → regija (Nav2 u plavu sobu) → pronađi → podigni → nosi kroz dvoja vrata →
  odloži u crvenoj sobi**.
- Trenutni `main_task` (skeniranje s mjesta) u ovom svijetu **neće naći kutiju** dok se ne doda
  korak navigacije u regiju ([[R-15_region_goal_nav2]]).
- Prolaz 1.0 m uz robota od 0.85 m daje 7.3 cm po strani. To je tijesno za Nav2 DWB, pa je
  determinističko provlačenje ([[D-15_door_transit_behaviour]]) praktički nužno.
- Uklonjeni su `target_table` (0.775 m) i stari `wall_with_door`.

## Odnos prema zahtjevima
Ispunjava [[R-10_mappable_world]], [[R-11_door_80cm]] (1.0 m je u duhu „recimo 80cm“, uz nužnu
rezervu za 0.85 m široka robota) i [[R-13_destination_place]]. **Zamjenjuje [[D-08_door_widened]].**
