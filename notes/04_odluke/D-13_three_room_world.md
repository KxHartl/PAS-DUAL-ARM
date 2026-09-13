---
id: D-13
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-10_mappable_world]]", "[[R-11_door_80cm]]", "[[R-13_destination_place]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]", "[[R-20_place_at_destination]]"]
problems: ["[[P-12_door_too_narrow]]"]
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

| Soba | Boja | Područje | Sadržaj |
|---|---|---|---|
| **HOME** (srednja, kut L-a) | siva | x ∈ [-2, 2], y ∈ [-2, 2] | početna poza robota, start mapiranja |
| **KUTIJA** (desno, −y) | plava | x ∈ [-2, 2], y ∈ [-6, -2] | `pick_table` (0, -4.4) + `aruco_box` (0, -4.28) |
| **ODREDIŠTE** („gore“ = naprijed, +x) | crvena | x ∈ [2, 6], y ∈ [-2, 2] | `place_table` (4.3, 0) |

- Otvori HOME↔PLAVA (zid y = -2, x ∈ [-0.45, 0.45]) i HOME↔CRVENA (zid x = 2, y ∈ [-0.45, 0.45])
  su široki **0.9 m** (standardna vrata).
- Zidovi su visoki 1.2 m, pa kamera iz HOME sobe ne vidi kutiju preko zida i robot mora
  navigirati u plavu sobu.
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
- Prolaz 0.9 m uz bazu širine 0.6 m daje ±0.15 m zazora. Nav2 footprint ±0.30 + inflacija 0.05
  ostavlja 0.2 m slobodnog koridora.
- Uklonjeni su `target_table` (0.775 m) i stari `wall_with_door`.

## Odnos prema zahtjevima
Ispunjava [[R-10_mappable_world]], [[R-11_door_80cm]] (0.9 m su standardna vrata, „recimo 80cm“) i
[[R-13_destination_place]]. **Zamjenjuje [[D-08_door_widened]].**
