---
id: P-39
type: problem
status: neprovjereno
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
solutions: ["[[S-06_navigation]]"]
decisions: ["[[D-16_zones_from_detected_features]]"]
updated: 2026-09-14
---
# P-39 — Nav2 ulazi u prolaz pod kutem i struže uz stol

## Simptom
Nakon prihvaćene karte (run 44) robot na Nav2 cilj u drugoj sobi kreće dijagonalno: prolaz
od 1.0 m pokušava proći ukoso, a pored stola prolazi preblizu i pod kutem, pa zapne.

## Uzrok
**Potvrđeno (izračun + mjerenje u costmapu 14. 9.):** nije planer, nego prihvatni kriteriji i
to što upravljač nije vidio zone.

Zahvaćena širina robota pri yaw grešci θ je `L·sin θ + W·cos θ` (L = 1.04 m, W = 0.854 m):

| yaw greška | zahvaćena širina | detektirani otvor 0.95 m |
|---|---|---|
| 0° | 0.854 m | ✅ 4.8 cm po strani |
| 2.9° (0.05 rad) | 0.905 m | ✅ 2.3 cm po strani |
| 6.9° (0.12 rad) | 0.972 m | ❌ ne stane |
| **14.3° (0.25 rad = stari `yaw_goal_tolerance`)** | **1.085 m** | ❌ ne stane |

Pet neovisnih uzroka, svaki dovoljan sam za sebe:
1. `yaw_goal_tolerance: 0.25` — cilj ispred vrata primljen kao „stigao" uz 14° zakreta.
2. `xy_goal_tolerance: 0.20` — a bočne rezerve u otvoru ima 4.8 cm.
3. `RotationShimController.angular_dist_threshold: 0.12` — prestane okretati na 6.9°, što
   traži 0.972 m. To je, a ne tolerancija cilja, ono što odlučuje kako robot ulazi u vrata.
4. Keepout maska („lijevci") bila je u `filters` **samo globalnog** costmapa → DWB je nije
   vidio i smio je presjeći kut koji je planeru bio zabranjen.
5. `footprint` 0.90 m (procjena) vs izmjerenih 0.854 m ([[P-35_arm_span_too_wide_for_door]]), uz
   `inflation_radius` 0.48 > upisanog radijusa. U otvoru od 0.95 m to daje 2.5 cm po strani, pa
   `ObstacleFootprint` odbacuje gotovo svaku trajektoriju — zato mu je `scale` bio spušten na
   0.02, čime je izgubljen jedini kritičar koji uopće gleda pravi pravokutnik.

Uz to su zone i waypointi bili magični brojevi na dva mjesta (`generate_keepout_mask.py` i
`go_to_room.py`), pa su se mogli razići.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 13. 9. (`nav2_params.yaml`) | ručno crtana keepout maska s lijevcima uz oba prolaza + `go_to_room.py` s tvrdo upisanim waypointima 1.2 m od vrata | robot i dalje ulazi ukoso | maska je bila samo na globalnom costmapu; tolerancije ostale 0.20 / 0.25 |
| 2 | 14. 9. `497a69a` | detekcija vrata iz spremljene karte (`scripts/check_doors.py`, offline) | oboja vrata nađena: (0.00, −3.00) i (3.00, −0.01), promašaj centra 1.5 cm, širina **0.95 m** | karta zadeblja zid ~2.5 cm po strani; zone se grade na 0.95 m, ne na 1.0 m |
| 3 | 14. 9. `497a69a` | stolovi iz karte kao klasteri slobodnostojećih nogu | 8 nogu, 2 stola, centri (−0.04, −6.53) i (6.54, −0.01); karta ima točno 9 komponenti, bez šuma | pojas 0.075–0.125 m u RGB-D detektoru je bio mrtav (ploha je na 0.75 m) |
| 4 | 14. 9. `497a69a` | zone iz detektiranih značajki (`nav_zones`), tolerancije 0.10 / 0.05, shim 0.06, footprint 0.854, `ObstacleFootprint.scale` 1.0, keepout i na lokalnom costmapu | headless: 8/8 kontrolera, filter aktivan na **oba** costmapa, 14/14 sondi costmapa točno | zone stvarno dolaze do upravljača |
| 5 | 14. 9. `497a69a` | izmjeren kut kojim **planirana putanja** siječe prag vrata, 5 slučajeva preko `/compute_path_to_pose` | 0.00–1.07° od okomice, 2.5 cm od osi; traži 0.854–0.873 m | i jedan jedini cilj kroz cijelu sobu sad prolazi okomito |
| 6 | 14. 9. | isto, ali s praznom keepout maskom (A/B, zone isključene) | najgori prolaz 2.92° i 5.6 cm od osi (vs 0.34° i 2.5 cm sa zonama) | **globalnu putanju uglavnom ispravlja već ispravljen footprint**; zone popravljaju najgori slučaj i daju graf i poze. Glavni uzrok kvara bile su tolerancije i nevidljivost zona lokalnom upravljaču |

## Trenutno rješenje
[[D-16_zones_from_detected_features]]. Zone se generiraju iz karte
(`nav_zones.py`), Nav2 i dalje vozi, a `room_navigator.py` vodi po grafu soba s poštenim
gateom prije svakog prolaza. Parametri: `nav2_params.yaml` (goal checker, shim, footprint,
kritičari, `filters` na `local_costmap`). Offline provjere: `scripts/check_doors.py`,
`scripts/check_zones.py`.

## Sljedeći korak
Vožnja u GUI-ju (runovi 45+). Kriterij: tri uzastopna prolaza po smjeru bez dodira,
|Δyaw| na pragu < 0.05 rad, minimalni bočni razmak > 3 cm, skok `map→odom` < 0.2 m.
**Sve dosad je mjereno na planeru i costmapu — nijedan metar nije odvožen.**

## Ne ponavljati
- Popuštanje `yaw_goal_tolerance` da bi cilj „prošao": za ovaj robot i ova vrata 0.25 rad je
  geometrijski nemoguć.
- Spuštanje `ObstacleFootprint.scale` da DWB nađe trajektoriju: to skriva da je footprint kriv.
- Keepout zone samo na globalnom costmapu.
- Zone upisivati ručno na dva mjesta ([[D-16_zones_from_detected_features]]).
- Širenje vrata radi Nav2 ([[P-12_door_too_narrow]]).
