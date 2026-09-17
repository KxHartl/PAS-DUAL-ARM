---
id: P-40
type: problem
status: djelomicno_potvrdeno
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]"]
solutions: ["[[S-06_navigation]]"]
decisions: ["[[D-18_verified_baseline_first]]"]
updated: 2026-09-14
---
# P-40 — AMCL poza odstupa ~7 cm od onoga što lidar mjeri u prolazu

## Simptom
Run 59, dionica 4/5 `home -> red`, abort:
`only -0.4 cm beside the robot (left 0.423 m, right 0.579 m), limit 0.5 cm`.
Robot pritom nije udario dovratnik; gate je pošteno stao.

Isti obrazac kroz cijelu povijest runova: otkloni od osi rastu **monotono unutar
jednog runa** — run 54 je izmjerio 0.022 → 0.042 → 0.076 → 0.095 m. Korisnik je to
opisao kao „nakupljanje grešaka".

## Uzrok
**Potvrđeno (izračun iz brojeva runa 59):**

```
left 0.423 + right 0.579 = 1.002 m     -> lidar TOČNO vidi otvor od 1.00 m
(0.579 - 0.423) / 2      = 0.078 m     -> robot je 7.8 cm od osi otvora
AMCL u istom trenutku: pose (+2.53, +0.02)
detektirana vrata (nav_zones):   y = +0.015
                                       -> AMCL misli da je robot na osi
```

Aritmetika se zatvara i s treće strane: `gap = min(left, right) − 0.427 = −0.004 m`,
što je točno `-0.4 cm` iz poruke. Dakle mjerenje lidarom je ispravno, a **poza na
kojoj sve ostalo vozi je pomaknuta ~7 cm**, uz fizički budžet od 7.3 cm po strani.

Nije upravljač: DWB je vodio robota na os koju mu je zadala kriva poza.

**Tri doprinosa, svaki izmjeren:**

1. **Karta je snimljena starim lidarom.** `maps/seminar_map.*` je od 13. 9.
   (run 44); commit od 14. 9. podigao je lidar s 360 zraka / 10 mm / 10 Hz
   na 1080 zraka / 1 mm / 25 Hz. Karta taj senzor nikad nije vidjela.
2. **Rezolucija karte 0.05 m.** `scripts/check_map_geometry.py` (novo, 14. 9.) na njoj
   mjeri: zid od 0.10 m nacrtan **0.150–0.178 m** debelo (+2.5 do +3.9 cm po licu),
   otvor 1.00 m očitan kao **0.950 m**, os prolaza pomaknuta **+1.5 cm**, jedno lice
   zida uz svaka vrata zakrenuto **1.4–2.2°** uz RMS 14–16 mm, stepenica između lica
   s dvije strane otvora 27 i 31 mm. **Mjerilo karte je pritom savršeno (0.0 mm na
   4.243 m)** — dakle to nije SLAM drift nego kvantizacija rešetke.
3. **AMCL-ov mjerni model je preširok za taj senzor:** `sigma_hit: 0.2` (20 cm) uz
   `z_rand: 0.5`. Razlika od nekoliko centimetara gotovo ne mijenja težinu čestice, a
   likelihood field se ionako računa na rešetki karte. Lidar od 1 mm se ne koristi.

**Nije izmjereno (zato i postoji `loc_error`):** koliki je stvarni |AMCL − istina| i
gdje kroz rutu naraste. Do 14. 9. to nitko nije mjerio.

## Pokušaji
| # | datum / commit | što smo probali | rezultat | zaključak |
|---|---|---|---|---|
| 1 | 14. 9. (run 59, slika) | ručni izračun iz poruke aborta | lidar 1.002 m / 7.8 cm od osi vs AMCL „centriran" | greška je u lokalizaciji, ne u upravljaču; treba je **mjeriti**, ne procjenjivati |
| 2 | 14. 9. | `scripts/check_map_geometry.py` na postojećoj karti | zid 0.15 m (stvarno 0.10), otvor 0.950 m, os +1.5 cm, mjerilo 0.0 mm | uzrok je rezolucija 0.05 m, ne drift; karta uz to nikad nije vidjela novi lidar |
| 3 | 14. 9. | `slam_params.yaml`: `resolution` 0.05 → 0.02, `smear_deviation` 0.1 → 0.03, graf 0.2 → 0.1 m / rad, `min_laser_range` 0.12 → 0.05, `scan_buffer_size` 10 → 20 | čeka novu turu (run 60) | — |
| 4 | 14. 9. | `loc_error` + `bridge_debug.yaml` (`debug_truth:=true`), i telemetrija lidar-vs-poza u `room_navigator._report_disagreement` | čeka run | ground truth **samo za debug**; telemetrija u navigatoru radi bez njega, dakle i na fizičkom robotu |
| 5 | 14. 9. (run 60, GUI) | nova teleop karta na 0.02 m novim lidarom | ✅ **doprinos karte izmjeren:** otvor 0.950 → **0.980 m**, os prolaza +1.5 → **0.0 cm**, zid 150–178 → **120 mm**, nagib lica 1.4–2.2° → **≤ 0.02°**, RMS 14–16 → **≤ 2.3 mm** | rezerva po strani 4.8 → **6.3 cm**. Time su doprinosi 1 i 2 (stari lidar, rezolucija) zatvoreni; ostaje koliko od ~7 cm otpada na doprinos 3 (AMCL) |
| 6 | 14. 9. (run 61, GUI) | vožnja na novoj karti uz **nepromijenjen** AMCL, telemetrija lidar-vs-poza | ⚠ 2/3 prolaza prošla (3.0 i 2.0 cm), treći abort. Razlika lidar-vs-AMCL: **+6.0 cm** (home→red), **−5.9 cm** (red→home), **−4.4 cm** (home→blue). Otvor iz lidara 0.998–1.000 m | **Prva dva su isti +6 cm u okviru karte** — „lijevo" se okrene sa smjerom vožnje — dakle AMCL ima **stalan pomak** (≈ +4 cm u x, −6 cm u y), ne drift. Potvrda s korisnikove slike: AMCL `x = −0.04`, iz lidara `x = 0.50 − 0.582 = −0.082` → +4.2 cm |
| 7 | 14. 9. | AMCL mjerni model: `sigma_hit` 0.2 → **0.05**, `z_hit/z_rand` 0.5/0.5 → **0.9/0.1**, `laser_likelihood_max_dist` 2.0 → **0.5**, `laser_max_range` 100 → **12**, `laser_min_range` −1 → **0.05**, `max_beams` 180 → **360**, `resample_interval` 1 → **2**, čestice 500/2000 → **1000/3000**, `transform_tolerance` 1.0 → **0.3** | čeka run 62 | `alpha1..5` **namjerno ostaju 0.2** — postavljaju se iz izmjerenog drifta odometrije (`loc_error`), a mijenjati ih u istom runu učinilo bi oboje nečitljivim |

## Trenutno rješenje
Ništa se još ne korigira. U kodu su **samo mjerila**:
- `src/pas_dual_arm_scripts/pas_dual_arm_scripts/loc_error.py` — |TF `map→base_footprint`
  − Gazebo poza| pokreće se samo uz `sim.launch.py debug_truth:=true`;
- `room_navigator._report_disagreement()` — razlika lidarske i lokalizirane osi u
  svakom prolazu, bez ground trutha;
- `scripts/check_map_geometry.py` — geometrijski gate karte, zove ga `save_map.sh`.

## Sljedeći korak
1. ~~Run 60: nova teleop karta na 0.02 m~~ — **napravljeno, gate prošao** (pokušaj 5).
2. ~~Run 61: vožnja na novoj karti uz nepromijenjen AMCL~~ — **napravljeno** (pokušaj 6):
   karta i zone su OK, ostaje stalan pomak AMCL-a od 4–6 cm.
3. **Run 62 (sljedeće):** novi AMCL mjerni model (pokušaj 7), i to s
   `sim.launch.py debug_truth:=true` da `loc_error` potvrdi smjer pomaka.
   **Kriterij:** razlika lidar-vs-AMCL **< 2 cm** u svakom prolazu i 5/5 dionica bez aborta.
   Ako pomak ostane: sljedeće je `alpha1..5` iz izmjerenog drifta odometrije.
4. Tek ako nakon toga ostane razlika: korekcija osi prolaza iz lidara — i to kao
   zaseban inkrement, s runom koji dokazuje da je lidarska mjera bolja
   ([[D-18_verified_baseline_first]]).

## Ne ponavljati
- Popuštanje `min_side_clearance` da prolaz „prođe": run 59 je već na 5 mm, a stvarna
  greška je 7 cm. To skriva pomaknutu pozu umjesto da je popravi.
- Zaključivanje o točnosti lokalizacije bez mjerenja; do 14. 9. se o 7 cm nagađalo.
- Uvođenje ground trutha bilo gdje u upravljački lanac — stack mora raditi i na
  fizičkom robotu.
