---
id: P-39
type: problem
status: djelomicno_potvrdeno
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
solutions: ["[[S-06_navigation]]"]
decisions: ["[[D-16_zones_from_detected_features]]", "[[D-17_closed_loop_door_transit]]"]
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

| 7 | 14. 9. (GUI, korisnik) | prva vožnja sa zonama | ❌ **robot se ne može okrenuti u mjestu kad stoji ispred stola — zapne za zonu.** Izmjereno: prolaz kroz halo stola bio je širok samo stol + 0.20 m, pa je pri okretu kut robota ulazio u halo pokraj prolaza. Usput otkriveno da je i `portal_standoff` 0.75 < potrebnih 0.923 m. | prolaz kroz halo → **puna širina halo-a**; `portal_standoff` 0.75 → 0.95; `chute_length` 1.20 → 0.85. `check_zones.py` sada provjerava okret u mjestu na **svakoj** pozi gdje navigator stane (i portali i prilaz stolu), iz poze pomaknute za toleranciju cilja i uz rezervu 0.15 m |
| 8 | 14. 9. | zaostali `velocity_smoother` iz ranijeg headless testa | ❌ novi `lifecycle_manager` ga našao u stanju `active` → `No transition matching 1 found` → **prekinut cijeli bringup**, karta se ne učita | uvijek provjeriti zaostale čvorove prije starta; upisano u [[01_pokretanje]] |
| 9 | 14. 9. | ekskluzivno poravnanje, lokalni keepout, sirovi sken i zatvorena petlja za uski tranzit | ✅ headless prolaz home→blue: poravnanje uspjelo, tranzit dovršen s minimalnim bočnim razmakom **4.7 cm**, na izlazu +0.2°; ❌ Nav2 prilaz stolu zapeo uz cilj na rubu halo zone | tranzit izdvojen iz DWB-a; `table_standoff` 0.60→0.80 m, potreban retest dolaska stolu i povratka |
| **A** | 14. 9. (GUI, korisnik) | **referentni run, Nav2 vozi sve dionice**, gate = `arms_ok` + `aligned_with` (8 cm / 5°), `square_corners` False, bočno ±0.30, bez lokalne inflacije | ✅ ručni RViz cilj → plava soba za **33.1 s**, Vrata 0 prijeđena s **1.8 cm i 4.2°**; ✅ `goto red` → 5 dionica, pred crvenim stolom za **60.6 s** | **ovo je stanje na koje se vraćamo.** Nije bilo commitano — postojalo je samo kao zapis u `STATE.md` |
| 10 | 14. 9. | na run A naslagan sloj: `realign_to_portal`, `_transit_doorway` (2.9 m s isključenim Nav2), `doorway_margin` umjesto `aligned_with`, prekid kod stola na 25 cm | ❌ **nikad odvoženo**; `aligned_with` ostao u datoteci, ali ga nitko ne zove | novi gate odbija **upravo prolaz iz runa A**: `0.95 − swept(4.2°) − 0.05 = −2.5 cm` → ABORT. Zamijenjen je kriterij koji je propustio stvarni prolaz, bez runa koji bi to opravdao |
| 11 | 14. 9. | na to naslagano još: `envelope_monitor` s `require_envelope` (po defaultu **odbija voziti**), mjerenje iz dovratnika, 3 pokušaja poravnanja, kutni `scan_filter` | ❌ **nikad odvoženo**; korisnik: „trenutačno sustav radi najgore ikad" | dodan još jedan razlog za abort na stanje koje već nije vozilo. Kod je sačuvan na grani `wip/door-transit-closed-loop`; `main` vraćen na run A |

| 12 | 14. 9. `b5ee64e` | prvi run s ispravnim usmjeravanjem (alat 2D Goal Pose): portal pogoden 9.8 cm / +2.7 stupnjeva, gate prosao | **`No valid trajectories out of 3711`**, status 6, pa Nav2 recovery zavrtio robota u mjestu (to je bio neobjasnjivi yaw) | izmjereno u lokalnom costmapu: u otvoru od **100 cm** slobodno je samo **80 cm**, a robot s paddingom 86.2 cm. DWB je bio u pravu; greska je u tome **sto costmap crta** |

| 13 | 14. 9. `a3cc8da` | run s 8 vrhova i rezolucijom 0.025 | ⚠ prolaz **uspio**, ali robot stane tocno kad ude u vrata, stoji **~30 s**, pa prode uredno. `Control loop missed` pao s 1184 na 3; nova greska je `Failed to make progress` (×3) | `scan_filter` s `half_width` 0.47 brise bliži dovratnik cim je bocni otklon > 3 cm (dovratnik je na 0.50). Bez povrata voxel sloj te celije **ne moze ni oznaciti ni raytraceom ocistiti**, pa oznake s prilaza ostanu zamrznute u prolazu. 30 s je `movement_time_allowance`; oslobodi ga recovery koji ocisti costmap. Maska suzena na **0.30 × 0.45**, prema stvarnoj sirini sasije na visini lasera |

| 14 | 14. 9. | povratak crvena -> home, isti run | ❌ **`ABORT before leg 2/3: 0.095 m off the lane centreline (limit 0.08)`** — gate je ispravno odbio, robot ostao stajati | kontradikcija u konfiguraciji: `xy_goal_tolerance` 0.10 dopusta Nav2-u da parkira 10 cm od portala, a gate trazi <= 8 cm, pa legalan dolazak moze biti nelegalno stanje **bez ikakvog izlaza**. Otkloni kroz run rastu 0.022 -> 0.042 -> 0.076 -> 0.095. Gate se ne smije popustiti (fizicki budzet 7.75 cm po strani), pa je pritegnuta isporuka: `xy_goal_tolerance` -> **0.05**, `required_movement_radius` -> **0.05** |

| 16 | 14. 9. | nadogradnja na PAL Tiago Omni Base lidar + SE(2) RotationShimController + fino uzorkovanje | ❌ robot stigao 2.9 cm od portala, ali zakrenut na **+132.6°** (`ABORT before leg 2/3`) | Uzrok: `use_final_approach_orientation: true` u NavFn-u prebrisao je kut cilja ($0.0^\circ$) kutom zadnjeg dijagonalnog koraka A* rešetke ($+132.6^\circ$); `RotationShimController` nepotrebno rotirao omni bazu; `debug_trajectory_details: true` serijalizirao 15.625 trajektorija na 20 Hz |
| 17 | 14. 9. | uklonjen `use_final_approach_orientation`, uklonjen `RotationShimController`, `debug_trajectory_details: false`; zadržano **20 Hz, 15.625 trajektorija, 1 cm / 0.01 rad rezolucija**, PAL lidar 1080 zraka / 25 Hz | u tijeku verifikacije | NavFn zadržava točan kut portala ($0.0^\circ$ / $-90.0^\circ$); DWB vodi omni bazu direktno bez shima i bez serijalizacijskog laga |

## Nalazi koji vrijede neovisno o tome koji je sloj u kodu
Izračunati 14. 9. pri analizi pokušaja 10 i 11. Vrijede i za referentni run A, pa ih
treba imati na umu pri svakoj sljedećoj izmjeni:

1. **`scan_filter` mora biti uzak koliko i robot NA VISINI LASERA, ne koliko je najširi.**
   Na 0.2086 m to je 0.195 m polu-širine (ploča u koju je laser ugrađen); ruke su 19 cm iznad.
   Sve šire briše dovratnike i noge stolova. Vrijednost je **0.30 × 0.45**. Staro (0.47):
   pa ih maska od 0.52 briše iz `/scan_filtered` — dakle iz `local_costmap.voxel_layer`
   **i** iz AMCL-a, i to baš dok robot prolazi kroz vrata. Vrijednost je 0.47.
2. **Keepout traka je gotovo jednako uska kao vrata.** Uz `lane_margin −0.05` guide walls
   stoje na ±0.525 m, pa je traka 1.05 m široka: robotu (0.845) ostaje **10.3 cm po strani**
   unutar trake, a fizička vrata (1.0 m) daju 7.75 cm. Traka dakle ne veže — veže dovratnik —
   ali je dovoljno blizu da se oboje mora računati zajedno.
   *(Ranija tvrdnja o 24.8 cm bila je iz zastarjele vrijednosti u bilješkama, ne iz koda.)*
3. **Ploča stola prepušta noge 5 cm po strani** (ploča 0.80, noge 0.70), a lidar na 0.209 m
   vidi samo noge. Ono u što robot udara nije ni u jednom senzorskom sloju.
4. **Kod stola je problem obilazak, ne prilaz** (korisnik, 14. 9.) — **izmjereno na živoj
   maski 14. 9.**: presjek kroz crveni stol pokazuje da je od `x=5.64` do `x=6.19` maska
   potpuno prazna punom širinom halo-a, a tek od `x=6.40` zatvorena. Gate pun 1.80 m nije
   čeoni ulaz nego slobodan koridor uz sam prednji brid ploče, pa je put s jedne strane
   stola na drugu najkraći baš uz stol. Uz to postoji samo **jedno** prilazno lice, pa cilj
   na suprotnoj strani leži u keepoutu i NavFn ga s `tolerance: 0.5` privuče na rub halo-a.
5. `_front_clear()` iz pokušaja 10 nije mogao opaliti: tražio je povrat u koridoru koji je
   maska iz točke 1 već obrisala.
6. **Costmap izmislja prepreku koje nema - tri izvora, svi nasi:**
   `footprint_padding` 0.01 (2 cm, dvostruko brojanje otkad je footprint izmjeren),
   **rezolucija 5 cm** (rezerva od 4 cm po strani je manja od celije, pa se dovratnik
   zaokruzi prema unutra), i **lidar od 360 zraka** (1 stupanj, na 1.8 m to je 3 cm razmaka).
   Prva dva rijesena 14. 9. (padding 0, lokalna rezolucija 0.025); lidar ostaje otvoren
   jer dira model robota, kartu i AMCL.
7. **RViz „Nav2 Goal" zaobilazi `room_navigator`.** `nav2_rviz_plugins/GoalTool` šalje
   izravno akciju `navigate_to_pose`, dakle nema portalnih poza ni okomitog ulaza — planer
   vuče dijagonalu prema cilju. Alat `rviz_default_plugins/SetGoal` („2D Goal Pose"), koji
   objavljuje na `/goal_pose` i ide kroz navigator, **u konfiguraciji dugo nije ni postojao**
   (dodan 14. 9.). Provjera: ako `room_navigator` u logu nema ništa osim `zone graph:`, cilj
   je otišao mimo njega.

## Trenutno rješenje
[[D-16_zones_from_detected_features]] i [[D-18_verified_baseline_first]]. Zone i graf dolaze
iz karte, a **Nav2 vozi svaku dionicu** — i prilaznu, i onu kroz vrata. Kod vrata postoje
samo dvije stvari: portalne poze na osi prolaza (dva cilja po vratima) i preduvjet
`arms_ok()` + `aligned_with()` (8 cm / 5°) koji pošteno stane umjesto da struže.
To je stanje runa A. [[D-17_closed_loop_door_transit]] je **povučena**.

## Sljedeći korak
1. Ponoviti run A i zabilježiti ga u [[runovi]] — dok se ne ponovi, ne dodaje se ništa.
2. Zatim **jedan** inkrement: obilazak stola (točka 4 gore), pa run, pa sljedeći.

Pravilo iz [[D-18_verified_baseline_first]]: ništa što može **odbiti vožnju** ne ulazi u kod
bez runa koji dokazuje da je to odbijanje potrebno.
