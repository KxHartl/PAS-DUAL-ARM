---
id: P-39
type: problem
status: rijeseno
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
| 2 | 14. 9. | detekcija vrata iz spremljene karte (`scripts/check_doors.py`, offline) | oboja vrata nađena: (0.00, −3.00) i (3.00, −0.01), promašaj centra 1.5 cm, širina **0.95 m** | karta zadeblja zid ~2.5 cm po strani; zone se grade na 0.95 m, ne na 1.0 m |
| 3 | 14. 9. | stolovi iz karte kao klasteri slobodnostojećih nogu | 8 nogu, 2 stola, centri (−0.04, −6.53) i (6.54, −0.01); karta ima točno 9 komponenti, bez šuma | pojas 0.075–0.125 m u RGB-D detektoru je bio mrtav (ploha je na 0.75 m) |
| 4 | 14. 9. | zone iz detektiranih značajki (`nav_zones`), tolerancije 0.10 / 0.05, shim 0.06, footprint 0.854, `ObstacleFootprint.scale` 1.0, keepout i na lokalnom costmapu | headless: 8/8 kontrolera, filter aktivan na **oba** costmapa, 14/14 sondi costmapa točno | zone stvarno dolaze do upravljača |
| 5 | 14. 9. | izmjeren kut kojim **planirana putanja** siječe prag vrata, 5 slučajeva preko `/compute_path_to_pose` | 0.00–1.07° od okomice, 2.5 cm od osi; traži 0.854–0.873 m | i jedan jedini cilj kroz cijelu sobu sad prolazi okomito |
| 6 | 14. 9. | isto, ali s praznom keepout maskom (A/B, zone isključene) | najgori prolaz 2.92° i 5.6 cm od osi (vs 0.34° i 2.5 cm sa zonama) | **globalnu putanju uglavnom ispravlja već ispravljen footprint**; zone popravljaju najgori slučaj i daju graf i poze. Glavni uzrok kvara bile su tolerancije i nevidljivost zona lokalnom upravljaču |

| 7 | 14. 9. (GUI, korisnik) | prva vožnja sa zonama | ❌ **robot se ne može okrenuti u mjestu kad stoji ispred stola — zapne za zonu.** Izmjereno: prolaz kroz halo stola bio je širok samo stol + 0.20 m, pa je pri okretu kut robota ulazio u halo pokraj prolaza. Usput otkriveno da je i `portal_standoff` 0.75 < potrebnih 0.923 m. | prolaz kroz halo → **puna širina halo-a**; `portal_standoff` 0.75 → 0.95; `chute_length` 1.20 → 0.85. `check_zones.py` sada provjerava okret u mjestu na **svakoj** pozi gdje navigator stane (i portali i prilaz stolu), iz poze pomaknute za toleranciju cilja i uz rezervu 0.15 m |
| 8 | 14. 9. | zaostali `velocity_smoother` iz ranijeg headless testa | ❌ novi `lifecycle_manager` ga našao u stanju `active` → `No transition matching 1 found` → **prekinut cijeli bringup**, karta se ne učita | uvijek provjeriti zaostale čvorove prije starta; upisano u [[01_pokretanje]] |
| 9 | 14. 9. | ekskluzivno poravnanje, lokalni keepout, sirovi sken i zatvorena petlja za uski tranzit | ✅ headless prolaz home→blue: poravnanje uspjelo, tranzit dovršen s minimalnim bočnim razmakom **4.7 cm**, na izlazu +0.2°; ❌ Nav2 prilaz stolu zapeo uz cilj na rubu halo zone | tranzit izdvojen iz DWB-a; `table_standoff` 0.60→0.80 m, potreban retest dolaska stolu i povratka |
| **A** | 14. 9. (GUI, korisnik) | **referentni run, Nav2 vozi sve dionice**, gate = `arms_ok` + `aligned_with` (8 cm / 5°), `square_corners` False, bočno ±0.30, bez lokalne inflacije | ✅ ručni RViz cilj → plava soba za **33.1 s**, Vrata 0 prijeđena s **1.8 cm i 4.2°**; ✅ `goto red` → 5 dionica, pred crvenim stolom za **60.6 s** | **ovo je stanje na koje se vraćamo.** Nije bilo commitano — postojalo je samo kao zapis u `STATE.md` |
| 10 | 14. 9. | na run A naslagan sloj: `realign_to_portal`, `_transit_doorway` (2.9 m s isključenim Nav2), `doorway_margin` umjesto `aligned_with`, prekid kod stola na 25 cm | ❌ **nikad odvoženo**; `aligned_with` ostao u datoteci, ali ga nitko ne zove | novi gate odbija **upravo prolaz iz runa A**: `0.95 − swept(4.2°) − 0.05 = −2.5 cm` → ABORT. Zamijenjen je kriterij koji je propustio stvarni prolaz, bez runa koji bi to opravdao |
| 11 | 14. 9. | na to naslagano još: `envelope_monitor` s `require_envelope` (po defaultu **odbija voziti**), mjerenje iz dovratnika, 3 pokušaja poravnanja, kutni `scan_filter` | ❌ **nikad odvoženo**; korisnik: „trenutačno sustav radi najgore ikad" | dodan još jedan razlog za abort na stanje koje već nije vozilo. Kod je sačuvan na grani `wip/door-transit-closed-loop`; `main` vraćen na run A |

| 12 | 14. 9. | prvi run s ispravnim usmjeravanjem (alat 2D Goal Pose): portal pogoden 9.8 cm / +2.7 stupnjeva, gate prosao | **`No valid trajectories out of 3711`**, status 6, pa Nav2 recovery zavrtio robota u mjestu (to je bio neobjasnjivi yaw) | izmjereno u lokalnom costmapu: u otvoru od **100 cm** slobodno je samo **80 cm**, a robot s paddingom 86.2 cm. DWB je bio u pravu; greska je u tome **sto costmap crta** |

| 13 | 14. 9. | run s 8 vrhova i rezolucijom 0.025 | ⚠ prolaz **uspio**, ali robot stane tocno kad ude u vrata, stoji **~30 s**, pa prode uredno. `Control loop missed` pao s 1184 na 3; nova greska je `Failed to make progress` (×3) | `scan_filter` s `half_width` 0.47 brise bliži dovratnik cim je bocni otklon > 3 cm (dovratnik je na 0.50). Bez povrata voxel sloj te celije **ne moze ni oznaciti ni raytraceom ocistiti**, pa oznake s prilaza ostanu zamrznute u prolazu. 30 s je `movement_time_allowance`; oslobodi ga recovery koji ocisti costmap. Maska suzena na **0.30 × 0.45**, prema stvarnoj sirini sasije na visini lasera |

| 14 | 14. 9. | povratak crvena -> home, isti run | ❌ **`ABORT before leg 2/3: 0.095 m off the lane centreline (limit 0.08)`** — gate je ispravno odbio, robot ostao stajati | kontradikcija u konfiguraciji: `xy_goal_tolerance` 0.10 dopusta Nav2-u da parkira 10 cm od portala, a gate trazi <= 8 cm, pa legalan dolazak moze biti nelegalno stanje **bez ikakvog izlaza**. Otkloni kroz run rastu 0.022 -> 0.042 -> 0.076 -> 0.095. Gate se ne smije popustiti (fizicki budzet 7.75 cm po strani), pa je pritegnuta isporuka: `xy_goal_tolerance` -> **0.05**, `required_movement_radius` -> **0.05** |

| 16 | 14. 9. | nadogradnja na PAL Tiago Omni Base lidar + SE(2) RotationShimController + fino uzorkovanje | ❌ robot stigao 2.9 cm od portala, ali zakrenut na **+132.6°** (`ABORT before leg 2/3`) | Uzrok: `use_final_approach_orientation: true` u NavFn-u prebrisao je kut cilja ($0.0^\circ$) kutom zadnjeg dijagonalnog koraka A* rešetke ($+132.6^\circ$); `RotationShimController` nepotrebno rotirao omni bazu; `debug_trajectory_details: true` serijalizirao 15.625 trajektorija na 20 Hz |
| 17 | 14. 9. | uklonjen `use_final_approach_orientation`, uklonjen `RotationShimController`, `debug_trajectory_details: false`; zadržano **20 Hz, 15.625 trajektorija, 1 cm / 0.01 rad rezolucija**, PAL lidar 1080 zraka / 25 Hz | u tijeku verifikacije | NavFn zadržava točan kut portala ($0.0^\circ$ / $-90.0^\circ$); DWB vodi omni bazu direktno bez shima i bez serijalizacijskog laga |

| 18 | 14. 9. (run 62, GUI) | vožnja oko stolova nakon što su vrata riješena | ❌ **zapeo dvaput**; robot uđe u halo bočno i DWB ostane bez trajektorija. Izmjereno u `nav_zones`: halo crvenog stola je `x ∈ [5.609, 7.409] × y ∈ [−0.892, 0.908]`, a `table_gate` ga buši **punom visinom**: `x ∈ [5.609, 6.159] × y ∈ [−0.892, 0.908]` | Od halo-a ostaje samo pravokutnik **istočno** od stola — docstring tvrdi „block the three sides", kod to ne radi. Ispred stola je slobodan koridor 0.55 × 1.80 m uz sam rub ploče; planeru je to najkraći put preko sobe. Uz to gate seže do `x = 6.159`, a rub ploče je na `6.109` → **5 cm ispod ploče** |
| 19 | 14. 9. | uzrok zašto je gate uopće bio pun: `table_standoff` 0.60 < opisanog radijusa 0.673 | pri okretu u mjestu na prilaznoj pozi kut robota zađe u halo (isto što i pokušaj 7) | Popravljen je bio **otvor**, a ne **odmak**. Kod vrata je isto riješeno ispravno: `portal_standoff = 0.673 + 0.10 + 0.15 = 0.95` |
| 20 | 14. 9. | **halo zatvoren**, `table_gate_margin` uklonjen; `table_standoff` 0.60 → **0.95** (ista izvedba kao portal); novi `table_overhang` **0.05 m** (ploča 0.80 vs detektiranih 0.70 — lidar na 0.209 m vidi samo noge) | `check_zones.py` prolazi: prilazne poze 1.90 m od centra, okret u mjestu čist, **soba i dalje prohodna** (novi test: erozija slobodnog prostora za polovicu širine robota + povezanost svih 11 poza) | Nijedna ruta kroz sobu sad ne može proći bliže stolu od `table_clearance` = 0.55 m. Čeka run 63 |

| 21 | 14. 9. (run 63, GUI, korisnik) | vožnja oko stola sa **zatvorenim** halo-om | ❌ korisnik: „nije rješenje staviti cijeli taj dio oko stola no-go — robot mora doći do stola da uzme i ostavi kutiju". **Veći nalaz (korisnik):** globalna putanja ide **točno uz rub zone**, a rub robota u zonu ne smije — takva putanja je neizvediva | Halo oko stola **uklonjen** u cijelosti. Stol ostaje prepreka u slojevima koji vide stvarne prepreke (static layer = noge, globalni `inflation_layer`) |
| 22 | 14. 9. | uzrok „putanja liže rub zone": NavFn planira **točku** (centar), DWB-ov `ObstacleFootprint` provjerava **pravokutnik 1.04 × 0.854 m** | **Potvrđeno u konfiguraciji:** `inflation_layer` je *plugin*, `keepout_filter` je *filter*, a nav2 filtere obrađuje **nakon** svih plugina → keepout ćelije se upisuju tek kad je inflation gotov, pa se **zone nikad ne napuhuju**. Vidljivo i u RViz-u: zidovi imaju cyan pojas, zone imaju oštar rub | Rješenje: **dvije maske istih zona**. `/keepout_filter_mask` (sirova) → lokalni costmap, gdje se provjerava stvarni otisak; `/keepout_filter_mask_planner` (**napuhana za upisani radijus 0.427 m**) → globalni costmap, gdje planer računa točku. Drugi `costmap_filter_info_server` u `nav2.launch.py` |
| 23 | 14. 9. | `check_zones.py`: nova provjera da napuhana maska **ne zatvori prolaz** | prvo je javila 1.050 m umjesto izračunatih 0.426 m → **našla grešku u mom kodu**: pravokutnici zona dolaze s obrnutim koordinatama (`x[−0.640, −1.290]`), pa je širenje polovicu njih **sužavalo**. Nakon popravka: **0.410 m** za centar robota (izračun 0.426, razlika je kvantizacija ćelije od 2 cm) | Prolaz i dalje otvoren s rezervom: traži se 2 × tolerancija cilja = 0.20 m. Na samim vratima ionako veže inflation oko zidova (0.98 − 2 × 0.427 = 0.126 m za centar), pa napuhana maska tamo ne steže ništa — djeluje samo ondje gdje je i trebala, uz vanjske strane lijevaka |

| 24 | 14. 9. (run 64, GUI) | cilj s druge strane plavog stola | ❌ **timeout 240 s**, robot stoji sjeverno od stola. `No valid trajectories` = **0**, `Failed to make progress` ×4, `collision_monitor` → `FootprintApproach` **20×** | Robota ne zaustavlja costmap nego **`collision_monitor`**. On projicira stvarni otisak naprijed za `HALF_LENGTH + v × time_before_collision` i skalira brzinu. Putanja je zavijala oko stola na **0.451 m**; kako `v → 0`, projekcija i dalje seže **0.52 m** (polovica duljine robota), pa je **nijedna brzina ne oslobađa**. To je zaglavljenje, ne opreznost |
| 25 | 14. 9. | korisnikov nalaz: „zone su postavljene po širini, a po duljini zapne" | **Potvrđeno brojem:** sve margine su izvedene iz **upisanog** radijusa 0.427 m (= polovica širine), a ono što veže pri čeonom prilasku je polovica **duljine** 0.52 m. Razlika 9.3 cm je točno raspon u kojem je putanja legalna, a nevozna | Ne može se popraviti podizanjem tvrdog radijusa: otvor je 0.98 m, a 0.98 − 2 × 0.427 = 12.6 cm za centar; na 0.52 bi se vrata zatvorila |
| 26 | 14. 9. | novi alat `scripts/check_costmap_path.py` — offline imitacija globalnog planera (karta + napuhane zone + `inflation_layer`, Dijkstra po NavFn-ovoj cijeni) | Prvi pokušaj je javio 27 nedostupnih parova → **našao grešku u modelu**: napuhavao je i zone, a nav2 ih ne napuhuje (filteri idu nakon plugina). Nakon ispravka reproducira stvarni broj: **0.451 m** razmaka od stvarnih prepreka | Sad postoji brojka kojom se promjena costmapa ocjenjuje **prije** vožnje; gate pada ako putanja prođe bliže od 0.52 m |
| 27 | 14. 9. | `inflation_radius` 0.45 → **0.85**, `cost_scaling_factor` 5.0 → **2.0** (gradijent, **ne** tvrdi radijus) | offline: razmak **0.451 → 0.851 m**, sve poze i dalje dostupne, koridor kroz vrata nepromijenjen (12.6 cm za centar) | Na 0.45 je gradijent bio širok **2.3 cm** (0.427 → 0.45), pa planer nije imao razloga preferirati sredinu slobodnog prostora i crtao je uz rub. Čeka run 65 |

| 28 | 14. 9. (korisnik) | zahtjev: umjesto zaustavljanja, ponašati se kao u magnetskom polju — prepreke odbijaju, put privlači | **Privlačenje** već postoji (`PathDist.scale: 96`, `GoalDist: 24`). **Odbijanje u lokalnom costmapu ne postoji**: `local_costmap` nema `inflation_layer`, pa su sve ne-lethal celije cijene **0**, a `ObstacleFootprintCritic` na lethal **odbaci** trajektoriju. To nije polje nego litica: 0, pa beskonacno | Polje zato mora zivjeti u **globalnom** sloju (napuhavanje 0.85/2.0, pokusaj 27) i u `collision_monitor`-u, koji brzinu **skalira** proporcionalno. Lokalno odbojno polje je ovdje **geometrijski nemoguce**: otvor 0.98 m i robot 0.854 m daju **6.3 cm po strani**, pa svaki odbojni pojas siri od 6.3 cm zatvara vrata — sto je i bio razlog zasto je `inflation_layer` izbacen iz lokalnog costmapa (vec zabiljezena slijepa ulica) |

| 29 | 14. 9. (run 65, GUI) | `inflation_radius` 0.85 | ❌ prolaz pao: `Failed to create a plan from potential when a legal potential was found` ×5 + `No valid trajectories out of 17575` ×42 | **NavFn vadi putanju gradijentnim spustom po potencijalu.** Cijena `50 + 0.8 × v` zasićuje na 253; polje dosega 0.85 m stavi cijeli koridor otvora (centar 0.427–0.49 m od dovratnika) na 228–252, dakle ravno na stropu. Loše putanje koje ipak izvuče liježu uz dovratnik, pa i DWB ostane bez trajektorija |
| 30 | 14. 9. | `inflation_radius` → **0.45** (korisnik) | koridor otvora opet na cijeni 0, prohodan; razmak od stvarnih prepreka natrag na 0.451 m | **Ovo je NavFn-ov strukturni kompromis:** polje ne smije doseći dalje od 0.45 m ili se otvor od 0.98 m ne da isplanirati, a na 0.45 m je polje široko 2.3 cm pa ga praktički nema. Izlaz nije u ugađanju ovog broja nego u: (a) `SmacPlanner2D` — A* po istom costmapu, bez gradijentnog spusta, ima `cost_travel_multiplier`; ili (b) **graduirana** planerska maska samo oko stolova (`nav_zones` već ima zasebnu masku za planer), koja polje stavlja tamo gdje treba i nigdje blizu vrata |

| 31 | 14. 9. (run 67, GUI) | povratak na `inflation_radius` 0.45 | ❌ robot opet stao kraj crvenog stola — **točno kako je offline i predviđeno** (0.451 m < 0.52 m) | Potvrda da prag iz `check_costmap_path.py` vrijedi: predvidio je ishod prije vožnje |
| 32 | 14. 9. | **graduirano polje oko stolova, u maski umjesto u `inflation_layer`-u** (`Zones.table_field`: radijus 0.90 m, vrh 60) | offline: razmak putanje od stvarnih prepreka **0.451 → 0.720 m**; maska na osi oba prolaza **0**; sve tri offline provjere prolaze | **Ključna razlika:** `inflation_layer` je globalan i neizbježno pogodi otvor, a masku crtamo gdje hoćemo. Provjereno u nav2 izvoru: `Costmap2D(OccupancyGrid)` pretvara 0–100 **linearno** u 0–254, a `KeepoutFilter` uzima **maksimum** — dakle vrijednost ispod 100 je cijena, ne zid. Vrh 60 → cijena 152 → NavFn 172 naspram 50 za slobodan pod: dovoljno da savije putanju, daleko od stropa 253 koji ruši gradijentni spust. Polje ide u **obje** maske, pa odbijanje ima i DWB, ne samo planer |

| 33 | 14. 9. | **jedno polje umjesto tri izvora odbijanja** ([[D-20_single_potential_field_costmap]]): `inflation_layer` ugašen, `Zones.field` računa cijelu plohu iz karte, brazde nulte cijene kroz vrata i niz prilaz stolu, anizotropna jezgra kod stola | offline sve tri provjere prolaze: brazde nulte cijene s netaknutom jezgrom, koridor kroz vrata **0.110 / 0.130 m** za centar (sam otvor dopušta 0.126 — zone nisu ograničenje), razmak putanje od stvarnih prepreka **0.835 m**, dock čelo **10.0 cm** od ploče | Gate je usput našao **dvije prave greške**: rubna ćelija brazde nije bila nulta (adresiranje ćelije po donjem lijevom kutu, trebao je cijeli `resolution` zazora), i provjera koridora kroz vrata mjerila je prema toleranciji cilja umjesto prema samom otvoru |

| 34 | 14. 9. (run 70, GUI, korisnik) | vožnja s jedinstvenim poljem | ✅ **„super radi“** — oba prolaza u oba smjera, obilazak stolova bez zapinjanja, dock i izlazak unatrag | **Riješeno.** Rješenje je [[D-20_single_potential_field_costmap]]: jedno polje umjesto tri izvora odbijanja, brazde nulte cijene kroz prolaze i prilaze, anizotropna jezgra kod stola |

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
[[D-20_single_potential_field_costmap]] (naslijedio [[D-16_zones_from_detected_features]]) i [[D-18_verified_baseline_first]].
Jedinstveno potencijalno polje u `nav_zones.py` (`field_width: 0.30 m`, `field_peak: 35`) sa nultim brazdama kroz prolaze i prilaz stolu. Globalni `inflation_layer` je ugašen (`enabled: false`). Uvedena geometrija dockinga (`table_dock_safety: 0.10 m`) uz obvezni jednodimenzionalni izlazak unatrag (`_undock_leg`). Dva keepout filtera (sirovi za DWB, napuhan za 0.427 m za NavFn). Putanje drže stabilan razmak 0.715 m od stvarnih prepreka.

Pravilo iz [[D-18_verified_baseline_first]]: ništa što može **odbiti vožnju** ne ulazi u kod
bez runa koji dokazuje da je to odbijanje potrebno.

