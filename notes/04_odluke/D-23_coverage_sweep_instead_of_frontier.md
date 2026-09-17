---
id: D-23
type: odluka
status: odlozena
deviation: false
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]"]
problems: ["[[P-10_skid_steer_cannot_turn]]", "[[P-11_nav2_slam_drift]]", "[[P-47_headless_batch_map_odom_stale]]", "[[P-48_no_path_alignment_critic]]"]
superseded_by: ""
updated: 2026-09-17
---
# D-23 — Mapiranje pokrivanjem soba umjesto frontier istraživanja

## Kontekst
Frontier istraživanje ([[P-48]], [[D-22]]) je udžbenički odgovor (Yamauchi, 1997) i u kodu je
radilo točno ono što piše — a izgledalo je besmisleno. Run 17. 9. 23:33 to pokazuje brojkama:

| Mjera iz runa | Vrijednost |
|---|---|
| poslanih ciljeva u cijelom runu | **3** |
| prvi cilj | `(-0.60, 0.58)` — **0.83 m od robota, na 136°** (iza i lijevo) |
| veličina klastera tog cilja | **4127 ćelija**, jedan jedini povezani klaster |
| vrijeme potrošeno na njega | **90 s**, pa timeout |
| drugi i treći cilj | iza zida, oba „navigation did not succeed" → `stalled` |
| `nav_zones` cijeli run | `no doorways detected in /map - publishing no zones` |

## Uzrok, ne simptom
**Frontier je loš cilj kad senzor nadmašuje sobu.** Lidar doseže 25 m, soba je 6 m — robot vidi
cijelu sobu čim u nju uđe. Ono što ostaje nemapirano nije daleko, nego **zasjenjeno**: iza noge
stola, iza vlastitog tijela. Te sjene čine **prsten** granice oko robota, a težište prstena je
sredina prstena — mjesto gdje robot već stoji. Odatle cilj od 83 cm iza leđa.

Iz istog razloga **prolaz nikad nije prepoznat**: detektor traži klaster koji je *sam po sebi*
tanak i širok kao vrata, a vrata su dio istog povezanog klastera od 4127 ćelija.

## Opcije
1. Popravljati bodovanje granica (najbliža točka klastera umjesto težišta, minimalna udaljenost,
   cijena po duljini putanje). Radi, ali ruta ostaje neizgledna i nepredvidiva — svaki ciklus
   iznova bira cilj, pa se ne može ni nacrtati ni unaprijed provjeriti.
2. **Pokrivanje (boustrophedon) po sobi, pa korak kroz prolaz.** Ruta se izračuna **prije** nego
   se robot pomakne, pa se može nacrtati, provjeriti offline i gledati.

## Odluka
17. 9. 2026., opcija 2 — na korisnikov zahtjev („neka lijepo vozi gore dolje, lijevo desno, jer
imamo omnibazu; ide od zida do zida; zapiše prolaze; ode u sljedeću sobu kroz prolaz").

Novi čvor `room_sweeper` (`explore_mode:=sweep`, zadano). Petlja:

```
nađi prolaze → zapečati ih → pokrij sobu trakama → dopuni rupe →
prođi kroz neposjećen prolaz → ponovi        (kraj: nema prolaza i nema nepoznatog uz dohvat)
```

Tri stvari koje to čine ispravnim, a ne samo ljepšim:

- **Trake koriste bazu.** I traka i korak na sljedeću traku su **čiste translacije** — smjer se
  tijekom pokrivanja **ne mijenja nijednom**. Okret u mjestu je jedino gibanje u kojem ova baza
  kliže ([[P-10]]), a svaki klizaj ide u scan matcher ([[P-11]]). Pokrivanje bez ijednog okreta je
  najbolji ulaz koji SLAM može dobiti — a na toj se karti poslije vozi otvor od 0.98 m.
- **Prolazi se traže kao prekid u zidu, ne kao oblik klastera.** `door_candidates` je dobio
  `both_sides=False`: dok se mapira, daleka strana vrata je nepoznata, pa stroga provjera ne nađe
  **ništa** — što je doslovno ono što `nav_zones` javlja cijeli run.
- **Kroz vrata i dalje vozi Nav2.** Koridor za **središte** robota kroz otvor od 1.0 m je ~12 cm;
  [[P-45]] je izmjerio da vlastiti vozač to ne drži, a Nav2 isti prolaz vozi 5/5 jer provjerava
  **otisak**. Pokrivanje se vozi izravno (otvoren prostor, `collision_monitor` i dalje filtrira),
  prolaz nikad.

## Izmjereno offline prije ijednog runa (`scripts/check_sweep.py`)
| Mjera | Vrijednost |
|---|---|
| prolaza nađeno na **polumapiranoj** karti | **2 / 2**, centri `(0.00, −2.99)` i `(3.01, 0.00)`, širina 0.980 m |
| isto sa starom strogom provjerom | **0 / 2** ← uzrok `no doorways detected` |
| prolaza na gotovoj karti, `both_sides=True` | **2 / 2**, nepromijenjeno → misija nije dirnuta |
| ruta u polaznoj sobi | 34.2 m², 8 točaka, 4 poteza po x + 3 po y, **0 dijagonalnih** |
| pokrivenost | najdalja ćelija **0.95 m** od rute (granica 1.35 m) |
| izlazak iz sobe kroz vrata | **0 točaka** — pečaćenje drži rutu u sobi |

## Posljedice
- [[D-22]] (profil mapiranja `FollowPathExplore`) **ostaje** i dalje vrijedi, ali sada se koristi
  samo ako se izričito traži `explore_mode:=frontier`. Prolazne poze idu zadanim stablom.
- `frontier_explorer` se **ne briše** — ostaje kao usporedba i kao zapis pokušaja ([[D-18]]).
- Pretpostavka koja se uvodi i mora se izgovoriti: **sobe su pravokutne i poravnate s osima karte.**
  Ne pretpostavlja se ni koliko ih je, ni kojim redom, ni kako se zovu.
- `room_sweeper` mjeri starost `/scan_filtered`, ne `map → odom`: slam_toolbox reobjavljuje tu
  transformaciju 50×/s sa svježim žigom i kad skenovi stanu, pa je watchdog na njoj slijep ([[P-47]]).

## Prva vožnja (18. 9.) i što je popravljeno
Korisnik: *„na početku se robot ok giba, ali previše vremena je u početnoj sobi i nikad nije prošao
kroz prolaz."* Gibanje je dakle bilo dobro — tri greške su trošile vrijeme, sve tri moje:

| # | Nalaz iz loga | Uzrok | Popravak |
|---|---|---|---|
| 1 | `soba 1: soba 5.0 m², 6 točaka`, pa poslije `34.0 m²` | ruta planirana dok je SLAM tek gradio sobu | `wait_for_map()` — čeka da poznata površina prestane rasti (`map_settle_*`) |
| 2 | `patching: još 4485 nepoznatih ćelija` nakon čiste sobe | mjera zatvorenosti brojala **druge sobe iza vrata**, pa se dopunska tura okidala uvijek | `unknown_reachable(..., sealed)` — „rupe u ovoj sobi" i „je li karta gotova" su sad dva pitanja |
| 3 | `straight line blocked, planning instead` na pola dionica | test ravne linije tražio isti zazor (0.60 m) kao postavljanje traka | zaseban `line_clearance` 0.50 m |
| 4 | — | soba je mogla pojesti cijeli run | `room_budget` (240 s): kad istekne, sljedeći prolaz vrijedi više od još jedne trake |

Uz #3: zabrana klizanja kroz vrata **ne smije** visjeti o broju. Najveći zazor u otvoru od 0.98 m
izmjeren je **0.490 m**, a prag je 0.50 — margina od 10 mm, i vrata od 1.05 m bi tiho postala
prohodna klizanjem. Zato `line_is_clear` sada dobiva **pečate prolaza** i zabrana vrijedi po
konstrukciji, neovisno o širini vrata.

Izmjereno nakon popravka: „samo ova soba" daje **0** nepoznatih ćelija za čistu polaznu sobu, dok
„cijela zgrada" daje > 0 dok god neka soba nije posjećena — dakle dopunska tura se više ne okida bez
razloga.

## Druga vožnja (18. 9., 00:15) — čvor se srušio prije prolaza
Korisnik: *„u trenutku … mapiranje home sobe je bilo gotovo, ali on je nastavio. Zašto nismo otišli
u drugu sobu?"* Odgovor iz loga: **nije ni stigao do tog koraka.**

```
ValueError: operands could not be broadcast together with shapes (596,597) (595,595) (596,597)
  room_sweeper.py:163 in unknown_reachable   ->   standable &= ~sealed
  room_sweeper.py:648 in closure(this_room_only=True)
```

| # | Nalaz | Uzrok | Popravak |
|---|---|---|---|
| 5 | čvor umro nakon prve ture, 225 s u run | **pečati prolaza su bili keširani**, a SLAM karta raste pa joj se mijenja oblik rešetke; maska iz ranije ture više ne pristaje | `seals()` gradi masku **iz trenutne karte pri svakom pozivu**; ništa se ne kešira |
| 6 | `karta se smirila (13795 ćelija)` → `soba 5.0 m²` od 34 | čekanje je završilo **pošteno** — karta se stvarno prestala mijenjati, jer slam_toolbox dodaje sken u graf tek kad se robot **pomakne** (`minimum_travel_distance`). Karta mirnog robota se smiri **rano**, ne **potpuno** | ruta se **preplanira dok soba raste** (`replan_growth` 1.4, `max_replans` 3) — čekanje više nije jedina obrana |

Uvjet pada je reproduciran offline (maska s rešetke 595×595 nad kartom 597×597 i dalje baca istu
iznimku; maska izgrađena iznova prolazi), pa popravak nije pretpostavka.

> [!warning] Ne kešurati ništa što je vezano uz oblik SLAM rešetke
> Karta raste tijekom runa. Svaka maska, indeks ili polje izvedeno iz `map.info` vrijedi samo za
> onu poruku iz koje je nastalo.

## Treća vožnja (18. 9., 00:29) — pokrivanje radi, ali se vozi bez razloga
Preplaniranje je proradilo (`soba je narasla 5.0 → 34.1 m²`), soba je pometena do
`nepoznato u sobi: 0`, i robot je krenuo na prolaz. Prolaz je pao, i tada:

```
+122.1  swept: nepoznato u sobi: 0, ukupno: 690
+122.1  doorway: Prolaz (2.97, 0.00), izmjereno 0.98 m
+163.0  WARN prolaz nije prošao; pokušavam sljedeći
+163.1  sweeping: soba 1: soba 34.1 m², 8 točaka     ← ISTA soba, drugi put
```

| # | Nalaz | Uzrok | Popravak |
|---|---|---|---|
| 7 | ista soba pometena dvaput | petlja se nakon palog prolaza vraćala na trake | pali prolaz vodi na **sljedeći prolaz**, nikad natrag na trake — soba je već pokrivena |
| 8 | sve četiri trake vožene kroz **praznu** sobu | trake su se planirale nad cijelom sobom, bez obzira ima li se što vidjeti | trake se režu na dionice blizu **nepoznatog** (`interest_mask`); soba bez nepoznatog daje **praznu rutu** |
| 9 | `prolaz nije prošao`, bez razloga u logu | `nav_goal` nije ispisivao Nav2 status | sada ispisuje `ABORTED/CANCELED` i koji je cilj bio |

Točka 8 je principijelna, ne ušteda: **lidar od 25 m vidi praznu sobu od 6 m iz bilo koje točke.**
Voziti u njoj četiri trake nije pokrivanje nego gluma pokrivanja, i košta stotinjak sekundi. Trake
postoje zbog **zasjenjenja**, pa se i voze samo tamo gdje zasjenjenja ima.

Provjera je usklađena: `check_sweep` sada ima etapu sa **sjenom iza prepreke** (inače bi nad gotovom
kartom vježbao praznu rutu i ne bi mjerio ništa) i traži oboje — ruta kad ima što vidjeti
(4 točke, najdalja nepoznata ćelija 0.74 m od rute), **prazna ruta kad nema**.

## Četvrta vožnja (18. 9., 00:41) — soba je dobra, prolaz pada na `nav_zones`
Pokrivanje je prošlo kako treba: `replanning 5.0 → 34.1 m²`, pa `nepoznato u sobi: 0`, pa odlazak na
prolaz. Prolaz je pao **dvaput**, a novi ispis statusa je konačno rekao zašto:

```
+169.4  doorway: Prolaz (2.97, -0.00), izmjereno 0.98 m
+198.9  WARN ispred prolaza: Nav2 ABORTED — cilj (1.87, -0.00, 0°)
+228.8  WARN ispred prolaza: Nav2 ABORTED — cilj (1.87, -0.00, 0°)
```

29.5 s po pokušaju = točno `progress_checker.movement_time_allowance` (30 s). Robot se **nije
pomaknuo**. Uzrok je u istom logu, sedam sekundi ranije:

```
nav_zones: door 0 at (2.97, -0.00) does not connect two detected rooms: [None, 'home']
nav_zones: room labels "home" and "blue" resolve to the same room
nav_zones: 1 room(s) ['home'], 1 door(s), 5 table(s), 2 keepout rectangles
nav_zones: 1 room(s) ['home'], 1 door(s), 6 table(s), 2 keepout rectangles
```

**Pet pa šest „stolova" u svijetu koji ih ima dva** — `table_candidates` je našao noge stolova u
fragmentima zidova koje skenovi još nisu spojili. Svaki je dobio halo zabrane na globalnom costmapu,
a poza za poravnanje pred vratima je pala u jedan od njih.

| # | Nalaz | Uzrok | Popravak |
|---|---|---|---|
| 10 | Nav2 ABORTED na pozi pred vratima, 2× | **zone se grade IZ karte, pa se ne mogu graditi DOK se karta gradi** | `zones:=false` tijekom mapiranja; `nav_zones` i `room_navigator` se dižu **nakon predaje karte**, 7 s iza `map_server`-a, pa je prva karta koju vide spremljena |
| 11 | svaki run završavao tracebackom | `stop()` objavljuje nakon što je kontekst ugašen | `stop()` više ne diže iznimku na izlazu |

To je ista pogreška u razmišljanju kao #8, na drugom mjestu: **izvedena veličina vrijedi samo za
ulaz iz kojeg je izvedena.** Zone izvedene iz polumapirane karte nisu „približno točne zone", nego
izmišljene prepreke.

> [!note] Cijena koju treba znati
> Dok su zone ugašene, tijekom mapiranja nema `room_navigator`-a, pa ručni RViz cilj (`/goal_pose`
> je remapiran na `/bt_goal_pose`) nema tko primiti. Mapiranje vozi sweeper; ručni cilj se vraća
> čim karta bude spremljena.

## Peta vožnja (18. 9., 00:48) i odluka da se stane

Gašenje zona nije riješilo prolaz. Nav2 sada odbija **oba** prolaza, na pozi poravnanja koja je u
otvorenom prostoru usred polazne sobe:

```
+156.1  swept: soba 1 pokrivena; nepoznato u sobi: 0
+156.1  doorway: Prolaz (2.97, 0.00), izmjereno 0.98 m
+196.1  WARN ispred prolaza: Nav2 ABORTED — cilj (1.87, 0.00, 0°)
+247.7  WARN ispred prolaza: Nav2 ABORTED — cilj (1.87, 0.00, 0°)
+287.0  WARN ispred prolaza: Nav2 ABORTED — cilj (0.01, -1.85, -90°)
```

**Uzrok nije utvrđen.** U cijelom runu nema nijednog `WARN` ni `ERROR` iz `controller_server`-a,
`planner_server`-a ni `bt_navigator`-a (`quiet:=true` drži nav2 na razini `warn`, a ništa se nije
oglasilo). Dakle: Nav2 je bio podignut, cilj je primio, vratio `ABORTED` — i nije rekao zašto.
Sljedeći korak bi bio run s `quiet:=false`, ali se na tome stalo.

### Što radi, a što ne (stanje na kraju)
| | |
|---|---|
| pokrivanje sobe trakama, bez ijednog okreta | ✅ odvoženo, više puta |
| preplaniranje dok karta raste (5.0 → 34.1 m²) | ✅ odvoženo |
| prepoznavanje prolaza s jedne strane, mjerenje otvora uživo | ✅ 0.98 m, odvoženo |
| **prolazak kroz prolaz** | ❌ **nijednom**, Nav2 `ABORTED` bez objašnjenja |

## Odluka: odloženo (18. 9.)
Korisnik je prekinuo rad na automatskom mapiranju. **Kod ostaje u repozitoriju** (`room_sweeper`,
`frontier_explorer`, `scenario_auto_map.launch.py`, `scripts/check_sweep.py`) i sve offline provjere
prolaze, ali je **izbačen iz README-a i `docs/RUNNING.md`** — predaja navodi samo **ručno mapiranje**,
koje je odvoženo i kojim je napravljena karta u repozitoriju.

> [!warning] Ne navoditi automatsko mapiranje kao mogućnost sustava
> Nijedan run nije prošao do kraja. U seminaru, videu i na slajdovima mapiranje je **ručno**.

🧪 **Odloženo 18. 9. nakon pet vožnji.** Ako se ikad nastavi, jedini sljedeći korak je run s
`quiet:=false` da se vidi zašto Nav2 odbija pozu pred vratima — sve prije toga je nagađanje.
