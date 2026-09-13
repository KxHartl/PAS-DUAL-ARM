---
id: D-16
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]"]
problems: ["[[P-39_nav2_enters_doorway_at_an_angle]]", "[[P-12_door_too_narrow]]", "[[P-35_arm_span_too_wide_for_door]]"]
superseded_by: ""
updated: 2026-09-14
---
# D-16 — Zone orijentacije generirane iz prepoznatih značajki

## Kontekst
Robot je ulazio u prolaz od 1.0 m pod kutem i strugao uz stol
([[P-39_nav2_enters_doorway_at_an_angle]]). Traženo ponašanje nije najkraća putanja nego
**predvidljivo pravokutno gibanje**: oko svake kritične značajke mora postojati prostor unutar
kojeg je robot prisiljen biti orijentiran okomito/paralelno na nju. Korisnik je 14. 9. tražio da
zone **ne budu ručno upisane**, nego da ih SLAM prepozna i da se postave na prepoznati otvor vrata.

## Opcije
1. **Samo pritegnuti Nav2** (tolerancije, kritičari) i zadržati ručno upisane waypointe.
   Najmanje posla, ali zone ostaju magični brojevi na dva mjesta i ne prate svijet.
2. **Izaći iz Nav2 u vratima** — determinističko poravnanje pa ravna vožnja
   ([[D-15_door_transit_behaviour]]). Najizvjesnije, ali Nav2 prestaje biti ono što vozi, a
   [MAIL] traži Nav2 do regije.
3. **Zone iz detektiranih značajki, Nav2 i dalje vozi** (izabrano).

## Odluka
Izabrana opcija 3, 14. 9. 2026. (`497a69a`).

- `feature_registry` iz karte daje **vrata** (centar, normala zida, širina) i **stolove**
  (klasteri slobodnostojećih nogu; ravnina skena na 0.209 m siječe noge, pa je stol u karti
  čim je soba mapirana — pouzdanije od RGB-D plohe, koja ovisi o tome gleda li kamera).
- `nav_zones` iz toga gradi: **lijevke** uz svaka vrata koji ostavljaju traku širine izmjerenog
  otvora i dugu 1.2 m sa svake strane; **halo** oko svakog stola, **otvoren na licu s kojeg se
  prilazi**; te **graf soba** — sobe su ono na što se slobodan prostor raspadne kad se prolazi
  zatvore. Objavljuje `/keepout_filter_mask` izravno (umjesto `filter_mask_server` i
  predgotovljenog PGM-a), `/nav_graph` i `/nav_zones_markers`.
- `room_navigator` vozi po tom grafu: **dva cilja po vratima** na osi prolaza, kut se umeće da
  dionica kroz praznu sobu skrene umjesto da presiječe dijagonalu, a prije svakog prolaza se
  provjerava poza ruku i stvarna poravnatost.
- Nav2 ostaje vozač; promijenjeno je **u čemu smije planirati**.

**Jedini ručni podatak su imena soba.** Occupancy grid nema boju, pa parametar `room_labels`
pridružuje ime („blue") onoj detektiranoj sobi koja sadrži zadanu točku. Sva geometrija — centri,
normale, širine, portali, prilazne poze — dolazi iz karte.

## Posljedice
- Zone prate svijet: nova karta → nove zone, bez ijednog broja u kodu. Nema više razilaženja
  između maske i waypointa (prije: `generate_keepout_mask.py` i `go_to_room.py`, oba obrisana).
- Lijevac je namjerno **5 cm po strani širi** od detektiranog otvora. Guide walls su u costmapu
  letalne; da su uže od vrata, one bi, a ne sam prolaz, mjerile stane li robot.
- Provjerljivo bez simulatora: `scripts/check_doors.py` i `scripts/check_zones.py` puštaju iste
  detektore i istu geometriju nad spremljenom kartom.
- Cijena: dva nova čvora (~600 redaka) i ovisnost o tome da detekcija nađe oboja vrata. Ako ne
  nađe, `nav_zones` ne objavljuje zone i `room_navigator` odbija planirati — pošteno stane
  ([[D-12_honesty_abort_over_fake]]), ne vozi naslijepo.
- [[D-15_door_transit_behaviour]] ostaje kao plan B ako se uživo pokaže da 4.8 cm po strani nije
  dovoljno za DWB; bočno centriranje mecanum strafeom uz fiksni yaw je sljedeći korak, a ne
  širenje vrata.

## Odnos prema zahtjevima
Ispunjava [[R-15_region_goal_nav2]] (Nav2 i dalje vozi do regije koju zada korisnik) i radi u
korist [[R-18_door_pass_empty]] / [[R-19_door_pass_with_box]]. Za seminar: pokazuje lanac
**SLAM → semantička karta (vrata, stolovi, sobe) → graf navigacije → Nav2**, umjesto ručno
upisanih koordinata. Nije odstupanje.
