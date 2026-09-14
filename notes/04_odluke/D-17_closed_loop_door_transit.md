---
id: D-17
type: odluka
status: zamijenjena
deviation: false
requirements: ["[[R-15_region_goal_nav2]]", "[[R-18_door_pass_empty]]", "[[R-19_door_pass_with_box]]"]
problems: ["[[P-39_nav2_enters_doorway_at_an_angle]]"]
superseded_by: "[[D-18_verified_baseline_first]]"
updated: 2026-09-14
---
# D-17 — Zatvorena petlja u uskom prolazu (POVUČENA)

> [!warning] Povučena 14. 9. 2026., zamijenjena s [[D-18_verified_baseline_first]]
> Ova je odluka zamijenila gate `aligned_with()` (8 cm / 5°) izračunom `doorway_margin`,
> a taj izračun **odbija upravo onaj prolaz koji je istog dana stvarno uspio**: pri 4.2° i
> savršenom centriranju daje −2.5 cm. Uz to je tranzit od 2.9 m vozila zatvorena petlja s
> isključenim Nav2. **Nijedan metar po ovoj odluci nije odvožen** — headless prolaz naveden
> niže bio je jedini test, a u GUI-ju je sustav prestao voziti.
> Kod je sačuvan na grani `wip/door-transit-closed-loop`. Dijelovi se mogu vratiti, ali
> svaki uz run koji dokazuje da je potreban ([[P-39_nav2_enters_doorway_at_an_angle]],
> pokušaji 10 i 11).

Nav2 vodi robota do portalne poze i po sobi. DWB je u otvoru širine 0.95 m povremeno
zadavao kut koji smanjuje bočni razmak ispod sigurnosnog praga ili nije nalazio valjanu
trajektoriju. Zato `room_navigator` nakon zaustavljanja preuzima ekskluzivni upravljački
kanal kroz `cmd_vel_relay` za poravnanje (bočni mecanum pomak i okret) te za ravni
tranzit do izlaznog portala. To je dopuna [[D-16_zones_from_detected_features]]:
zone i graf i dalje nastaju iz detektiranih vrata i stolova, a Nav2 ostaje vozač ostalih dionica.

Tranzitna petlja koristi svježu AMCL pozu, odometriju, sirovi `/scan` za dovratnike,
filtrirani sken ispred robota i zglobove ruku. Napredovanje se pauzira kad geometrijska
rezerva širine postane negativna; robot se bočno ispravlja, a nakon 3 s bez oporavka
prekida. Ispod 2.5 cm izmjerenog bočnog razmaka, pri opasnosti od stola ili zastarjelim
povratnim podacima odmah šalje nulu. Relay ima watchdog 0.30 s i najam 60 s.

U headless Gazebu jedan prolaz home→blue uspio je s najmanjim izmjerenim bočnim
razmakom 4.7 cm i završnim odstupanjem kuta 0.2°. Dolazak do stola i povratni
smjerovi zahtijevaju zasebnu provjeru; ova odluka nije potvrda cijele misije.
