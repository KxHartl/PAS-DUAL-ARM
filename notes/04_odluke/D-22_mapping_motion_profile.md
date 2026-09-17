---
id: D-22
type: odluka
status: vazeca
deviation: false
requirements: ["[[R-14_slam_mapping]]", "[[R-15_region_goal_nav2]]"]
problems: ["[[P-10_skid_steer_cannot_turn]]", "[[P-11_nav2_slam_drift]]", "[[P-48_no_path_alignment_critic]]"]
superseded_by: ""
updated: 2026-09-17
---
# D-22 — Mapiranje vozi vlastiti profil gibanja, misija ostaje netaknuta

## Kontekst
Do 17. 9. je automatsko mapiranje vozilo istim Nav2 profilom kao misija. Posljedica, vidljiva u
GUI-ju i mjerljiva iz logova runa od 17. 9.:

- **pirueta na kraju svake dionice** — `frontier_explorer` je slao yaw uz svaki cilj granice, a
  `general_goal_checker` traži 0.025 rad (1.4°) uz `RotateToGoal.scale: 64`;
- **`spin` recovery** (1.57 rad u mjestu) nakon svakog palog cilja — dvaput u tom runu;
- **vožnja unatrag kao obično gibanje** — `min_vel_x: -0.30`.

Okret u mjestu je upravo gibanje u kojem ova baza kliže (`mu2 = 0`, [[P-10_skid_steer_cannot_turn]]):
odometrija laže o yawu, scan matcher to ispravlja, i korisnik u RViz-u vidi kako se scan okreće dok
zidovi stoje. [[P-11_nav2_slam_drift]] isto stoji na spinovima uz SLAM.

Cijena tih okreta plaća se **kartom na kojoj će misija poslije voziti kroz otvor od 0.98 m sa
7.3 cm rezerve po strani**. Orijentacija na granici pritom ne nosi nijedan bit informacije — lidar
vidi 360°.

## Opcije
1. Popustiti `yaw_goal_tolerance` globalno — pokvarilo bi misijske portalne poze, gdje je uska
   tolerancija izmjerena potreba ([[P-39_nav2_enters_doorway_at_an_angle]]).
2. Maknuti `spin` i vožnju unatrag globalno — dira odvožen misijski lanac, protiv
   [[D-18_verified_baseline_first]].
3. **Drugi profil, biran po cilju.** Nav2 to već podržava: `FollowPath` BT čvor ima ulaze
   `controller_id` i `goal_checker_id`, a `NavigateToPose` ima polje `behavior_tree`.

## Odluka
17. 9. 2026., opcija 3. Uvedeni su:

- `behavior_trees/explore_to_pose.xml` — kopija zadanog stabla bez `Spin` u oporavku
  (`ClearCostmap`, `Wait`, `BackUp` ostaju);
- `explore_goal_checker` — `xy 0.25 m`, `yaw 3.15 rad` (bilo koji kurs);
- `FollowPathExplore` — `min_vel_x: -0.10`, bez `RotateToGoal`, s dodanim `PathAlign` (32) i
  `GoalAlign` (8), `max_vel_theta: 0.25`.

> [!warning] Prva verzija je zaustavila robota
> `min_vel_x: 0.0` bez kritike koja nagrađuje okretanje prema putanji učinila je **mirovanje**
> najbolje ocijenjenom opcijom — 28 s stajanja bez ijedne greške u logu
> ([[P-48_no_path_alignment_critic]]). Vožnja unatrag je zato vraćena kao **sporo** gibanje
> (−0.10 m/s), a nedostajuća ideja („okreni se prema putanji") dodana kao kritika.

`frontier_explorer` šalje **ciljeve granica** tim stablom i s kursom koji robot već ima; **poze
vrata i dalje idu zadanim stablom** i uskim `general_goal_checker`-om, jer je poravnanje pred
otvorom onaj jedan okret u mjestu koji se isplati.

## Posljedice
- Misijski lanac (`FollowPath`, `general_goal_checker`, zadano stablo) nije dirnut ni u jednom retku.
- Unatrag i dalje postoji, ali samo kao `BackUp` oporavak — izlaz iz zaglavljenja, ne obrazac vožnje.
- Bez `Spin` oporavka robot ima jedan izlaz manje iz uske situacije; pokrivaju ga čišćenje costmapa,
  `BackUp` i crna lista u istraživaču. Ako se u vožnji pokaže da to nije dovoljno, to je red u
  P-kartici, ne razlog za vraćanje spina uz SLAM.
- Mapiranje se okreće sporije (0.25 rad/s), pa je nešto sporije — to je ista brzina koju ručna tura
  već koristi iz istog razloga.

## Odnos prema zahtjevima
Ispunjava [[R-14_slam_mapping]] i [[R-15_region_goal_nav2]] bez odstupanja: mijenja se **kako** se
mapira, ne **što** se traži. U seminaru ide kao mjerena posljedica [[P-10_skid_steer_cannot_turn]] —
profil gibanja pri mapiranju bira se prema tome što senzor treba, a ne prema tome što je zadano.

🧪 **Napisano 17. 9., čeka vožnju** — upute [[automated_mapping]].
